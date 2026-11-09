#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <exception>
#include <functional>
#include <future>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

#include <threepp/canvas/EglContext.hpp>
#include <threepp/loaders/GLTFLoader.hpp>
#include <threepp/threepp.hpp>

#include <tesseract/common/joint_state.h>
#include <tesseract/common/logging.h>
#include <tesseract/environment/environment.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/visualization/markers/marker.h>
#include <tesseract/visualization/threejs/conversions.h>
#include <tesseract/visualization/threepp/threepp_visualization.h>

// This namespace is also called threepp, so the library has to be named from the global scope
namespace tp = ::threepp;

namespace tesseract::visualization::threepp
{
using tesseract::visualization::threejs::ViewerMarker;

namespace
{
constexpr double UNTIMED_TRAJECTORY_DURATION = 5.0;
constexpr float PI_2 = 1.57079632679489661923F;

/** @brief GLAD's function pointers are process wide, so one object renders at a time */
std::atomic_bool context_in_use{ false };

using JointValues = std::unordered_map<std::string, double>;

struct JointInfo
{
  std::string name;
  tp::Vector3 axis;
  tesseract::scene_graph::JointType type;
};

/** @brief Every joint's value at each trajectory time, including mimic joints */
struct Playback
{
  std::vector<double> times;
  std::vector<JointValues> states;
  double duration() const { return times.back(); }
};

JointValues toJointValues(const tesseract::scene_graph::SceneState& state)
{
  JointValues values;
  values.reserve(state.joints.size());
  for (const auto& [id, value] : state.joints)
    values.emplace(id.name(), value);
  return values;
}

JointValues sample(const Playback& playback, double time)
{
  const auto& times = playback.times;
  if (time <= times.front() || times.size() == 1)
    return playback.states.front();
  if (time >= times.back())
    return playback.states.back();

  const auto after = static_cast<std::size_t>(std::upper_bound(times.begin(), times.end(), time) - times.begin());
  const std::size_t before = after - 1;
  const double span = times[after] - times[before];
  const double blend = span > 0 ? (time - times[before]) / span : 0.0;

  JointValues values = playback.states[before];
  for (auto& [name, value] : values)
    if (const auto it = playback.states[after].find(name); it != playback.states[after].end())
      value += blend * (it->second - value);
  return values;
}

tp::Vector3 toVector(const std::array<double, 3>& v)
{
  return { static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2]) };
}

tp::Color toColor(const ViewerMarker& marker)
{
  return { static_cast<float>(marker.color[0]), static_cast<float>(marker.color[1]), static_cast<float>(marker.color[2]) };
}

std::shared_ptr<tp::Material> surfaceMaterial(const ViewerMarker& marker)
{
  auto material = tp::MeshPhongMaterial::create();
  material->color = toColor(marker);
  material->opacity = static_cast<float>(marker.color[3]);
  material->transparent = marker.color[3] < 1.0;
  material->side = tp::Side::Double;
  return material;
}

/** @brief The same geometry conventions as the three.js client, which convertMarker's output expects */
std::shared_ptr<tp::Object3D> markerObject(const ViewerMarker& marker)
{
  const auto length = [&marker] { return static_cast<float>(marker.length.value()); };
  const auto radius = [&marker] { return static_cast<float>(marker.radius.value()); };

  // An arrow is placed by its origin, ignoring the marker's pose
  if (marker.marker_type == "arrow")
    return tp::ArrowHelper::create(toVector(marker.direction.value()), toVector(marker.origin.value()), length(), toColor(marker));

  std::shared_ptr<tp::Object3D> object;
  if (marker.marker_type == "axes")
    object = tp::AxesHelper::create(static_cast<float>(marker.size.value()));
  else if (marker.marker_type == "box")
  {
    const auto& extents = marker.extents.value();
    object = tp::Mesh::create(tp::BoxGeometry::create(static_cast<float>(extents[0]), static_cast<float>(extents[1]), static_cast<float>(extents[2])),
                              surfaceMaterial(marker));
  }
  else if (marker.marker_type == "sphere")
    object = tp::Mesh::create(tp::SphereGeometry::create(radius(), 32, 16), surfaceMaterial(marker));
  else if (marker.marker_type == "cylinder")
    object = tp::Mesh::create(tp::CylinderGeometry::create(radius(), radius(), length(), 32), surfaceMaterial(marker));
  else if (marker.marker_type == "cone")
    object = tp::Mesh::create(tp::ConeGeometry::create(radius(), length(), 32), surfaceMaterial(marker));
  else if (marker.marker_type == "capsule")
    object = tp::Mesh::create(tp::CapsuleGeometry::create(radius(), length()), surfaceMaterial(marker));
  else if (marker.marker_type == "lines")
  {
    std::vector<tp::Vector3> points;
    for (const auto& vertex : marker.vertices.value())
      points.push_back(toVector(vertex));
    auto geometry = tp::BufferGeometry::create();
    geometry->setFromPoints(points);
    auto material = tp::LineBasicMaterial::create();
    material->color = toColor(marker);
    object = tp::Line::create(geometry, material);
  }
  else
    throw std::invalid_argument("Unknown marker type " + marker.marker_type);

  object->position.copy(toVector(marker.position));
  const auto& q = marker.quaternion;  // w, x, y, z
  object->quaternion.set(static_cast<float>(q[1]), static_cast<float>(q[2]), static_cast<float>(q[3]), static_cast<float>(q[0]));
  return object;
}

/** @brief Everything that touches OpenGL. It is created, used and destroyed on the render thread */
class Renderer
{
public:
  explicit Renderer(const ThreeppVisualizationOptions& options, bool headless) : headless_(headless)
  {
    if (headless)
    {
      tp::EglContext::Parameters parameters;
      parameters.width = options.width;
      parameters.height = options.height;
      parameters.requireHardware = false;  // Containers and CI have no GPU
      egl_ = std::make_unique<tp::EglContext>(parameters);
      gl_ = std::make_unique<tp::GLRenderer>(std::pair<int, int>{ options.width, options.height });
    }
    else
    {
      canvas_ = std::make_unique<tp::Canvas>(tp::Canvas::Parameters()
                                                 .title(options.title)
                                                 .size(options.width, options.height)
                                                 .exitOnKeyEscape(false));
      gl_ = std::make_unique<tp::GLRenderer>(*canvas_);
    }
    gl_->setClearColor(tp::Color(0x000000));

    scene_ = tp::Scene::create();
    // Matches the brightness of the three.js client, whose r153 lights are not physically scaled
    scene_->add(tp::HemisphereLight::create(0xffffff, 0x202018, 2 * PI_2));

    // tesseract is Z up, three.js and threepp are Y up
    z_up_ = tp::Group::create();
    z_up_->rotateX(-PI_2);
    scene_->add(z_up_);

    camera_ = tp::PerspectiveCamera::create(45, static_cast<float>(options.width) / static_cast<float>(options.height), 0.1F, 1000);
    camera_->position.set(3, 3, -1.5F);
    camera_->lookAt(0, 0, 0);

    if (canvas_ != nullptr)
    {
      controls_ = std::make_unique<tp::OrbitControls>(*camera_, *canvas_);
      canvas_->onWindowResize([this](tp::WindowSize size) {
        camera_->aspect = size.aspect();
        camera_->updateProjectionMatrix();
        gl_->setSize(size);
      });
    }
  }

  tp::Canvas* canvas() { return canvas_.get(); }

  std::string glRendererName() const
  {
    if (egl_ != nullptr)
      return egl_->renderer();
    return "OpenGL window";
  }

  void loadEnvironment(const std::vector<std::uint8_t>& glb, std::vector<JointInfo> joints, const JointValues& values)
  {
    tp::GLTFLoader loader;
    auto result = loader.load(glb);
    if (!result || result->scene == nullptr)
      throw std::runtime_error("threepp could not load the scene");

    z_up_->clear();
    z_up_->add(result->scene);
    root_ = result->scene;
    links_.clear();
    joint_nodes_.clear();
    root_->traverse([this](tp::Object3D& object) {
      if (object.userData.count("tesseract_link") != 0)
        links_[object.name] = &object;
      else if (object.userData.count("tesseract_joint") != 0)
        joint_nodes_[object.name] = &object;
    });
    joints_ = std::move(joints);

    applyJointValues(values);
    for (auto& entry : markers_)
      attach(entry);
    if (playback_)
      applyJointValues(sample(*playback_, playback_time_));
  }

  void applyJointValues(const JointValues& values)
  {
    using tesseract::scene_graph::JointType;
    for (const auto& joint : joints_)
    {
      const auto node = joint_nodes_.find("joint_" + joint.name);
      const auto value = values.find(joint.name);
      if (node == joint_nodes_.end() || value == values.end())
        continue;
      const auto amount = static_cast<float>(value->second);
      if (joint.type == JointType::REVOLUTE || joint.type == JointType::CONTINUOUS)
        node->second->quaternion.setFromAxisAngle(joint.axis, amount);
      else if (joint.type == JointType::PRISMATIC)
        node->second->position.copy(tp::Vector3(joint.axis).multiplyScalar(amount));
    }
  }

  void setStaticState(const JointValues& values)
  {
    playback_.reset();
    applyJointValues(values);
  }

  void setPlayback(Playback playback)
  {
    playback_ = std::move(playback);
    playback_time_ = 0;
    playing_ = !headless_;
    applyJointValues(sample(*playback_, playback_time_));
  }

  void seek(double time)
  {
    if (!playback_)
      return;
    playing_ = false;
    playback_time_ = std::clamp(time, 0.0, playback_->duration());
    applyJointValues(sample(*playback_, playback_time_));
  }

  void addMarkers(std::vector<ViewerMarker> markers, const std::string& ns)
  {
    for (auto& marker : markers)
    {
      markers_.push_back(MarkerEntry{ std::move(marker), ns, nullptr });
      attach(markers_.back());
    }
  }

  void clearMarkers(const std::string& ns)
  {
    const auto end = std::remove_if(markers_.begin(), markers_.end(), [&ns](MarkerEntry& entry) {
      if (!ns.empty() && entry.ns != ns)
        return false;
      if (entry.object != nullptr)
        entry.object->removeFromParent();
      return true;
    });
    markers_.erase(end, markers_.end());
  }

  /** @brief Advance playback by the time since the last frame, looping */
  void advance()
  {
    const auto now = std::chrono::steady_clock::now();
    const double delta = std::chrono::duration<double>(now - last_frame_).count();
    last_frame_ = now;
    if (!playback_ || !playing_ || playback_->duration() <= 0)
      return;
    playback_time_ = std::fmod(playback_time_ + delta, playback_->duration());
    applyJointValues(sample(*playback_, playback_time_));
  }

  void render()
  {
    if (controls_ != nullptr)
      controls_->update();
    gl_->render(*scene_, *camera_);
  }

  ThreeppImage capture()
  {
    render();
    const tp::WindowSize size = gl_->size();
    ThreeppImage image;
    image.width = size.width();
    image.height = size.height();
    const std::size_t row = static_cast<std::size_t>(image.width) * 4;
    std::vector<std::uint8_t> flipped(row * static_cast<std::size_t>(image.height));
    gl_->readPixels(tp::Vector2(0, 0), { image.width, image.height }, tp::Format::RGBA, flipped.data());

    // OpenGL's first row is the bottom one
    image.rgba.resize(flipped.size());
    for (int y = 0; y < image.height; ++y)
      std::copy_n(flipped.begin() + static_cast<std::ptrdiff_t>(row) * (image.height - 1 - y), row,
                  image.rgba.begin() + static_cast<std::ptrdiff_t>(row) * y);
    return image;
  }

  void saveImage(const std::string& path)
  {
    render();
    gl_->writeFramebuffer(path);
  }

private:
  struct MarkerEntry
  {
    ViewerMarker marker;
    std::string ns;
    std::shared_ptr<tp::Object3D> object;
  };

  /** @brief Links can come and go with the scene, so markers are built again on every load */
  void attach(MarkerEntry& entry)
  {
    entry.object = nullptr;
    const auto link = links_.find("link_" + entry.marker.parent_link_name);
    if (link == links_.end())
      return;  // Kept, in case a later scene has the link
    entry.object = markerObject(entry.marker);
    link->second->add(entry.object);
  }

  bool headless_;
  // The context must outlive every other member, and members are destroyed in reverse order
  std::unique_ptr<tp::EglContext> egl_;
  std::unique_ptr<tp::Canvas> canvas_;
  std::unique_ptr<tp::GLRenderer> gl_;
  std::unique_ptr<tp::OrbitControls> controls_;
  std::shared_ptr<tp::Scene> scene_;
  std::shared_ptr<tp::Group> z_up_;
  std::shared_ptr<tp::Group> root_;
  std::shared_ptr<tp::PerspectiveCamera> camera_;

  std::unordered_map<std::string, tp::Object3D*> links_;
  std::unordered_map<std::string, tp::Object3D*> joint_nodes_;
  std::vector<JointInfo> joints_;
  std::vector<MarkerEntry> markers_;

  std::optional<Playback> playback_;
  double playback_time_{ 0 };
  bool playing_{ false };
  std::chrono::steady_clock::time_point last_frame_{ std::chrono::steady_clock::now() };
};

bool resolveHeadless(ThreeppDisplayMode mode)
{
  if (mode != ThreeppDisplayMode::AUTO)
    return mode == ThreeppDisplayMode::HEADLESS;
#ifdef __linux__
  const char* display = std::getenv("DISPLAY");
  return display == nullptr || display[0] == '\0';
#else
  return false;
#endif
}

int parseSize(const char* variable, const char* text)
{
  std::size_t parsed = 0;
  int value = 0;
  try
  {
    value = std::stoi(text, &parsed);
  }
  catch (const std::exception&)
  {
    parsed = 0;
  }
  if (parsed == 0 || text[parsed] != '\0' || value <= 0)
    throw std::invalid_argument(std::string("Invalid ") + variable + ": " + text);
  return value;
}
}  // namespace

ThreeppDisplayMode ThreeppVisualizationOptions::parseMode(const std::string& text)
{
  if (text == "auto")
    return ThreeppDisplayMode::AUTO;
  if (text == "window")
    return ThreeppDisplayMode::WINDOW;
  if (text == "headless")
    return ThreeppDisplayMode::HEADLESS;
  throw std::invalid_argument("Invalid threepp display mode, expected auto, window or headless: " + text);
}

ThreeppVisualizationOptions ThreeppVisualizationOptions::fromEnvironment()
{
  ThreeppVisualizationOptions options;
  if (const char* mode = std::getenv("TESSERACT_VISUALIZATION_THREEPP_MODE"))
    options.mode = parseMode(mode);
  if (const char* width = std::getenv("TESSERACT_VISUALIZATION_THREEPP_WIDTH"))
    options.width = parseSize("TESSERACT_VISUALIZATION_THREEPP_WIDTH", width);
  if (const char* height = std::getenv("TESSERACT_VISUALIZATION_THREEPP_HEIGHT"))
    options.height = parseSize("TESSERACT_VISUALIZATION_THREEPP_HEIGHT", height);
  return options;
}

/** @brief The thread that owns the Renderer, and the queue of work for it */
class ThreeppVisualization::RenderThread
{
public:
  using Command = std::function<void(Renderer&)>;

  /** @throws std::runtime_error if the context could not be created */
  explicit RenderThread(const ThreeppVisualizationOptions& options)
  {
    if (context_in_use.exchange(true))
      throw std::runtime_error("another ThreeppVisualization is already rendering in this process");

    std::promise<std::string> ready;
    auto started = ready.get_future();
    thread_ = std::thread([this, options, &ready] { run(options, ready); });
    const std::string failure = started.get();
    if (!failure.empty())
    {
      thread_.join();
      context_in_use = false;
      throw std::runtime_error(failure);
    }
  }

  ~RenderThread()
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    wake_.notify_all();
    thread_.join();
    context_in_use = false;
  }

  RenderThread(const RenderThread&) = delete;
  RenderThread& operator=(const RenderThread&) = delete;
  RenderThread(RenderThread&&) = delete;
  RenderThread& operator=(RenderThread&&) = delete;

  bool running() const { return running_; }

  void post(Command command)
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (stop_ || !running_)
        return;
      queue_.push_back(std::move(command));
    }
    wake_.notify_all();
  }

  /** @brief Run after everything already posted and wait. Empty if the thread has stopped */
  template <class Result>
  std::optional<Result> call(std::function<Result(Renderer&)> function)
  {
    auto promise = std::make_shared<std::promise<Result>>();
    auto result = promise->get_future();
    post([promise, function = std::move(function)](Renderer& renderer) {
      try
      {
        promise->set_value(function(renderer));
      }
      catch (...)
      {
        promise->set_exception(std::current_exception());
      }
    });
    // Dropped commands break their promise, which get() reports
    return result.get();
  }

private:
  void run(const ThreeppVisualizationOptions& options, std::promise<std::string>& ready)
  {
    try
    {
      Renderer renderer(options, resolveHeadless(options.mode));
      running_ = true;
      ready.set_value("");

      if (auto* canvas = renderer.canvas())
      {
        // Polls input and swaps buffers around every frame
        bool open = true;
        while (open && !stopRequested())
          open = canvas->animateOnce([&] {
            drain(renderer);
            renderer.advance();
            renderer.render();
          });
      }
      else
      {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!stop_)
        {
          wake_.wait(lock, [this] { return stop_ || !queue_.empty(); });
          lock.unlock();
          drain(renderer);
          lock.lock();
        }
      }
      running_ = false;
      // Commands queued after the last drain are dropped here, while the context is still alive
      const std::lock_guard<std::mutex> lock(mutex_);
      queue_.clear();
    }
    catch (const std::exception& e)
    {
      running_ = false;
      try
      {
        ready.set_value(e.what());
      }
      catch (const std::future_error&)
      {
        TESSERACT_LOG_ERROR("tesseract_visualization_threepp: rendering stopped: {}", e.what());
      }
    }
  }

  bool stopRequested()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return stop_;
  }

  void drain(Renderer& renderer)
  {
    std::deque<Command> commands;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      commands.swap(queue_);
    }
    for (auto& command : commands)
    {
      try
      {
        command(renderer);
      }
      catch (const std::exception& e)
      {
        TESSERACT_LOG_ERROR("tesseract_visualization_threepp: plotting failed: {}", e.what());
      }
    }
  }

  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Command> queue_;
  bool stop_{ false };
  std::atomic_bool running_{ false };
  std::thread thread_;
};

ThreeppVisualization::ThreeppVisualization() = default;

ThreeppVisualization::ThreeppVisualization(ThreeppVisualizationOptions options) : options_(std::move(options)) {}

ThreeppVisualization::~ThreeppVisualization() = default;

bool ThreeppVisualization::ensureStarted() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!start_attempted_)
  {
    start_attempted_ = true;
    try
    {
      const ThreeppVisualizationOptions options = options_ ? *options_ : ThreeppVisualizationOptions::fromEnvironment();
      render_thread_ = std::make_unique<RenderThread>(options);
    }
    catch (const std::exception& e)
    {
      TESSERACT_LOG_ERROR("tesseract_visualization_threepp: could not start rendering: {}", e.what());
    }
  }
  return render_thread_ != nullptr && render_thread_->running();
}

template <class Function>
void ThreeppVisualization::guard(Function&& function)
{
  if (!ensureStarted())
    return;
  try
  {
    function();
  }
  catch (const std::exception& e)
  {
    TESSERACT_LOG_ERROR("tesseract_visualization_threepp: plotting failed: {}", e.what());
  }
}

std::string ThreeppVisualization::rootLink() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return root_link_;
}

bool ThreeppVisualization::isConnected() const { return ensureStarted(); }

void ThreeppVisualization::waitForConnection(long /*seconds*/) const { ensureStarted(); }

void ThreeppVisualization::plotEnvironment(const tesseract::environment::Environment& env, std::string /*ns*/)
{
  guard([&] {
    // Reading meshes is the slow part, so it stays off the render thread and a window stays responsive
    auto glb = threejs::environmentToGlb(env);

    std::vector<JointInfo> joints;
    for (const auto& id : env.getJointIds())
    {
      const auto joint = env.getJoint(id);
      joints.push_back(JointInfo{ joint->getName(),
                                  tp::Vector3(static_cast<float>(joint->axis.x()), static_cast<float>(joint->axis.y()),
                                              static_cast<float>(joint->axis.z())),
                                  joint->type });
    }

    {
      const std::lock_guard<std::mutex> lock(mutex_);
      root_link_ = env.getRootLinkId().name();
    }
    render_thread_->post([glb = std::move(glb), joints = std::move(joints), values = toJointValues(env.getState())](
                             Renderer& renderer) mutable { renderer.loadEnvironment(glb, std::move(joints), values); });
  });
}

void ThreeppVisualization::plotEnvironmentState(const tesseract::scene_graph::SceneState& state, std::string /*ns*/)
{
  if (state.joints.empty())
    return;
  guard([&] {
    render_thread_->post([values = toJointValues(state)](Renderer& renderer) { renderer.setStaticState(values); });
  });
}

void ThreeppVisualization::plotTrajectory(const tesseract::common::JointTrajectory& traj,
                                          const tesseract::scene_graph::StateSolver& state_solver,
                                          std::string /*ns*/)
{
  guard([&] {
    if (traj.states.empty())
      throw std::invalid_argument("Trajectory is empty");

    Playback playback;
    playback.times.reserve(traj.states.size());
    playback.states.reserve(traj.states.size());
    for (const auto& state : traj.states)
    {
      playback.times.push_back(state.time);
      // The solver fills in mimic joints, which a trajectory does not list
      playback.states.push_back(toJointValues(state_solver.getState(state.joint_ids, state.position)));
    }

    // Animation runs on time, so a path with no timing is spread evenly, as Ignition does
    const bool untimed = playback.times.size() > 1 && std::all_of(playback.times.begin(), playback.times.end(), [&](double time) {
                           return time == playback.times.front();
                         });
    if (untimed)
      for (std::size_t i = 0; i < playback.times.size(); ++i)
        playback.times[i] = UNTIMED_TRAJECTORY_DURATION * static_cast<double>(i) / static_cast<double>(playback.times.size() - 1);

    render_thread_->post([playback = std::move(playback)](Renderer& renderer) mutable { renderer.setPlayback(std::move(playback)); });
  });
}

void ThreeppVisualization::plotMarker(const tesseract::visualization::Marker& marker, std::string ns)
{
  guard([&] {
    render_thread_->post([markers = threejs::convertMarker(marker, rootLink()), ns = std::move(ns)](Renderer& renderer) mutable {
      renderer.addMarkers(std::move(markers), ns);
    });
  });
}

void ThreeppVisualization::plotMarkers(const std::vector<std::shared_ptr<tesseract::visualization::Marker>>& markers, std::string ns)
{
  guard([&] {
    const std::string root = rootLink();
    std::vector<ViewerMarker> converted;
    for (const auto& marker : markers)
    {
      auto part = threejs::convertMarker(*marker, root);
      converted.insert(converted.end(), std::make_move_iterator(part.begin()), std::make_move_iterator(part.end()));
    }
    render_thread_->post([converted = std::move(converted), ns = std::move(ns)](Renderer& renderer) mutable {
      renderer.addMarkers(std::move(converted), ns);
    });
  });
}

void ThreeppVisualization::clear(std::string ns)
{
  guard([&] { render_thread_->post([ns = std::move(ns)](Renderer& renderer) { renderer.clearMarkers(ns); }); });
}

void ThreeppVisualization::waitForInput(std::string message)
{
  std::cout << message << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

std::optional<ThreeppImage> ThreeppVisualization::renderImage()
{
  if (!ensureStarted())
    return std::nullopt;
  try
  {
    return render_thread_->call<ThreeppImage>([](Renderer& renderer) { return renderer.capture(); });
  }
  catch (const std::exception& e)
  {
    TESSERACT_LOG_ERROR("tesseract_visualization_threepp: rendering an image failed: {}", e.what());
    return std::nullopt;
  }
}

bool ThreeppVisualization::saveImage(const std::string& path)
{
  if (!ensureStarted())
    return false;
  try
  {
    return render_thread_->call<bool>([path](Renderer& renderer) {
      renderer.saveImage(path);
      return true;
    }).value_or(false);
  }
  catch (const std::exception& e)
  {
    TESSERACT_LOG_ERROR("tesseract_visualization_threepp: saving {} failed: {}", path, e.what());
    return false;
  }
}

void ThreeppVisualization::seekTrajectory(double time_s)
{
  guard([&] { render_thread_->post([time_s](Renderer& renderer) { renderer.seek(time_s); }); });
}

std::string ThreeppVisualization::glRendererName()
{
  if (!ensureStarted())
    return {};
  try
  {
    return render_thread_->call<std::string>([](Renderer& renderer) { return renderer.glRendererName(); }).value_or("");
  }
  catch (const std::exception& e)
  {
    TESSERACT_LOG_ERROR("tesseract_visualization_threepp: {}", e.what());
    return {};
  }
}

}  // namespace tesseract::visualization::threepp
