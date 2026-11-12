#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <exception>
#include <cmath>
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
#include <threepp/threepp.hpp>

#include <tesseract/common/joint_state.h>
#include <tesseract/common/logging.h>
#include <tesseract/environment/environment.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/visualization/markers/marker.h>
#include <tesseract/threepp/conversions.h>
#include <tesseract/threepp/tesseract_scene.h>
#include <tesseract/visualization/threepp/threepp_visualization.h>

namespace tesseract::visualization::threepp
{
using namespace ::tesseract::threepp;

namespace
{

/** @brief GLAD's function pointers are process wide, so one object renders at a time */
std::atomic_bool context_in_use{ false };

/** @brief Everything that touches OpenGL. It is created, used and destroyed on the render thread */
class Renderer
{
public:
  explicit Renderer(const ThreeppVisualizationOptions& options, bool headless)
    : headless_(headless), scene_(TesseractSceneOptions{ options.ground_grid })
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

  TesseractScene& scene() { return scene_; }

  /** @brief A window plays trajectories. Headless, they wait at the start for seekTrajectory */
  void setPlayback(JointValueTrajectory playback) { scene_.setTrajectory(std::move(playback), !headless_); }

  /** @brief Advance playback by the time since the last frame, looping */
  void advance()
  {
    const auto now = std::chrono::steady_clock::now();
    const double delta = std::chrono::duration<double>(now - last_frame_).count();
    last_frame_ = now;
    scene_.advance(delta);
  }

  void render()
  {
    if (controls_ != nullptr)
      controls_->update();
    gl_->render(scene_.scene(), *camera_);
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
  bool headless_;
  // The context must outlive every other member, and members are destroyed in reverse order
  std::unique_ptr<tp::EglContext> egl_;
  std::unique_ptr<tp::Canvas> canvas_;
  std::unique_ptr<tp::GLRenderer> gl_;
  std::unique_ptr<tp::OrbitControls> controls_;
  std::shared_ptr<tp::PerspectiveCamera> camera_;

  // After gl_, so that it is destroyed first, while the renderer can still release its resources
  TesseractScene scene_;
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
    // Reading meshes and building the graph is the slow part, so a window stays responsive
    auto scene = environmentToScene(env);
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      root_link_ = env.getRootLinkId().name();
    }
    render_thread_->post([scene = std::move(scene), values = sceneStateToJointValues(env.getState())](Renderer& renderer) mutable {
      renderer.scene().setEnvironment(std::move(scene), values);
    });
  });
}

void ThreeppVisualization::plotEnvironmentState(const tesseract::scene_graph::SceneState& state, std::string /*ns*/)
{
  if (state.joints.empty())
    return;
  guard([&] {
    render_thread_->post([values = sceneStateToJointValues(state)](Renderer& renderer) { renderer.scene().setJointValues(values); });
  });
}

void ThreeppVisualization::plotTrajectory(const tesseract::common::JointTrajectory& traj,
                                          const tesseract::scene_graph::StateSolver& state_solver,
                                          std::string /*ns*/)
{
  guard([&] {
    render_thread_->post([playback = toJointValueTrajectory(traj, state_solver)](Renderer& renderer) mutable {
      renderer.setPlayback(std::move(playback));
    });
  });
}

void ThreeppVisualization::plotMarker(const tesseract::visualization::Marker& marker, std::string ns)
{
  guard([&] {
    render_thread_->post([objects = markerToObjects(marker, rootLink()), ns = std::move(ns)](Renderer& renderer) mutable {
      renderer.scene().addMarkers(std::move(objects), ns);
    });
  });
}

void ThreeppVisualization::plotMarkers(const std::vector<std::shared_ptr<tesseract::visualization::Marker>>& markers, std::string ns)
{
  guard([&] {
    const std::string root = rootLink();
    std::vector<MarkerObject> objects;
    for (const auto& marker : markers)
    {
      auto part = markerToObjects(*marker, root);
      objects.insert(objects.end(), std::make_move_iterator(part.begin()), std::make_move_iterator(part.end()));
    }
    render_thread_->post([objects = std::move(objects), ns = std::move(ns)](Renderer& renderer) mutable {
      renderer.scene().addMarkers(std::move(objects), ns);
    });
  });
}

void ThreeppVisualization::clear(std::string ns)
{
  guard([&] { render_thread_->post([ns = std::move(ns)](Renderer& renderer) { renderer.scene().clearMarkers(ns); }); });
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
  guard([&] { render_thread_->post([time_s](Renderer& renderer) { renderer.scene().seek(time_s); }); });
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
