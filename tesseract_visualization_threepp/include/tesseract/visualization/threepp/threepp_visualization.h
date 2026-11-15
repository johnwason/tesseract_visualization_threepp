/**
 * @file threepp_visualization.h
 * @brief A tesseract::visualization::Visualization that renders in process with threepp
 */
#ifndef TESSERACT_VISUALIZATION_THREEPP_THREEPP_VISUALIZATION_H
#define TESSERACT_VISUALIZATION_THREEPP_THREEPP_VISUALIZATION_H

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <tesseract/visualization/visualization.h>

namespace tesseract::visualization::threepp
{
enum class ThreeppDisplayMode
{
  AUTO,      ///< A window if the DISPLAY variable is set, otherwise headless. Always a window off Linux
  WINDOW,    ///< A GLFW window with orbit controls. The trajectory plays in a loop
  HEADLESS,  ///< An EGL context with no window or display server. Linux only
};

/** @brief How the scene is lit */
enum class ThreeppLighting
{
  HEMISPHERE,   ///< A hemisphere light, as the three.js client uses by default. More contrast on CAD meshes
  ENVIRONMENT,  ///< Also an environment map, so metallic and rough PBR materials and normal maps look as intended
};

struct ThreeppVisualizationOptions
{
  ThreeppDisplayMode mode{ ThreeppDisplayMode::AUTO };
  int width{ 1280 };
  int height{ 720 };
  std::string title{ "Tesseract" };
  bool ground_grid{ true };  ///< Draw a 10 m grid with 0.5 m cells on the Z = 0 plane
  ThreeppLighting lighting{ ThreeppLighting::HEMISPHERE };

  /** @throws std::invalid_argument unless text is auto, window or headless */
  static ThreeppDisplayMode parseMode(const std::string& text);

  /** @throws std::invalid_argument unless text is hemisphere or environment */
  static ThreeppLighting parseLighting(const std::string& text);

  /**
   * @brief Options from TESSERACT_VISUALIZATION_THREEPP_MODE, _WIDTH, _HEIGHT and _LIGHTING
   * @throws std::invalid_argument if a value is not valid
   */
  static ThreeppVisualizationOptions fromEnvironment();
};

/** @brief A rendered frame, top row first, four bytes per pixel */
struct ThreeppImage
{
  int width{ 0 };
  int height{ 0 };
  std::vector<std::uint8_t> rgba;
};

/**
 * @brief Owns a render thread with one OpenGL context, scene and camera per object
 * @details The render thread starts on first use. All OpenGL work happens on it, so the plot
 * functions can be called from any thread, and run in the order they are called. A failure to start
 * or plot is logged rather than thrown, so that the viewer cannot abort an application's planning.
 *
 * threepp's OpenGL function pointers are process wide, so only one object can render at a time. Any
 * other logs an error and stays disconnected until the first is destroyed.
 *
 * On macOS, GLFW requires the main thread, so this class cannot open a window there.
 */
class ThreeppVisualization : public tesseract::visualization::Visualization
{
public:
  /** @brief Options come from the environment when the render thread starts */
  ThreeppVisualization();
  explicit ThreeppVisualization(ThreeppVisualizationOptions options);
  ~ThreeppVisualization() override;
  ThreeppVisualization(const ThreeppVisualization&) = delete;
  ThreeppVisualization& operator=(const ThreeppVisualization&) = delete;
  ThreeppVisualization(ThreeppVisualization&&) = delete;
  ThreeppVisualization& operator=(ThreeppVisualization&&) = delete;

  /** @brief Render the scene as it stands after every earlier plot call. Empty if not rendering */
  std::optional<ThreeppImage> renderImage();

  /** @brief Render and write a .png, .jpg or .bmp file. False if it could not be written */
  bool saveImage(const std::string& path);

  /** @brief Stop any playback and show the trajectory at a time, in seconds, clamped to its length */
  void seekTrajectory(double time_s);

  /** @brief The OpenGL renderer in use, such as "llvmpipe". Empty if not rendering */
  std::string glRendererName();

  /** @brief True while the render thread runs: the window is open, or the headless context is live */
  bool isConnected() const override;
  /** @brief Returns once the render thread has started or failed to. A window needs no client to connect */
  void waitForConnection(long seconds = 0) const override;
  void plotEnvironment(const tesseract::environment::Environment& env, std::string ns = "") override;
  void plotEnvironmentState(const tesseract::scene_graph::SceneState& state, std::string ns = "") override;
  void plotTrajectory(const tesseract::common::JointTrajectory& traj,
                      const tesseract::scene_graph::StateSolver& state_solver,
                      std::string ns = "") override;
  void plotMarker(const tesseract::visualization::Marker& marker, std::string ns = "") override;
  void plotMarkers(const std::vector<std::shared_ptr<tesseract::visualization::Marker>>& markers,
                   std::string ns = "") override;
  void clear(std::string ns = "") override;
  void waitForInput(std::string message = "Hit enter key to continue!") override;

private:
  class RenderThread;

  bool ensureStarted() const;
  template <class Function>
  void guard(Function&& function);
  std::string rootLink() const;

  mutable std::mutex mutex_;
  std::optional<ThreeppVisualizationOptions> options_;
  mutable std::unique_ptr<RenderThread> render_thread_;
  mutable bool start_attempted_{ false };
  std::string root_link_{ "world" };
};

}  // namespace tesseract::visualization::threepp

#endif  // TESSERACT_VISUALIZATION_THREEPP_THREEPP_VISUALIZATION_H
