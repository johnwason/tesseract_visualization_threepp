/**
 * @file tesseract_scene.h
 * @brief A threepp scene of a tesseract environment, with markers and trajectory playback
 *
 * It has no window, camera or OpenGL. Render scene() with any threepp renderer and camera, from a
 * viewer, a host application or a simulated sensor.
 */
#ifndef TESSERACT_THREEPP_TESSERACT_SCENE_H
#define TESSERACT_THREEPP_TESSERACT_SCENE_H

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <threepp/objects/Group.hpp>
#include <threepp/scenes/Scene.hpp>

#include <tesseract/threepp/conversions.h>

namespace tesseract::threepp
{
struct TesseractSceneOptions
{
  bool ground_grid{ true };  ///< Draw a 10 m grid with 0.5 m cells on the Z = 0 plane
};

/**
 * @brief The environment, its joint state, markers and an optional trajectory, as one threepp scene
 * @details The scene is Y up, as threepp is, and holds tesseract's Z up frame below a rotated group.
 * Not thread safe: use it from the thread that renders it. Destroy it while the renderer that drew it
 * still exists, so that GPU resources are released.
 */
class TesseractScene
{
public:
  explicit TesseractScene(const TesseractSceneOptions& options = TesseractSceneOptions());

  tp::Scene& scene() { return *scene_; }
  const EnvironmentScene& environment() const { return environment_; }

  /** @brief Replace the environment. Markers move to the new links, and playback continues */
  void setEnvironment(EnvironmentScene environment, const JointValues& values);

  /** @brief Show a joint state, stopping any playback */
  void setJointValues(const JointValues& values);

  /** @brief Show a trajectory from its start, and play it if play is true */
  void setTrajectory(JointValueTrajectory trajectory, bool play);
  /** @brief Show the trajectory at a time, clamped to its length, and stop playing */
  void seek(double time);
  /** @brief Advance playback, looping. Does nothing unless playing */
  void advance(double seconds);
  bool playing() const;

  /**
   * @brief Add markers under a namespace
   * @details A marker whose link is not in the environment is kept, and appears if a later environment has the link.
   */
  void addMarkers(std::vector<MarkerObject> markers, const std::string& ns);
  /** @brief Remove the markers of a namespace, or all of them if ns is empty */
  void clearMarkers(const std::string& ns);

private:
  struct MarkerEntry
  {
    MarkerObject marker;
    std::string ns;
  };

  void attach(const MarkerEntry& entry);
  void showPlaybackTime();

  std::shared_ptr<tp::Scene> scene_;
  std::shared_ptr<tp::Group> z_up_;
  EnvironmentScene environment_;
  std::vector<MarkerEntry> markers_;

  std::optional<JointValueTrajectory> playback_;
  double playback_time_{ 0 };
  bool playing_{ false };
};

}  // namespace tesseract::threepp

#endif  // TESSERACT_THREEPP_TESSERACT_SCENE_H
