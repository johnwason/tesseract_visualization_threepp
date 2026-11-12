#include <algorithm>
#include <cmath>
#include <utility>

#include <threepp/threepp.hpp>

#include <tesseract/threepp/tesseract_scene.h>

namespace tesseract::threepp
{
namespace
{
constexpr float PI_2 = 1.57079632679489661923F;

void detach(const MarkerObject& marker)
{
  if (marker.object->parent != nullptr)
    marker.object->removeFromParent();
}
}  // namespace

TesseractScene::TesseractScene(const TesseractSceneOptions& options) : scene_(tp::Scene::create()), z_up_(tp::Group::create())
{
  // Matches the brightness of the three.js client, whose r153 lights are not physically scaled
  scene_->add(tp::HemisphereLight::create(0xffffff, 0x202018, 2 * PI_2));

  // tesseract is Z up, three.js and threepp are Y up
  z_up_->rotateX(-PI_2);
  scene_->add(z_up_);

  // The grid is flat in threepp's Y up frame, so it goes in the scene rather than under z_up_
  if (options.ground_grid)
    scene_->add(tp::GridHelper::create(10, 20, 0x888888, 0x444444));
}

void TesseractScene::setEnvironment(EnvironmentScene environment, const JointValues& values)
{
  // The old graph goes last, so that markers can move straight from it to the new one
  const EnvironmentScene old_environment = std::exchange(environment_, std::move(environment));
  z_up_->add(environment_.root);
  for (const auto& entry : markers_)
    attach(entry);
  if (old_environment.root != nullptr)
    z_up_->remove(*old_environment.root);

  threepp::setJointValues(environment_, values);
  if (playback_)
    showPlaybackTime();
}

void TesseractScene::setJointValues(const JointValues& values)
{
  playback_.reset();
  playing_ = false;
  threepp::setJointValues(environment_, values);
}

void TesseractScene::setTrajectory(JointValueTrajectory trajectory, bool play)
{
  playback_ = std::move(trajectory);
  playback_time_ = 0;
  playing_ = play;
  showPlaybackTime();
}

void TesseractScene::seek(double time)
{
  if (!playback_)
    return;
  playing_ = false;
  playback_time_ = std::clamp(time, 0.0, playback_->duration());
  showPlaybackTime();
}

void TesseractScene::advance(double seconds)
{
  if (!playing())
    return;
  playback_time_ = std::fmod(playback_time_ + seconds, playback_->duration());
  showPlaybackTime();
}

bool TesseractScene::playing() const { return playback_ && playing_ && playback_->duration() > 0; }

void TesseractScene::addMarkers(std::vector<MarkerObject> markers, const std::string& ns)
{
  for (auto& marker : markers)
  {
    markers_.push_back(MarkerEntry{ std::move(marker), ns });
    attach(markers_.back());
  }
}

void TesseractScene::clearMarkers(const std::string& ns)
{
  const auto end = std::remove_if(markers_.begin(), markers_.end(), [&ns](const MarkerEntry& entry) {
    if (!ns.empty() && entry.ns != ns)
      return false;
    detach(entry.marker);
    return true;
  });
  markers_.erase(end, markers_.end());
}

void TesseractScene::attach(const MarkerEntry& entry)
{
  const auto link = environment_.links.find(entry.marker.parent_link);
  if (link == environment_.links.end())
    detach(entry.marker);
  else
    link->second->add(entry.marker.object);
}

void TesseractScene::showPlaybackTime() { threepp::setJointValues(environment_, playback_->sample(playback_time_)); }

}  // namespace tesseract::threepp
