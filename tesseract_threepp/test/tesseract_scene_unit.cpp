// TesseractScene needs no window or OpenGL, so these tests run anywhere

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>

#include <threepp/threepp.hpp>

#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/visualization/markers/axis_marker.h>
#include <tesseract/threepp/tesseract_scene.h>

using namespace tesseract::threepp;

namespace
{
constexpr char URDF[] = R"(
<robot name="slider" tesseract:make_convex="false" xmlns:tesseract="http://tesseract">
  <link name="world"/>
  <link name="carriage"/>
  <joint name="slide" type="prismatic">
    <parent link="world"/><child link="carriage"/>
    <axis xyz="0 0 1"/>
    <limit lower="-10" upper="10" effort="1" velocity="1"/>
  </joint>
</robot>)";

constexpr char OTHER_URDF[] = R"(
<robot name="other" tesseract:make_convex="false" xmlns:tesseract="http://tesseract">
  <link name="world"/>
  <link name="elsewhere"/>
  <joint name="fixed" type="fixed"><parent link="world"/><child link="elsewhere"/></joint>
</robot>)";

EnvironmentScene makeScene(const char* urdf)
{
  tesseract::environment::Environment env;
  if (!env.init(std::string(urdf), std::make_shared<tesseract::common::GeneralResourceLocator>()))
    throw std::runtime_error("Could not load the test URDF");
  return environmentToScene(env);
}

std::shared_ptr<tp::Object3D> axisOn(const std::string& link, TesseractScene& scene, const std::string& ns)
{
  tesseract::visualization::AxisMarker marker(Eigen::Isometry3d::Identity());
  marker.setParentLink(link);
  auto objects = markerToObjects(marker, "world");
  auto object = objects.at(0).object;
  scene.addMarkers(std::move(objects), ns);
  return object;
}

float slidePosition(const TesseractScene& scene) { return scene.environment().joints.at("slide").node->position.z; }

JointValueTrajectory slideTrajectory()
{
  JointValueTrajectory trajectory;
  trajectory.times = { 0.0, 2.0 };
  trajectory.states = { { { "slide", 0.0 } }, { { "slide", 1.0 } } };
  return trajectory;
}
}  // namespace

TEST(TesseractScene, MarkersFollowTheirLinkAcrossEnvironments)
{
  TesseractScene scene;
  scene.setEnvironment(makeScene(URDF), {});
  const auto on_carriage = axisOn("carriage", scene, "");
  EXPECT_EQ(on_carriage->parent, scene.environment().links.at("carriage"));

  // A scene without the link hides the marker, and one with it brings the marker back
  scene.setEnvironment(makeScene(OTHER_URDF), {});
  EXPECT_EQ(on_carriage->parent, nullptr);
  scene.setEnvironment(makeScene(URDF), {});
  EXPECT_EQ(on_carriage->parent, scene.environment().links.at("carriage"));
}

TEST(TesseractScene, MarkersAreClearedByNamespace)
{
  TesseractScene scene;
  scene.setEnvironment(makeScene(URDF), {});
  const auto a = axisOn("world", scene, "a");
  const auto b = axisOn("world", scene, "b");

  scene.clearMarkers("a");
  EXPECT_EQ(a->parent, nullptr);
  EXPECT_NE(b->parent, nullptr);

  scene.clearMarkers("");
  EXPECT_EQ(b->parent, nullptr);

  // Cleared markers stay gone when the environment is replaced
  scene.setEnvironment(makeScene(URDF), {});
  EXPECT_EQ(a->parent, nullptr);
  EXPECT_EQ(b->parent, nullptr);
}

TEST(TesseractScene, PlaybackLoopsAndSeekStopsIt)
{
  TesseractScene scene;
  scene.setEnvironment(makeScene(URDF), {});
  scene.setTrajectory(slideTrajectory(), true);
  EXPECT_TRUE(scene.playing());
  EXPECT_FLOAT_EQ(slidePosition(scene), 0.0F);

  scene.advance(0.5);
  EXPECT_FLOAT_EQ(slidePosition(scene), 0.25F);
  scene.advance(2.0);  // Loops back to 0.5 s
  EXPECT_FLOAT_EQ(slidePosition(scene), 0.25F);

  scene.seek(1.0);
  EXPECT_FALSE(scene.playing());
  EXPECT_FLOAT_EQ(slidePosition(scene), 0.5F);
  scene.advance(1.0);
  EXPECT_FLOAT_EQ(slidePosition(scene), 0.5F) << "A paused trajectory does not advance";
}

TEST(TesseractScene, JointValuesReplaceTheTrajectory)
{
  TesseractScene scene;
  scene.setEnvironment(makeScene(URDF), {});
  scene.setTrajectory(slideTrajectory(), true);
  scene.setJointValues({ { "slide", 3.0 } });
  EXPECT_FALSE(scene.playing());
  EXPECT_FLOAT_EQ(slidePosition(scene), 3.0F);

  // With no trajectory, a new environment shows the values it is given
  scene.setEnvironment(makeScene(URDF), { { "slide", 2.0 } });
  EXPECT_FLOAT_EQ(slidePosition(scene), 2.0F);
}

TEST(TesseractScene, TrajectoryContinuesAcrossEnvironments)
{
  TesseractScene scene;
  scene.setEnvironment(makeScene(URDF), {});
  scene.setTrajectory(slideTrajectory(), false);
  scene.seek(2.0);
  scene.setEnvironment(makeScene(URDF), { { "slide", 0.0 } });
  EXPECT_FLOAT_EQ(slidePosition(scene), 1.0F);
}
