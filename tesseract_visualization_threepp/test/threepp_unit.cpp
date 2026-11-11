#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <tesseract/common/joint_state.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/visualization/markers/axis_marker.h>
#include <tesseract/visualization/threepp/threepp_visualization.h>
#include <tesseract/visualization/visualization_loader.h>

using namespace tesseract::visualization::threepp;

namespace
{
// A red box that the joint carries out of the camera's view
constexpr char URDF[] = R"(
<robot name="r" tesseract:make_convex="false" xmlns:tesseract="http://tesseract.dev">
  <link name="world"/>
  <link name="moving">
    <visual>
      <geometry><box size="0.6 0.6 0.6"/></geometry>
      <material name="red"><color rgba="1 0 0 1"/></material>
    </visual>
  </link>
  <joint name="slide" type="prismatic">
    <parent link="world"/>
    <child link="moving"/>
    <axis xyz="0 0 1"/>
    <limit lower="-100" upper="100" effort="1" velocity="1"/>
  </joint>
</robot>)";

ThreeppVisualizationOptions headless()
{
  ThreeppVisualizationOptions options;
  options.mode = ThreeppDisplayMode::HEADLESS;
  options.width = 320;
  options.height = 240;
  options.ground_grid = false;  // So that an empty scene is entirely background
  return options;
}

std::unique_ptr<tesseract::environment::Environment> makeEnvironment()
{
  auto env = std::make_unique<tesseract::environment::Environment>();
  if (!env->init(std::string(URDF), std::make_shared<tesseract::common::GeneralResourceLocator>()))
    throw std::runtime_error("Could not load the test URDF");
  return env;
}

/** @brief Pixels where red clearly dominates */
int redPixels(const ThreeppImage& image)
{
  int count = 0;
  for (std::size_t i = 0; i + 3 < image.rgba.size(); i += 4)
    if (image.rgba[i] > 40 && image.rgba[i] > 2 * image.rgba[i + 1] && image.rgba[i] > 2 * image.rgba[i + 2])
      ++count;
  return count;
}

/** @brief Pixels that are not the black background */
int litPixels(const ThreeppImage& image)
{
  int count = 0;
  for (std::size_t i = 0; i + 3 < image.rgba.size(); i += 4)
    if (image.rgba[i] > 8 || image.rgba[i + 1] > 8 || image.rgba[i + 2] > 8)
      ++count;
  return count;
}

tesseract::common::JointTrajectory slideTrajectory(const tesseract::environment::Environment& env, double a, double b, double end_time)
{
  const auto ids = env.getStateSolver()->getActiveJointIds();
  tesseract::common::JointTrajectory trajectory;
  for (const auto& [value, time] : { std::pair<double, double>{ a, 0.0 }, std::pair<double, double>{ b, end_time } })
  {
    tesseract::common::JointState state(ids, (Eigen::VectorXd(1) << value).finished());
    state.time = time;
    trajectory.push_back(state);
  }
  return trajectory;
}
}  // namespace

TEST(ThreeppVisualization, RendersAnEmptySceneHeadless)
{
  ThreeppVisualization viewer(headless());
  EXPECT_TRUE(viewer.isConnected());
  EXPECT_FALSE(viewer.glRendererName().empty());
  const auto image = viewer.renderImage();
  ASSERT_TRUE(image.has_value());
  EXPECT_EQ(image->width, 320);
  EXPECT_EQ(image->height, 240);
  EXPECT_EQ(image->rgba.size(), std::size_t{ 320 * 240 * 4 });
  EXPECT_EQ(litPixels(*image), 0);
}

TEST(ThreeppVisualization, GroundGridIsDrawnByDefault)
{
  ThreeppVisualizationOptions options = headless();
  options.ground_grid = ThreeppVisualizationOptions{}.ground_grid;
  ThreeppVisualization viewer(options);
  const auto image = viewer.renderImage();
  ASSERT_TRUE(image.has_value());
  EXPECT_GT(litPixels(*image), 200);
}

TEST(ThreeppVisualization, PlottedEnvironmentAppears)
{
  ThreeppVisualization viewer(headless());
  auto env = makeEnvironment();
  viewer.plotEnvironment(*env);
  const auto image = viewer.renderImage();
  ASSERT_TRUE(image.has_value());
  EXPECT_GT(redPixels(*image), 200);
}

TEST(ThreeppVisualization, JointStateMovesTheLink)
{
  ThreeppVisualization viewer(headless());
  auto env = makeEnvironment();
  viewer.plotEnvironment(*env);
  ASSERT_GT(redPixels(*viewer.renderImage()), 200);

  tesseract::scene_graph::SceneState away = env->getState();
  for (auto& [id, value] : away.joints)
    value = 50.0;
  viewer.plotEnvironmentState(away);
  EXPECT_EQ(redPixels(*viewer.renderImage()), 0);

  viewer.plotEnvironmentState(env->getState());
  EXPECT_GT(redPixels(*viewer.renderImage()), 200);
}

TEST(ThreeppVisualization, TrajectorySeekIsInterpolated)
{
  ThreeppVisualization viewer(headless());
  auto env = makeEnvironment();
  viewer.plotEnvironment(*env);
  // Both ends are out of view, so only a blend of the two puts the box on screen
  viewer.plotTrajectory(slideTrajectory(*env, -100.0, 100.0, 2.0), *env->getStateSolver());

  // Headless playback waits at the start
  EXPECT_EQ(redPixels(*viewer.renderImage()), 0);
  viewer.seekTrajectory(1.0);
  EXPECT_GT(redPixels(*viewer.renderImage()), 200);
  viewer.seekTrajectory(2.0);
  EXPECT_EQ(redPixels(*viewer.renderImage()), 0);
}

TEST(ThreeppVisualization, MarkersCanBeClearedByNamespace)
{
  ThreeppVisualization viewer(headless());
  auto env = makeEnvironment();
  tesseract::scene_graph::SceneState away = env->getState();
  for (auto& [id, value] : away.joints)
    value = 50.0;
  viewer.plotEnvironment(*env);
  viewer.plotEnvironmentState(away);
  ASSERT_EQ(litPixels(*viewer.renderImage()), 0);

  tesseract::visualization::AxisMarker marker(Eigen::Isometry3d::Identity());
  marker.setScale(Eigen::Vector3d::Constant(1.0));
  viewer.plotMarker(marker, "ns_a");
  const int with_marker = litPixels(*viewer.renderImage());
  EXPECT_GT(with_marker, 50);

  viewer.clear("other");
  EXPECT_EQ(litPixels(*viewer.renderImage()), with_marker);
  viewer.clear("ns_a");
  EXPECT_EQ(litPixels(*viewer.renderImage()), 0);
}

TEST(ThreeppVisualization, MarkersSurviveAnEnvironmentReload)
{
  ThreeppVisualization viewer(headless());
  auto env = makeEnvironment();
  viewer.plotEnvironment(*env);
  tesseract::visualization::AxisMarker marker(Eigen::Isometry3d::Identity());
  viewer.plotMarker(marker);
  const auto before = viewer.renderImage();
  ASSERT_TRUE(before.has_value());

  for (int i = 0; i < 3; ++i)
    viewer.plotEnvironment(*env);
  const auto after = viewer.renderImage();
  ASSERT_TRUE(after.has_value());
  EXPECT_EQ(litPixels(*after), litPixels(*before));
}

TEST(ThreeppVisualization, ASecondObjectDoesNotRenderWhileTheFirstDoes)
{
  auto first = std::make_unique<ThreeppVisualization>(headless());
  ASSERT_TRUE(first->isConnected());
  {
    ThreeppVisualization second(headless());
    EXPECT_FALSE(second.isConnected());
    EXPECT_FALSE(second.renderImage().has_value());
  }
  EXPECT_TRUE(first->renderImage().has_value());
  first.reset();

  ThreeppVisualization third(headless());
  EXPECT_TRUE(third.isConnected());
  EXPECT_TRUE(third.renderImage().has_value());
}

TEST(ThreeppVisualization, InvalidModeIsRejected)
{
  EXPECT_THROW(ThreeppVisualizationOptions::parseMode("sideways"), std::invalid_argument);
  EXPECT_EQ(ThreeppVisualizationOptions::parseMode("headless"), ThreeppDisplayMode::HEADLESS);
}

TEST(ThreeppVisualization, LightingOptionIsParsed)
{
  EXPECT_THROW(ThreeppVisualizationOptions::parseLighting("disco"), std::invalid_argument);
  EXPECT_EQ(ThreeppVisualizationOptions::parseLighting("hemisphere"), ThreeppLighting::HEMISPHERE);
  EXPECT_EQ(ThreeppVisualizationOptions::parseLighting("environment"), ThreeppLighting::ENVIRONMENT);
  EXPECT_EQ(ThreeppVisualizationOptions{}.lighting, ThreeppLighting::HEMISPHERE);
}

TEST(ThreeppVisualization, EnvironmentLightingChangesTheImage)
{
  // The red link's material is partly metallic, so the environment map changes how it shades
  const auto render = [](ThreeppLighting lighting) {
    ThreeppVisualizationOptions options = headless();
    options.lighting = lighting;
    ThreeppVisualization viewer(options);
    auto env = makeEnvironment();
    viewer.plotEnvironment(*env);
    return viewer.renderImage();
  };
  const auto hemisphere = render(ThreeppLighting::HEMISPHERE);
  const auto environment = render(ThreeppLighting::ENVIRONMENT);
  ASSERT_TRUE(hemisphere.has_value());
  ASSERT_TRUE(environment.has_value());
  EXPECT_GT(redPixels(*environment), 200);
  EXPECT_NE(hemisphere->rgba, environment->rgba);
}

TEST(ThreeppVisualization, TheLoaderProvidesAPluginInstance)
{
  setenv("TESSERACT_VISUALIZATION_THREEPP_MODE", "headless", 1);
  setenv("TESSERACT_VISUALIZATION_THREEPP_WIDTH", "160", 1);
  setenv("TESSERACT_VISUALIZATION_THREEPP_HEIGHT", "120", 1);
  tesseract::visualization::VisualizationLoader loader;
  auto viewer = loader.get("TesseractThreeppVisualizationPlugin");
  ASSERT_NE(viewer, nullptr);
  EXPECT_TRUE(viewer->isConnected());
  EXPECT_EQ(viewer.get(), loader.get("TesseractThreeppVisualizationPlugin").get());
}

/** @brief Skipped without a display. tools/test_threepp_window.sh provides a virtual one */
TEST(ThreeppVisualization, WindowRendersUnderADisplay)
{
  if (std::getenv("DISPLAY") == nullptr)
    GTEST_SKIP() << "No display";
  ThreeppVisualizationOptions options = headless();
  options.mode = ThreeppDisplayMode::WINDOW;
  ThreeppVisualization viewer(options);
  ASSERT_TRUE(viewer.isConnected());
  auto env = makeEnvironment();
  viewer.plotEnvironment(*env);
  const auto image = viewer.renderImage();
  ASSERT_TRUE(image.has_value());
  EXPECT_GT(redPixels(*image), 200);
}
