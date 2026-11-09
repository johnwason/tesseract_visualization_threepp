/**
 * @file abb_irb2400_viewer.cpp
 * @brief Shows the ABB IRB2400 with threepp, playing a trajectory in a window, or saving an image headless
 *
 * Needs TESSERACT_RESOURCE_PATH to contain tesseract/support. Usage: abb_irb2400_viewer [image.png]
 * With an image path it renders headless and exits. Otherwise it opens a window until enter is pressed.
 */
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include <tesseract/common/joint_state.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/visualization/threepp/threepp_visualization.h>

using namespace tesseract::visualization::threepp;

int main(int argc, char** argv)
{
  const std::string prefix = "package://tesseract/support/urdf/abb_irb2400";
  auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
  auto urdf = locator->locateResource(prefix + ".urdf");
  auto srdf = locator->locateResource(prefix + ".srdf");
  tesseract::environment::Environment env;
  if (urdf == nullptr || srdf == nullptr || !env.init(std::filesystem::path(urdf->getFilePath()), std::filesystem::path(srdf->getFilePath()), locator))
  {
    std::cerr << "Could not load the ABB IRB2400. Is TESSERACT_RESOURCE_PATH set?" << std::endl;
    return 1;
  }

  ThreeppVisualizationOptions options = ThreeppVisualizationOptions::fromEnvironment();
  const bool to_image = argc > 1;
  if (to_image)
    options.mode = ThreeppDisplayMode::HEADLESS;

  ThreeppVisualization viewer(options);
  viewer.plotEnvironment(env);
  if (!viewer.isConnected())
    return 1;

  // Swing the first joint and raise the second
  const auto ids = env.getStateSolver()->getActiveJointIds();
  tesseract::common::JointTrajectory trajectory;
  for (int i = 0; i <= 40; ++i)
  {
    const double phase = static_cast<double>(i) / 40.0;
    Eigen::VectorXd position = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(ids.size()));
    position[0] = std::sin(2.0 * EIGEN_PI * phase);
    position[1] = 0.5 * std::sin(EIGEN_PI * phase);
    tesseract::common::JointState state(ids, position);
    state.time = phase * 4.0;
    trajectory.push_back(state);
  }
  viewer.plotTrajectory(trajectory, *env.getStateSolver());

  if (to_image)
  {
    viewer.seekTrajectory(1.0);
    return viewer.saveImage(argv[1]) ? 0 : 1;
  }
  viewer.waitForInput("Press enter to close the viewer");
  return 0;
}
