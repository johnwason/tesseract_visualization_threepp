/**
 * @file conversions.h
 * @brief Pure conversions from tesseract types to threepp objects. No window, renderer or OpenGL.
 *
 * Nothing here depends on the viewer. Unlike threepp_visualization.h, this header exposes threepp
 * types, so including it needs C++20.
 *
 * threepp objects are ordinary CPU data until first rendered, so these can run on any thread.
 */
#ifndef TESSERACT_VISUALIZATION_THREEPP_CONVERSIONS_H
#define TESSERACT_VISUALIZATION_THREEPP_CONVERSIONS_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <threepp/core/Object3D.hpp>
#include <threepp/materials/Material.hpp>
#include <threepp/math/Vector3.hpp>
#include <threepp/objects/Group.hpp>
#include <threepp/objects/Mesh.hpp>

#include <tesseract/common/fwd.h>
#include <tesseract/environment/fwd.h>
#include <tesseract/geometry/fwd.h>
#include <tesseract/scene_graph/fwd.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/state_solver/fwd.h>
#include <tesseract/visualization/fwd.h>

namespace tesseract::visualization::threepp
{
// This namespace is also called threepp, so the library has to be named from the global scope
namespace tp = ::threepp;

/** @brief Joint values by joint name */
using JointValues = std::unordered_map<std::string, double>;

JointValues sceneStateToJointValues(const tesseract::scene_graph::SceneState& state);

/** @brief The node that a joint moves */
struct SceneJoint
{
  tp::Object3D* node{ nullptr };
  tp::Vector3 axis;
  tesseract::scene_graph::JointType type{ tesseract::scene_graph::JointType::FIXED };
};

/**
 * @brief An environment as a threepp graph, in tesseract's Z up frame
 * @details The tree alternates links and joints. Below each link, a group holds the joint's origin
 * and a child of it holds the joint's motion, with the child link below that. Links and joints are
 * looked up by name, and those pointers are valid while root is alive.
 */
struct EnvironmentScene
{
  std::shared_ptr<tp::Group> root;
  std::unordered_map<std::string, tp::Object3D*> links;
  std::unordered_map<std::string, SceneJoint> joints;
};

/** @brief Build the graph at zero joint values. Geometry that cannot be drawn logs a warning and is skipped */
EnvironmentScene environmentToScene(const tesseract::environment::Environment& env);

/** @brief Move revolute, continuous and prismatic joints. Names not in the scene are ignored */
void setJointValues(const EnvironmentScene& scene, const JointValues& values);

/**
 * @brief Convert a geometry, with cylinders, capsules and cones along Z as in tesseract
 * @details Supported: boxes, spheres, cylinders, capsules, cones and polygon meshes, including convex
 * and compound ones. Anything else logs a warning and returns nothing.
 * @param geometry The geometry
 * @param material Used unless a mesh brings its own material. STL meshes never do, as their color is unreliable
 */
std::vector<std::shared_ptr<tp::Mesh>> geometryToMeshes(const tesseract::geometry::Geometry& geometry,
                                                        const std::shared_ptr<tp::Material>& material);

/** @brief An object to add to a link, with its pose relative to that link already set */
struct MarkerObject
{
  std::string parent_link;
  std::shared_ptr<tp::Object3D> object;
};

/**
 * @brief Convert a tesseract marker
 * @details Supported: AXIS, ARROW, GEOMETRY, TOOLPATH and CONTACT_RESULTS. One marker can produce
 * several objects. Anything else logs a warning and returns nothing.
 * @param marker The marker
 * @param default_parent_link Used when the marker has no parent link, usually the root link
 */
std::vector<MarkerObject> markerToObjects(const tesseract::visualization::Marker& marker,
                                          const std::string& default_parent_link);

/** @brief Every joint's value at each trajectory time, including mimic joints */
struct JointValueTrajectory
{
  std::vector<double> times;
  std::vector<JointValues> states;

  double duration() const { return times.back(); }

  /** @brief Values at a time, linearly interpolated and clamped to the trajectory */
  JointValues sample(double time) const;
};

/**
 * @brief Evaluate a trajectory with the state solver, which fills in mimic joints
 * @details If every state has the same time, states are spread evenly over 5 seconds, as Ignition does.
 * @throws std::invalid_argument if the trajectory is empty
 */
JointValueTrajectory toJointValueTrajectory(const tesseract::common::JointTrajectory& trajectory,
                                            const tesseract::scene_graph::StateSolver& state_solver);

}  // namespace tesseract::visualization::threepp

#endif  // TESSERACT_VISUALIZATION_THREEPP_CONVERSIONS_H
