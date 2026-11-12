// The conversions need no window or OpenGL, so these tests run anywhere

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <threepp/threepp.hpp>

#include <tesseract/common/joint_state.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/visualization/markers/axis_marker.h>
#include <tesseract/visualization/markers/contact_results_marker.h>
#include <tesseract/visualization/markers/geometry_marker.h>
#include <tesseract/threepp/conversions.h>

using namespace tesseract::threepp;
namespace g = tesseract::geometry;

namespace
{
constexpr char ARM[] = R"(
<robot name="arm" tesseract:make_convex="false" xmlns:tesseract="http://tesseract">
  <link name="base"/>
  <link name="a"><visual><geometry><box size="1 2 3"/></geometry></visual></link>
  <link name="b"/>
  <link name="c"/>
  <link name="d"/>
  <joint name="j_a" type="revolute">
    <parent link="base"/><child link="a"/>
    <origin xyz="0.1 0.2 0.3" rpy="0.4 -0.5 0.6"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="j_b" type="prismatic">
    <parent link="a"/><child link="b"/>
    <origin xyz="0.7 0 -0.2" rpy="1.0 0.2 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="j_c" type="continuous">
    <parent link="b"/><child link="c"/>
    <origin xyz="0 0.3 0.4" rpy="0 0 -1.1"/>
    <axis xyz="1 0 0"/>
    <limit effort="1" velocity="1"/>
  </joint>
  <joint name="j_d" type="fixed">
    <parent link="c"/><child link="d"/>
    <origin xyz="0.5 0.5 0" rpy="0.3 0 0"/>
  </joint>
</robot>)";

constexpr char TEXTURED[] = R"(
<robot name="textured" tesseract:make_convex="false" xmlns:tesseract="http://tesseract">
  <link name="base"/>
  <link name="mesh">
    <visual><geometry><mesh filename="package://tesseract/support/meshes/tesseract_material_mesh.glb"/></geometry></visual>
  </link>
  <joint name="j" type="fixed"><parent link="base"/><child link="mesh"/></joint>
</robot>)";

std::unique_ptr<tesseract::environment::Environment> makeEnvironment(const char* urdf)
{
  auto env = std::make_unique<tesseract::environment::Environment>();
  if (!env->init(std::string(urdf), std::make_shared<tesseract::common::GeneralResourceLocator>()))
    throw std::runtime_error("Could not load the test URDF");
  return env;
}

tp::Box3 boundsOf(const g::Geometry& geometry)
{
  const auto meshes = geometryToMeshes(geometry, tp::MeshBasicMaterial::create());
  if (meshes.size() != 1)
    throw std::runtime_error("Expected one mesh");
  auto buffer = meshes[0]->geometry();
  buffer->computeBoundingBox();
  return *buffer->boundingBox;
}

void expectBounds(const tp::Box3& box, const tp::Vector3& half)
{
  EXPECT_NEAR(box.min().x, -half.x, 1e-3);
  EXPECT_NEAR(box.max().x, half.x, 1e-3);
  EXPECT_NEAR(box.min().y, -half.y, 1e-3);
  EXPECT_NEAR(box.max().y, half.y, 1e-3);
  EXPECT_NEAR(box.min().z, -half.z, 1e-3);
  EXPECT_NEAR(box.max().z, half.z, 1e-3);
}
}  // namespace

TEST(Conversions, PrimitivesHaveTesseractSizesAndAxes)
{
  expectBounds(boundsOf(g::Box(1, 2, 3)), { 0.5F, 1.0F, 1.5F });
  expectBounds(boundsOf(g::Sphere(0.5)), { 0.5F, 0.5F, 0.5F });
  expectBounds(boundsOf(g::Cylinder(0.25, 2)), { 0.25F, 0.25F, 1.0F });
  expectBounds(boundsOf(g::Cone(0.25, 2)), { 0.25F, 0.25F, 1.0F });
  // A capsule's length is the straight section between its end caps
  expectBounds(boundsOf(g::Capsule(0.25, 2)), { 0.25F, 0.25F, 1.25F });
}

TEST(Conversions, ConeApexIsAtPlusZ)
{
  const auto meshes = geometryToMeshes(g::Cone(0.25, 2), tp::MeshBasicMaterial::create());
  ASSERT_EQ(meshes.size(), 1U);
  const auto* positions = meshes[0]->geometry()->getAttribute<float>("position");
  ASSERT_NE(positions, nullptr);

  tp::Vector3 point;
  float top_radius = 0;
  float bottom_radius = 0;
  for (int i = 0; i < positions->count(); ++i)
  {
    positions->setFromBufferAttribute(point, i);
    const float radius = std::hypot(point.x, point.y);
    if (point.z > 0.99F)
      top_radius = std::max(top_radius, radius);
    if (point.z < -0.99F)
      bottom_radius = std::max(bottom_radius, radius);
  }
  EXPECT_NEAR(top_radius, 0.0F, 1e-3);
  EXPECT_NEAR(bottom_radius, 0.25F, 1e-3);
}

TEST(Conversions, UnsupportedGeometryIsSkipped)
{
  EXPECT_TRUE(geometryToMeshes(g::Plane(0, 0, 1, 0), tp::MeshBasicMaterial::create()).empty());
}

TEST(Conversions, SceneHasEveryLinkAndJoint)
{
  const auto env = makeEnvironment(ARM);
  const EnvironmentScene scene = environmentToScene(*env);
  EXPECT_EQ(scene.links.size(), 5U);
  EXPECT_EQ(scene.joints.size(), 4U);
  ASSERT_EQ(scene.links.count("a"), 1U);
  EXPECT_EQ(scene.links.at("a")->children.size(), 2U) << "The box visual and the origin of joint j_b";
}

TEST(Conversions, JointMotionMatchesForwardKinematics)
{
  const auto env = makeEnvironment(ARM);
  const EnvironmentScene scene = environmentToScene(*env);

  const auto ids = env->getStateSolver()->getActiveJointIds();
  ASSERT_EQ(ids.size(), 3U);
  const auto state = env->getStateSolver()->getState(ids, Eigen::Vector3d(0.7, -1.2, 2.1));
  setJointValues(scene, sceneStateToJointValues(state));
  scene.root->updateMatrixWorld(true);

  for (const auto& [id, expected] : state.link_transforms)
  {
    ASSERT_EQ(scene.links.count(id.name()), 1U) << id.name();
    const auto& actual = scene.links.at(id.name())->matrixWorld->elements;
    Eigen::Matrix4d expected_matrix = expected.matrix();
    for (int column = 0; column < 4; ++column)
      for (int row = 0; row < 4; ++row)
        EXPECT_NEAR(actual[static_cast<std::size_t>(column * 4 + row)], expected_matrix(row, column), 1e-4)
            << id.name() << " at " << row << "," << column;
  }
}

TEST(Conversions, UnknownJointsAreIgnored)
{
  const auto env = makeEnvironment(ARM);
  const EnvironmentScene scene = environmentToScene(*env);
  EXPECT_NO_THROW(setJointValues(scene, { { "not_a_joint", 1.0 } }));
}

TEST(Conversions, TexturedMeshKeepsItsTexture)
{
  const auto env = makeEnvironment(TEXTURED);
  const EnvironmentScene scene = environmentToScene(*env);
  int textured = 0;
  scene.root->traverseType<tp::Mesh>([&textured](tp::Mesh& mesh) {
    const auto material = std::dynamic_pointer_cast<tp::MeshStandardMaterial>(mesh.material());
    if (material != nullptr && material->map != nullptr)
    {
      EXPECT_TRUE(mesh.geometry()->hasAttribute("uv"));
      ++textured;
    }
  });
  EXPECT_GT(textured, 0);
}

TEST(Conversions, MarkersGoToTheirLinkOrTheDefault)
{
  tesseract::visualization::AxisMarker axis(Eigen::Isometry3d::Identity());
  auto objects = markerToObjects(axis, "world");
  ASSERT_EQ(objects.size(), 1U);
  EXPECT_EQ(objects[0].parent_link, "world");

  axis.setParentLink("tool");
  objects = markerToObjects(axis, "world");
  ASSERT_EQ(objects.size(), 1U);
  EXPECT_EQ(objects[0].parent_link, "tool");
}

TEST(Conversions, GeometryMarkersIncludeCapsules)
{
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.translation() = Eigen::Vector3d(1, 2, 3);
  const tesseract::visualization::GeometryMarker marker(std::make_shared<g::Capsule>(0.1, 0.5), origin);
  const auto objects = markerToObjects(marker, "world");
  ASSERT_EQ(objects.size(), 1U);
  EXPECT_FLOAT_EQ(objects[0].object->position.z, 3.0F);
}

TEST(Conversions, ContactArrowsStartAtTheOtherLinksPoint)
{
  namespace vis = tesseract::visualization;
  using tesseract::common::LinkId;
  const LinkId a("a");
  const LinkId b("b");
  tesseract::collision::ContactResult result;
  result.link_ids = { a, b };
  result.distance = 0.5;
  result.nearest_points = { Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1) };
  const auto margin = [](const LinkId&, const LinkId&) { return 0.0; };

  const auto arrows = [&](std::vector<LinkId> selected) {
    return markerToObjects(vis::ContactResultsMarker(std::move(selected), { result }, margin), "world");
  };

  auto objects = arrows({ a });
  ASSERT_EQ(objects.size(), 1U);
  EXPECT_FLOAT_EQ(objects[0].object->position.z, 1.0F);

  objects = arrows({ b });
  ASSERT_EQ(objects.size(), 1U);
  EXPECT_FLOAT_EQ(objects[0].object->position.z, 0.0F);

  objects = arrows({});
  ASSERT_EQ(objects.size(), 1U);
  EXPECT_FLOAT_EQ(objects[0].object->position.z, 0.0F);

  objects = arrows({ a, b });
  ASSERT_EQ(objects.size(), 2U);
  EXPECT_FLOAT_EQ(objects[0].object->position.z, 0.0F);
  EXPECT_FLOAT_EQ(objects[1].object->position.z, 1.0F);
}

TEST(Conversions, TrajectoriesInterpolateAndUntimedOnesAreSpread)
{
  const auto env = makeEnvironment(ARM);
  const auto solver = env->getStateSolver();
  const auto ids = solver->getActiveJointIds();

  tesseract::common::JointTrajectory timed;
  for (const auto& [value, time] : { std::pair<double, double>{ 0.0, 0.0 }, std::pair<double, double>{ 1.0, 2.0 } })
  {
    tesseract::common::JointState state(ids, Eigen::Vector3d::Constant(value));
    state.time = time;
    timed.push_back(state);
  }
  const JointValueTrajectory values = toJointValueTrajectory(timed, *solver);
  EXPECT_DOUBLE_EQ(values.duration(), 2.0);
  EXPECT_NEAR(values.sample(0.5).at("j_a"), 0.25, 1e-9);
  EXPECT_NEAR(values.sample(-1.0).at("j_a"), 0.0, 1e-9);
  EXPECT_NEAR(values.sample(9.0).at("j_a"), 1.0, 1e-9);

  for (auto& state : timed.states)
    state.time = 0;
  EXPECT_DOUBLE_EQ(toJointValueTrajectory(timed, *solver).duration(), 5.0);

  EXPECT_THROW(toJointValueTrajectory(tesseract::common::JointTrajectory(), *solver), std::invalid_argument);
}
