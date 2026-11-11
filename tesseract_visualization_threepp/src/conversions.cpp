#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <threepp/threepp.hpp>

#include <tesseract/collision/types.h>
#include <tesseract/common/joint_state.h>
#include <tesseract/common/logging.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/impl/mesh_material.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/visualization/markers/arrow_marker.h>
#include <tesseract/visualization/markers/axis_marker.h>
#include <tesseract/visualization/markers/contact_results_marker.h>
#include <tesseract/visualization/markers/geometry_marker.h>
#include <tesseract/visualization/markers/marker.h>
#include <tesseract/visualization/markers/toolpath_marker.h>
#include <tesseract/visualization/threepp/conversions.h>

namespace tesseract::visualization::threepp
{
namespace
{
namespace g = tesseract::geometry;
namespace vis = tesseract::visualization;

constexpr float PI_2 = 1.57079632679489661923F;
constexpr double UNTIMED_TRAJECTORY_DURATION = 5.0;
constexpr float DEFAULT_ROUGHNESS = 0.3F;
constexpr float DEFAULT_METALNESS = 0.3F;
constexpr float ALPHA_CUTOFF = 0.4F;
constexpr unsigned int ROUND_SEGMENTS = 32;

tp::Vector3 toVector(const Eigen::Vector3d& v)
{
  return { static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()) };
}

tp::Color toColor(const Eigen::Vector4d& c)
{
  return { static_cast<float>(c[0]), static_cast<float>(c[1]), static_cast<float>(c[2]) };
}

void setPose(tp::Object3D& object, const Eigen::Isometry3d& pose)
{
  const Eigen::Quaterniond q(pose.linear());
  object.position.copy(toVector(pose.translation()));
  object.quaternion.set(static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()), static_cast<float>(q.w()));
}

bool endsWithStl(std::string url)
{
  std::transform(url.begin(), url.end(), url.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return url.size() >= 4 && url.compare(url.size() - 4, 4, ".stl") == 0;
}

/** @brief threepp builds round shapes along +Y and tesseract along +Z. +90 degrees about X maps Y onto Z */
std::shared_ptr<tp::BufferGeometry> alongZ(std::shared_ptr<tp::BufferGeometry> geometry)
{
  geometry->rotateX(PI_2);
  return geometry;
}

std::shared_ptr<tp::BufferGeometry> primitiveGeometry(const g::Geometry& geometry)
{
  if (const auto* box = dynamic_cast<const g::Box*>(&geometry))
    return tp::BoxGeometry::create(static_cast<float>(box->getX()), static_cast<float>(box->getY()), static_cast<float>(box->getZ()));
  if (const auto* sphere = dynamic_cast<const g::Sphere*>(&geometry))
    return tp::SphereGeometry::create(static_cast<float>(sphere->getRadius()), ROUND_SEGMENTS, ROUND_SEGMENTS / 2);
  if (const auto* cylinder = dynamic_cast<const g::Cylinder*>(&geometry))
  {
    const auto radius = static_cast<float>(cylinder->getRadius());
    return alongZ(tp::CylinderGeometry::create(radius, radius, static_cast<float>(cylinder->getLength()), ROUND_SEGMENTS));
  }
  if (const auto* capsule = dynamic_cast<const g::Capsule*>(&geometry))
    return alongZ(tp::CapsuleGeometry::create(static_cast<float>(capsule->getRadius()), static_cast<float>(capsule->getLength()), ROUND_SEGMENTS / 2, ROUND_SEGMENTS));
  if (const auto* cone = dynamic_cast<const g::Cone*>(&geometry))
    return alongZ(tp::ConeGeometry::create(static_cast<float>(cone->getRadius()), static_cast<float>(cone->getLength()), ROUND_SEGMENTS));
  return nullptr;
}

tp::TextureWrapping toWrapping(g::MeshTextureWrapMode mode)
{
  switch (mode)
  {
    case g::MeshTextureWrapMode::CLAMP_TO_EDGE:
      return tp::TextureWrapping::ClampToEdge;
    case g::MeshTextureWrapMode::MIRRORED_REPEAT:
      return tp::TextureWrapping::MirroredRepeat;
    default:
      return tp::TextureWrapping::Repeat;
  }
}

/** @brief The threepp filter, or nothing to keep threepp's default */
std::optional<tp::Filter> toFilter(g::MeshTextureFilter filter)
{
  switch (filter)
  {
    case g::MeshTextureFilter::NEAREST:
      return tp::Filter::Nearest;
    case g::MeshTextureFilter::LINEAR:
      return tp::Filter::Linear;
    case g::MeshTextureFilter::NEAREST_MIPMAP_NEAREST:
      return tp::Filter::NearestMipmapNearest;
    case g::MeshTextureFilter::LINEAR_MIPMAP_NEAREST:
      return tp::Filter::LinearMipmapNearest;
    case g::MeshTextureFilter::NEAREST_MIPMAP_LINEAR:
      return tp::Filter::NearestMipmapLinear;
    case g::MeshTextureFilter::LINEAR_MIPMAP_LINEAR:
      return tp::Filter::LinearMipmapLinear;
    default:
      return std::nullopt;
  }
}

/** @brief The geometry attribute threepp samples for a UV set: "uv", "uv2" and "uv3" (Texture::channel 0, 1, 2) */
std::string uvAttributeName(int channel) { return channel == 0 ? "uv" : "uv" + std::to_string(channel + 1); }

constexpr int MAX_UV_SETS = 3;

/** @brief The mesh's own material, or null if it has none that can be trusted */
std::shared_ptr<tp::Material> meshMaterial(const g::PolygonMesh& mesh, tp::BufferGeometry& geometry)
{
  const auto resource = mesh.getResource();
  const auto source = mesh.getMaterial();
  if (resource == nullptr || source == nullptr || endsWithStl(resource->getUrl()))
    return nullptr;

  auto material = tp::MeshStandardMaterial::create();
  const Eigen::Vector4d color = source->getBaseColorFactor();
  material->color = toColor(color);
  material->roughness = static_cast<float>(source->getRoughnessFactor());
  material->metalness = static_cast<float>(source->getMetallicFactor());
  material->emissive = toColor(source->getEmissiveFactor());
  material->emissiveIntensity = static_cast<float>(source->getEmissiveStrength());
  material->side = source->getDoubleSided() ? tp::Side::Double : tp::Side::Front;

  // glTF 2.0 alpha modes, as three.js's GLTFLoader maps them
  switch (source->getAlphaMode())
  {
    case g::MeshMaterialAlphaMode::BLEND:
      material->opacity = static_cast<float>(color[3]);
      material->transparent = true;
      material->depthWrite = false;
      break;
    case g::MeshMaterialAlphaMode::MASK:
      material->opacity = static_cast<float>(color[3]);
      material->alphaTest = static_cast<float>(source->getAlphaCutoff());
      break;
    default:
      material->opacity = 1.0F;
      break;
  }

  const auto& textures = mesh.getTextures();
  if (textures == nullptr || textures->empty())
    return material;

  // Each UV channel of the source mesh used by a texture becomes one threepp UV set, in order of use
  std::map<unsigned, int> uv_sets;
  // Textures that share an image and UV set, as the metallic and roughness maps always do, share one decoded texture
  std::map<std::pair<const tesseract::common::Resource*, int>, std::shared_ptr<tp::Texture>> decoded;
  tp::TextureLoader loader(false);

  for (const auto& texture : *textures)
  {
    const auto image = texture->getTextureImage();
    const auto& uvs = texture->getUVs();
    if (image == nullptr || uvs == nullptr || uvs->size() != mesh.getVertices()->size())
      continue;

    auto set = uv_sets.find(texture->getUVChannel());
    if (set == uv_sets.end())
    {
      if (static_cast<int>(uv_sets.size()) >= MAX_UV_SETS)
      {
        TESSERACT_LOG_WARN("tesseract_visualization_threepp: texture {} needs more than {} UV sets and was skipped", image->getUrl(), MAX_UV_SETS);
        continue;
      }
      const int channel = static_cast<int>(uv_sets.size());
      std::vector<float> flat;
      flat.reserve(uvs->size() * 2);
      for (const auto& uv : *uvs)
        flat.insert(flat.end(), { static_cast<float>(uv.x()), static_cast<float>(uv.y()) });
      geometry.setAttribute(uvAttributeName(channel), tp::FloatBufferAttribute::create(std::move(flat), 2));
      set = uv_sets.emplace(texture->getUVChannel(), channel).first;
    }

    // Color textures are sRGB; data textures (metallic-roughness, normal, occlusion) are linear
    const bool is_color = texture->getType() == g::MeshTextureType::BASE_COLOR || texture->getType() == g::MeshTextureType::EMISSIVE;
    auto& map = decoded[{ image.get(), set->second }];
    if (map == nullptr)
    {
      map = loader.loadFromMemory(image->getUrl(), image->getResourceContents(), is_color ? tp::ColorSpace::sRGB : tp::ColorSpace::NoColorSpace);
      if (map == nullptr)
      {
        TESSERACT_LOG_WARN("tesseract_visualization_threepp: could not decode texture {}", image->getUrl());
        continue;
      }
      map->wrapS = toWrapping(texture->getWrapU());
      map->wrapT = toWrapping(texture->getWrapV());
      if (auto filter = toFilter(texture->getMagFilter()))
        map->magFilter = *filter;
      if (auto filter = toFilter(texture->getMinFilter()))
        map->minFilter = *filter;
      map->channel = set->second;
      map->needsUpdate();
    }

    switch (texture->getType())
    {
      case g::MeshTextureType::BASE_COLOR:
        material->map = map;
        break;
      case g::MeshTextureType::METALLIC_ROUGHNESS:
        // glTF packs roughness in green and metalness in blue, as threepp samples them
        material->metalnessMap = map;
        material->roughnessMap = map;
        break;
      case g::MeshTextureType::NORMAL:
      {
        material->normalMap = map;
        const auto scale = static_cast<float>(source->getNormalScale());
        material->normalScale = tp::Vector2(scale, scale);
        break;
      }
      case g::MeshTextureType::OCCLUSION:
        material->aoMap = map;
        material->aoMapIntensity = static_cast<float>(source->getOcclusionStrength());
        break;
      case g::MeshTextureType::EMISSIVE:
        material->emissiveMap = map;
        break;
    }
  }
  return material;
}

std::shared_ptr<tp::Mesh> polygonMeshToMesh(const g::PolygonMesh& mesh, const std::shared_ptr<tp::Material>& fallback)
{
  const auto& vertices = *mesh.getVertices();
  std::vector<float> positions;
  positions.reserve(vertices.size() * 3);
  for (const auto& v : vertices)
    positions.insert(positions.end(), { static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()) });

  // Faces are [n, i0, ..., i(n-1)]. Meshes are triangles in practice; other polygons are fanned.
  const Eigen::VectorXi& faces = *mesh.getFaces();
  std::vector<unsigned int> indices;
  indices.reserve(static_cast<std::size_t>(faces.size()) / 4 * 3);
  for (Eigen::Index i = 0; i < faces.size(); i += faces[i] + 1)
    for (int k = 1; k + 1 < faces[i]; ++k)
      indices.insert(indices.end(),
                     { static_cast<unsigned int>(faces[i + 1]), static_cast<unsigned int>(faces[i + 1 + k]), static_cast<unsigned int>(faces[i + 2 + k]) });

  auto geometry = tp::BufferGeometry::create();
  geometry->setAttribute("position", tp::FloatBufferAttribute::create(std::move(positions), 3));
  geometry->setIndex(std::move(indices));

  auto material = meshMaterial(mesh, *geometry);

  const auto& normals = mesh.getNormals();
  if (normals != nullptr && normals->size() == vertices.size())
  {
    std::vector<float> flat;
    flat.reserve(normals->size() * 3);
    for (const auto& n : *normals)
      flat.insert(flat.end(), { static_cast<float>(n.x()), static_cast<float>(n.y()), static_cast<float>(n.z()) });
    geometry->setAttribute("normal", tp::FloatBufferAttribute::create(std::move(flat), 3));
  }
  else
  {
    // Separate vertices per face give each face its own normal. Sharing them would round off hard edges.
    geometry = geometry->toNonIndexed();
    geometry->computeVertexNormals();
  }
  return tp::Mesh::create(geometry, material != nullptr ? material : fallback);
}

void appendLink(const tesseract::environment::Environment& env,
                const std::vector<std::shared_ptr<const tesseract::scene_graph::Joint>>& joints,
                const tesseract::common::LinkId& link_id,
                tp::Object3D& parent,
                EnvironmentScene& scene)
{
  const auto link = env.getLink(link_id);
  auto link_node = tp::Group::create();
  link_node->name = link->getName();
  scene.links[link->getName()] = link_node.get();
  parent.add(link_node);

  for (const auto& visual : link->visual)
  {
    auto material = tp::MeshStandardMaterial::create();
    const Eigen::Vector4d color = visual->material != nullptr ? Eigen::Vector4d(visual->material->color) : Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);
    material->color = toColor(color);
    material->opacity = static_cast<float>(color[3]);
    material->alphaTest = ALPHA_CUTOFF;
    material->roughness = DEFAULT_ROUGHNESS;
    material->metalness = DEFAULT_METALNESS;
    for (auto& mesh : geometryToMeshes(*visual->geometry, material))
    {
      mesh->name = link->getName() + "_visual_" + visual->name;
      setPose(*mesh, visual->origin);
      link_node->add(mesh);
    }
  }

  for (const auto& joint : joints)
  {
    if (joint->parent_link_id != link_id)
      continue;
    auto origin_node = tp::Group::create();
    origin_node->name = joint->getName() + "_origin";
    setPose(*origin_node, joint->parent_to_joint_origin_transform);
    link_node->add(origin_node);

    auto joint_node = tp::Group::create();
    joint_node->name = joint->getName();
    origin_node->add(joint_node);
    scene.joints[joint->getName()] = SceneJoint{ joint_node.get(), toVector(joint->axis), joint->type };

    appendLink(env, joints, joint->child_link_id, *joint_node, scene);
  }
}

/** @brief Pose relative to the parent link is carried by the object */
std::shared_ptr<tp::Object3D> arrowObject(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, double length, const Eigen::Vector4d& color)
{
  return tp::ArrowHelper::create(toVector(direction), toVector(origin), static_cast<float>(length), toColor(color));
}

std::shared_ptr<tp::Object3D> axesObject(const Eigen::Isometry3d& pose, double size)
{
  auto axes = tp::AxesHelper::create(static_cast<float>(size));
  setPose(*axes, pose);
  return axes;
}

void appendArrowBetween(std::vector<MarkerObject>& out,
                        const Eigen::Vector3d& from,
                        const Eigen::Vector3d& to,
                        const Eigen::Vector4d& color,
                        const std::string& parent)
{
  const Eigen::Vector3d delta = to - from;
  const double length = delta.norm();
  if (length > 1e-9)
    out.push_back(MarkerObject{ parent, arrowObject(from, delta / length, length, color) });
}

void appendGeometry(std::vector<MarkerObject>& out, const vis::GeometryMarker& marker, const std::string& parent)
{
  auto material = tp::MeshPhongMaterial::create();
  material->side = tp::Side::Double;
  if (marker.geom == nullptr)
    return;
  for (auto& mesh : geometryToMeshes(*marker.geom, material))
  {
    setPose(*mesh, marker.origin);
    out.push_back(MarkerObject{ parent, std::move(mesh) });
  }
}

void appendToolpath(std::vector<MarkerObject>& out, const vis::ToolpathMarker& marker, const std::string& parent)
{
  for (const auto& segment : marker.toolpath)
  {
    if (marker.show_path && segment.size() > 1)
    {
      std::vector<tp::Vector3> points;
      points.reserve(segment.size());
      for (const auto& pose : segment)
        points.push_back(toVector(pose.translation()));
      auto geometry = tp::BufferGeometry::create();
      geometry->setFromPoints(points);
      auto material = tp::LineBasicMaterial::create();
      material->color = tp::Color(0.0F, 1.0F, 0.0F);
      out.push_back(MarkerObject{ parent, tp::Line::create(geometry, material) });
    }
    if (marker.show_axis)
      for (const auto& pose : segment)
        out.push_back(MarkerObject{ parent, axesObject(pose, marker.scale.x()) });
  }
}

// Same colors and arrows as tesseract's Ignition visualization
void appendContactResults(std::vector<MarkerObject>& out, const vis::ContactResultsMarker& marker, const std::string& parent)
{
  namespace c = tesseract::collision;
  const Eigen::Vector4d sweep_color_0(0, 0, 1, 1);
  const Eigen::Vector4d sweep_color_1(0, 0, 0.5, 1);

  const auto selected = [&marker](const tesseract::common::LinkId& id) {
    return std::find(marker.link_ids.begin(), marker.link_ids.end(), id) != marker.link_ids.end();
  };

  for (const auto& result : marker.dist_results)
  {
    const double margin = marker.margin_fn ? marker.margin_fn(result.link_ids[0], result.link_ids[1]) :
                                             marker.margin_data.getCollisionMargin(result.link_ids[0], result.link_ids[1]);
    Eigen::Vector4d color(0, 1, 0, 1);
    if (result.distance < 0)
      color = Eigen::Vector4d(1, 0, 0, 1);
    else if (result.distance < margin)
      color = Eigen::Vector4d(1, 1, 0, 1);

    if (result.cc_type[0] == c::ContinuousCollisionType::CCType_Between)
      appendArrowBetween(out, result.transform[0] * result.nearest_points_local[0], result.cc_transform[0] * result.nearest_points_local[0], sweep_color_0, parent);
    if (result.cc_type[1] == c::ContinuousCollisionType::CCType_Between)
      appendArrowBetween(out, result.transform[1] * result.nearest_points_local[1], result.cc_transform[1] * result.nearest_points_local[1], sweep_color_1, parent);

    const bool first = selected(result.link_ids[0]);
    const bool second = selected(result.link_ids[1]);
    if (first && second)
    {
      appendArrowBetween(out, result.nearest_points[0], result.nearest_points[1], color, parent);
      appendArrowBetween(out, result.nearest_points[1], result.nearest_points[0], color, parent);
    }
    else if (first)
    {
      appendArrowBetween(out, result.nearest_points[1], result.nearest_points[0], color, parent);
    }
    else
    {
      appendArrowBetween(out, result.nearest_points[0], result.nearest_points[1], color, parent);
    }
  }
}
}  // namespace

JointValues sceneStateToJointValues(const tesseract::scene_graph::SceneState& state)
{
  JointValues values;
  values.reserve(state.joints.size());
  for (const auto& [id, value] : state.joints)
    values.emplace(id.name(), value);
  return values;
}

EnvironmentScene environmentToScene(const tesseract::environment::Environment& env)
{
  std::vector<std::shared_ptr<const tesseract::scene_graph::Joint>> joints;
  for (const auto& id : env.getJointIds())
    joints.push_back(env.getJoint(id));

  EnvironmentScene scene;
  scene.root = tp::Group::create();
  appendLink(env, joints, env.getRootLinkId(), *scene.root, scene);
  return scene;
}

void setJointValues(const EnvironmentScene& scene, const JointValues& values)
{
  using tesseract::scene_graph::JointType;
  for (const auto& [name, value] : values)
  {
    const auto joint = scene.joints.find(name);
    if (joint == scene.joints.end())
      continue;
    const auto amount = static_cast<float>(value);
    if (joint->second.type == JointType::REVOLUTE || joint->second.type == JointType::CONTINUOUS)
      joint->second.node->quaternion.setFromAxisAngle(joint->second.axis, amount);
    else if (joint->second.type == JointType::PRISMATIC)
      joint->second.node->position.copy(tp::Vector3(joint->second.axis).multiplyScalar(amount));
  }
}

std::vector<std::shared_ptr<tp::Mesh>> geometryToMeshes(const g::Geometry& geometry, const std::shared_ptr<tp::Material>& material)
{
  if (const auto* compound = dynamic_cast<const g::CompoundMesh*>(&geometry))
  {
    std::vector<std::shared_ptr<tp::Mesh>> meshes;
    for (const auto& mesh : compound->getMeshes())
      meshes.push_back(polygonMeshToMesh(*mesh, material));
    return meshes;
  }
  if (const auto* mesh = dynamic_cast<const g::PolygonMesh*>(&geometry))
    return { polygonMeshToMesh(*mesh, material) };
  if (auto primitive = primitiveGeometry(geometry))
    return { tp::Mesh::create(primitive, material) };

  TESSERACT_LOG_WARN("tesseract_visualization_threepp: skipping geometry of unsupported type {}", static_cast<int>(geometry.getType()));
  return {};
}

std::vector<MarkerObject> markerToObjects(const vis::Marker& marker, const std::string& default_parent_link)
{
  const std::string& parent = marker.getParentLink().empty() ? default_parent_link : marker.getParentLink();
  std::vector<MarkerObject> out;

  switch (marker.getType())
  {
    case static_cast<int>(vis::MarkerType::AXIS):
    {
      const auto& axis = dynamic_cast<const vis::AxisMarker&>(marker);
      out.push_back(MarkerObject{ parent, axesObject(axis.axis, axis.getScale().x()) });
      break;
    }
    case static_cast<int>(vis::MarkerType::ARROW):
    {
      // ArrowMarker's two-point constructor puts the pose at the middle of the arrow and points +Z
      // along it, whatever the class comment says about the base. Its pose is treated as the center.
      const auto& arrow = dynamic_cast<const vis::ArrowMarker&>(marker);
      const double length = arrow.shaft_length + arrow.head_length;
      const Eigen::Vector3d direction = arrow.pose.linear().col(2);
      const Eigen::Vector4d color = arrow.material != nullptr ? Eigen::Vector4d(arrow.material->color) : Eigen::Vector4d(1, 1, 1, 1);
      out.push_back(MarkerObject{ parent, arrowObject(arrow.pose.translation() - 0.5 * length * direction, direction, length, color) });
      break;
    }
    case static_cast<int>(vis::MarkerType::GEOMETRY):
      appendGeometry(out, dynamic_cast<const vis::GeometryMarker&>(marker), parent);
      break;
    case static_cast<int>(vis::MarkerType::TOOLPATH):
      appendToolpath(out, dynamic_cast<const vis::ToolpathMarker&>(marker), parent);
      break;
    case static_cast<int>(vis::MarkerType::CONTACT_RESULTS):
      appendContactResults(out, dynamic_cast<const vis::ContactResultsMarker&>(marker), parent);
      break;
    default:
      TESSERACT_LOG_WARN("tesseract_visualization_threepp: marker type {} cannot be shown", marker.getType());
      break;
  }
  return out;
}

JointValues JointValueTrajectory::sample(double time) const
{
  if (time <= times.front() || times.size() == 1)
    return states.front();
  if (time >= times.back())
    return states.back();

  const auto after = static_cast<std::size_t>(std::upper_bound(times.begin(), times.end(), time) - times.begin());
  const std::size_t before = after - 1;
  const double span = times[after] - times[before];
  const double blend = span > 0 ? (time - times[before]) / span : 0.0;

  JointValues values = states[before];
  for (auto& [name, value] : values)
    if (const auto it = states[after].find(name); it != states[after].end())
      value += blend * (it->second - value);
  return values;
}

JointValueTrajectory toJointValueTrajectory(const tesseract::common::JointTrajectory& trajectory,
                                            const tesseract::scene_graph::StateSolver& state_solver)
{
  if (trajectory.states.empty())
    throw std::invalid_argument("Trajectory is empty");

  JointValueTrajectory out;
  out.times.reserve(trajectory.states.size());
  out.states.reserve(trajectory.states.size());
  for (const auto& state : trajectory.states)
  {
    out.times.push_back(state.time);
    out.states.push_back(sceneStateToJointValues(state_solver.getState(state.joint_ids, state.position)));
  }

  const bool untimed = out.times.size() > 1 && std::all_of(out.times.begin(), out.times.end(), [&out](double time) { return time == out.times.front(); });
  if (untimed)
    for (std::size_t i = 0; i < out.times.size(); ++i)
      out.times[i] = UNTIMED_TRAJECTORY_DURATION * static_cast<double>(i) / static_cast<double>(out.times.size() - 1);
  return out;
}

}  // namespace tesseract::visualization::threepp
