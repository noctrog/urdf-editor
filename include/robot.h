#pragma once

#include <raylib.h>
#include <raymath.h>

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <pugixml.hpp>
#include <vector>

namespace urdf {

// Extracts URDF fixed-axis roll-pitch-yaw Euler angles from a rotation matrix.
//
// The rotation convention is R = Rz(yaw) * Ry(pitch) * Rx(roll). The result
// is undefined at gimbal lock (pitch of +/-90 degrees).
//
// Args:
//   m: Matrix whose rotational components are converted to Euler angles.
//
// Returns:
//   Roll, pitch, and yaw in radians.
inline Vector3 MatrixToXYZ(const Matrix& m) {
    Vector3 xyz{Vector3Zero()};

    xyz.x = std::atan2(m.m6, m.m10);
    xyz.y = std::atan2(-m.m2, std::sqrt(m.m0 * m.m0 + m.m1 * m.m1));
    xyz.z = std::atan2(m.m1, m.m0);

    return xyz;
}

// Extracts the translation component of a homogeneous transform.
inline Vector3 PosFromMatrix(const Matrix& m) { return Vector3{m.m12, m.m13, m.m14}; }

// Stores a URDF pose as position and roll-pitch-yaw Euler angles.
struct Origin {
    // Creates the identity pose.
    Origin();
    // Parses space-separated URDF xyz and rpy attributes.
    Origin(const char* xyz, const char* rpy);
    // Creates a pose from position and roll-pitch-yaw radians.
    Origin(const Vector3& xyz, const Vector3& rpy);

    // Converts this pose to a homogeneous transform.
    Matrix toMatrix(void) const;

    Vector3 xyz;
    Vector3 rpy;
};

using OriginRawPtr = Origin*;

// Defines the common interface for renderable URDF geometry primitives.
struct GeometryType {
    virtual Model generateGeometry() = 0;
};

// Represents a URDF box with dimensions along the local X, Y, and Z axes.
struct Box : public GeometryType {
    Box();
    Box(const char* size);
    Box(const Vector3& size);
    virtual ~Box() = default;

    Vector3 size;

    virtual Model generateGeometry() override;
};

// Represents a URDF cylinder aligned with its local Z axis.
struct Cylinder : public GeometryType {
    Cylinder();
    Cylinder(const char* radius, const char* length);
    virtual ~Cylinder() = default;

    float radius;
    float length;

    virtual Model generateGeometry() override;
};

// Represents a URDF sphere centered at its local origin.
struct Sphere : public GeometryType {
    Sphere();
    Sphere(const char* radius);
    Sphere(float radius);
    virtual ~Sphere() = default;

    float radius;

    virtual Model generateGeometry() override;
};

// Represents externally stored mesh geometry and its URDF scale.
struct Mesh : public GeometryType {
    Mesh(const char* filename, const std::string& resolved_path = "",
         const Vector3& scale = {1.0f, 1.0f, 1.0f});
    virtual ~Mesh() = default;

    std::string filename;
    std::string resolved_path;
    Vector3 scale{1.0f, 1.0f, 1.0f};

    virtual Model generateGeometry() override;
};

using GeometryTypePtr = std::shared_ptr<GeometryType>;

// Wraps one concrete URDF geometry type.
struct Geometry {
    GeometryTypePtr type;
};

// Describes a globally named URDF material and its optional appearance data.
struct Material {
    std::string name;
    std::optional<Vector4> rgba;
    std::optional<std::string> texture_file;
};

// Stores the six independent elements of a symmetric inertia tensor.
struct Inertia {
    Inertia(float ixx, float iyy, float izz, float ixy, float ixz, float iyz);

    float ixx, iyy, izz, ixy, ixz, iyz;
};

// Stores a link's mass, inertia tensor, and center-of-mass pose.
struct Inertial {
    explicit Inertial(const char* xyz = nullptr, const char* rpy = nullptr, float mass = 0.0F,
                      float ixx = 0.0F, float iyy = 0.0F, float izz = 0.0F, float ixy = 0.0F,
                      float ixz = 0.0F, float iyz = 0.0F);

    Origin origin;
    float mass;
    Inertia inertia;
};

// Describes a renderable link element and its optional material reference.
struct Visual {
    std::optional<std::string> name;
    std::optional<Origin> origin;
    Geometry geometry;
    std::optional<std::string> material_name;
};

// Describes a non-rendered collision shape associated with a link.
struct Collision {
    std::optional<std::string> name;
    std::optional<Origin> origin;
    Geometry geometry;
};

// Stores a joint's motion axis in the joint frame.
struct Axis {
    Axis();
    explicit Axis(const Vector3& _xyz);
    explicit Axis(const char* s_xyz);

    Vector3 xyz;
};

// Stores optional damping and friction coefficients for a joint.
struct Dynamics {
    Dynamics();
    Dynamics(float _damping, float _friction);

    float damping;
    float friction;
};

// Stores lower/upper position, effort, and velocity limits for a joint.
struct Limit {
    Limit(float lower, float upper, float effort, float velocity);

    float lower;
    float upper;
    float effort;
    float velocity;
};

// Stores the joint and affine relation used by a mimic joint.
struct Mimic {
    std::string joint;
    float multiplier{1.0f};
    float offset{0.0f};
};

// Stores optional rising and falling calibration reference positions.
struct Calibration {
    float rising{0.0f};
    float falling{0.0f};
};

// Stores soft joint-limit controller parameters.
struct SafetyController {
    float k_velocity{0.0f};
    float k_position{0.0f};
    float soft_lower_limit{0.0f};
    float soft_upper_limit{0.0f};
};

// Contains the serializable URDF data owned by one link node.
struct Link {
    Link() = default;
    explicit Link(std::string name);

    std::string name;
    std::optional<Inertial> inertial;
    std::vector<Visual> visual;
    std::vector<Collision> collision;
};

// Contains the serializable URDF data owned by one joint node.
struct Joint {
    Joint(const char* name, const char* parent, const char* child, const char* type);

    enum Type : int {
        kRevolute = 0,
        kContinuous,
        kPrismatic,
        kFixed,
        kFloating,
        kPlanar,
        kNumJointTypes
    };

    // Returns whether this joint contributes a configurable motion transform.
    bool isArticulated() const {
        return type == kRevolute || type == kContinuous || type == kPrismatic;
    }

    std::string name;
    Type type;
    std::string parent;
    std::string child;
    std::optional<Origin> origin;
    std::optional<Axis> axis;
    std::optional<Calibration> calibration;
    std::optional<Dynamics> dynamics;
    // Present for revolute and prismatic joints, where URDF requires limits.
    std::optional<Limit> limit;
    std::optional<Mimic> mimic;
    std::optional<SafetyController> safety_controller;
};

// Provides a polymorphic base for selectable link and joint tree nodes.
struct TreeNode {
    TreeNode() = default;
    virtual ~TreeNode() = default;
};

struct LinkNode;
struct JointNode;

using TreeNodePtr = std::shared_ptr<TreeNode>;
using LinkNodePtr = std::shared_ptr<LinkNode>;
using JointNodePtr = std::shared_ptr<JointNode>;

// Identifies the visual or collision element hit by a scene ray.
struct HitResult {
    LinkNodePtr link;
    enum Type { kVisual, kCollision } type;
    // Index in link.visual for kVisual or link.collision for kCollision.
    int index = 0;
};

// A link node in the editor tree, including children and generated models.
struct LinkNode : TreeNode {
    LinkNode();
    LinkNode(Link link, JointNodePtr parent_joint);

    // Appends a default visual and its matching render-model slot.
    void addVisual();
    // Removes a visual and its matching render model.
    void deleteVisual(int i);
    // Appends a default collision and its matching render-model slot.
    void addCollision();
    // Removes a collision and its matching render model.
    void deleteCollision(int i);

    Link link;
    JointNodePtr parent;
    std::vector<JointNodePtr> children;

    std::vector<Model> visual_models;
    std::vector<Model> collision_models;

    // Homogeneous transform from the link frame into the world frame.
    // Naming convention: a_T_b maps a point in frame b into frame a.
    Matrix w_T_l;
};

// A joint node linking a parent link to its child link in the editor tree.
struct JointNode : TreeNode {
    JointNode(Joint joint, LinkNodePtr parent, LinkNodePtr child);

    Joint joint;
    LinkNodePtr parent;
    LinkNodePtr child;
    // Angle for revolute/continuous joints or displacement for prismatic joints.
    float q{0.0f};
};

// Reports a URDF validation error or warning.
struct ValidationMessage {
    enum Level { kError, kWarning } level;
    std::string message;
};

// Owns the editable URDF tree, shared materials, and generated scene resources.
class Robot {
   public:
    // Creates a robot from its root link and optional global materials.
    explicit Robot(LinkNodePtr root, const std::map<std::string, Material>& materials = {});
    ~Robot();

    // Recomputes world transforms for every link, visual, and collision model.
    void forwardKinematics();
    // Recomputes transforms for link and all of its descendants.
    static void forwardKinematics(LinkNodePtr& link);

    // Generates all visual and collision models from their URDF geometry.
    void buildGeometry();
    // Applies the link's material references to its generated visual models.
    void updateMaterial(const LinkNodePtr& link);

    // Writes a human-readable representation of the link/joint hierarchy to stdout.
    void printTree() const;

    // Sets the shader used by generated visual models.
    void setShader(const Shader& shader);

    // Draws all link visuals and, optionally, collision geometry.
    //
    // Args:
    //   highlighted: Link rendered with the hover treatment, if any.
    //   selected: Link rendered with the selection treatment, if any.
    //   show_collisions: Whether collision models should be drawn.
    void draw(const LinkNodePtr& highlighted = nullptr, const LinkNodePtr& selected = nullptr,
              bool show_collisions = true) const;

    // Returns the root link of the editable tree.
    LinkNodePtr getRoot() const;

    // Performs ray-vs-mesh picking against visual and collision geometry.
    std::optional<HitResult> getLink(const Ray& ray);

    // Invokes func once for every link in tree traversal order.
    void forEveryLink(const std::function<void(const LinkNodePtr&)>& func) const;
    // Invokes func once for every joint in tree traversal order.
    void forEveryJoint(const std::function<void(const JointNodePtr&)>& func) const;

    // Checks the tree for invalid or suspicious URDF data.
    std::vector<ValidationMessage> validate() const;

    // Returns the global material definitions.
    const std::map<std::string, Material>& getMaterials() const;
    // Returns mutable global material definitions for command implementations.
    std::map<std::string, Material>& getMutableMaterials();

    // Returns raw transmission XML retained for lossless load/save round trips.
    const std::vector<std::string>& getTransmissions() const;
    // Replaces the raw transmission XML retained for export.
    void setTransmissions(std::vector<std::string> transmissions);

    // Loads texture resources for all global materials that reference texture files.
    void loadTextures();
    // Replaces the GPU texture for one material after its texture path changes.
    void reloadTexture(const std::string& material_name);

   private:
    // Loads and caches the texture referenced by one material.
    void loadTextureForMaterial(const std::string& name, const Material& mat);
    // Converts an optional origin to an identity transform when it is absent.
    static Matrix originToMatrix(const std::optional<Origin>& origin);

    LinkNodePtr root_;

    Shader visual_shader_;

    std::map<std::string, Material> materials_;
    std::map<std::string, Texture2D> loaded_textures_;
    // Raw XML is retained because the editor does not otherwise model transmissions.
    std::vector<std::string> transmissions_;
};

using RobotPtr = std::shared_ptr<Robot>;

// Computes the inertia tensor for a uniform-density primitive about its center of mass.
//
// Off-diagonal terms are zero for axis-aligned primitives. Mesh geometry is not supported.
//
// Args:
//   mass: Total mass of the primitive.
//   geometry: Box, cylinder, or sphere geometry to evaluate.
//
// Returns:
//   The primitive's inertia tensor, or zeros for unsupported geometry.
Inertia computeInertia(float mass, const GeometryTypePtr& geometry);

// Returns an independent copy of a concrete geometry object.
GeometryTypePtr cloneGeometry(const GeometryTypePtr& src);
// Clones source and every descendant, attaching the clone below attach_parent.
JointNodePtr cloneSubtree(const LinkNodePtr& source, const LinkNodePtr& attach_parent,
                          const RobotPtr& robot);

// Resolves an absolute, relative, or package:// mesh path from a URDF directory.
std::string resolveFilePath(const std::string& filename, const std::string& urdf_dir);

// Parses a URDF file into an editable robot tree and shared materials.
RobotPtr buildRobot(const char* urdf_file);

// Serializes a robot to a URDF file.
void exportRobot(const Robot& robot, std::string out_filename);

// Returns the document's <robot> root node, including namespace-tolerant matches.
pugi::xml_node findRoot(const pugi::xml_document& doc);

// Converts XML nodes to editor data structures.
//
// File-relative geometry references are resolved against urdf_dir.
Link xmlNodeToLink(const pugi::xml_node& xml_node, const std::string& urdf_dir = "");
Joint xmlNodeToJoint(const pugi::xml_node& xml_node);

std::optional<Inertial> xmlNodeToInertial(const pugi::xml_node& xml_node);
std::optional<Origin> xmlNodeToOrigin(const pugi::xml_node& xml_node);
Geometry xmlNodeToGeometry(const pugi::xml_node& xml_node, const std::string& urdf_dir = "");
std::optional<Material> xmlNodeToMaterial(const pugi::xml_node& xml_node);

// Serializes editor data structures into existing XML nodes.
void linkToXmlNode(pugi::xml_node& xml_node, const Link& link);
void jointToXmlNode(pugi::xml_node& xml_node, const Joint& joint);

void inertialToXmlNode(pugi::xml_node& xml_node, const Inertial& inertial);
void originToXmlNode(pugi::xml_node& xml_node, const Origin& origin);
void geometryToXmlNode(pugi::xml_node& xml_node, const Geometry& geometry);
void materialToXmlNode(pugi::xml_node& xml_node, const Material& material);

}  // namespace urdf
