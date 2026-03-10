#pragma once

#include <raylib.h>
#include <raymath.h>

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <pugixml.hpp>
#include <vector>

namespace urdf {

// Extract XYZ intrinsic Euler angles (roll-pitch-yaw) from a rotation matrix.
// Uses the convention: R = Rx(roll) * Ry(pitch) * Rz(yaw).
// Assumes no gimbal lock (pitch != +/-90 degrees).
inline Vector3 MatrixToXYZ(const Matrix& m) {
    Vector3 xyz{Vector3Zero()};

    xyz.x = std::atan2(-m.m9, m.m10);
    xyz.y = std::atan2(m.m8, std::sqrt(m.m0 * m.m0 + m.m4 * m.m4));
    xyz.z = std::atan2(-m.m4, m.m0);

    return xyz;
}

inline Vector3 PosFromMatrix(const Matrix& m) { return Vector3{m.m12, m.m13, m.m14}; }

struct Origin {
    Origin();
    Origin(const char* xyz, const char* rpy);
    Origin(const Vector3& xyz, const Vector3& rpy);

    Matrix toMatrix(void) const;

    Vector3 xyz;
    Vector3 rpy;
};

using OriginRawPtr = Origin*;

struct GeometryType {
    virtual Model generateGeometry() = 0;
};

struct Box : public GeometryType {
    Box();
    Box(const char* size);
    Box(const Vector3& size);
    virtual ~Box() = default;

    Vector3 size;

    virtual Model generateGeometry() override;
};

struct Cylinder : public GeometryType {
    Cylinder();
    Cylinder(const char* radius, const char* length);
    virtual ~Cylinder() = default;

    float radius;
    float length;

    virtual Model generateGeometry() override;
};

struct Sphere : public GeometryType {
    Sphere();
    Sphere(const char* radius);
    Sphere(float radius);
    virtual ~Sphere() = default;

    float radius;

    virtual Model generateGeometry() override;
};

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

struct Geometry {
    GeometryTypePtr type;
};

struct Material {
    std::string name;
    std::optional<Vector4> rgba;
    std::optional<std::string> texture_file;
};

struct Inertia {
    Inertia(float ixx, float iyy, float izz, float ixy, float ixz, float iyz);

    float ixx, iyy, izz, ixy, ixz, iyz;
};

struct Inertial {
    explicit Inertial(const char* xyz = nullptr, const char* rpy = nullptr, float mass = 0.0F,
                      float ixx = 0.0F, float iyy = 0.0F, float izz = 0.0F, float ixy = 0.0F,
                      float ixz = 0.0F, float iyz = 0.0F);

    Origin origin;
    float mass;
    Inertia inertia;
};

struct Visual {
    std::optional<std::string> name;
    std::optional<Origin> origin;
    Geometry geometry;
    std::optional<std::string> material_name;
};

struct Collision {
    std::optional<std::string> name;
    std::optional<Origin> origin;
    Geometry geometry;
};

struct Axis {
    Axis();
    explicit Axis(const Vector3& _xyz);
    explicit Axis(const char* s_xyz);

    Vector3 xyz;
};

struct Dynamics {
    Dynamics();
    Dynamics(float _damping, float _friction);

    float damping;
    float friction;
};

struct Limit {
    Limit(float lower, float upper, float effort, float velocity);

    float lower;
    float upper;
    float effort;
    float velocity;
};

struct Mimic {
    std::string joint;
    float multiplier{1.0f};
    float offset{0.0f};
};

struct Calibration {
    float rising{0.0f};
    float falling{0.0f};
};

struct Link {
    Link() = default;
    explicit Link(std::string name);

    std::string name;
    std::optional<Inertial> inertial;
    std::vector<Visual> visual;
    std::vector<Collision> collision;
};

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

    std::string name;
    Type type;
    std::string parent;
    std::string child;
    std::optional<Origin> origin;
    std::optional<Axis> axis;
    std::optional<Calibration> calibration;
    std::optional<Dynamics> dynamics;
    std::optional<Limit> limit;  // required only for prismatic and revolute
    std::optional<Mimic> mimic;
};

struct TreeNode {
    TreeNode() = default;
    virtual ~TreeNode() = default;
};

struct LinkNode;
struct JointNode;

using TreeNodePtr = std::shared_ptr<TreeNode>;
using LinkNodePtr = std::shared_ptr<LinkNode>;
using JointNodePtr = std::shared_ptr<JointNode>;

struct HitResult {
    LinkNodePtr link;
    enum Type { kVisual, kCollision } type;
    int index = 0;  // visual index when kVisual, collision index when kCollision
};

struct LinkNode : TreeNode {
    LinkNode();
    LinkNode(Link link, JointNodePtr parent_joint);

    void addVisual();
    void deleteVisual(int i);
    void addCollision();
    void deleteCollision(int i);

    Link link;
    JointNodePtr parent;
    std::vector<JointNodePtr> children;

    std::vector<Model> visual_models;
    std::vector<Model> collision_models;

    // Homogeneous transform from link frame into world frame.
    // Naming convention: a_T_b = transform that maps a point in frame b to frame a.
    Matrix w_T_l;
};

struct JointNode : TreeNode {
    JointNode(Joint joint, LinkNodePtr parent, LinkNodePtr child);

    Joint joint;
    LinkNodePtr parent;
    LinkNodePtr child;
};

class Robot {
   public:
    explicit Robot(LinkNodePtr root, const std::map<std::string, Material>& materials = {});
    ~Robot();

    void forwardKinematics();
    static void forwardKinematics(LinkNodePtr& link);

    void buildGeometry();
    void updateMaterial(const LinkNodePtr& link);

    void printTree() const;

    void setShader(const Shader& shader);

    void draw(const LinkNodePtr& highlighted = nullptr,
              const LinkNodePtr& selected = nullptr) const;

    LinkNodePtr getRoot() const;

    std::optional<HitResult> getLink(const Ray& ray);

    void forEveryLink(const std::function<void(const LinkNodePtr&)>& func) const;
    void forEveryJoint(const std::function<void(const JointNodePtr&)>& func) const;

    const std::map<std::string, Material>& getMaterials() const;
    std::map<std::string, Material>& getMutableMaterials();

   private:
    static Matrix originToMatrix(std::optional<Origin>& origin);

    LinkNodePtr root_;

    Shader visual_shader_;

    // std::map<std::string, std::vector<float>> q; // vector motivation: joints with multiple dof

    std::map<std::string, Material> materials_;
};

using RobotPtr = std::shared_ptr<Robot>;

RobotPtr buildRobot(const char* urdf_file);

void exportRobot(const Robot& robot, std::string out_filename);

pugi::xml_node findRoot(const pugi::xml_document& doc);

// XML to data structures
Link xmlNodeToLink(const pugi::xml_node& xml_node, const std::string& urdf_dir = "");
Joint xmlNodeToJoint(const pugi::xml_node& xml_node);

std::optional<Inertial> xmlNodeToInertial(const pugi::xml_node& xml_node);
std::optional<Origin> xmlNodeToOrigin(const pugi::xml_node& xml_node);
Geometry xmlNodeToGeometry(const pugi::xml_node& xml_node, const std::string& urdf_dir = "");
std::optional<Material> xmlNodeToMaterial(const pugi::xml_node& xml_node);

// Data structures to XML
void linkToXmlNode(pugi::xml_node& xml_node, const Link& link);
void jointToXmlNode(pugi::xml_node& xml_node, const Joint& joint);

void inertialToXmlNode(pugi::xml_node& xml_node, const Inertial& inertial);
void originToXmlNode(pugi::xml_node& xml_node, const Origin& origin);
void geometryToXmlNode(pugi::xml_node& xml_node, const Geometry& geometry);
void materialToXmlNode(pugi::xml_node& xml_node, const Material& material);

}  // namespace urdf
