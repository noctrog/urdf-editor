#include <vector>
#include <variant>
#include <optional>
#include <memory>

#include <pugixml.hpp>
#include <raylib.h>

namespace urdf {

struct Origin {
    Origin(const char *xyz, const char *rpy);
    Origin(const Vector3& xyz, const Vector3& rpy);

    Vector3 xyz_;
    Vector3 rpy_;
};

struct Box {
    Box();
    Box(const char *size);
    Box(const Vector3& size);

    Vector3 size_;
};

struct Cylinder {
    Cylinder();
    Cylinder(const char *radius, const char *length);
    Cylinder(float radius, float length);

    float radius_;
    float length_;
};

struct Sphere {
    Sphere();
    Sphere(const char *radius);
    Sphere(float radius);

    float radius_;
};

struct Mesh {
    std::string filename;
};

// TODO: implement mesh geometry

using GeometryType = std::variant<Box, Cylinder, Sphere, Mesh>;

struct Geometry {
    GeometryType type;
};

struct Material {
    std::optional<std::string> name;
    std::optional<Vector4> rgba;
    std::optional<std::string> texture_file;
};

struct Inertia {
    Inertia(float ixx, float iyy, float izz, float ixy, float ixz, float iyz);

    float ixx_, iyy_, izz_, ixy_, ixz_, iyz_;
};

struct Inertial {
    Inertial(const char *xyz = nullptr, const char *rpy = nullptr,
             float mass = 0.0f,
             float ixx = 0.0f, float iyy = 0.0f, float izz = 0.0f,
             float ixy = 0.0f, float ixz = 0.0f, float iyz = 0.0f);

    Origin origin_;
    float mass_;
    Inertia inertia_;
};

struct Visual {
    std::optional<std::string> name;
    std::optional<Origin> origin;
    Geometry geometry;
    std::optional<Material> material;
};

struct Collision {
    std::optional<std::string> name;
    std::optional<Origin> origin;
    Geometry geometry;
};

struct Axis {
    Axis(const Vector3& _xyz);
    Axis(const char *xyz);

    Vector3 xyz;
};

struct Dynamics {
    Dynamics(float _damping, float _friction);

    float damping;
    float friction;
};

struct Limit
{
    Limit(float lower, float upper, float effort, float velocity);

    float lower;
    float upper;
    float effort;
    float velocity;
};

struct Link {
    Link();

    std::string name;
    std::optional<Inertial> inertial;
    std::optional<Visual> visual;
    std::optional<Collision> collision;
};

struct Joint {
    Joint(const char *name, const char *parent, const char *child, const char *type);

    enum Type {
        REVOLUTE, CONTINUOUS, PRISMATIC, FIXED, FLOATING, PLANAR
    };

    std::string name;
    Type type;
    std::string parent;
    std::string child;
    std::optional<Origin> origin;
    std::optional<Axis> axis;
    // TODO: calibration
    std::optional<Dynamics> dynamics;
    std::optional<Limit> limit;  // required only for prismatic and revolute
    // TODO: mimic
};

struct LinkNode;
struct JointNode;

struct LinkNode
{
    Link link;
    std::shared_ptr<JointNode> parent;
    std::vector<std::shared_ptr<JointNode>> children;

    Model visual_model;
    Model collision_model;
};

struct JointNode
{
    Joint joint;
    std::shared_ptr<LinkNode> parent;
    std::shared_ptr<LinkNode> child;
};

class Parser {
public:
    Parser(const char *urdf_file);
    ~Parser();

    std::shared_ptr<LinkNode> build_robot(void);

    void print_tree(std::shared_ptr<LinkNode> tree_root);

private:
    pugi::xml_node find_root();

    Link xml_node_to_link(const pugi::xml_node& xml_node);
    Joint xml_node_to_joint(const pugi::xml_node& xml_node);

    std::optional<Inertial> xml_node_to_inertial(const pugi::xml_node& xml_node);
    std::optional<Origin> xml_node_to_origin(const pugi::xml_node& xml_node);
    Geometry xml_node_to_geometry(const pugi::xml_node& xml_node);
    std::optional<Material> xml_node_to_material(const pugi::xml_node& xml_node);

    pugi::xml_document doc_;
};

} // namespace urdf
