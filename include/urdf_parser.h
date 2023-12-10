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
    Box(const char *size);
    Box(const Vector3& size);

    Vector3 size_;
};

struct Cylinder {
    Cylinder(const char *radius, const char *length);
    Cylinder(float radius, float length);

    float radius_;
    float length_;
};

struct Sphere {
    Sphere(const char *radius);
    Sphere(float radius);

    float radius_;
};

// TODO: implement mesh geometry

using GeometryType = std::variant<Box, Cylinder, Sphere>;

struct Geometry {
    Geometry(const GeometryType& geom);

    GeometryType geom_;
};

struct Material {
    Material(const char *name);
    Material(const char *name, const Vector4& rgba);
    Material(const char *name, const char *texture_file);

    std::string name_;
    Vector4 rgba_;
    std::string texture_file_;
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
    Visual(const char *name, Origin *origin_p, Geometry *geometry_p, Material *material_p);

    std::string name_;
    Origin origin_;
    Geometry geometry_;
    Material material_;
};

struct Collision {
    Collision(const char *name, Origin *origin_p, Geometry *geometry_p);

    std::string name_;
    Origin origin_;
    Geometry geometry_;
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

    void print_tree(void);

private:
    pugi::xml_node find_root();

    Link xml_node_to_link(const pugi::xml_node& xml_node);
    Joint xml_node_to_joint(const pugi::xml_node& xml_node);

    pugi::xml_document doc_;
    std::shared_ptr<LinkNode> tree_root_;
};

} // namespace urdf
