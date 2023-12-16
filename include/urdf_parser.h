#include <vector>
#include <variant>
#include <optional>
#include <map>

#include <pugixml.hpp>
#include <raylib.h>

namespace urdf {

struct Origin {
    Origin(const char *xyz, const char *rpy);
    Origin(const Vector3& xyz, const Vector3& rpy);

    Vector3 xyz;
    Vector3 rpy;
};

struct GeometryType {
    virtual ::Mesh generateGeometry() = 0;
};

struct Box : GeometryType {
    Box();
    Box(const char *size);
    Box(const Vector3& size);
    virtual ~Box() = default;

    Vector3 size;

    virtual ::Mesh generateGeometry() override;
};

struct Cylinder : GeometryType {
    Cylinder();
    Cylinder(const char *radius, const char *length);
    Cylinder(float radius, float length);
    virtual ~Cylinder() = default;

    float radius;
    float length;

    virtual ::Mesh generateGeometry() override;
};

struct Sphere : GeometryType {
    Sphere();
    Sphere(const char *radius);
    Sphere(float radius);
    virtual ~Sphere() = default;

    float radius;

    virtual ::Mesh generateGeometry() override;
};

struct Mesh {
    std::string filename;

    void generateGeometry(::Mesh& mesh);
};

// TODO: implement mesh geometry

struct Geometry {
    std::shared_ptr<GeometryType> type;
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
    std::vector<Collision> collision;
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

using LinkNodePtr = std::shared_ptr<LinkNode>;
using JointNodePtr = std::shared_ptr<JointNode>;

struct LinkNode
{
    LinkNode();
    LinkNode(const Link& link, const JointNodePtr& parent_joint);

    Link link;
    std::shared_ptr<JointNode> parent;
    std::vector<std::shared_ptr<JointNode>> children;

    ::Mesh visual_mesh;
    std::vector<::Mesh> collision_mesh;

    Model visual_model;
    std::vector<Model> collision_model;

    Matrix T;
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

    LinkNodePtr build_robot(void);

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

class Robot {
public:
    Robot(const LinkNodePtr& root);

    void forward_kinematics(void);

    void build_geometry(void);

    void draw(void);

    void print_tree(void);

private:
    static Matrix origin_to_matrix(std::optional<Origin>& origin);

    LinkNodePtr root_;

    std::map<std::string, float> q;
};

} // namespace urdf
