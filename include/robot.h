#pragma once

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

struct Mesh : GeometryType {
    Mesh(const char *filename);

    std::string filename;

    virtual ::Mesh generateGeometry() override;
};

// TODO: implement mesh geometry

struct Geometry {
    std::shared_ptr<GeometryType> type;
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
    Inertial(const char *xyz = nullptr, const char *rpy = nullptr,
             float mass = 0.0f,
             float ixx = 0.0f, float iyy = 0.0f, float izz = 0.0f,
             float ixy = 0.0f, float ixz = 0.0f, float iyz = 0.0f);

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

struct TreeNode {
    TreeNode() = default;
    virtual ~TreeNode() = 0;
};

struct LinkNode;
struct JointNode;

using TreeNodePtr = std::shared_ptr<TreeNode>;
using LinkNodePtr = std::shared_ptr<LinkNode>;
using JointNodePtr = std::shared_ptr<JointNode>;

struct LinkNode : TreeNode
{
    LinkNode();
    LinkNode(const Link& link, const JointNodePtr& parent_joint);

    void AddCollision();
    void DeleteCollision(int i);

    Link link;
    std::shared_ptr<JointNode> parent;
    std::vector<std::shared_ptr<JointNode>> children;

    ::Mesh visual_mesh;
    std::vector<::Mesh> collision_mesh;

    Model visual_model;
    std::vector<Model> collision_models;

    Matrix T;
};

struct JointNode : TreeNode
{
    JointNode(const Joint& joint, const LinkNodePtr& parent, const LinkNodePtr& child);

    Joint joint;
    std::shared_ptr<LinkNode> parent;
    std::shared_ptr<LinkNode> child;
};

class Robot {
public:
    Robot(const LinkNodePtr& root,
          const std::map<std::string, Material>& materials = {});
    ~Robot();

    void forward_kinematics();
    void forward_kinematics(LinkNodePtr& link);

    void build_geometry(void);

    void print_tree(void) const;

    void set_shader(const Shader& shader);

    void draw(const LinkNodePtr& highlighted = nullptr) const;

    LinkNodePtr get_root(void) const;

    LinkNodePtr get_link(const Ray& ray);

    const std::map<std::string, Material>& get_materials(void) const;

private:
    static Matrix origin_to_matrix(std::optional<Origin>& origin);

    LinkNodePtr root_;

    Shader visual_shader_;

    // std::map<std::string, std::vector<float>> q; // vector motivation: joints with multiple dof

    std::map<std::string, Material> materials_;
};

class Parser {
public:
    Parser();
    ~Parser();

    Robot build_robot(const char *urdf_file);

    void export_robot(const Robot& robot, std::string out_filename);

private:
    pugi::xml_node find_root(const pugi::xml_document& doc);

    // XML to data structures
    Link xml_node_to_link(const pugi::xml_node& xml_node);
    Joint xml_node_to_joint(const pugi::xml_node& xml_node);

    std::optional<Inertial> xml_node_to_inertial(const pugi::xml_node& xml_node);
    std::optional<Origin> xml_node_to_origin(const pugi::xml_node& xml_node);
    Geometry xml_node_to_geometry(const pugi::xml_node& xml_node);
    std::optional<Material> xml_node_to_material(const pugi::xml_node& xml_node);

    // Data structures to XML
    void link_to_xml_node(pugi::xml_node& xml_node, const Link& link);
    void joint_to_xml_node(pugi::xml_node& xml_node, const Joint& joint);

    void inertial_to_xml_node(pugi::xml_node& xml_node,  const Inertial& inertial);
    void origin_to_xml_node(pugi::xml_node& xml_node, const Origin& origin);
    void geometry_to_xml_node(pugi::xml_node& xml_node, const Geometry& geometry);
    void material_to_xml_node(pugi::xml_node& xml_node, const Material& material);
};

} // namespace urdf
