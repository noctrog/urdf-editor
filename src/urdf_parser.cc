#include <sstream>
#include <set>
#include <map>
#include <deque>

#include <urdf_parser.h>
#include <loguru.hpp>

namespace urdf {

static void store_float(const char *s, float& f) {
    std::stringstream iss(s);
    iss >> f;
    CHECK_F(iss && (iss >> std::ws).eof(), "store_float: invalid string");
}

static void store_vec(const char *s, Vector3& vec) {
    std::stringstream iss(s);
    iss >> vec.x >> vec.y >> vec.z;
    CHECK_F(iss && (iss >> std::ws).eof(), "store_vec: invalid string");
}

Origin::Origin(const char *xyz_s, const char *rpy_s)
{
    xyz_.x = xyz_.y = xyz_.z = 0.0f;
    rpy_.x = rpy_.y = rpy_.z = 0.0f;

    if (xyz_s) store_vec(xyz_s, xyz_);
    if (rpy_s) store_vec(rpy_s, rpy_);

}

Origin::Origin(const Vector3& xyz, const Vector3& rpy)
    : xyz_(xyz), rpy_(rpy)
{

}

Box::Box(const char *size)
{
    store_vec(size, size_);
}

Box::Box(const Vector3& size)
    : size_(size)
{

}

Cylinder::Cylinder(const char *radius, const char *length)
{
    radius_ = length_ = 0.0f;
    if (radius) store_float(radius, radius_);
    if (length) store_float(length, length_);
}

Sphere::Sphere(const char *radius)
{
    store_float(radius, radius_);
}

Sphere::Sphere(float radius)
    : radius_(radius)
{

}

Geometry::Geometry(const GeometryType& geom)
    :geom_(geom)
{

}

Material::Material(const char *name)
    : name_(name)
{
    rgba_.x = rgba_.y = rgba_.z = 0.3f;
    rgba_.w = 1.0f;
}

Material::Material(const char *name, const Vector4& rgba)
    : name_(name), rgba_(rgba)
{

}

Material::Material(const char *name, const char *texture_file)
    : name_(name), texture_file_(texture_file)
{
    rgba_.x = rgba_.y = rgba_.z = 0.3f;
    rgba_.w = 1.0f;
}

Inertia::Inertia(float ixx, float iyy, float izz, float ixy, float ixz, float iyz)
    : ixx_(ixx), iyy_(iyy), izz_(izz), ixy_(ixy), ixz_(ixz), iyz_(iyz)
{

}

Inertial::Inertial(const char *xyz, const char *rpy,
                   float mass,
                   float ixx, float iyy, float izz,
                   float ixy, float ixz, float iyz)
    : origin_(xyz, rpy), mass_(mass), inertia_(ixx, iyy, izz, ixy, ixz, iyz)
{

}

Axis::Axis(const Vector3& _xyz) : xyz(_xyz) { }

Axis::Axis(const char *s)
{
    store_vec(s, xyz);
}

Dynamics::Dynamics(float _damping, float _friction) : damping(_damping), friction(_friction) { }

Limit::Limit(float _lower, float _upper, float _effort, float _velocity)
    : lower(_lower), upper(_upper), effort(_effort), velocity(_velocity)
{

}

Link::Link()
{

}

Joint::Joint(const char *_name, const char *_parent, const char *_child, const char *_type)
    : name(_name), parent(_parent), child(_child)
{
    std::string stype(_type);
    if (stype == "revolute") {
        type = REVOLUTE;
    } else if (stype == "continuous") {
        type = CONTINUOUS;
    } else if (stype == "prismatic") {
        type = PRISMATIC;
    } else if (stype == "fixed") {
        type = FIXED;
    } else if (stype == "floating") {
        type = FLOATING;
    } else if (stype == "planar") {
        type = PLANAR;
    }
}

Parser::Parser(const char *urdf_file)
{
    pugi::xml_parse_result result = doc_.load_file(urdf_file);
    CHECK_F(result, "Failed to open %s", urdf_file);

    bool is_root_robot = doc_.root().name() != std::string("robot");
    CHECK_F(is_root_robot, "The urdf root name (%s) is not robot", doc_.root().name());

    pugi::xml_node root_link = find_root();
    CHECK_F(not root_link.empty(), "No root link found");

    LinkNode tree_root;
    tree_root.link = xml_node_to_link(root_link);

    // Parent link to all child joints
    std::map<std::string, std::vector<Joint>> joint_p_hash;
    for (pugi::xml_node joint : doc_.child("robot").children("joint")) {
        Joint j = xml_node_to_joint(joint);
        if (joint_p_hash.find(j.parent) == joint_p_hash.end()) {
            joint_p_hash.insert({ j.parent, std::vector<Joint>({j}) });
        } else {
            joint_p_hash[j.parent].push_back(j);
        }
    }

    // Link name to link
    std::map<std::string, Link> link_hash;
    for (pugi::xml_node link : doc_.child("robot").children("link")) {
        Link l = xml_node_to_link(link);
        CHECK_F(link_hash.find(l.name) == link_hash.end());
        link_hash.insert({l.name, l});
    }

    tree_root_ = std::make_shared<LinkNode>(tree_root);
    std::deque<std::shared_ptr<LinkNode>> deq{tree_root_};
    while (not deq.empty()) {
        std::shared_ptr<LinkNode> current_root = deq.front();
        deq.pop_front();

        const auto it = joint_p_hash.find(current_root->link.name);
        if (it == joint_p_hash.end()) {
            continue;
        }

        for (const auto& joint : it->second) {
            auto joint_node = std::make_shared<JointNode>(JointNode {joint, current_root, nullptr});
            CHECK_F(link_hash.find(joint.child) != link_hash.end());

            auto child_node = std::make_shared<LinkNode>(LinkNode {link_hash[joint.child], joint_node});
            joint_node->child = child_node;
            current_root->children.push_back(joint_node);
            deq.push_back(child_node);
        }
    }

    LOG_F(INFO, "URDF Tree built successfully!");

    print_tree();
}

Parser::~Parser()
{

}

void Parser::print_tree(void)
{
    std::deque<std::shared_ptr<LinkNode>> deq {tree_root_};

    while (not deq.empty()) {
        const auto current_link = deq.front();
        deq.pop_front();

        LOG_F(INFO, "%s", current_link->link.name.c_str());

        for (const auto& joint : current_link->children) {
            deq.push_back(joint->child);
        }
    }
}

pugi::xml_node Parser::find_root()
{
    pugi::xml_node root_link;
    std::set<std::string> child_link_names;

    for (pugi::xml_node joint : doc_.child("robot").children("joint")) {
        child_link_names.insert(joint.child("child").attribute("link").value());
    }
    for (pugi::xml_node link : doc_.child("robot").children("link")) {
        if (child_link_names.find(link.attribute("name").value()) == child_link_names.end()) {
            root_link = link;
            break;
        }
    }

    return root_link;
}

Link Parser::xml_node_to_link(const pugi::xml_node& xml_node)
{
    Link link;

    CHECK_F(static_cast<bool>(xml_node.attribute("name")), "Link has no name");
    link.name = xml_node.attribute("name").as_string();

    if (xml_node.child("inertial")) {
        link.inertial = Inertial(
            xml_node.child("inertial").child("origin").attribute("xyz").as_string(),
            xml_node.child("inertial").child("origin").attribute("rpy").as_string(),
            xml_node.child("inertial").child("mass").attribute("value").as_float(),
            xml_node.child("inertial").child("inertia").attribute("ixx").as_float(),
            xml_node.child("inertial").child("inertia").attribute("iyy").as_float(),
            xml_node.child("inertial").child("inertia").attribute("izz").as_float(),
            xml_node.child("inertial").child("inertia").attribute("ixy").as_float(),
            xml_node.child("inertial").child("inertia").attribute("ixz").as_float(),
            xml_node.child("inertial").child("inertia").attribute("iyz").as_float()
        );
    }
    if (xml_node.child("visual")) {

    }
    if (xml_node.child("collision")) {

    }
    return link;
}

Joint Parser::xml_node_to_joint(const pugi::xml_node& xml_node)
{
    Joint joint(
        xml_node.attribute("name").as_string(),
        xml_node.child("parent").attribute("link").as_string(),
        xml_node.child("child").attribute("link").as_string(),
        xml_node.child("type").attribute("name").as_string()
    );
    if (xml_node.child("origin")) {
        joint.origin = Origin(
            xml_node.child("origin").attribute("xyz").as_string(),
            xml_node.child("origin").attribute("rpy").as_string()
        );
    }
    if (xml_node.child("axis")) {
        joint.axis = Axis(
            xml_node.child("axis").attribute("xyz").as_string()
        );
    }
    if (xml_node.child("dynamics")) {
        joint.dynamics = Dynamics(
            xml_node.child("dynamics").attribute("damping").as_float(),
            xml_node.child("dynamics").attribute("friction").as_float()
        );
    }
    if (xml_node.child("limit")) {
        joint.limit = Limit(
            xml_node.child("limit").attribute("lower").as_float(),
            xml_node.child("limit").attribute("upper").as_float(),
            xml_node.child("limit").attribute("effort").as_float(),
            xml_node.child("limit").attribute("velocity").as_float()
        );
    }
    return joint;
}

}  // namespace urdf

