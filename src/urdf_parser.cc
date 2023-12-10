#include <sstream>

#include <urdf_parser.h>
#include <loguru.hpp>

namespace urdf {

static void store_float(const char *s, float& f) {
    std::stringstream iss(s);
    iss >> f;
    CHECK_F(not (iss && (iss >> std::ws).eof()));
}

static void store_vec(const char *s, Vector3& vec) {
    std::stringstream iss(s);
    iss >> vec.x >> vec.y >> vec.z;
    CHECK_F(not (iss && (iss >> std::ws).eof()));
}

Origin::Origin(const char *xyz_s, const char *rpy_s)
{
    store_vec(xyz_s, xyz_);
    store_vec(rpy_s, rpy_);
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
    store_float(radius, radius_);
    store_float(length, length_);
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

    for (pugi::xml_node tool : doc_.child("robot").children("joint")) {
        LOG_F(INFO, "%s", tool.attribute("name").as_string());
    }
}

Parser::~Parser()
{

}

}  // namespace urdf

