#include <raylib.h>
#include <raymath.h>
#include <external/par_shapes.h>

#include <cstring>
#include <functional>
#include <set>
#include <map>
#include <deque>

#include <robot.h>
#include <loguru.hpp>
#include <utility>

#include <import_mesh.h>

namespace urdf {
using std::string;

static inline Matrix MatMul(const Matrix& left, const Matrix& right)
{
    return MatrixMultiply(right, left);
}

static void storeFloat(const char *s, float& f) {
    std::stringstream iss(s);
    iss >> f;
    CHECK_F(iss && (iss >> std::ws).eof(), "storeFloat: invalid string");
}

static void storeVec3(const char *s, Vector3& vec) {
    std::stringstream iss(s);
    iss >> vec.x >> vec.y >> vec.z;
    CHECK_F(iss && (iss >> std::ws).eof(), "store_vec3: invalid string");
}

static void storeVec4(const char *s, Vector4& vec) {
    std::stringstream iss(s);
    iss >> vec.x >> vec.y >> vec.z >> vec.w;
    CHECK_F(iss && (iss >> std::ws).eof(), "store_vec4: invalid string");
}

Origin::Origin() : xyz{}, rpy{} { }

Origin::Origin(const char *xyz_s, const char *rpy_s)
{
    xyz.x = xyz.y = xyz.z = 0.0F;
    rpy.x = rpy.y = rpy.z = 0.0F;

    if (!xyz_s || std::strlen(xyz_s) == 0) {
        LOG_F(WARNING, "Origin xyz is empty");
    } else {
        storeVec3(xyz_s, xyz);
    }

    if (!rpy_s || std::strlen(rpy_s) == 0) {
        LOG_F(WARNING, "Origin rpy is empty");
    } else {
        storeVec3(rpy_s, rpy);
    }
}

Origin::Origin(const Vector3& xyz, const Vector3& rpy)
    : xyz(xyz), rpy(rpy)
{

}

Matrix Origin::toMatrix() const
{
    Matrix rot_mat {MatrixRotateXYZ(rpy)};
    Matrix pos_mat {MatrixTranslate(xyz.x, xyz.y, xyz.z)};
    return MatrixMultiply(rot_mat, pos_mat);
}

Box::Box()
{
    size.x = size.y = size.z = 1.0F;
}

Box::Box(const char *_size)
{
    storeVec3(_size, size);
}

Box::Box(const Vector3& size)
    : size(size)
{

}

Model Box::generateGeometry()
{
    ::Mesh mesh = GenMeshCube(size.x, size.y, size.z);
    return LoadModelFromMesh(mesh);
}

Cylinder::Cylinder()
{
    radius = length = 1.0F;
}

Cylinder::Cylinder(const char *_radius, const char *_length)
{
    radius = length = 0.0F;
    if (_radius) storeFloat(_radius, radius);
    if (_length) storeFloat(_length, length);
}

Model Cylinder::generateGeometry()
{
    ::Mesh mesh = GenMeshCenteredCylinder(radius, length, 32);
    return LoadModelFromMesh(mesh);
}

Sphere::Sphere()
{
    radius = 1.0F;
}

Sphere::Sphere(const char *_radius)
{
    storeFloat(_radius, radius);
}

Sphere::Sphere(float _radius)
    : radius(_radius)
{

}

Model Sphere::generateGeometry()
{
    ::Mesh mesh = GenMeshSphere(radius, 32, 32);
    return LoadModelFromMesh(mesh);
}

Mesh::Mesh(const char *filename)
    : filename(filename)
{
}

Model Mesh::generateGeometry()
{
    return LoadModelFromCollada(filename);
}

Inertia::Inertia(float ixx, float iyy, float izz, float ixy, float ixz, float iyz)
    : ixx(ixx), iyy(iyy), izz(izz), ixy(ixy), ixz(ixz), iyz(iyz)
{

}

Inertial::Inertial(const char *xyz, const char *rpy,
                   float mass,
                   float ixx, float iyy, float izz,
                   float ixy, float ixz, float iyz)
    : origin(xyz, rpy), mass(mass), inertia(ixx, iyy, izz, ixy, ixz, iyz)
{

}

Axis::Axis() : xyz{} { }

Axis::Axis(const Vector3& _xyz) : xyz(_xyz) { }

Axis::Axis(const char *s_xyz)
{
    storeVec3(s_xyz, xyz);
}

Dynamics::Dynamics() : damping(0.0F), friction(0.0F) { }

Dynamics::Dynamics(float _damping, float _friction) : damping(_damping), friction(_friction) { }

Limit::Limit(float _lower, float _upper, float _effort, float _velocity)
    : lower(_lower), upper(_upper), effort(_effort), velocity(_velocity)
{

}

Link::Link(string name) : name(std::move(name)) {}

Joint::Joint(const char *_name, const char *_parent, const char *_child, const char *_type)
    : name(_name), parent(_parent), child(_child)
{
    std::string stype(_type);
    if (stype == "revolute") {
        type = kRevolute;
    } else if (stype == "continuous") {
        type = kContinuous;
    } else if (stype == "prismatic") {
        type = kPrismatic;
    } else if (stype == "fixed") {
        type = kFixed;
    } else if (stype == "floating") {
        type = kFloating;
    } else if (stype == "planar") {
        type = kPlanar;
    } else {
        LOG_F(WARNING, "Unknown joint type: %s", _type);
        type = kFixed;
    }
}

LinkNode::LinkNode()
    : link("New link"), w_T_l(MatrixIdentity())
{

}

LinkNode::LinkNode(Link link, JointNodePtr parent_joint)
    : link(std::move(link)), parent(std::move(parent_joint)), w_T_l(MatrixIdentity())
{

}

void LinkNode::addCollision()
{
    link.collision.emplace_back();
    link.collision.back().geometry.type = std::make_shared<urdf::Box>();
    collision_models.push_back(link.collision.back().geometry.type->generateGeometry());
}

void LinkNode::deleteCollision(int i)
{
    link.collision.erase(link.collision.begin() + i);
    UnloadModel(collision_models[i]);
    collision_models.erase(collision_models.begin() + i);
}

JointNode::JointNode(Joint joint, LinkNodePtr parent, LinkNodePtr child)
    : joint(std::move(joint)), parent(std::move(parent)), child(std::move(child))
{

}

RobotPtr buildRobot(const char *urdf_file)
{
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(urdf_file);
    CHECK_F(result, "Failed to open %s", urdf_file);

    bool is_root_robot = doc.root().name() != std::string("robot");
    CHECK_F(is_root_robot, "The urdf root name (%s) is not robot", doc.root().name());
    LOG_F(INFO, "Building robot...");

    pugi::xml_node root_link = findRoot(doc);
    CHECK_F(not root_link.empty(), "No root link found");

    auto tree_root = std::make_shared<LinkNode>();
    tree_root->link = xmlNodeToLink(root_link);
    tree_root->w_T_l = MatrixIdentity();

    // Create all materials
    // TODO(ramon): support loading materials that are defined within the link
    std::map<std::string, Material> materials;
    for (pugi::xml_node& mat : doc.child("robot").children("material")) {
        if (const auto m = xmlNodeToMaterial(mat)) {
            materials.insert({m->name, *m});
        }
    }
    // TODO(ramon): every link uses the same shader, the material properties change the shader color.

    // Parent link to all child joints
    std::map<std::string, std::vector<Joint>> joint_p_hash;
    for (const pugi::xml_node& joint : doc.child("robot").children("joint")) {
        Joint j = xmlNodeToJoint(joint);
        if (joint_p_hash.find(j.parent) == joint_p_hash.end()) {
            joint_p_hash.insert({ j.parent, std::vector<Joint>({j}) });
        } else {
            joint_p_hash[j.parent].push_back(j);
        }
    }

    // Link name to link
    std::map<std::string, Link> link_hash;
    for (const pugi::xml_node& link : doc.child("robot").children("link")) {
        Link l = xmlNodeToLink(link);
        CHECK_F(link_hash.find(l.name) == link_hash.end());
        link_hash.insert({l.name, l});
    }

    std::deque<LinkNodePtr> deq{tree_root};
    while (not deq.empty()) {
        LinkNodePtr current_root = deq.front();
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

    RobotPtr robot = std::make_shared<Robot>(tree_root, materials);

    LOG_F(INFO, "URDF Tree built successfully!");

    return robot;
}

pugi::xml_node findRoot(const pugi::xml_document& doc)
{
    pugi::xml_node root_link;
    std::set<std::string> child_link_names;

    for (const pugi::xml_node joint : doc.child("robot").children("joint")) {
        child_link_names.insert(joint.child("child").attribute("link").value());
    }
    for (const pugi::xml_node link : doc.child("robot").children("link")) {
        if (child_link_names.find(link.attribute("name").value()) == child_link_names.end()) {
            root_link = link;
            break;
        }
    }

    return root_link;
}

Link xmlNodeToLink(const pugi::xml_node& xml_node)
{
    CHECK_F(static_cast<bool>(xml_node.attribute("name")), "Link has no name");
    Link link(xml_node.attribute("name").as_string());

    link.inertial = xmlNodeToInertial(xml_node.child("inertial"));

    const pugi::xml_node& vis_node = xml_node.child("visual");
    if (vis_node) {
        link.visual = Visual{};

        // --- Name
        std::string visual_name = vis_node.attribute("name").as_string();
        if (not visual_name.empty()) link.visual->name = visual_name;

        // --- Origin
        const auto& ori_node = vis_node.child("origin");
        link.visual->origin = xmlNodeToOrigin(ori_node);

        // --- Geometry
        const auto& geom_node = vis_node.child("geometry");
        link.visual->geometry = xmlNodeToGeometry(geom_node);

        // --- Material
        if (const auto& mat_node = vis_node.child("material")) {
            link.visual->material_name = mat_node.attribute("name").as_string();
            CHECK_F(not link.visual->material_name->empty(), "Materials need to have a name!");
        }
    }

    for (const auto& col_node : xml_node.children("collision")) {
        Collision col { };

        // --- Name
        std::string collision_name = col_node.attribute("name").as_string();
        if (not collision_name.empty()) col.name = collision_name;

        // --- Origin
        const auto& ori_node = col_node.child("origin");
        col.origin = xmlNodeToOrigin(ori_node);

        // --- Geometry
        const auto& geom_node = col_node.child("geometry");
        col.geometry = xmlNodeToGeometry(geom_node);

        link.collision.push_back(col);
    }

    return link;
}

Joint xmlNodeToJoint(const pugi::xml_node& xml_node)
{
    Joint joint(
        xml_node.attribute("name").as_string(),
        xml_node.child("parent").attribute("link").as_string(),
        xml_node.child("child").attribute("link").as_string(),
        xml_node.attribute("type").as_string()
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

std::optional<Origin> xmlNodeToOrigin(const pugi::xml_node& xml_node)
{
    if (xml_node) {
        return Origin(
            xml_node.attribute("xyz").as_string(),
            xml_node.attribute("rpy").as_string()
        );
    } else {
        return std::nullopt;
    }
}

std::optional<Inertial> xmlNodeToInertial(const pugi::xml_node& xml_node)
{
    if (xml_node) {
        return Inertial(
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
    } else {
        return std::nullopt;
    }
}

Geometry xmlNodeToGeometry(const pugi::xml_node& geom_node)
{
    CHECK_F(static_cast<bool>(geom_node), "Missing geometry tag!");

    Geometry geom;
    if (geom_node.child("box")) {
        geom.type = std::make_shared<Box>(geom_node.child("box").attribute("size").as_string());
    } else if (geom_node.child("cylinder")) {
        geom.type = std::make_shared<Cylinder>(
            geom_node.child("cylinder").attribute("radius").as_string(),
            geom_node.child("cylinder").attribute("length").as_string()
        );
    } else if (geom_node.child("sphere")) {
        geom.type = std::make_shared<Sphere>(geom_node.child("sphere").attribute("radius").as_string());
    } else if (geom_node.child("mesh")) {
        LOG_F(INFO, "Mesh visuals are not implemented yet!"); // TODO(ramon)
        geom.type = std::make_shared<Box>();
    }

    return geom;
}

std::optional<Material> xmlNodeToMaterial(const pugi::xml_node& mat_node)
{
    if (mat_node) {
        Material mat;

        mat.name = mat_node.attribute("name").as_string();
        CHECK_F(not mat.name.empty(), "Materials need to have a name!");

        if (mat_node.child("color")) {
            CHECK_F(static_cast<bool>(mat_node.child("color").attribute("rgba")),
                    "The material color tag needs to have an rgba value");
            mat.rgba = Vector4{};
            storeVec4(mat_node.child("color").attribute("rgba").as_string(), mat.rgba.value());
        }

        if (mat_node.child("texture")) {
            CHECK_F(static_cast<bool>(mat_node.child("texture").attribute("filename")),
                    "The texture tag needs to have a filename attribute!");
            mat.texture_file = mat_node.child("texture").attribute("filename").as_string();
        }

        return mat;
    } else {
        return std::nullopt;
    }
}

void exportRobot(const Robot& robot, std::string out_filename)
{
    pugi::xml_document out_doc;
    const LinkNodePtr root_node = robot.getRoot();

    pugi::xml_node robot_root = out_doc.append_child("robot");

    // save links
    std::deque<LinkNodePtr> deq {root_node};
    while (not deq.empty()) {
        const auto& current_link = deq.front();
        deq.pop_front();

        pugi::xml_node link_node = robot_root.append_child("link");
        linkToXmlNode(link_node, current_link->link);

        for (const auto& joint : current_link->children) {
            deq.push_back(joint->child);
        }
    }

    // save joints
    deq = std::deque<LinkNodePtr> {root_node};
    while (not deq.empty()) {
        const auto& current_link = deq.front();
        deq.pop_front();

        for (const auto& joint : current_link->children) {
            pugi::xml_node joint_node = robot_root.append_child("joint");
            jointToXmlNode(joint_node, joint->joint);

            deq.push_back(joint->child);
        }
    }

    // save materials
    for (const auto& mat : robot.getMaterials()) {
        pugi::xml_node mat_node = robot_root.append_child("material");
        materialToXmlNode(mat_node, mat.second);
    }

    if (!out_doc.save_file(out_filename.c_str())) {
        LOG_F(ERROR, "File %s could not be opened or written!", out_filename.c_str());
    }
}

void linkToXmlNode(pugi::xml_node& xml_node, const Link& link)
{
    xml_node.set_name("link");
    xml_node.append_attribute("name") = link.name.c_str();

    if (link.inertial) {
        pugi::xml_node inertial_node = xml_node.append_child("inertial");
        inertialToXmlNode(inertial_node, *link.inertial);
    }

    if (link.visual) {
        pugi::xml_node visual_node = xml_node.append_child("visual");
        if (link.visual->name) {
            visual_node.append_attribute("name") = link.visual->name->c_str();
        }
        if (link.visual->origin) {
            pugi::xml_node origin_node = visual_node.append_child("origin");
            originToXmlNode(origin_node, *link.visual->origin);
        }
        pugi::xml_node geometry_node = visual_node.append_child("geometry");
        geometryToXmlNode(geometry_node, link.visual->geometry);
        if (link.visual->material_name) {
            visual_node.append_child("material").append_attribute("name")
                = link.visual->material_name->c_str();
        }
    }

    for (const Collision& col : link.collision) {
        pugi::xml_node collision_node = xml_node.append_child("collision");

        if (col.name) {
            collision_node.append_attribute("name") = col.name->c_str();
        }

        if (col.origin) {
            pugi::xml_node origin_node = collision_node.append_child("origin");
            originToXmlNode(origin_node, *col.origin);
        }

        pugi::xml_node geometry_node = collision_node.append_child("geometry");
        geometryToXmlNode(geometry_node, col.geometry);
    }

    if (not xml_node) LOG_F(ERROR, "Link node is empty!");
}

void jointToXmlNode(pugi::xml_node& joint_node, const Joint& joint)
{
    joint_node.set_name("joint");
    switch (joint.type) {
        case Joint::kRevolute:
            joint_node.append_attribute("type") = "revolute";
            break;
        case Joint::kContinuous:
            joint_node.append_attribute("type") = "continuous";
            break;
        case Joint::kPrismatic:
            joint_node.append_attribute("type") = "prismatic";
            break;
        case Joint::kFixed:
            joint_node.append_attribute("type") = "fixed";
            break;
        case Joint::kFloating:
            joint_node.append_attribute("type") = "floating";
            break;
        case Joint::kPlanar:
            joint_node.append_attribute("type") = "planar";
            break;
        default:
            LOG_F(ERROR, "Unknown joint type.");
    }

    pugi::xml_node parent_node = joint_node.append_child("parent");
    parent_node.append_attribute("link") = joint.parent.c_str();
    pugi::xml_node child_node = joint_node.append_child("child");
    child_node.append_attribute("link") = joint.child.c_str();

    if (joint.origin) {
        pugi::xml_node origin_node = joint_node.append_child("origin");
        originToXmlNode(origin_node, *joint.origin);
    }

    if (joint.axis) {
        pugi::xml_node axis_node = joint_node.append_child("axis");
        axis_node.append_attribute("xyz") = (std::to_string(joint.axis->xyz.x) + " " +
                                             std::to_string(joint.axis->xyz.y) + " " +
                                             std::to_string(joint.axis->xyz.z)).c_str();
    }

    if (joint.dynamics) {
        pugi::xml_node dynamics_node = joint_node.append_child("dynamics");
        dynamics_node.append_attribute("damping") = joint.dynamics->damping;
        dynamics_node.append_attribute("friction") = joint.dynamics->friction;
    }

    if (joint.limit) {
        pugi::xml_node limit_node = joint_node.append_child("limit");
        limit_node.append_attribute("lower") = joint.limit->lower;
        limit_node.append_attribute("upper") = joint.limit->upper;
        limit_node.append_attribute("effort") = joint.limit->effort;
        limit_node.append_attribute("velocity") = joint.limit->velocity;
    }
}

void inertialToXmlNode(pugi::xml_node& xml_node, const Inertial& inertial)
{
    xml_node.set_name("inertial");

    pugi::xml_node mass_node = xml_node.append_child("mass");
    mass_node.append_attribute("value") = inertial.mass;

    pugi::xml_node inertia_node = xml_node.append_child("inertia");
    inertia_node.append_attribute("ixx") = inertial.inertia.ixx;
    inertia_node.append_attribute("iyy") = inertial.inertia.iyy;
    inertia_node.append_attribute("izz") = inertial.inertia.izz;
    inertia_node.append_attribute("ixy") = inertial.inertia.ixy;
    inertia_node.append_attribute("ixz") = inertial.inertia.ixz;
    inertia_node.append_attribute("iyz") = inertial.inertia.iyz;

    pugi::xml_node origin_node = xml_node.append_child("origin");
    originToXmlNode(origin_node, inertial.origin);
}

void originToXmlNode(pugi::xml_node& xml_node, const Origin& origin)
{
    xml_node.set_name("origin");

    xml_node.append_attribute("xyz") = (std::to_string(origin.xyz.x) + " " +
        std::to_string(origin.xyz.y) + " " +
        std::to_string(origin.xyz.z)).c_str();
    xml_node.append_attribute("rpy") = (std::to_string(origin.rpy.x) + " " +
        std::to_string(origin.rpy.y) + " " +
        std::to_string(origin.rpy.z)).c_str();
}

void geometryToXmlNode(pugi::xml_node& xml_node, const Geometry& geometry)
{
    xml_node.set_name("geometry");

    if (std::dynamic_pointer_cast<Box>(geometry.type)) {
        const Box& box = *std::dynamic_pointer_cast<Box>(geometry.type);
        pugi::xml_node box_node = xml_node.append_child("box");
        box_node.append_attribute("size") = (std::to_string(box.size.x) + " " +
                                             std::to_string(box.size.y) + " " +
                                             std::to_string(box.size.z)).c_str();
    } else if (std::dynamic_pointer_cast<Cylinder>(geometry.type)) {
        const Cylinder& cylinder = *std::dynamic_pointer_cast<Cylinder>(geometry.type);
        pugi::xml_node cylinder_node = xml_node.append_child("cylinder");
        cylinder_node.append_attribute("radius") = std::to_string(cylinder.radius).c_str();
        cylinder_node.append_attribute("length") = std::to_string(cylinder.length).c_str();
    } else if (std::dynamic_pointer_cast<Sphere>(geometry.type)) {
        const Sphere& sphere = *std::dynamic_pointer_cast<Sphere>(geometry.type);
        pugi::xml_node sphere_node = xml_node.append_child("sphere");
        sphere_node.append_attribute("radius") = std::to_string(sphere.radius).c_str();
    } else {
        LOG_F(WARNING, "Geometry type not implemented yet");
    }
}

void materialToXmlNode(pugi::xml_node& xml_node, const Material& material)
{
    xml_node.set_name("material");
    xml_node.append_attribute("name") = material.name.c_str();
    if (material.rgba) {
        pugi::xml_node color_node = xml_node.append_child("color");
        color_node.append_attribute("rgba") = (std::to_string(material.rgba->x) + " " +
                                               std::to_string(material.rgba->y) + " " +
                                               std::to_string(material.rgba->z) + " " +
                                               std::to_string(material.rgba->w)).c_str();
    }
    if (material.texture_file) {
        pugi::xml_node texture_node = xml_node.append_child("texture");
        texture_node.append_attribute("filename") = material.texture_file->c_str();
    }
}

Robot::Robot(LinkNodePtr root,
      const std::map<std::string, Material>& materials)
    : root_(std::move(root)), materials_(materials)
{

}

Robot::~Robot()
{
    auto func = [&](const LinkNodePtr& link){
        UnloadModel(link->visual_model);
        for (const Model& mod : link->collision_models) {
            UnloadModel(mod);
        }
    };

    forEveryLink(func);
}

void Robot::forwardKinematics()
{
    forwardKinematics(root_);
}

void Robot::forwardKinematics(LinkNodePtr& link)
{
    std::function<void (LinkNodePtr&)> recursion = [&](LinkNodePtr& node){
        const Matrix& w_t_p = node->w_T_l;

        for (auto& joint_node : node->children) {
            // Forward kinematics
            const Matrix p_t_j = originToMatrix(joint_node->joint.origin);
            const Matrix j_t_c = MatrixIdentity();  // TODO(ramon): joint -> child transform
            const Matrix w_t_c = MatMul(w_t_p, MatMul(p_t_j, j_t_c));
            joint_node->child->w_T_l = w_t_c;

            // Update visual transform
            const Matrix c_t_v = originToMatrix(joint_node->child->link.visual->origin);
            const Matrix w_t_v = MatMul(w_t_c, c_t_v);
            joint_node->child->visual_model.transform = w_t_v;

            // Update collision transform
            for (size_t i = 0; i < joint_node->child->collision_models.size(); ++i) {
                const Matrix c_t_col = originToMatrix(joint_node->child->link.collision[i].origin);
                const Matrix w_t_col = MatMul(w_t_c, c_t_col);
                joint_node->child->collision_models[i].transform = w_t_col;
            }

            recursion(joint_node->child);
        }
    };

    recursion(link);
}

Matrix Robot::originToMatrix(std::optional<Origin>& origin)
{
    Matrix t = MatrixIdentity();

    if (origin) {
        t = origin->toMatrix();
    }

    return t;
}

void Robot::buildGeometry()
{
    auto func = [&](const LinkNodePtr& link){
        // Visual model
        if (link->link.visual) {
            link->visual_model = link->link.visual->geometry.type->generateGeometry();
            link->visual_model.materials[0].shader = visual_shader_;
            updateMaterial(link);
        }

        // Collision model
        for (const Collision& col : link->link.collision) {
            // link->collision_mesh.push_back(col.geometry.type->generateGeometry());
            link->collision_models.push_back(col.geometry.type->generateGeometry());
        }
    };

    forEveryLink(func);
}

void Robot::updateMaterial(const LinkNodePtr& link)
{
    auto set_material_diffuse_color = [](const Model& model, const Vector4& color){
        model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color.r = static_cast<char>(color.x * 255.0F);
        model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color.g = static_cast<char>(color.y * 255.0F);
        model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color.b = static_cast<char>(color.z * 255.0F);
        model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color.a = static_cast<char>(color.w * 255.0F);
    };

    if (link->link.visual->material_name) {
        const Material& mat = materials_[*link->link.visual->material_name];
        if (mat.rgba) {
            set_material_diffuse_color(link->visual_model, *mat.rgba);
        } else {
            set_material_diffuse_color(link->visual_model, Vector4{127, 127, 127, 255});
            LOG_F(INFO, "Link: %s. RGBA not found, using default color. Texture is not implemented yet.", link->link.name.c_str());
        }
    } else {
        set_material_diffuse_color(link->visual_model, Vector4{127, 127, 127, 255});
        LOG_F(INFO, "Link: %s. No material name specified. Using default color", link->link.name.c_str());
    }
}

void Robot::draw(const LinkNodePtr& highlighted, const LinkNodePtr& selected) const
{
    auto draw_link = [&](const LinkNodePtr& link){
        if (link->link.visual) {
            if (highlighted.get() == link.get()) {
                DrawModel(link->visual_model, Vector3Zero(), 1.0, RED);
            } else {
                DrawModel(link->visual_model, Vector3Zero(), 1.0, WHITE);
            }
        }
        for (const Model& model : link->collision_models) {
            if (selected.get() == link.get()) {
                DrawModelWires(model, Vector3Zero(), 1.0, GREEN);
            } else {
                DrawModelWires(model, Vector3Zero(), 1.0, GRAY);
            }
        }
    };

    forEveryLink(draw_link);
}

void Robot::setShader(const Shader& sh)
{
    visual_shader_ = sh;

    auto func = [&](const LinkNodePtr& link){
        if (link->visual_model.materialCount > 0) {
            if (::Material *mat = link->visual_model.materials) {
                mat[0].shader = visual_shader_;
            }
        } else {
            LOG_F(WARNING, "Link %s visual model does not have materials", link->link.name.c_str());
            LOG_F(WARNING, "Link %s has %d meshes", link->link.name.c_str(), link->visual_model.meshCount);
        }
    };

    forEveryLink(func);
}

void Robot::printTree() const
{
    auto func = [](const LinkNodePtr& link){ LOG_F(INFO, "%s", link->link.name.c_str()); };
    forEveryLink(func);
}

LinkNodePtr Robot::getRoot() const
{
    return root_;
}

const std::map<std::string, Material>& Robot::getMaterials() const
{
    return materials_;
}

void Robot::forEveryLink(const std::function<void (const LinkNodePtr&)>& func) const
{
    std::deque<LinkNodePtr> deq {root_};

    while (not deq.empty()) {
        const auto& current_link = deq.front();
        deq.pop_front();

        func(current_link);

        for (const auto& joint : current_link->children) {
            deq.push_back(joint->child);
        }
    }
}

void Robot::forEveryJoint(const std::function<void(const JointNodePtr&)>& func) const
{
    std::deque<LinkNodePtr> deq {root_};

    while (not deq.empty()) {
        const auto& current_link = deq.front();
        deq.pop_front();

        for (const auto& joint : current_link->children) {
            func(joint);
            deq.push_back(joint->child);
        }
    }
}

} // namespace urdf
