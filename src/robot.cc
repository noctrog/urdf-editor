#include <external/par_shapes.h>
#include <fmt/format.h>
#include <import_mesh.h>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <robot.h>

#include <cfloat>
#include <cmath>
#include <cstring>
#include <deque>
#include <filesystem>
#include <functional>
#include <loguru.hpp>
#include <map>
#include <set>
#include <utility>

namespace urdf {
using std::string;

constexpr float kDefaultGeometrySize = 1.0F;
constexpr int kMeshSubdivisions = 32;

static inline Matrix MatMul(const Matrix& left, const Matrix& right) {
    return MatrixMultiply(right, left);
}

static void storeFloat(const char* s, float& f) {
    std::stringstream iss(s);
    iss >> f;
    CHECK_F(iss && (iss >> std::ws).eof(), "storeFloat: invalid string");
}

static void storeVec3(const char* s, Vector3& vec) {
    std::stringstream iss(s);
    iss >> vec.x >> vec.y >> vec.z;
    CHECK_F(iss && (iss >> std::ws).eof(), "store_vec3: invalid string");
}

static void storeVec4(const char* s, Vector4& vec) {
    std::stringstream iss(s);
    iss >> vec.x >> vec.y >> vec.z >> vec.w;
    CHECK_F(iss && (iss >> std::ws).eof(), "store_vec4: invalid string");
}

std::string resolveFilePath(const std::string& mesh_filename, const std::string& urdf_dir) {
    namespace fs = std::filesystem;

    const std::string package_prefix = "package://";
    if (mesh_filename.rfind(package_prefix, 0) == 0) {
        std::string remainder = mesh_filename.substr(package_prefix.size());
        auto slash_pos = remainder.find('/');
        std::string relative_path = (slash_pos != std::string::npos) ? remainder.substr(slash_pos + 1) : "";

        // Walk up from urdf_dir looking for package.xml
        fs::path dir = urdf_dir;
        while (!dir.empty()) {
            if (fs::exists(dir / "package.xml")) {
                return (dir / relative_path).string();
            }
            fs::path parent = dir.parent_path();
            if (parent == dir) break;
            dir = parent;
        }

        LOG_F(WARNING, "Could not find package.xml for '%s', treating as relative to URDF dir",
              mesh_filename.c_str());
        return (fs::path(urdf_dir) / relative_path).string();
    }

    if (fs::path(mesh_filename).is_absolute()) {
        return mesh_filename;
    }

    return (fs::path(urdf_dir) / mesh_filename).string();
}

Origin::Origin() : xyz{}, rpy{} {}

Origin::Origin(const char* xyz_s, const char* rpy_s) {
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

Origin::Origin(const Vector3& xyz, const Vector3& rpy) : xyz(xyz), rpy(rpy) {}

Matrix Origin::toMatrix() const {
    Matrix rot_mat{MatrixRotateXYZ(rpy)};
    Matrix pos_mat{MatrixTranslate(xyz.x, xyz.y, xyz.z)};
    return MatrixMultiply(rot_mat, pos_mat);
}

Box::Box() { size.x = size.y = size.z = kDefaultGeometrySize; }

Box::Box(const char* _size) { storeVec3(_size, size); }

Box::Box(const Vector3& size) : size(size) {}

Model Box::generateGeometry() {
    ::Mesh mesh = GenMeshCube(size.x, size.y, size.z);
    return LoadModelFromMesh(mesh);
}

Cylinder::Cylinder() { radius = length = kDefaultGeometrySize; }

Cylinder::Cylinder(const char* _radius, const char* _length) {
    radius = length = 0.0F;
    if (_radius) storeFloat(_radius, radius);
    if (_length) storeFloat(_length, length);
}

Model Cylinder::generateGeometry() {
    ::Mesh mesh = GenMeshCenteredCylinder(radius, length, kMeshSubdivisions);
    return LoadModelFromMesh(mesh);
}

Sphere::Sphere() { radius = kDefaultGeometrySize; }

Sphere::Sphere(const char* _radius) { storeFloat(_radius, radius); }

Sphere::Sphere(float _radius) : radius(_radius) {}

Model Sphere::generateGeometry() {
    ::Mesh mesh = GenMeshSphere(radius, kMeshSubdivisions, kMeshSubdivisions);
    return LoadModelFromMesh(mesh);
}

Mesh::Mesh(const char* filename, const std::string& resolved_path, const Vector3& scale)
    : filename(filename),
      resolved_path(resolved_path.empty() ? filename : resolved_path),
      scale(scale) {}

Model Mesh::generateGeometry() {
    Model model = LoadModelFromFile(resolved_path);
    model.transform = MatrixMultiply(MatrixScale(scale.x, scale.y, scale.z), model.transform);
    return model;
}

Inertia::Inertia(float ixx, float iyy, float izz, float ixy, float ixz, float iyz)
    : ixx(ixx), iyy(iyy), izz(izz), ixy(ixy), ixz(ixz), iyz(iyz) {}

Inertial::Inertial(const char* xyz, const char* rpy, float mass, float ixx, float iyy, float izz,
                   float ixy, float ixz, float iyz)
    : origin(xyz, rpy), mass(mass), inertia(ixx, iyy, izz, ixy, ixz, iyz) {}

Axis::Axis() : xyz{} {}

Axis::Axis(const Vector3& _xyz) : xyz(_xyz) {}

Axis::Axis(const char* s_xyz) { storeVec3(s_xyz, xyz); }

Dynamics::Dynamics() : damping(0.0F), friction(0.0F) {}

Dynamics::Dynamics(float _damping, float _friction) : damping(_damping), friction(_friction) {}

Limit::Limit(float _lower, float _upper, float _effort, float _velocity)
    : lower(_lower), upper(_upper), effort(_effort), velocity(_velocity) {}

Link::Link(string name) : name(std::move(name)) {}

Joint::Joint(const char* _name, const char* _parent, const char* _child, const char* _type)
    : name(_name), parent(_parent), child(_child) {
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

LinkNode::LinkNode() : link("New link"), w_T_l(MatrixIdentity()) {}

LinkNode::LinkNode(Link link, JointNodePtr parent_joint)
    : link(std::move(link)), parent(std::move(parent_joint)), w_T_l(MatrixIdentity()) {}

void LinkNode::addVisual() {
    link.visual.emplace_back();
    link.visual.back().geometry.type = std::make_shared<urdf::Box>();
    visual_models.push_back(link.visual.back().geometry.type->generateGeometry());
}

void LinkNode::deleteVisual(int i) {
    link.visual.erase(link.visual.begin() + i);
    UnloadModel(visual_models[i]);
    visual_models.erase(visual_models.begin() + i);
}

void LinkNode::addCollision() {
    link.collision.emplace_back();
    link.collision.back().geometry.type = std::make_shared<urdf::Box>();
    collision_models.push_back(link.collision.back().geometry.type->generateGeometry());
}

void LinkNode::deleteCollision(int i) {
    link.collision.erase(link.collision.begin() + i);
    UnloadModel(collision_models[i]);
    collision_models.erase(collision_models.begin() + i);
}

JointNode::JointNode(Joint joint, LinkNodePtr parent, LinkNodePtr child)
    : joint(std::move(joint)), parent(std::move(parent)), child(std::move(child)) {}

GeometryTypePtr cloneGeometry(const GeometryTypePtr& src) {
    if (auto box = std::dynamic_pointer_cast<Box>(src)) {
        return std::make_shared<Box>(box->size);
    }
    if (auto cyl = std::dynamic_pointer_cast<Cylinder>(src)) {
        auto c = std::make_shared<Cylinder>();
        c->radius = cyl->radius;
        c->length = cyl->length;
        return c;
    }
    if (auto sph = std::dynamic_pointer_cast<Sphere>(src)) {
        return std::make_shared<Sphere>(sph->radius);
    }
    if (auto mesh = std::dynamic_pointer_cast<Mesh>(src)) {
        return std::make_shared<Mesh>(mesh->filename.c_str(), mesh->resolved_path, mesh->scale);
    }
    return nullptr;
}

static std::string deduplicateName(const std::string& base, const std::set<std::string>& existing) {
    if (existing.count(base) == 0) return base;
    std::string candidate = base + "_copy";
    if (existing.count(candidate) == 0) return candidate;
    for (int i = 2;; ++i) {
        candidate = base + "_copy" + std::to_string(i);
        if (existing.count(candidate) == 0) return candidate;
    }
}

JointNodePtr cloneSubtree(const LinkNodePtr& source, const LinkNodePtr& attach_parent,
                          const RobotPtr& robot) {
    // Collect all existing names
    std::set<std::string> link_names;
    std::set<std::string> joint_names;
    robot->forEveryLink([&](const LinkNodePtr& l) { link_names.insert(l->link.name); });
    robot->forEveryJoint([&](const JointNodePtr& j) { joint_names.insert(j->joint.name); });

    // Recursive clone
    std::function<JointNodePtr(const JointNodePtr&, const LinkNodePtr&)> recurse =
        [&](const JointNodePtr& src_joint, const LinkNodePtr& new_parent) -> JointNodePtr {
        const LinkNodePtr& src_link = src_joint->child;

        // Clone link data
        Link new_link;
        std::string link_name = deduplicateName(src_link->link.name, link_names);
        link_names.insert(link_name);
        new_link.name = link_name;
        new_link.inertial = src_link->link.inertial;

        for (const Visual& vis : src_link->link.visual) {
            Visual new_vis = vis;
            new_vis.geometry.type = cloneGeometry(vis.geometry.type);
            new_link.visual.push_back(new_vis);
        }
        for (const Collision& col : src_link->link.collision) {
            Collision new_col = col;
            new_col.geometry.type = cloneGeometry(col.geometry.type);
            new_link.collision.push_back(new_col);
        }

        auto new_link_node = std::make_shared<LinkNode>(std::move(new_link), nullptr);

        // Clone joint data
        std::string jname = deduplicateName(src_joint->joint.name, joint_names);
        joint_names.insert(jname);
        Joint new_joint(jname.c_str(), new_parent->link.name.c_str(),
                        new_link_node->link.name.c_str(), "fixed");
        new_joint.type = src_joint->joint.type;
        new_joint.origin = src_joint->joint.origin;
        new_joint.axis = src_joint->joint.axis;
        new_joint.dynamics = src_joint->joint.dynamics;
        new_joint.limit = src_joint->joint.limit;
        new_joint.mimic = src_joint->joint.mimic;
        new_joint.calibration = src_joint->joint.calibration;
        new_joint.safety_controller = src_joint->joint.safety_controller;

        auto new_joint_node =
            std::make_shared<JointNode>(std::move(new_joint), new_parent, new_link_node);
        new_link_node->parent = new_joint_node;

        // Recurse into children
        for (const auto& child_joint : src_link->children) {
            auto cloned_child = recurse(child_joint, new_link_node);
            new_link_node->children.push_back(cloned_child);
        }

        return new_joint_node;
    };

    // The source link must have a parent joint — clone from that joint's perspective
    // but attach to attach_parent
    if (!source->parent) return nullptr;
    return recurse(source->parent, attach_parent);
}

RobotPtr buildRobot(const char* urdf_file) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(urdf_file);
    CHECK_F(result, "Failed to open %s", urdf_file);

    bool is_root_robot = doc.root().name() != std::string("robot");
    CHECK_F(is_root_robot, "The urdf root name (%s) is not robot", doc.root().name());
    LOG_F(INFO, "Building robot from: %s", urdf_file);

    std::string urdf_dir = std::filesystem::path(urdf_file).parent_path().string();

    pugi::xml_node root_link = findRoot(doc);
    CHECK_F(not root_link.empty(), "No root link found");

    auto tree_root = std::make_shared<LinkNode>();
    tree_root->link = xmlNodeToLink(root_link, urdf_dir);
    tree_root->w_T_l = MatrixIdentity();

    // Create all materials (global definitions first)
    std::map<std::string, Material> materials;
    for (pugi::xml_node& mat : doc.child("robot").children("material")) {
        if (const auto m = xmlNodeToMaterial(mat)) {
            materials.insert({m->name, *m});
        }
    }

    // Promote inline material definitions (materials defined inside <visual>) to the global map
    for (const pugi::xml_node& link : doc.child("robot").children("link")) {
        for (const pugi::xml_node& vis : link.children("visual")) {
            const pugi::xml_node& mat_node = vis.child("material");
            if (mat_node && (mat_node.child("color") || mat_node.child("texture"))) {
                if (const auto m = xmlNodeToMaterial(mat_node)) {
                    materials.insert({m->name, *m});
                }
            }
        }
    }
    // Resolve texture paths relative to the URDF directory
    for (auto& [name, mat] : materials) {
        if (mat.texture_file) {
            mat.texture_file = resolveFilePath(*mat.texture_file, urdf_dir);
        }
    }

    // TODO(ramon): every link uses the same shader, the material properties change the shader
    // color.

    // Parent link to all child joints
    std::map<std::string, std::vector<Joint>> joint_p_hash;
    for (const pugi::xml_node& joint : doc.child("robot").children("joint")) {
        Joint j = xmlNodeToJoint(joint);
        joint_p_hash[j.parent].push_back(j);
    }

    // Link name to link
    std::map<std::string, Link> link_hash;
    for (const pugi::xml_node& link : doc.child("robot").children("link")) {
        Link l = xmlNodeToLink(link, urdf_dir);
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
            auto joint_node = std::make_shared<JointNode>(JointNode{joint, current_root, nullptr});
            CHECK_F(link_hash.find(joint.child) != link_hash.end());

            auto child_node =
                std::make_shared<LinkNode>(LinkNode{link_hash[joint.child], joint_node});
            joint_node->child = child_node;
            current_root->children.push_back(joint_node);
            deq.push_back(child_node);
        }
    }

    RobotPtr robot = std::make_shared<Robot>(tree_root, materials);

    LOG_F(INFO, "URDF Tree built successfully!");

    return robot;
}

pugi::xml_node findRoot(const pugi::xml_document& doc) {
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

Link xmlNodeToLink(const pugi::xml_node& xml_node, const std::string& urdf_dir) {
    CHECK_F(static_cast<bool>(xml_node.attribute("name")), "Link has no name");
    Link link(xml_node.attribute("name").as_string());

    link.inertial = xmlNodeToInertial(xml_node.child("inertial"));

    for (const auto& vis_node : xml_node.children("visual")) {
        Visual vis{};

        // --- Name
        std::string visual_name = vis_node.attribute("name").as_string();
        if (not visual_name.empty()) vis.name = visual_name;

        // --- Origin
        const auto& ori_node = vis_node.child("origin");
        vis.origin = xmlNodeToOrigin(ori_node);

        // --- Geometry
        const auto& geom_node = vis_node.child("geometry");
        vis.geometry = xmlNodeToGeometry(geom_node, urdf_dir);

        // --- Material
        if (const auto& mat_node = vis_node.child("material")) {
            vis.material_name = mat_node.attribute("name").as_string();
            CHECK_F(not vis.material_name->empty(), "Materials need to have a name!");
        }

        link.visual.push_back(vis);
    }

    for (const auto& col_node : xml_node.children("collision")) {
        Collision col{};

        // --- Name
        std::string collision_name = col_node.attribute("name").as_string();
        if (not collision_name.empty()) col.name = collision_name;

        // --- Origin
        const auto& ori_node = col_node.child("origin");
        col.origin = xmlNodeToOrigin(ori_node);

        // --- Geometry
        const auto& geom_node = col_node.child("geometry");
        col.geometry = xmlNodeToGeometry(geom_node, urdf_dir);

        link.collision.push_back(col);
    }

    return link;
}

Joint xmlNodeToJoint(const pugi::xml_node& xml_node) {
    Joint joint(xml_node.attribute("name").as_string(),
                xml_node.child("parent").attribute("link").as_string(),
                xml_node.child("child").attribute("link").as_string(),
                xml_node.attribute("type").as_string());
    if (auto origin_node = xml_node.child("origin")) {
        joint.origin = Origin(origin_node.attribute("xyz").as_string(),
                              origin_node.attribute("rpy").as_string());
    }
    if (auto axis_node = xml_node.child("axis")) {
        joint.axis = Axis(axis_node.attribute("xyz").as_string());
    }
    if (auto dynamics_node = xml_node.child("dynamics")) {
        joint.dynamics = Dynamics(dynamics_node.attribute("damping").as_float(),
                                  dynamics_node.attribute("friction").as_float());
    }
    if (auto limit_node = xml_node.child("limit")) {
        joint.limit = Limit(limit_node.attribute("lower").as_float(),
                            limit_node.attribute("upper").as_float(),
                            limit_node.attribute("effort").as_float(),
                            limit_node.attribute("velocity").as_float());
    }
    if (auto mimic_node = xml_node.child("mimic")) {
        Mimic m;
        m.joint = mimic_node.attribute("joint").as_string();
        m.multiplier = mimic_node.attribute("multiplier").as_float(1.0f);
        m.offset = mimic_node.attribute("offset").as_float(0.0f);
        joint.mimic = m;
    }
    if (auto cal_node = xml_node.child("calibration")) {
        Calibration c;
        c.rising = cal_node.attribute("rising").as_float(0.0f);
        c.falling = cal_node.attribute("falling").as_float(0.0f);
        joint.calibration = c;
    }
    if (auto sc_node = xml_node.child("safety_controller")) {
        SafetyController sc;
        sc.k_velocity = sc_node.attribute("k_velocity").as_float(0.0f);
        sc.k_position = sc_node.attribute("k_position").as_float(0.0f);
        sc.soft_lower_limit = sc_node.attribute("soft_lower_limit").as_float(0.0f);
        sc.soft_upper_limit = sc_node.attribute("soft_upper_limit").as_float(0.0f);
        joint.safety_controller = sc;
    }
    return joint;
}

std::optional<Origin> xmlNodeToOrigin(const pugi::xml_node& xml_node) {
    if (!xml_node) return std::nullopt;
    return Origin(xml_node.attribute("xyz").as_string(), xml_node.attribute("rpy").as_string());
}

std::optional<Inertial> xmlNodeToInertial(const pugi::xml_node& xml_node) {
    if (!xml_node) return std::nullopt;

    return Inertial(xml_node.child("origin").attribute("xyz").as_string(),
                    xml_node.child("origin").attribute("rpy").as_string(),
                    xml_node.child("mass").attribute("value").as_float(),
                    xml_node.child("inertia").attribute("ixx").as_float(),
                    xml_node.child("inertia").attribute("iyy").as_float(),
                    xml_node.child("inertia").attribute("izz").as_float(),
                    xml_node.child("inertia").attribute("ixy").as_float(),
                    xml_node.child("inertia").attribute("ixz").as_float(),
                    xml_node.child("inertia").attribute("iyz").as_float());
}

Geometry xmlNodeToGeometry(const pugi::xml_node& geom_node, const std::string& urdf_dir) {
    CHECK_F(static_cast<bool>(geom_node), "Missing geometry tag!");

    Geometry geom;
    if (geom_node.child("box")) {
        geom.type = std::make_shared<Box>(geom_node.child("box").attribute("size").as_string());
    } else if (geom_node.child("cylinder")) {
        geom.type =
            std::make_shared<Cylinder>(geom_node.child("cylinder").attribute("radius").as_string(),
                                       geom_node.child("cylinder").attribute("length").as_string());
    } else if (geom_node.child("sphere")) {
        geom.type =
            std::make_shared<Sphere>(geom_node.child("sphere").attribute("radius").as_string());
    } else if (geom_node.child("mesh")) {
        std::string filename = geom_node.child("mesh").attribute("filename").as_string();
        std::string resolved = resolveFilePath(filename, urdf_dir);
        LOG_F(1, "Mesh: %s -> %s", filename.c_str(), resolved.c_str());
        Vector3 scale{1.0f, 1.0f, 1.0f};
        if (geom_node.child("mesh").attribute("scale")) {
            storeVec3(geom_node.child("mesh").attribute("scale").as_string(), scale);
        }
        geom.type = std::make_shared<Mesh>(filename.c_str(), resolved, scale);
    }

    return geom;
}

std::optional<Material> xmlNodeToMaterial(const pugi::xml_node& mat_node) {
    if (!mat_node) return std::nullopt;

    Material mat;

    mat.name = mat_node.attribute("name").as_string();
    CHECK_F(not mat.name.empty(), "Materials need to have a name!");

    if (auto color_node = mat_node.child("color")) {
        CHECK_F(static_cast<bool>(color_node.attribute("rgba")),
                "The material color tag needs to have an rgba value");
        mat.rgba = Vector4{};
        storeVec4(color_node.attribute("rgba").as_string(), mat.rgba.value());
    }

    if (auto texture_node = mat_node.child("texture")) {
        CHECK_F(static_cast<bool>(texture_node.attribute("filename")),
                "The texture tag needs to have a filename attribute!");
        mat.texture_file = texture_node.attribute("filename").as_string();
    }

    return mat;
}

void exportRobot(const Robot& robot, std::string out_filename) {
    pugi::xml_document out_doc;
    const LinkNodePtr root_node = robot.getRoot();

    pugi::xml_node robot_root = out_doc.append_child("robot");

    // save links
    std::deque<LinkNodePtr> deq{root_node};
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
    deq = std::deque<LinkNodePtr>{root_node};
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

void linkToXmlNode(pugi::xml_node& xml_node, const Link& link) {
    xml_node.set_name("link");
    xml_node.append_attribute("name") = link.name.c_str();

    if (link.inertial) {
        pugi::xml_node inertial_node = xml_node.append_child("inertial");
        inertialToXmlNode(inertial_node, *link.inertial);
    }

    for (const Visual& vis : link.visual) {
        pugi::xml_node visual_node = xml_node.append_child("visual");
        if (vis.name) {
            visual_node.append_attribute("name") = vis.name->c_str();
        }
        if (vis.origin) {
            pugi::xml_node origin_node = visual_node.append_child("origin");
            originToXmlNode(origin_node, *vis.origin);
        }
        pugi::xml_node geometry_node = visual_node.append_child("geometry");
        geometryToXmlNode(geometry_node, vis.geometry);
        if (vis.material_name) {
            visual_node.append_child("material").append_attribute("name") =
                vis.material_name->c_str();
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

void jointToXmlNode(pugi::xml_node& joint_node, const Joint& joint) {
    joint_node.set_name("joint");
    joint_node.append_attribute("name") = joint.name.c_str();
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
        axis_node.append_attribute("xyz") =
            fmt::format("{} {} {}", joint.axis->xyz.x, joint.axis->xyz.y, joint.axis->xyz.z)
                .c_str();
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

    if (joint.mimic) {
        pugi::xml_node mimic_node = joint_node.append_child("mimic");
        mimic_node.append_attribute("joint") = joint.mimic->joint.c_str();
        mimic_node.append_attribute("multiplier") = joint.mimic->multiplier;
        mimic_node.append_attribute("offset") = joint.mimic->offset;
    }

    if (joint.calibration) {
        pugi::xml_node cal_node = joint_node.append_child("calibration");
        cal_node.append_attribute("rising") = joint.calibration->rising;
        cal_node.append_attribute("falling") = joint.calibration->falling;
    }

    if (joint.safety_controller) {
        pugi::xml_node sc_node = joint_node.append_child("safety_controller");
        sc_node.append_attribute("k_velocity") = joint.safety_controller->k_velocity;
        sc_node.append_attribute("k_position") = joint.safety_controller->k_position;
        sc_node.append_attribute("soft_lower_limit") = joint.safety_controller->soft_lower_limit;
        sc_node.append_attribute("soft_upper_limit") = joint.safety_controller->soft_upper_limit;
    }
}

void inertialToXmlNode(pugi::xml_node& xml_node, const Inertial& inertial) {
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

void originToXmlNode(pugi::xml_node& xml_node, const Origin& origin) {
    xml_node.set_name("origin");

    xml_node.append_attribute("xyz") =
        fmt::format("{} {} {}", origin.xyz.x, origin.xyz.y, origin.xyz.z).c_str();
    xml_node.append_attribute("rpy") =
        fmt::format("{} {} {}", origin.rpy.x, origin.rpy.y, origin.rpy.z).c_str();
}

void geometryToXmlNode(pugi::xml_node& xml_node, const Geometry& geometry) {
    xml_node.set_name("geometry");

    if (auto box = std::dynamic_pointer_cast<Box>(geometry.type)) {
        pugi::xml_node box_node = xml_node.append_child("box");
        box_node.append_attribute("size") =
            fmt::format("{} {} {}", box->size.x, box->size.y, box->size.z).c_str();
    } else if (auto cylinder = std::dynamic_pointer_cast<Cylinder>(geometry.type)) {
        pugi::xml_node cylinder_node = xml_node.append_child("cylinder");
        cylinder_node.append_attribute("radius") = fmt::format("{}", cylinder->radius).c_str();
        cylinder_node.append_attribute("length") = fmt::format("{}", cylinder->length).c_str();
    } else if (auto sphere = std::dynamic_pointer_cast<Sphere>(geometry.type)) {
        pugi::xml_node sphere_node = xml_node.append_child("sphere");
        sphere_node.append_attribute("radius") = fmt::format("{}", sphere->radius).c_str();
    } else if (auto mesh = std::dynamic_pointer_cast<Mesh>(geometry.type)) {
        pugi::xml_node mesh_node = xml_node.append_child("mesh");
        mesh_node.append_attribute("filename") = mesh->filename.c_str();
        if (mesh->scale.x != 1.0f || mesh->scale.y != 1.0f || mesh->scale.z != 1.0f) {
            mesh_node.append_attribute("scale") =
                fmt::format("{} {} {}", mesh->scale.x, mesh->scale.y, mesh->scale.z).c_str();
        }
    } else {
        LOG_F(WARNING, "Geometry type not implemented yet");
    }
}

void materialToXmlNode(pugi::xml_node& xml_node, const Material& material) {
    xml_node.set_name("material");
    xml_node.append_attribute("name") = material.name.c_str();
    if (material.rgba) {
        pugi::xml_node color_node = xml_node.append_child("color");
        color_node.append_attribute("rgba") =
            fmt::format("{} {} {} {}", material.rgba->x, material.rgba->y, material.rgba->z,
                        material.rgba->w)
                .c_str();
    }
    if (material.texture_file) {
        pugi::xml_node texture_node = xml_node.append_child("texture");
        texture_node.append_attribute("filename") = material.texture_file->c_str();
    }
}

Robot::Robot(LinkNodePtr root, const std::map<std::string, Material>& materials)
    : root_(std::move(root)), materials_(materials) {}

Robot::~Robot() {
    auto func = [&](const LinkNodePtr& link) {
        for (const Model& mod : link->visual_models) {
            UnloadModel(mod);
        }
        for (const Model& mod : link->collision_models) {
            UnloadModel(mod);
        }
    };

    forEveryLink(func);

    for (auto& [name, tex] : loaded_textures_) {
        UnloadTexture(tex);
    }
}

void Robot::forwardKinematics() { forwardKinematics(root_); }

// Recursively compute forward kinematics starting from the given link.
// For each child joint, the world-frame transform of the child link is:
//   w_T_child = w_T_parent * p_T_joint * j_T_child
// where p_T_joint comes from the joint origin and j_T_child is identity
// (no joint-to-child offset currently). Visual and collision transforms
// are then composed on top: w_T_visual = w_T_child * c_T_visual.
void Robot::forwardKinematics(LinkNodePtr& link) {
    std::function<void(LinkNodePtr&)> recursion = [&](LinkNodePtr& node) {
        const Matrix& w_t_p = node->w_T_l;

        for (auto& joint_node : node->children) {
            const Matrix p_t_j = originToMatrix(joint_node->joint.origin);
            Matrix j_t_c = MatrixIdentity();
            if (joint_node->joint.isArticulated()) {
                Vector3 ax = joint_node->joint.axis ? joint_node->joint.axis->xyz
                                                    : Vector3{0, 0, 1};
                if (joint_node->joint.type == Joint::kPrismatic) {
                    Vector3 d = Vector3Scale(ax, joint_node->q);
                    j_t_c = MatrixTranslate(d.x, d.y, d.z);
                } else {
                    j_t_c = MatrixRotate(ax, joint_node->q);
                }
            }
            const Matrix w_t_c = MatMul(w_t_p, MatMul(p_t_j, j_t_c));
            joint_node->child->w_T_l = w_t_c;

            // Update visual transforms
            for (size_t i = 0; i < joint_node->child->visual_models.size(); ++i) {
                auto& vis_origin = joint_node->child->link.visual[i].origin;
                const Matrix c_t_v = originToMatrix(vis_origin);
                const Matrix w_t_v = MatMul(w_t_c, c_t_v);
                joint_node->child->visual_models[i].transform = w_t_v;
            }

            // Update collision transforms
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

Matrix Robot::originToMatrix(std::optional<Origin>& origin) {
    return origin ? origin->toMatrix() : MatrixIdentity();
}

void Robot::buildGeometry() {
    loadTextures();

    auto func = [&](const LinkNodePtr& link) {
        // Visual models
        for (size_t i = 0; i < link->link.visual.size(); ++i) {
            Model model = link->link.visual[i].geometry.type->generateGeometry();
            model.materials[0].shader = visual_shader_;
            link->visual_models.push_back(model);
        }
        updateMaterial(link);

        // Collision models
        for (const Collision& col : link->link.collision) {
            link->collision_models.push_back(col.geometry.type->generateGeometry());
        }
    };

    forEveryLink(func);
}

void Robot::loadTextures() {
    for (const auto& [name, mat] : materials_) {
        loadTextureForMaterial(name, mat);
    }
}

void Robot::reloadTexture(const std::string& material_name) {
    auto it = loaded_textures_.find(material_name);
    if (it != loaded_textures_.end()) {
        UnloadTexture(it->second);
        loaded_textures_.erase(it);
    }

    auto mat_it = materials_.find(material_name);
    if (mat_it != materials_.end()) {
        loadTextureForMaterial(material_name, mat_it->second);
    }
}

void Robot::loadTextureForMaterial(const std::string& name, const Material& mat) {
    if (!mat.texture_file || mat.texture_file->empty()) return;

    if (!std::filesystem::exists(*mat.texture_file)) {
        LOG_F(WARNING, "Texture file not found for material '%s': %s", name.c_str(),
              mat.texture_file->c_str());
        return;
    }

    loaded_textures_[name] = LoadTexture(mat.texture_file->c_str());
}

void Robot::updateMaterial(const LinkNodePtr& link) {
    const Vector4 default_grey{0.5f, 0.5f, 0.5f, 1.0f};
    const Texture2D default_texture = {rlGetTextureIdDefault(), 1, 1, 1,
                                       PIXELFORMAT_UNCOMPRESSED_R8G8B8A8};

    for (size_t i = 0; i < link->link.visual.size(); ++i) {
        if (i >= link->visual_models.size()) break;

        Vector4 color = default_grey;
        Texture2D texture = default_texture;

        const auto& mat_name = link->link.visual[i].material_name;
        if (mat_name) {
            auto it = materials_.find(*mat_name);
            if (it != materials_.end() && it->second.rgba) {
                color = *it->second.rgba;
            }

            auto tex_it = loaded_textures_.find(*mat_name);
            if (tex_it != loaded_textures_.end()) {
                texture = tex_it->second;
            }
        }

        MaterialMap& diffuse = link->visual_models[i].materials[0].maps[MATERIAL_MAP_DIFFUSE];
        diffuse.color = ColorFromNormalized(color);
        diffuse.texture = texture;
    }
}

void Robot::draw(const LinkNodePtr& highlighted, const LinkNodePtr& selected,
                 bool show_collisions) const {
    auto draw_link = [&](const LinkNodePtr& link) {
        for (const Model& vm : link->visual_models) {
            if (vm.meshCount > 0) {
                if (highlighted.get() == link.get()) {
                    DrawModel(vm, Vector3Zero(), 1.0, RED);
                } else {
                    DrawModel(vm, Vector3Zero(), 1.0, WHITE);
                }
            }
        }
        if (show_collisions) {
            for (const Model& model : link->collision_models) {
                if (selected.get() == link.get()) {
                    DrawModelWires(model, Vector3Zero(), 1.0, GREEN);
                } else {
                    DrawModelWires(model, Vector3Zero(), 1.0, GRAY);
                }
            }
        }
    };

    forEveryLink(draw_link);
}

void Robot::setShader(const Shader& sh) {
    visual_shader_ = sh;

    auto func = [&](const LinkNodePtr& link) {
        for (Model& vm : link->visual_models) {
            if (vm.materialCount > 0) {
                if (::Material* mat = vm.materials) {
                    mat[0].shader = visual_shader_;
                }
            }
        }
    };

    forEveryLink(func);
}

void Robot::printTree() const {
    auto func = [](const LinkNodePtr& link) { LOG_F(INFO, "%s", link->link.name.c_str()); };
    forEveryLink(func);
}

LinkNodePtr Robot::getRoot() const { return root_; }

std::optional<HitResult> Robot::getLink(const Ray& ray) {
    float closest = FLT_MAX;
    std::optional<HitResult> result;

    forEveryLink([&](const LinkNodePtr& link) {
        // Test visual meshes
        for (int vi = 0; vi < static_cast<int>(link->visual_models.size()); vi++) {
            const Model& vm = link->visual_models[vi];
            for (int m = 0; m < vm.meshCount; m++) {
                RayCollision rc = GetRayCollisionMesh(ray, vm.meshes[m], vm.transform);
                if (rc.hit && rc.distance < closest) {
                    closest = rc.distance;
                    result = {link, HitResult::kVisual, vi};
                }
            }
        }
        // Test collision meshes
        int num_collisions = static_cast<int>(link->collision_models.size());
        for (int i = 0; i < num_collisions; i++) {
            const Model& cm = link->collision_models[i];
            for (int m = 0; m < cm.meshCount; m++) {
                RayCollision rc = GetRayCollisionMesh(ray, cm.meshes[m], cm.transform);
                if (rc.hit && rc.distance < closest) {
                    closest = rc.distance;
                    result = {link, HitResult::kCollision, i};
                }
            }
        }
    });

    return result;
}

std::vector<ValidationMessage> Robot::validate() const {
    std::vector<ValidationMessage> msgs;

    // Collect link names and check for duplicates/empty
    std::map<std::string, int> link_name_counts;
    forEveryLink([&](const LinkNodePtr& link) { link_name_counts[link->link.name]++; });
    for (const auto& [name, count] : link_name_counts) {
        if (name.empty()) {
            msgs.push_back({ValidationMessage::kError, "Empty link name found"});
        }
        if (count > 1) {
            msgs.push_back(
                {ValidationMessage::kError, fmt::format("Duplicate link name: '{}'", name)});
        }
    }

    // Collect joint names and check for duplicates/empty
    std::map<std::string, int> joint_name_counts;
    forEveryJoint([&](const JointNodePtr& joint) { joint_name_counts[joint->joint.name]++; });
    for (const auto& [name, count] : joint_name_counts) {
        if (name.empty()) {
            msgs.push_back({ValidationMessage::kError, "Empty joint name found"});
        }
        if (count > 1) {
            msgs.push_back(
                {ValidationMessage::kError, fmt::format("Duplicate joint name: '{}'", name)});
        }
    }

    // Joint-level checks
    forEveryJoint([&](const JointNodePtr& jn) {
        const Joint& j = jn->joint;

        // Revolute/prismatic require limit
        if ((j.type == Joint::kRevolute || j.type == Joint::kPrismatic) && !j.limit) {
            msgs.push_back({ValidationMessage::kError,
                            fmt::format("Joint '{}': revolute/prismatic requires <limit>", j.name)});
        }

        // Non-unit axis
        if (j.axis) {
            float len = Vector3Length(j.axis->xyz);
            if (std::abs(len - 1.0f) > 0.01f) {
                msgs.push_back({ValidationMessage::kWarning,
                                fmt::format("Joint '{}': axis vector not unit length ({:.3f})",
                                            j.name, len)});
            }
        }

        // Limit lower >= upper
        if (j.limit && j.limit->lower >= j.limit->upper) {
            msgs.push_back({ValidationMessage::kWarning,
                            fmt::format("Joint '{}': limit lower ({}) >= upper ({})", j.name,
                                        j.limit->lower, j.limit->upper)});
        }

        // Mimic references non-existent joint
        if (j.mimic && joint_name_counts.count(j.mimic->joint) == 0) {
            msgs.push_back({ValidationMessage::kWarning,
                            fmt::format("Joint '{}': mimic references non-existent joint '{}'",
                                        j.name, j.mimic->joint)});
        }
    });

    // Link-level checks
    forEveryLink([&](const LinkNodePtr& ln) {
        // Zero or negative mass
        if (ln->link.inertial && ln->link.inertial->mass <= 0.0f) {
            msgs.push_back({ValidationMessage::kWarning,
                            fmt::format("Link '{}': zero or negative mass", ln->link.name)});
        }

        // Visual references non-existent material
        for (const auto& vis : ln->link.visual) {
            if (vis.material_name && materials_.count(*vis.material_name) == 0) {
                msgs.push_back(
                    {ValidationMessage::kWarning,
                     fmt::format("Link '{}': visual references non-existent material '{}'",
                                 ln->link.name, *vis.material_name)});
            }
        }
    });

    return msgs;
}

const std::map<std::string, Material>& Robot::getMaterials() const { return materials_; }

std::map<std::string, Material>& Robot::getMutableMaterials() { return materials_; }

void Robot::forEveryLink(const std::function<void(const LinkNodePtr&)>& func) const {
    std::deque<LinkNodePtr> deq{root_};

    while (not deq.empty()) {
        const auto& current_link = deq.front();
        deq.pop_front();

        func(current_link);

        for (const auto& joint : current_link->children) {
            deq.push_back(joint->child);
        }
    }
}

void Robot::forEveryJoint(const std::function<void(const JointNodePtr&)>& func) const {
    std::deque<LinkNodePtr> deq{root_};

    while (not deq.empty()) {
        const auto& current_link = deq.front();
        deq.pop_front();

        for (const auto& joint : current_link->children) {
            func(joint);
            deq.push_back(joint->child);
        }
    }
}

}  // namespace urdf
