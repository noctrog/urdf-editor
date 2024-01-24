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

#include <raymath.h>

namespace urdf {

static inline Matrix MatMul(const Matrix& left, const Matrix& right)
{
    return MatrixMultiply(right, left);
}

::Mesh GenMeshCenteredCylinder(float radius, float height, int slices)
{
    ::Mesh mesh = { 0 };

    if (slices >= 3)
    {
        // Create a cylinder along Y-axis
        par_shapes_mesh* cylinder = par_shapes_create_cylinder(slices, 8);
        par_shapes_scale(cylinder, radius, radius, height);

        // Translate the cylinder to center it at the origin
        par_shapes_translate(cylinder, 0, 0, -height / 2.0f);

        // Create top cap
        float center_top[] = { 0, 0, height / 2 };
        float normal_top[] = { 0, 0, 1 };
        par_shapes_mesh* capTop = par_shapes_create_disk(radius, slices, center_top, normal_top);
        capTop->tcoords = PAR_MALLOC(float, 2 * capTop->npoints);
        for (int i = 0; i < 2 * capTop->npoints; i++) capTop->tcoords[i] = 0.0f;

        // Create bottom cap
        float center_bot[] = { 0, 0, 0 };
        float normal_bot[] = { 0, 0, 1 };
        par_shapes_mesh* capBottom = par_shapes_create_disk(radius, slices, center_bot, normal_bot);
        float x_vec[] = {1, 0, 0};
        par_shapes_rotate(capBottom, -PI, x_vec);
        par_shapes_translate(capBottom, 0, 0, -height / 2.0f);
        capBottom->tcoords = PAR_MALLOC(float, 2 * capBottom->npoints);
        for (int i = 0; i < 2 * capBottom->npoints; i++) capBottom->tcoords[i] = 0.95f;

        // Merge cylinder and caps
        par_shapes_merge_and_free(cylinder, capTop);
        par_shapes_merge_and_free(cylinder, capBottom);


        mesh.vertices = (float *)RL_MALLOC(cylinder->ntriangles*3*3*sizeof(float));
        mesh.texcoords = (float *)RL_MALLOC(cylinder->ntriangles*3*2*sizeof(float));
        mesh.normals = (float *)RL_MALLOC(cylinder->ntriangles*3*3*sizeof(float));

        mesh.vertexCount = cylinder->ntriangles*3;
        mesh.triangleCount = cylinder->ntriangles;

        for (int k = 0; k < mesh.vertexCount; k++)
        {
            mesh.vertices[k*3] = cylinder->points[cylinder->triangles[k]*3];
            mesh.vertices[k*3 + 1] = cylinder->points[cylinder->triangles[k]*3 + 1];
            mesh.vertices[k*3 + 2] = cylinder->points[cylinder->triangles[k]*3 + 2];

            mesh.normals[k*3] = cylinder->normals[cylinder->triangles[k]*3];
            mesh.normals[k*3 + 1] = cylinder->normals[cylinder->triangles[k]*3 + 1];
            mesh.normals[k*3 + 2] = cylinder->normals[cylinder->triangles[k]*3 + 2];

            mesh.texcoords[k*2] = cylinder->tcoords[cylinder->triangles[k]*2];
            mesh.texcoords[k*2 + 1] = cylinder->tcoords[cylinder->triangles[k]*2 + 1];
        }

        par_shapes_free_mesh(cylinder);

        // Upload vertex data to GPU (static mesh)
        UploadMesh(&mesh, false);
    }
    else LOG_F(WARNING, "MESH: Failed to generate mesh: cylinder");

    return mesh;
}

static void store_float(const char *s, float& f) {
    std::stringstream iss(s);
    iss >> f;
    CHECK_F(iss && (iss >> std::ws).eof(), "store_float: invalid string");
}

static void store_vec3(const char *s, Vector3& vec) {
    std::stringstream iss(s);
    iss >> vec.x >> vec.y >> vec.z;
    CHECK_F(iss && (iss >> std::ws).eof(), "store_vec3: invalid string");
}

static void store_vec4(const char *s, Vector4& vec) {
    std::stringstream iss(s);
    iss >> vec.x >> vec.y >> vec.z >> vec.w;
    CHECK_F(iss && (iss >> std::ws).eof(), "store_vec4: invalid string");
}

Origin::Origin() : xyz{}, rpy{} { }

Origin::Origin(const char *xyz_s, const char *rpy_s)
{
    xyz.x = xyz.y = xyz.z = 0.0f;
    rpy.x = rpy.y = rpy.z = 0.0f;

    if (!xyz_s || std::strlen(xyz_s) == 0) {
        LOG_F(WARNING, "Origin xyz is empty");
    } else {
        store_vec3(xyz_s, xyz);
    }

    if (!rpy_s || std::strlen(rpy_s) == 0) {
        LOG_F(WARNING, "Origin rpy is empty");
    } else {
        store_vec3(rpy_s, rpy);
    }
}

Origin::Origin(const Vector3& xyz, const Vector3& rpy)
    : xyz(xyz), rpy(rpy)
{

}

Box::Box()
{
    size.x = size.y = size.z = 1.0f;
}

Box::Box(const char *_size)
{
    store_vec3(_size, size);
}

Box::Box(const Vector3& size)
    : size(size)
{

}

::Mesh Box::generateGeometry()
{
    return GenMeshCube(size.x, size.y, size.z);
}

Cylinder::Cylinder()
{
    radius = length = 1.0f;
}

Cylinder::Cylinder(const char *_radius, const char *_length)
{
    radius = length = 0.0f;
    if (_radius) store_float(_radius, radius);
    if (_length) store_float(_length, length);
}

::Mesh Cylinder::generateGeometry()
{
    ::Mesh mesh = GenMeshCenteredCylinder(radius, length, 32);


    return mesh;
}

Sphere::Sphere()
{
    radius = 1.0f;
}

Sphere::Sphere(const char *_radius)
{
    store_float(_radius, radius);
}

Sphere::Sphere(float _radius)
    : radius(_radius)
{

}

::Mesh Sphere::generateGeometry()
{
    return GenMeshSphere(radius, 32, 32);
}

Mesh::Mesh(const char *filename)
    : filename(filename)
{

}

::Mesh Mesh::generateGeometry()
{
    return GenMeshSphere(1.0, 32, 32); // TODO import geometry
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

Axis::Axis(const char *s)
{
    store_vec3(s, xyz);
}

Dynamics::Dynamics() : damping(0.0f), friction(0.0f) { }

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

TreeNode::~TreeNode() { }

LinkNode::LinkNode()
    : T(MatrixIdentity())
{

}

LinkNode::LinkNode(const Link& link, const JointNodePtr& parent_joint)
    : link(link), parent(parent_joint), T(MatrixIdentity())
{

}

void LinkNode::AddCollision()
{
    link.collision.push_back(urdf::Collision());
    link.collision.back().geometry.type = std::make_shared<urdf::Box>();
    collision_mesh.emplace_back(link.collision.back().geometry.type->generateGeometry());
    collision_models.push_back(LoadModelFromMesh(collision_mesh.back()));
}

void LinkNode::DeleteCollision(int i)
{
    link.collision.erase(link.collision.begin() + i);
    UnloadMesh(collision_mesh[i]);
    collision_mesh.erase(collision_mesh.begin() + i);
    collision_models.erase(collision_models.begin() + i);
}

JointNode::JointNode(const Joint& joint, const LinkNodePtr& parent, const LinkNodePtr& child)
    : joint(joint), parent(parent), child(child)
{

}

Parser::Parser()
{

}

Parser::~Parser()
{

}

std::shared_ptr<Robot> Parser::build_robot(const char *urdf_file)
{
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(urdf_file);
    CHECK_F(result, "Failed to open %s", urdf_file);

    bool is_root_robot = doc.root().name() != std::string("robot");
    CHECK_F(is_root_robot, "The urdf root name (%s) is not robot", doc.root().name());
    LOG_F(INFO, "Building robot...");

    pugi::xml_node root_link = find_root(doc);
    CHECK_F(not root_link.empty(), "No root link found");

    auto tree_root = std::make_shared<LinkNode>();
    tree_root->link = xml_node_to_link(root_link);
    tree_root->T = MatrixIdentity();

    // Create all materials
    // TODO: support loading materials that are defined within the link
    std::map<std::string, Material> materials;
    for (pugi::xml_node& mat : doc.child("robot").children("material")) {
        if (const auto m = xml_node_to_material(mat)) {
            materials.insert({m->name, *m});
        }
    }
    // TODO: every link uses the same shader, the material properties change the shader color.

    // Parent link to all child joints
    std::map<std::string, std::vector<Joint>> joint_p_hash;
    for (const pugi::xml_node& joint : doc.child("robot").children("joint")) {
        Joint j = xml_node_to_joint(joint);
        if (joint_p_hash.find(j.parent) == joint_p_hash.end()) {
            joint_p_hash.insert({ j.parent, std::vector<Joint>({j}) });
        } else {
            joint_p_hash[j.parent].push_back(j);
        }
    }

    // Link name to link
    std::map<std::string, Link> link_hash;
    for (const pugi::xml_node& link : doc.child("robot").children("link")) {
        Link l = xml_node_to_link(link);
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

    std::shared_ptr<Robot> robot = std::make_shared<Robot>(tree_root, materials);

    LOG_F(INFO, "URDF Tree built successfully!");

    return robot;
}

pugi::xml_node Parser::find_root(const pugi::xml_document& doc)
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

Link Parser::xml_node_to_link(const pugi::xml_node& xml_node)
{
    Link link;

    CHECK_F(static_cast<bool>(xml_node.attribute("name")), "Link has no name");
    link.name = xml_node.attribute("name").as_string();

    link.inertial = xml_node_to_inertial(xml_node.child("inertial"));

    const pugi::xml_node& vis_node = xml_node.child("visual");
    if (vis_node) {
        link.visual = Visual{};

        // --- Name
        std::string visual_name = vis_node.attribute("name").as_string();
        if (not visual_name.empty()) link.visual->name = visual_name;

        // --- Origin
        const auto& ori_node = vis_node.child("origin");
        link.visual->origin = xml_node_to_origin(ori_node);

        // --- Geometry
        const auto& geom_node = vis_node.child("geometry");
        link.visual->geometry = xml_node_to_geometry(geom_node);

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
        col.origin = xml_node_to_origin(ori_node);

        // --- Geometry
        const auto& geom_node = col_node.child("geometry");
        col.geometry = xml_node_to_geometry(geom_node);

        link.collision.push_back(col);
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

std::optional<Origin> Parser::xml_node_to_origin(const pugi::xml_node& xml_node)
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

std::optional<Inertial> Parser::xml_node_to_inertial(const pugi::xml_node& xml_node)
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

Geometry Parser::xml_node_to_geometry(const pugi::xml_node& geom_node)
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
        LOG_F(INFO, "Mesh visuals are not implemented yet!"); // TODO
        geom.type = std::make_shared<Box>();
    }

    return geom;
}

std::optional<Material> Parser::xml_node_to_material(const pugi::xml_node& mat_node)
{
    if (mat_node) {
        Material mat;

        mat.name = mat_node.attribute("name").as_string();
        CHECK_F(not mat.name.empty(), "Materials need to have a name!");

        if (mat_node.child("color")) {
            CHECK_F(static_cast<bool>(mat_node.child("color").attribute("rgba")),
                    "The material color tag needs to have an rgba value");
            mat.rgba = Vector4{};
            store_vec4(mat_node.child("color").attribute("rgba").as_string(), mat.rgba.value());
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

void Parser::export_robot(const Robot& robot, std::string out_filename)
{
    pugi::xml_document out_doc;
    const LinkNodePtr root_node = robot.get_root();

    pugi::xml_node robot_root = out_doc.append_child("robot");

    // save links
    std::deque<LinkNodePtr> deq {root_node};
    while (not deq.empty()) {
        const auto& current_link = deq.front();
        deq.pop_front();

        pugi::xml_node link_node = robot_root.append_child("link");
        link_to_xml_node(link_node, current_link->link);

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
            joint_to_xml_node(joint_node, joint->joint);

            deq.push_back(joint->child);
        }
    }

    // save materials
    for (const auto& mat : robot.get_materials()) {
        pugi::xml_node mat_node = robot_root.append_child("material");
        material_to_xml_node(mat_node, mat.second);
    }

    if (!out_doc.save_file(out_filename.c_str())) {
        LOG_F(ERROR, "File %s could not be opened or written!", out_filename.c_str());
    }
}

void Parser::link_to_xml_node(pugi::xml_node& xml_node, const Link& link)
{
    xml_node.set_name("link");
    xml_node.append_attribute("name") = link.name.c_str();

    if (link.inertial) {
        pugi::xml_node inertial_node = xml_node.append_child("inertial");
        inertial_to_xml_node(inertial_node, *link.inertial);
    }

    if (link.visual) {
        pugi::xml_node visual_node = xml_node.append_child("visual");
        if (link.visual->name) {
            visual_node.append_attribute("name") = link.visual->name->c_str();
        }
        if (link.visual->origin) {
            pugi::xml_node origin_node = visual_node.append_child("origin");
            origin_to_xml_node(origin_node, *link.visual->origin);
        }
        pugi::xml_node geometry_node = visual_node.append_child("geometry");
        geometry_to_xml_node(geometry_node, link.visual->geometry);
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
            origin_to_xml_node(origin_node, *col.origin);
        }

        pugi::xml_node geometry_node = collision_node.append_child("geometry");
        geometry_to_xml_node(geometry_node, col.geometry);
    }

    if (not xml_node) LOG_F(ERROR, "Link node is empty!");
}

void Parser::joint_to_xml_node(pugi::xml_node& joint_node, const Joint& joint)
{
    joint_node.set_name("joint");
    switch (joint.type) {
        case Joint::REVOLUTE:
            joint_node.append_attribute("type") = "revolute";
            break;
        case Joint::CONTINUOUS:
            joint_node.append_attribute("type") = "continuous";
            break;
        case Joint::PRISMATIC:
            joint_node.append_attribute("type") = "prismatic";
            break;
        case Joint::FIXED:
            joint_node.append_attribute("type") = "fixed";
            break;
        case Joint::FLOATING:
            joint_node.append_attribute("type") = "floating";
            break;
        case Joint::PLANAR:
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
        origin_to_xml_node(origin_node, *joint.origin);
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

void Parser::inertial_to_xml_node(pugi::xml_node& xml_node, const Inertial& inertial)
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
    origin_to_xml_node(origin_node, inertial.origin);
}

void Parser::origin_to_xml_node(pugi::xml_node& xml_node, const Origin& origin)
{
    xml_node.set_name("origin");

    xml_node.append_attribute("xyz") = (std::to_string(origin.xyz.x) + " " +
        std::to_string(origin.xyz.y) + " " +
        std::to_string(origin.xyz.z)).c_str();
    xml_node.append_attribute("rpy") = (std::to_string(origin.rpy.x) + " " +
        std::to_string(origin.rpy.y) + " " +
        std::to_string(origin.rpy.z)).c_str();
}

void Parser::geometry_to_xml_node(pugi::xml_node& xml_node, const Geometry& geometry)
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

void Parser::material_to_xml_node(pugi::xml_node& xml_node, const Material& material)
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

Robot::Robot(const LinkNodePtr& root,
      const std::map<std::string, Material>& materials)
    : root_(root), materials_(materials)
{

}

Robot::~Robot()
{
    std::deque<LinkNodePtr> deq {root_};

    while (not deq.empty()) {
        const auto& link = deq.front();
        deq.pop_front();

        UnloadModel(link->visual_model);
        for (const Model& mod : link->collision_models) {
            UnloadModel(mod);
        }

        for (const auto& joint : link->children) {
            deq.push_back(joint->child);
        }
    }
}

void Robot::forward_kinematics(void)
{
    forward_kinematics(root_);
}

void Robot::forward_kinematics(LinkNodePtr& link)
{
    std::function<void (LinkNodePtr&)> recursion = [&](LinkNodePtr& node){
        const Matrix& w_T_p = node->T;

        for (auto& joint_node : node->children) {
            // Forward kinematics
            const Matrix p_T_j = origin_to_matrix(joint_node->joint.origin);
            const Matrix j_T_c = MatrixIdentity();  // TODO: joint -> child transform
            const Matrix w_T_c = MatMul(w_T_p, MatMul(p_T_j, j_T_c));
            joint_node->child->T = w_T_c;

            // Update visual transform
            const Matrix c_T_v = origin_to_matrix(joint_node->child->link.visual->origin);
            const Matrix w_T_v = MatMul(w_T_c, c_T_v);
            joint_node->child->visual_model.transform = w_T_v;

            // Update collision transform
            for (size_t i = 0; i < joint_node->child->collision_models.size(); ++i) {
                const Matrix c_T_col = origin_to_matrix(joint_node->child->link.collision[i].origin);
                const Matrix w_T_col = MatMul(w_T_c, c_T_col);
                joint_node->child->collision_models[i].transform = w_T_col;
            }

            recursion(joint_node->child);
        }
    };

    recursion(link);
}

Matrix Robot::origin_to_matrix(std::optional<Origin>& origin)
{
    Matrix T = MatrixIdentity();

    if (origin) {
        T = MatMul(
            MatrixTranslate(origin->xyz.x, origin->xyz.y, origin->xyz.z),
            MatrixRotateXYZ(origin->rpy)
        );
    }

    return T;
}

void Robot::build_geometry()
{
    std::deque<LinkNodePtr> deq {root_};

    auto set_material_diffuse_color = [](const LinkNodePtr& link, const Vector4& color){
        link->visual_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color.r = static_cast<char>(color.x * 255.0f);
        link->visual_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color.g = static_cast<char>(color.y * 255.0f);
        link->visual_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color.b = static_cast<char>(color.z * 255.0f);
        link->visual_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color.a = static_cast<char>(color.w * 255.0f);
    };

    while (not deq.empty()) {
        const LinkNodePtr link = deq.front();
        deq.pop_front();

        // Visual model
        if (link->link.visual) {
            link->visual_mesh = link->link.visual->geometry.type->generateGeometry();
            link->visual_model = LoadModelFromMesh(link->visual_mesh);
            link->visual_model.materials[0].shader = visual_shader_;

            if (link->link.visual->material_name) {
                const Material& mat = materials_[*link->link.visual->material_name];
                if (mat.rgba) {
                    set_material_diffuse_color(link, *mat.rgba);
                } else {
                    set_material_diffuse_color(link, Vector4{127, 127, 127, 255});
                    LOG_F(INFO, "Link: %s. RGBA not found, using default color. Texture is not implemented yet.", link->link.name.c_str());
                }
            } else {
                set_material_diffuse_color(link, Vector4{127, 127, 127, 255});
                LOG_F(INFO, "Link: %s. No material name specified. Using default color", link->link.name.c_str());
            }
        }

        // Collision model
        for (const Collision& col : link->link.collision) {
            link->collision_mesh.push_back(col.geometry.type->generateGeometry());
            link->collision_models.push_back(LoadModelFromMesh(link->collision_mesh.back()));
        }

        for (const auto& joint : link->children) {
            deq.push_back(joint->child);
        }
    }
}

void Robot::draw(const LinkNodePtr& highlighted) const
{
    std::deque<LinkNodePtr> deq {root_};

    while (not deq.empty()) {
        const LinkNodePtr& link = deq.front();
        deq.pop_front();

        if (link->link.visual) {
            if (highlighted.get() == link.get()) {
                DrawModel(link->visual_model, Vector3Zero(), 1.0, RED);
            } else {
                DrawModel(link->visual_model, Vector3Zero(), 1.0, WHITE);
            }
        }

        for (const Model& model : link->collision_models) {
            DrawModelWires(model, Vector3Zero(), 1.0, LIGHTGRAY);
        }

        for (const auto& joint : link->children) {
            deq.push_back(joint->child);
        }
    }
}

void Robot::set_shader(const Shader& sh)
{
    visual_shader_ = sh;

    std::deque<LinkNodePtr> deq {root_};

    while (not deq.empty()) {
        const auto& current_link = deq.front();
        deq.pop_front();

        if (current_link->visual_model.materialCount > 0) {
            if (::Material *mat = current_link->visual_model.materials) {
                mat[0].shader = visual_shader_;
            }
        } else {
            LOG_F(WARNING, "Link %s visual model does not have materials", current_link->link.name.c_str());
            LOG_F(WARNING, "Link %s has %d meshes", current_link->link.name.c_str(), current_link->visual_model.meshCount);
        }

        for (const auto& joint : current_link->children) {
            deq.push_back(joint->child);
        }
    }
}

void Robot::print_tree() const
{
    std::deque<LinkNodePtr> deq {root_};

    while (not deq.empty()) {
        const auto& current_link = deq.front();
        deq.pop_front();

        LOG_F(INFO, "%s", current_link->link.name.c_str());

        for (const auto& joint : current_link->children) {
            deq.push_back(joint->child);
        }
    }
}

LinkNodePtr Robot::get_root(void) const
{
    return root_;
}

const std::map<std::string, Material>& Robot::get_materials(void) const
{
    return materials_;
}


}  // namespace urdf

