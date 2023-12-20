#include <raylib.h>
#include <raymath.h>
#include <external/par_shapes.h>

#include <set>
#include <map>
#include <deque>

#include <urdf_parser.h>
#include <loguru.hpp>

#include <raymath.h>

namespace urdf {

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
        par_shapes_mesh* capTop = par_shapes_create_disk(radius,
                                                         slices,
                                                         (float[]){ 0, 0, height/2 },
                                                         (float[]){ 0, 0, 1 });
        capTop->tcoords = PAR_MALLOC(float, 2 * capTop->npoints);
        for (int i = 0; i < 2 * capTop->npoints; i++) capTop->tcoords[i] = 0.0f;

        // Create bottom cap
        par_shapes_mesh* capBottom = par_shapes_create_disk(radius,
                                                            slices,
                                                            (float[]){ 0, 0, 0 },
                                                            (float[]){ 0, 0, -1 });
        par_shapes_rotate(capBottom, -PI, (float[]){1, 0, 0});
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

Origin::Origin(const char *xyz_s, const char *rpy_s)
{
    xyz.x = xyz.y = xyz.z = 0.0f;
    rpy.x = rpy.y = rpy.z = 0.0f;

    if (xyz_s) store_vec3(xyz_s, xyz);
    if (rpy_s) store_vec3(rpy_s, rpy);

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
    store_vec3(s, xyz);
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

LinkNode::LinkNode()
    : T(MatrixIdentity())
{

}

LinkNode::LinkNode(const Link& link, const JointNodePtr& parent_joint)
    : link(link), parent(parent_joint), T(MatrixIdentity())
{

}

Parser::Parser(const char *urdf_file)
{
    pugi::xml_parse_result result = doc_.load_file(urdf_file);
    CHECK_F(result, "Failed to open %s", urdf_file);

    bool is_root_robot = doc_.root().name() != std::string("robot");
    CHECK_F(is_root_robot, "The urdf root name (%s) is not robot", doc_.root().name());
}

Parser::~Parser()
{

}

Robot Parser::build_robot(void)
{
    LOG_F(INFO, "Building robot...");

    pugi::xml_node root_link = find_root();
    CHECK_F(not root_link.empty(), "No root link found");

    auto tree_root = std::make_shared<LinkNode>();
    tree_root->link = xml_node_to_link(root_link);

    // Create all materials
    // TODO: support loading materials that are defined within the link
    std::map<std::string, Material> materials;
    for (pugi::xml_node& mat : doc_.child("robot").children("material")) {
        if (const auto m = xml_node_to_material(mat)) {
            materials.insert({m->name, *m});
        }
    }
    // TODO: every link uses the same shader, the material properties change the shader color.

    // Parent link to all child joints
    std::map<std::string, std::vector<Joint>> joint_p_hash;
    for (const pugi::xml_node& joint : doc_.child("robot").children("joint")) {
        Joint j = xml_node_to_joint(joint);
        if (joint_p_hash.find(j.parent) == joint_p_hash.end()) {
            joint_p_hash.insert({ j.parent, std::vector<Joint>({j}) });
        } else {
            joint_p_hash[j.parent].push_back(j);
        }
    }

    // Link name to link
    std::map<std::string, Link> link_hash;
    for (const pugi::xml_node& link : doc_.child("robot").children("link")) {
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

    Robot robot(tree_root, materials);

    LOG_F(INFO, "URDF Tree built successfully!");

    return robot;
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
        for (auto& joint_node : node->children) {
            const Matrix& w_T_p = node->T;

            // Forward kinematics
            const Matrix p_T_c = origin_to_matrix(joint_node->joint.origin);
            const Matrix w_T_c = MatrixMultiply(w_T_p, p_T_c);
            joint_node->child->T = w_T_c;

            // Update visual transform
            joint_node->child->visual_model.transform =
                origin_to_matrix(joint_node->child->link.visual->origin);

            // Update collision transform
            for (int i = 0; i < joint_node->child->collision_models.size(); ++i) {
                joint_node->child->collision_models[i].transform =
                    origin_to_matrix(joint_node->child->link.collision[i].origin);
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
        T = MatrixMultiply(
            MatrixRotateXYZ(origin->rpy),
            MatrixTranslate(origin->xyz.x, origin->xyz.y, origin->xyz.z)
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

void Robot::draw() const
{
    std::deque<LinkNodePtr> deq {root_};

    while (not deq.empty()) {
        const auto& link = deq.front();
        deq.pop_front();

        if (link->link.visual) {
            DrawModel(link->visual_model, Vector3Zero(), 1.0, WHITE);
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

        if (::Material *mat = current_link->visual_model.materials) {
            mat[0].shader = visual_shader_;
        }

        for (const auto& joint : current_link->children) {
            deq.push_back(joint->child);
        }
    }
}

void Robot::print_tree()
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


}  // namespace urdf

