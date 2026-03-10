#include <doctest/doctest.h>
#include <pugixml.hpp>
#include <robot.h>

TEST_CASE("Material round-trip (color)") {
    pugi::xml_document doc;
    doc.load_string(R"(<material name="red"><color rgba="1 0 0 1"/></material>)");
    pugi::xml_node node = doc.first_child();

    auto mat = urdf::xmlNodeToMaterial(node);
    REQUIRE(mat.has_value());
    CHECK(mat->name == "red");
    REQUIRE(mat->rgba.has_value());
    CHECK(mat->rgba->x == doctest::Approx(1.0f));
    CHECK(mat->rgba->y == doctest::Approx(0.0f));
    CHECK(mat->rgba->z == doctest::Approx(0.0f));
    CHECK(mat->rgba->w == doctest::Approx(1.0f));
    CHECK_FALSE(mat->texture_file.has_value());

    // Export
    pugi::xml_document out_doc;
    pugi::xml_node out_node = out_doc.append_child("material");
    urdf::materialToXmlNode(out_node, *mat);

    CHECK(std::string(out_node.attribute("name").as_string()) == "red");
    CHECK(std::string(out_node.child("color").attribute("rgba").as_string()) == "1 0 0 1");
}

TEST_CASE("Material with texture") {
    pugi::xml_document doc;
    doc.load_string(R"(<material name="tex_mat"><texture filename="foo.png"/></material>)");
    pugi::xml_node node = doc.first_child();

    auto mat = urdf::xmlNodeToMaterial(node);
    REQUIRE(mat.has_value());
    CHECK(mat->name == "tex_mat");
    CHECK_FALSE(mat->rgba.has_value());
    REQUIRE(mat->texture_file.has_value());
    CHECK(*mat->texture_file == "foo.png");
}

TEST_CASE("Origin round-trip") {
    pugi::xml_document doc;
    doc.load_string(R"(<origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>)");
    pugi::xml_node node = doc.first_child();

    auto origin = urdf::xmlNodeToOrigin(node);
    REQUIRE(origin.has_value());
    CHECK(origin->xyz.x == doctest::Approx(1.0f));
    CHECK(origin->xyz.y == doctest::Approx(2.0f));
    CHECK(origin->xyz.z == doctest::Approx(3.0f));
    CHECK(origin->rpy.x == doctest::Approx(0.1f));
    CHECK(origin->rpy.y == doctest::Approx(0.2f));
    CHECK(origin->rpy.z == doctest::Approx(0.3f));

    // Export and parse back
    pugi::xml_document out_doc;
    pugi::xml_node out_node = out_doc.append_child("origin");
    urdf::originToXmlNode(out_node, *origin);

    auto origin2 = urdf::xmlNodeToOrigin(out_node);
    REQUIRE(origin2.has_value());
    CHECK(origin2->xyz.x == doctest::Approx(origin->xyz.x));
    CHECK(origin2->xyz.y == doctest::Approx(origin->xyz.y));
    CHECK(origin2->xyz.z == doctest::Approx(origin->xyz.z));
    CHECK(origin2->rpy.x == doctest::Approx(origin->rpy.x));
    CHECK(origin2->rpy.y == doctest::Approx(origin->rpy.y));
    CHECK(origin2->rpy.z == doctest::Approx(origin->rpy.z));
}

TEST_CASE("Joint round-trip") {
    const char* xml = R"(
        <joint name="joint1" type="revolute">
            <parent link="base"/>
            <child link="arm"/>
            <origin xyz="0 0 1" rpy="0 0 0.5"/>
            <axis xyz="0 0 1"/>
            <dynamics damping="0.5" friction="0.1"/>
            <limit lower="-1.57" upper="1.57" effort="10" velocity="2"/>
        </joint>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);
    pugi::xml_node node = doc.first_child();

    urdf::Joint joint = urdf::xmlNodeToJoint(node);
    CHECK(joint.name == "joint1");
    CHECK(joint.type == urdf::Joint::kRevolute);
    CHECK(joint.parent == "base");
    CHECK(joint.child == "arm");

    REQUIRE(joint.origin.has_value());
    CHECK(joint.origin->xyz.z == doctest::Approx(1.0f));
    CHECK(joint.origin->rpy.z == doctest::Approx(0.5f));

    REQUIRE(joint.axis.has_value());
    CHECK(joint.axis->xyz.z == doctest::Approx(1.0f));

    REQUIRE(joint.dynamics.has_value());
    CHECK(joint.dynamics->damping == doctest::Approx(0.5f));
    CHECK(joint.dynamics->friction == doctest::Approx(0.1f));

    REQUIRE(joint.limit.has_value());
    CHECK(joint.limit->lower == doctest::Approx(-1.57f));
    CHECK(joint.limit->upper == doctest::Approx(1.57f));
    CHECK(joint.limit->effort == doctest::Approx(10.0f));
    CHECK(joint.limit->velocity == doctest::Approx(2.0f));

    // Export (jointToXmlNode writes the name attribute itself)
    pugi::xml_document out_doc;
    pugi::xml_node out_node = out_doc.append_child("joint");
    urdf::jointToXmlNode(out_node, joint);

    // Parse back
    urdf::Joint joint2 = urdf::xmlNodeToJoint(out_node);
    CHECK(joint2.name == "joint1");
    CHECK(joint2.type == urdf::Joint::kRevolute);
    CHECK(joint2.parent == "base");
    CHECK(joint2.child == "arm");
    REQUIRE(joint2.origin.has_value());
    CHECK(joint2.origin->xyz.z == doctest::Approx(1.0f));
    REQUIRE(joint2.axis.has_value());
    CHECK(joint2.axis->xyz.z == doctest::Approx(1.0f));
    REQUIRE(joint2.dynamics.has_value());
    CHECK(joint2.dynamics->damping == doctest::Approx(0.5f));
    REQUIRE(joint2.limit.has_value());
    CHECK(joint2.limit->lower == doctest::Approx(-1.57f));
}

TEST_CASE("Link with visual and collision") {
    const char* xml = R"(
        <link name="my_link">
            <visual>
                <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>
                <geometry>
                    <box size="1 2 3"/>
                </geometry>
                <material name="blue"/>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                    <box size="1 2 3"/>
                </geometry>
            </collision>
        </link>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);
    pugi::xml_node node = doc.first_child();

    urdf::Link link = urdf::xmlNodeToLink(node);
    CHECK(link.name == "my_link");

    REQUIRE(link.visual.size() == 1);
    REQUIRE(link.visual[0].origin.has_value());
    CHECK(link.visual[0].origin->xyz.x == doctest::Approx(0.1f));
    CHECK(link.visual[0].origin->xyz.y == doctest::Approx(0.2f));
    CHECK(link.visual[0].origin->xyz.z == doctest::Approx(0.3f));

    REQUIRE(link.visual[0].material_name.has_value());
    CHECK(*link.visual[0].material_name == "blue");

    auto box = std::dynamic_pointer_cast<urdf::Box>(link.visual[0].geometry.type);
    REQUIRE(box);
    CHECK(box->size.x == doctest::Approx(1.0f));
    CHECK(box->size.y == doctest::Approx(2.0f));
    CHECK(box->size.z == doctest::Approx(3.0f));

    REQUIRE(link.collision.size() == 1);

    // Export
    pugi::xml_document out_doc;
    pugi::xml_node out_node = out_doc.append_child("link");
    urdf::linkToXmlNode(out_node, link);

    // Parse back
    urdf::Link link2 = urdf::xmlNodeToLink(out_node);
    CHECK(link2.name == "my_link");
    REQUIRE(link2.visual.size() == 1);
    REQUIRE(link2.visual[0].material_name.has_value());
    CHECK(*link2.visual[0].material_name == "blue");
    CHECK(link2.collision.size() == 1);
}

TEST_CASE("jointToXmlNode writes name attribute") {
    urdf::Joint joint("my_joint", "parent_link", "child_link", "fixed");

    pugi::xml_document doc;
    pugi::xml_node node = doc.append_child("joint");
    urdf::jointToXmlNode(node, joint);

    CHECK(std::string(node.attribute("name").as_string()) == "my_joint");
    CHECK(std::string(node.attribute("type").as_string()) == "fixed");
}

TEST_CASE("jointToXmlNode name round-trips for all joint types") {
    const char* types[] = {"revolute", "continuous", "prismatic", "fixed", "floating", "planar"};

    for (const char* type : types) {
        CAPTURE(type);
        urdf::Joint joint("test_joint", "p", "c", type);

        pugi::xml_document doc;
        pugi::xml_node node = doc.append_child("joint");
        urdf::jointToXmlNode(node, joint);

        urdf::Joint parsed = urdf::xmlNodeToJoint(node);
        CHECK(parsed.name == "test_joint");
    }
}

TEST_CASE("findRoot identifies root link") {
    const char* xml = R"(
        <robot name="test_robot">
            <link name="base_link"/>
            <link name="child_link"/>
            <joint name="j1" type="fixed">
                <parent link="base_link"/>
                <child link="child_link"/>
            </joint>
        </robot>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);

    pugi::xml_node root = urdf::findRoot(doc);
    REQUIRE_FALSE(root.empty());
    CHECK(std::string(root.attribute("name").as_string()) == "base_link");
}

TEST_CASE("Joint mimic round-trip") {
    const char* xml = R"(
        <joint name="j1" type="revolute">
            <parent link="base"/>
            <child link="arm"/>
            <mimic joint="j0" multiplier="2.5" offset="0.1"/>
        </joint>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);
    urdf::Joint joint = urdf::xmlNodeToJoint(doc.first_child());

    REQUIRE(joint.mimic.has_value());
    CHECK(joint.mimic->joint == "j0");
    CHECK(joint.mimic->multiplier == doctest::Approx(2.5f));
    CHECK(joint.mimic->offset == doctest::Approx(0.1f));

    // Export and re-parse
    pugi::xml_document out_doc;
    pugi::xml_node out_node = out_doc.append_child("joint");
    urdf::jointToXmlNode(out_node, joint);

    urdf::Joint j2 = urdf::xmlNodeToJoint(out_node);
    REQUIRE(j2.mimic.has_value());
    CHECK(j2.mimic->joint == "j0");
    CHECK(j2.mimic->multiplier == doctest::Approx(2.5f));
    CHECK(j2.mimic->offset == doctest::Approx(0.1f));
}

TEST_CASE("Joint mimic defaults") {
    const char* xml = R"(
        <joint name="j1" type="revolute">
            <parent link="base"/>
            <child link="arm"/>
            <mimic joint="j0"/>
        </joint>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);
    urdf::Joint joint = urdf::xmlNodeToJoint(doc.first_child());

    REQUIRE(joint.mimic.has_value());
    CHECK(joint.mimic->joint == "j0");
    CHECK(joint.mimic->multiplier == doctest::Approx(1.0f));
    CHECK(joint.mimic->offset == doctest::Approx(0.0f));
}

TEST_CASE("Joint calibration round-trip") {
    const char* xml = R"(
        <joint name="j1" type="revolute">
            <parent link="base"/>
            <child link="arm"/>
            <calibration rising="1.5" falling="2.5"/>
        </joint>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);
    urdf::Joint joint = urdf::xmlNodeToJoint(doc.first_child());

    REQUIRE(joint.calibration.has_value());
    CHECK(joint.calibration->rising == doctest::Approx(1.5f));
    CHECK(joint.calibration->falling == doctest::Approx(2.5f));

    // Export and re-parse
    pugi::xml_document out_doc;
    pugi::xml_node out_node = out_doc.append_child("joint");
    urdf::jointToXmlNode(out_node, joint);

    urdf::Joint j2 = urdf::xmlNodeToJoint(out_node);
    REQUIRE(j2.calibration.has_value());
    CHECK(j2.calibration->rising == doctest::Approx(1.5f));
    CHECK(j2.calibration->falling == doctest::Approx(2.5f));
}

TEST_CASE("Joint safety_controller round-trip") {
    const char* xml = R"(
        <joint name="j1" type="revolute">
            <parent link="base"/>
            <child link="arm"/>
            <safety_controller k_velocity="10.5" k_position="20.0"
                               soft_lower_limit="-1.0" soft_upper_limit="1.0"/>
        </joint>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);
    urdf::Joint joint = urdf::xmlNodeToJoint(doc.first_child());

    REQUIRE(joint.safety_controller.has_value());
    CHECK(joint.safety_controller->k_velocity == doctest::Approx(10.5f));
    CHECK(joint.safety_controller->k_position == doctest::Approx(20.0f));
    CHECK(joint.safety_controller->soft_lower_limit == doctest::Approx(-1.0f));
    CHECK(joint.safety_controller->soft_upper_limit == doctest::Approx(1.0f));

    // Export and re-parse
    pugi::xml_document out_doc;
    pugi::xml_node out_node = out_doc.append_child("joint");
    urdf::jointToXmlNode(out_node, joint);

    urdf::Joint j2 = urdf::xmlNodeToJoint(out_node);
    REQUIRE(j2.safety_controller.has_value());
    CHECK(j2.safety_controller->k_velocity == doctest::Approx(10.5f));
    CHECK(j2.safety_controller->k_position == doctest::Approx(20.0f));
    CHECK(j2.safety_controller->soft_lower_limit == doctest::Approx(-1.0f));
    CHECK(j2.safety_controller->soft_upper_limit == doctest::Approx(1.0f));
}

TEST_CASE("Joint safety_controller absent by default") {
    const char* xml = R"(
        <joint name="j1" type="fixed">
            <parent link="base"/>
            <child link="arm"/>
        </joint>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);
    urdf::Joint joint = urdf::xmlNodeToJoint(doc.first_child());

    CHECK_FALSE(joint.safety_controller.has_value());
}

TEST_CASE("Mesh scale round-trip") {
    const char* xml = R"(
        <geometry>
            <mesh filename="robot.stl" scale="2 3 4"/>
        </geometry>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);
    urdf::Geometry geom = urdf::xmlNodeToGeometry(doc.first_child());

    auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geom.type);
    REQUIRE(mesh);
    CHECK(mesh->filename == "robot.stl");
    CHECK(mesh->scale.x == doctest::Approx(2.0f));
    CHECK(mesh->scale.y == doctest::Approx(3.0f));
    CHECK(mesh->scale.z == doctest::Approx(4.0f));

    // Export and re-parse
    pugi::xml_document out_doc;
    pugi::xml_node out_node = out_doc.append_child("geometry");
    urdf::geometryToXmlNode(out_node, geom);

    urdf::Geometry geom2 = urdf::xmlNodeToGeometry(out_node);
    auto mesh2 = std::dynamic_pointer_cast<urdf::Mesh>(geom2.type);
    REQUIRE(mesh2);
    CHECK(mesh2->scale.x == doctest::Approx(2.0f));
    CHECK(mesh2->scale.y == doctest::Approx(3.0f));
    CHECK(mesh2->scale.z == doctest::Approx(4.0f));
}

TEST_CASE("Mesh default scale not exported") {
    urdf::Geometry geom;
    geom.type = std::make_shared<urdf::Mesh>("test.stl");

    pugi::xml_document doc;
    pugi::xml_node node = doc.append_child("geometry");
    urdf::geometryToXmlNode(node, geom);

    // scale attribute should not be present when it's the default {1,1,1}
    CHECK_FALSE(node.child("mesh").attribute("scale"));
}

TEST_CASE("Multiple visuals per link") {
    const char* xml = R"(
        <link name="multi_vis_link">
            <visual>
                <geometry><box size="1 1 1"/></geometry>
                <material name="red"/>
            </visual>
            <visual>
                <origin xyz="0 0 1" rpy="0 0 0"/>
                <geometry><sphere radius="0.5"/></geometry>
                <material name="blue"/>
            </visual>
        </link>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);
    urdf::Link link = urdf::xmlNodeToLink(doc.first_child());

    REQUIRE(link.visual.size() == 2);

    auto box = std::dynamic_pointer_cast<urdf::Box>(link.visual[0].geometry.type);
    REQUIRE(box);
    CHECK(box->size.x == doctest::Approx(1.0f));
    REQUIRE(link.visual[0].material_name.has_value());
    CHECK(*link.visual[0].material_name == "red");

    auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(link.visual[1].geometry.type);
    REQUIRE(sphere);
    CHECK(sphere->radius == doctest::Approx(0.5f));
    REQUIRE(link.visual[1].origin.has_value());
    CHECK(link.visual[1].origin->xyz.z == doctest::Approx(1.0f));
    REQUIRE(link.visual[1].material_name.has_value());
    CHECK(*link.visual[1].material_name == "blue");

    // Export and re-parse
    pugi::xml_document out_doc;
    pugi::xml_node out_node = out_doc.append_child("link");
    urdf::linkToXmlNode(out_node, link);

    urdf::Link link2 = urdf::xmlNodeToLink(out_node);
    REQUIRE(link2.visual.size() == 2);
    CHECK(std::dynamic_pointer_cast<urdf::Box>(link2.visual[0].geometry.type));
    CHECK(std::dynamic_pointer_cast<urdf::Sphere>(link2.visual[1].geometry.type));
    REQUIRE(link2.visual[0].material_name.has_value());
    CHECK(*link2.visual[0].material_name == "red");
    REQUIRE(link2.visual[1].material_name.has_value());
    CHECK(*link2.visual[1].material_name == "blue");
}

TEST_CASE("Inline material promotion") {
    const char* xml = R"(
        <robot name="test">
            <link name="base_link">
                <visual>
                    <geometry><box size="1 1 1"/></geometry>
                    <material name="inline_red">
                        <color rgba="1 0 0 1"/>
                    </material>
                </visual>
            </link>
        </robot>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);

    // Simulate the inline material promotion logic from buildRobot
    std::map<std::string, urdf::Material> materials;
    for (const pugi::xml_node& link : doc.child("robot").children("link")) {
        for (const pugi::xml_node& vis : link.children("visual")) {
            const pugi::xml_node& mat_node = vis.child("material");
            if (mat_node && (mat_node.child("color") || mat_node.child("texture"))) {
                if (const auto m = urdf::xmlNodeToMaterial(mat_node)) {
                    materials.insert({m->name, *m});
                }
            }
        }
    }

    REQUIRE(materials.count("inline_red") == 1);
    REQUIRE(materials["inline_red"].rgba.has_value());
    CHECK(materials["inline_red"].rgba->x == doctest::Approx(1.0f));
    CHECK(materials["inline_red"].rgba->y == doctest::Approx(0.0f));
}

TEST_CASE("Inertial round-trip") {
    const char* xml = R"(
        <link name="test_link">
            <inertial>
                <origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>
                <mass value="5.0"/>
                <inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0.01" ixz="0.02" iyz="0.03"/>
            </inertial>
        </link>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);
    pugi::xml_node node = doc.first_child();

    urdf::Link link = urdf::xmlNodeToLink(node);
    REQUIRE(link.inertial.has_value());
    CHECK(link.inertial->mass == doctest::Approx(5.0f));
    CHECK(link.inertial->origin.xyz.x == doctest::Approx(1.0f));
    CHECK(link.inertial->origin.xyz.y == doctest::Approx(2.0f));
    CHECK(link.inertial->origin.xyz.z == doctest::Approx(3.0f));
    CHECK(link.inertial->origin.rpy.x == doctest::Approx(0.1f));
    CHECK(link.inertial->inertia.ixx == doctest::Approx(0.1f));
    CHECK(link.inertial->inertia.iyy == doctest::Approx(0.2f));
    CHECK(link.inertial->inertia.izz == doctest::Approx(0.3f));
    CHECK(link.inertial->inertia.ixy == doctest::Approx(0.01f));
    CHECK(link.inertial->inertia.ixz == doctest::Approx(0.02f));
    CHECK(link.inertial->inertia.iyz == doctest::Approx(0.03f));
}

// --- Validation tests ---

// Returns true if any validation message matches the given level and contains the substring.
static bool hasMessage(const std::vector<urdf::ValidationMessage>& msgs,
                       urdf::ValidationMessage::Level level, const std::string& substring) {
    for (const auto& m : msgs) {
        if (m.level == level && m.message.find(substring) != std::string::npos) return true;
    }
    return false;
}

// Build a minimal robot with a single root link and one child joint+link.
static urdf::RobotPtr buildTwoLinkRobot(const urdf::Joint& joint) {
    auto root = std::make_shared<urdf::LinkNode>(urdf::Link("base_link"), nullptr);
    auto child = std::make_shared<urdf::LinkNode>(urdf::Link("child_link"), nullptr);
    auto joint_node = std::make_shared<urdf::JointNode>(joint, root, child);
    child->parent = joint_node;
    root->children.push_back(joint_node);
    return std::make_shared<urdf::Robot>(root);
}

TEST_CASE("Validate: revolute joint missing limit is error") {
    auto robot = buildTwoLinkRobot(urdf::Joint("j1", "base_link", "child_link", "revolute"));
    auto msgs = robot->validate();
    CHECK(hasMessage(msgs, urdf::ValidationMessage::kError, "limit"));
}

TEST_CASE("Validate: non-unit axis is warning") {
    urdf::Joint j("j1", "base_link", "child_link", "fixed");
    j.axis = urdf::Axis(Vector3{2.0f, 0.0f, 0.0f});
    auto robot = buildTwoLinkRobot(j);
    auto msgs = robot->validate();
    CHECK(hasMessage(msgs, urdf::ValidationMessage::kWarning, "unit"));
}

TEST_CASE("Validate: valid robot has no messages") {
    auto root = std::make_shared<urdf::LinkNode>(urdf::Link("base_link"), nullptr);
    auto robot = std::make_shared<urdf::Robot>(root);
    auto msgs = robot->validate();
    CHECK(msgs.empty());
}

TEST_CASE("Validate: duplicate link names") {
    auto root = std::make_shared<urdf::LinkNode>(urdf::Link("base_link"), nullptr);
    auto child = std::make_shared<urdf::LinkNode>(urdf::Link("base_link"), nullptr);
    auto joint_node = std::make_shared<urdf::JointNode>(
        urdf::Joint("j1", "base_link", "base_link", "fixed"), root, child);
    child->parent = joint_node;
    root->children.push_back(joint_node);

    auto robot = std::make_shared<urdf::Robot>(root);
    auto msgs = robot->validate();
    CHECK(hasMessage(msgs, urdf::ValidationMessage::kError, "Duplicate link"));
}

TEST_CASE("Validate: limit lower >= upper is warning") {
    urdf::Joint j("j1", "base_link", "child_link", "revolute");
    j.limit = urdf::Limit(1.0f, 0.5f, 10.0f, 2.0f);  // lower > upper
    auto robot = buildTwoLinkRobot(j);
    auto msgs = robot->validate();
    CHECK(hasMessage(msgs, urdf::ValidationMessage::kWarning, "lower"));
}

TEST_CASE("Inline material reference-only not promoted") {
    const char* xml = R"(
        <robot name="test">
            <link name="base_link">
                <visual>
                    <geometry><box size="1 1 1"/></geometry>
                    <material name="existing_mat"/>
                </visual>
            </link>
        </robot>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);

    std::map<std::string, urdf::Material> materials;
    for (const pugi::xml_node& link : doc.child("robot").children("link")) {
        for (const pugi::xml_node& vis : link.children("visual")) {
            const pugi::xml_node& mat_node = vis.child("material");
            if (mat_node && (mat_node.child("color") || mat_node.child("texture"))) {
                if (const auto m = urdf::xmlNodeToMaterial(mat_node)) {
                    materials.insert({m->name, *m});
                }
            }
        }
    }

    // Reference-only materials (no color/texture children) should not be promoted
    CHECK(materials.empty());
}
