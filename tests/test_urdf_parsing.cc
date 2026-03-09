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

    // Export
    pugi::xml_document out_doc;
    pugi::xml_node out_node = out_doc.append_child("joint");
    out_node.append_attribute("name") = joint.name.c_str();
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

    REQUIRE(link.visual.has_value());
    REQUIRE(link.visual->origin.has_value());
    CHECK(link.visual->origin->xyz.x == doctest::Approx(0.1f));
    CHECK(link.visual->origin->xyz.y == doctest::Approx(0.2f));
    CHECK(link.visual->origin->xyz.z == doctest::Approx(0.3f));

    REQUIRE(link.visual->material_name.has_value());
    CHECK(*link.visual->material_name == "blue");

    auto box = std::dynamic_pointer_cast<urdf::Box>(link.visual->geometry.type);
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
    REQUIRE(link2.visual.has_value());
    REQUIRE(link2.visual->material_name.has_value());
    CHECK(*link2.visual->material_name == "blue");
    CHECK(link2.collision.size() == 1);
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
