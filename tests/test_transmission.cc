#include <doctest/doctest.h>
#include <pugixml.hpp>
#include <robot.h>

#include <sstream>

// Helper: build a minimal Robot with one root link and set transmissions.
static urdf::RobotPtr buildRobotWithTransmissions(
    const std::vector<std::string>& transmissions) {
    auto root = std::make_shared<urdf::LinkNode>(urdf::Link("base_link"), nullptr);
    auto robot = std::make_shared<urdf::Robot>(root);
    robot->setTransmissions(transmissions);
    return robot;
}

// Helper: append stored transmission XML strings as child elements of the given robot node.
static void appendTransmissions(pugi::xml_node& robot_root,
                                const std::vector<std::string>& transmissions) {
    for (const auto& tx : transmissions) {
        pugi::xml_document tx_doc;
        tx_doc.load_string(tx.c_str());
        if (tx_doc.first_child()) {
            robot_root.append_copy(tx_doc.first_child());
        }
    }
}

TEST_CASE("Transmission storage: set and get") {
    auto root = std::make_shared<urdf::LinkNode>(urdf::Link("base_link"), nullptr);
    auto robot = std::make_shared<urdf::Robot>(root);

    CHECK(robot->getTransmissions().empty());

    std::vector<std::string> txs = {"<transmission name=\"t1\"/>", "<transmission name=\"t2\"/>"};
    robot->setTransmissions(txs);

    REQUIRE(robot->getTransmissions().size() == 2);
    CHECK(robot->getTransmissions()[0] == txs[0]);
    CHECK(robot->getTransmissions()[1] == txs[1]);
}

TEST_CASE("Transmission export round-trip: single transmission") {
    std::string tx_xml = "<transmission name=\"trans1\">\n"
                         "\t<type>transmission_interface/SimpleTransmission</type>\n"
                         "\t<joint name=\"joint1\">\n"
                         "\t\t<hardwareInterface>hardware_interface/EffortJointInterface"
                         "</hardwareInterface>\n"
                         "\t</joint>\n"
                         "\t<actuator name=\"motor1\">\n"
                         "\t\t<mechanicalReduction>50</mechanicalReduction>\n"
                         "\t</actuator>\n"
                         "</transmission>\n";

    auto robot = buildRobotWithTransmissions({tx_xml});

    // Export the robot to XML
    // We test the export logic by reproducing what exportRobot does
    pugi::xml_document out_doc;
    pugi::xml_node robot_root = out_doc.append_child("robot");

    // Export links
    pugi::xml_node link_node = robot_root.append_child("link");
    urdf::linkToXmlNode(link_node, robot->getRoot()->link);

    // Export transmissions
    appendTransmissions(robot_root, robot->getTransmissions());

    // Verify the transmission element exists in the output
    pugi::xml_node tx_node = robot_root.child("transmission");
    REQUIRE_FALSE(tx_node.empty());
    CHECK(std::string(tx_node.attribute("name").as_string()) == "trans1");
    CHECK_FALSE(tx_node.child("type").empty());
    CHECK(std::string(tx_node.child("type").child_value()) ==
          "transmission_interface/SimpleTransmission");
    CHECK(std::string(tx_node.child("joint").attribute("name").as_string()) == "joint1");
    CHECK(std::string(tx_node.child("actuator").attribute("name").as_string()) == "motor1");
}

TEST_CASE("Transmission export round-trip: multiple transmissions") {
    std::vector<std::string> txs = {
        "<transmission name=\"t1\"><type>SimpleTransmission</type></transmission>\n",
        "<transmission name=\"t2\"><type>DifferentialTransmission</type></transmission>\n",
    };

    auto robot = buildRobotWithTransmissions(txs);

    pugi::xml_document out_doc;
    pugi::xml_node robot_root = out_doc.append_child("robot");
    pugi::xml_node link_node = robot_root.append_child("link");
    urdf::linkToXmlNode(link_node, robot->getRoot()->link);

    appendTransmissions(robot_root, robot->getTransmissions());

    // Count transmission children
    int count = 0;
    for (auto child = robot_root.child("transmission"); child;
         child = child.next_sibling("transmission")) {
        count++;
    }
    CHECK(count == 2);

    // Check names
    pugi::xml_node first = robot_root.child("transmission");
    CHECK(std::string(first.attribute("name").as_string()) == "t1");
    pugi::xml_node second = first.next_sibling("transmission");
    CHECK(std::string(second.attribute("name").as_string()) == "t2");
}

TEST_CASE("Transmission export: no transmissions produces no elements") {
    auto robot = buildRobotWithTransmissions({});

    pugi::xml_document out_doc;
    pugi::xml_node robot_root = out_doc.append_child("robot");
    pugi::xml_node link_node = robot_root.append_child("link");
    urdf::linkToXmlNode(link_node, robot->getRoot()->link);

    appendTransmissions(robot_root, robot->getTransmissions());

    CHECK(robot_root.child("transmission").empty());
}

TEST_CASE("Transmission parse: extract from URDF XML") {
    const char* xml = R"(
        <robot name="test_robot">
            <link name="base_link"/>
            <transmission name="trans1">
                <type>transmission_interface/SimpleTransmission</type>
                <joint name="joint1">
                    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
                </joint>
                <actuator name="motor1">
                    <mechanicalReduction>50</mechanicalReduction>
                </actuator>
            </transmission>
            <transmission name="trans2">
                <type>transmission_interface/SimpleTransmission</type>
                <joint name="joint2">
                    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
                </joint>
                <actuator name="motor2">
                    <mechanicalReduction>100</mechanicalReduction>
                </actuator>
            </transmission>
        </robot>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);

    // Simulate the parsing logic from buildRobot
    std::vector<std::string> transmissions;
    for (const pugi::xml_node& tx : doc.child("robot").children("transmission")) {
        std::ostringstream oss;
        tx.print(oss);
        transmissions.push_back(oss.str());
    }

    REQUIRE(transmissions.size() == 2);

    // Verify first transmission can be re-parsed
    pugi::xml_document tx1_doc;
    tx1_doc.load_string(transmissions[0].c_str());
    CHECK(std::string(tx1_doc.first_child().attribute("name").as_string()) == "trans1");

    // Verify second transmission
    pugi::xml_document tx2_doc;
    tx2_doc.load_string(transmissions[1].c_str());
    CHECK(std::string(tx2_doc.first_child().attribute("name").as_string()) == "trans2");
}

TEST_CASE("Transmission parse: no transmissions in URDF") {
    const char* xml = R"(
        <robot name="test_robot">
            <link name="base_link"/>
        </robot>
    )";

    pugi::xml_document doc;
    doc.load_string(xml);

    std::vector<std::string> transmissions;
    for (const pugi::xml_node& tx : doc.child("robot").children("transmission")) {
        std::ostringstream oss;
        tx.print(oss);
        transmissions.push_back(oss.str());
    }

    CHECK(transmissions.empty());
}

TEST_CASE("Transmission full round-trip: parse then export preserves content") {
    const char* xml = R"(
        <robot name="test_robot">
            <link name="base_link"/>
            <transmission name="trans1">
                <type>transmission_interface/SimpleTransmission</type>
                <joint name="joint1">
                    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
                </joint>
                <actuator name="motor1">
                    <mechanicalReduction>50</mechanicalReduction>
                </actuator>
            </transmission>
        </robot>
    )";

    // Parse
    pugi::xml_document doc;
    doc.load_string(xml);

    std::vector<std::string> transmissions;
    for (const pugi::xml_node& tx : doc.child("robot").children("transmission")) {
        std::ostringstream oss;
        tx.print(oss);
        transmissions.push_back(oss.str());
    }
    REQUIRE(transmissions.size() == 1);

    // Store in robot
    auto root = std::make_shared<urdf::LinkNode>(urdf::Link("base_link"), nullptr);
    auto robot = std::make_shared<urdf::Robot>(root);
    robot->setTransmissions(std::move(transmissions));

    // Export
    pugi::xml_document out_doc;
    pugi::xml_node robot_root = out_doc.append_child("robot");
    pugi::xml_node link_node = robot_root.append_child("link");
    link_node.append_attribute("name") = "base_link";

    appendTransmissions(robot_root, robot->getTransmissions());

    // Verify the exported transmission is structurally identical
    pugi::xml_node out_tx = robot_root.child("transmission");
    REQUIRE_FALSE(out_tx.empty());
    CHECK(std::string(out_tx.attribute("name").as_string()) == "trans1");
    CHECK(std::string(out_tx.child("type").child_value()) ==
          "transmission_interface/SimpleTransmission");
    CHECK(std::string(out_tx.child("joint").attribute("name").as_string()) == "joint1");
    CHECK(std::string(out_tx.child("joint").child("hardwareInterface").child_value()) ==
          "hardware_interface/EffortJointInterface");
    CHECK(std::string(out_tx.child("actuator").attribute("name").as_string()) == "motor1");
    CHECK(std::string(out_tx.child("actuator").child("mechanicalReduction").child_value()) == "50");
}
