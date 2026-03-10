#include <command.h>
#include <doctest/doctest.h>
#include <robot.h>

// Helper: build a simple robot tree:
//   root_link -> joint1 -> child1 -> joint2 -> grandchild
static auto makeTestRobot() {
    auto root = std::make_shared<urdf::LinkNode>(urdf::Link("root"), nullptr);

    auto child1 = std::make_shared<urdf::LinkNode>(urdf::Link("child1"), nullptr);
    auto joint1 = std::make_shared<urdf::JointNode>(
        urdf::Joint("joint1", "root", "child1", "revolute"), root, child1);
    child1->parent = joint1;
    root->children.push_back(joint1);

    auto grandchild = std::make_shared<urdf::LinkNode>(urdf::Link("grandchild"), nullptr);
    auto joint2 = std::make_shared<urdf::JointNode>(
        urdf::Joint("joint2", "child1", "grandchild", "fixed"), child1, grandchild);
    grandchild->parent = joint2;
    child1->children.push_back(joint2);

    auto robot = std::make_shared<urdf::Robot>(root);
    return std::make_tuple(robot, root, joint1, child1, joint2, grandchild);
}

TEST_CASE("DeleteJointCommand removes joint and subtree") {
    auto [robot, root, joint1, child1, joint2, grandchild] = makeTestRobot();
    urdf::TreeNodePtr selected = joint1;
    urdf::OriginRawPtr origin_ptr = nullptr;

    DeleteJointCommand cmd(joint1, robot, selected, origin_ptr);
    cmd.execute();

    CHECK(root->children.empty());
    CHECK(selected == nullptr);
}

TEST_CASE("DeleteJointCommand undo restores subtree") {
    auto [robot, root, joint1, child1, joint2, grandchild] = makeTestRobot();
    urdf::TreeNodePtr selected = joint1;
    urdf::OriginRawPtr origin_ptr = nullptr;

    DeleteJointCommand cmd(joint1, robot, selected, origin_ptr);
    cmd.execute();

    CHECK(root->children.empty());

    cmd.undo();

    REQUIRE(root->children.size() == 1);
    CHECK(root->children[0] == joint1);
    // The entire subtree is intact
    CHECK(joint1->child == child1);
    CHECK(child1->children.size() == 1);
    CHECK(child1->children[0] == joint2);
    CHECK(joint2->child == grandchild);
    // Selection restored to the joint
    CHECK(selected == joint1);
}

TEST_CASE("DeleteJointCommand preserves sibling order") {
    // root -> joint_a -> link_a
    //      -> joint_b -> link_b
    //      -> joint_c -> link_c
    auto root = std::make_shared<urdf::LinkNode>(urdf::Link("root"), nullptr);

    auto link_a = std::make_shared<urdf::LinkNode>(urdf::Link("link_a"), nullptr);
    auto joint_a = std::make_shared<urdf::JointNode>(
        urdf::Joint("joint_a", "root", "link_a", "fixed"), root, link_a);
    link_a->parent = joint_a;
    root->children.push_back(joint_a);

    auto link_b = std::make_shared<urdf::LinkNode>(urdf::Link("link_b"), nullptr);
    auto joint_b = std::make_shared<urdf::JointNode>(
        urdf::Joint("joint_b", "root", "link_b", "fixed"), root, link_b);
    link_b->parent = joint_b;
    root->children.push_back(joint_b);

    auto link_c = std::make_shared<urdf::LinkNode>(urdf::Link("link_c"), nullptr);
    auto joint_c = std::make_shared<urdf::JointNode>(
        urdf::Joint("joint_c", "root", "link_c", "fixed"), root, link_c);
    link_c->parent = joint_c;
    root->children.push_back(joint_c);

    auto robot = std::make_shared<urdf::Robot>(root);
    urdf::TreeNodePtr selected = joint_b;
    urdf::OriginRawPtr origin_ptr = nullptr;

    // Delete the middle joint
    DeleteJointCommand cmd(joint_b, robot, selected, origin_ptr);
    cmd.execute();

    REQUIRE(root->children.size() == 2);
    CHECK(root->children[0] == joint_a);
    CHECK(root->children[1] == joint_c);

    // Undo should restore joint_b in its original position (index 1)
    cmd.undo();

    REQUIRE(root->children.size() == 3);
    CHECK(root->children[0] == joint_a);
    CHECK(root->children[1] == joint_b);
    CHECK(root->children[2] == joint_c);
}

TEST_CASE("DeleteJointCommand clears origin pointer") {
    auto [robot, root, joint1, child1, joint2, grandchild] = makeTestRobot();
    urdf::TreeNodePtr selected = joint1;

    // Simulate an origin pointer pointing into the subtree
    urdf::Origin dummy_origin;
    urdf::OriginRawPtr origin_ptr = &dummy_origin;

    DeleteJointCommand cmd(joint1, robot, selected, origin_ptr);
    cmd.execute();

    CHECK(origin_ptr == nullptr);
}

TEST_CASE("DeleteJointCommand execute-undo-execute round-trip") {
    auto [robot, root, joint1, child1, joint2, grandchild] = makeTestRobot();
    urdf::TreeNodePtr selected = joint1;
    urdf::OriginRawPtr origin_ptr = nullptr;

    DeleteJointCommand cmd(joint1, robot, selected, origin_ptr);

    cmd.execute();
    CHECK(root->children.empty());

    cmd.undo();
    REQUIRE(root->children.size() == 1);
    CHECK(root->children[0] == joint1);

    // Re-execute should delete again
    cmd.execute();
    CHECK(root->children.empty());
    CHECK(selected == nullptr);
}
