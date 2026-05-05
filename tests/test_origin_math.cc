#include <doctest/doctest.h>
#include <robot.h>

#include <cmath>

static bool matrixApproxEqual(const Matrix& a, const Matrix& b, float eps = 1e-5f) {
    const float* fa = &a.m0;
    const float* fb = &b.m0;
    for (int i = 0; i < 16; i++) {
        if (std::fabs(fa[i] - fb[i]) > eps) return false;
    }
    return true;
}

TEST_CASE("Identity origin produces identity matrix") {
    urdf::Origin o({0, 0, 0}, {0, 0, 0});
    Matrix m = o.toMatrix();
    CHECK(matrixApproxEqual(m, MatrixIdentity()));
}

TEST_CASE("Pure translation") {
    urdf::Origin o({1, 2, 3}, {0, 0, 0});
    Matrix m = o.toMatrix();

    CHECK(m.m12 == doctest::Approx(1.0f));
    CHECK(m.m13 == doctest::Approx(2.0f));
    CHECK(m.m14 == doctest::Approx(3.0f));

    // Rotation part should be identity
    CHECK(m.m0 == doctest::Approx(1.0f));
    CHECK(m.m5 == doctest::Approx(1.0f));
    CHECK(m.m10 == doctest::Approx(1.0f));
}

TEST_CASE("PosFromMatrix extracts position") {
    Matrix m = MatrixIdentity();
    m.m12 = 4.0f;
    m.m13 = 5.0f;
    m.m14 = 6.0f;

    Vector3 pos = urdf::PosFromMatrix(m);
    CHECK(pos.x == doctest::Approx(4.0f));
    CHECK(pos.y == doctest::Approx(5.0f));
    CHECK(pos.z == doctest::Approx(6.0f));
}

TEST_CASE("RPY round-trip through matrix") {
    Vector3 cases[] = {
        {0.1f, 0.2f, 0.3f},
        {-0.5f, 0.3f, 0.7f},
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 0.5f, 0.0f},
        {-0.3f, -0.2f, -0.1f},
    };

    for (const auto& rpy : cases) {
        CAPTURE(rpy.x);
        CAPTURE(rpy.y);
        CAPTURE(rpy.z);

        urdf::Origin o({0, 0, 0}, rpy);
        Matrix m = o.toMatrix();
        Vector3 recovered = urdf::MatrixToXYZ(m);

        CHECK(recovered.x == doctest::Approx(rpy.x).epsilon(1e-4));
        CHECK(recovered.y == doctest::Approx(rpy.y).epsilon(1e-4));
        CHECK(recovered.z == doctest::Approx(rpy.z).epsilon(1e-4));
    }
}

TEST_CASE("URDF RPY uses fixed-axis roll pitch yaw") {
    urdf::Origin o({0, 0, 0}, {PI / 2.0f, 0.0f, PI / 2.0f});
    Matrix m = o.toMatrix();

    Vector3 x_axis = Vector3Transform({1.0f, 0.0f, 0.0f}, m);
    Vector3 y_axis = Vector3Transform({0.0f, 1.0f, 0.0f}, m);
    Vector3 z_axis = Vector3Transform({0.0f, 0.0f, 1.0f}, m);

    CHECK(x_axis.x == doctest::Approx(0.0f).epsilon(1e-5));
    CHECK(x_axis.y == doctest::Approx(1.0f).epsilon(1e-5));
    CHECK(x_axis.z == doctest::Approx(0.0f).epsilon(1e-5));

    CHECK(y_axis.x == doctest::Approx(0.0f).epsilon(1e-5));
    CHECK(y_axis.y == doctest::Approx(0.0f).epsilon(1e-5));
    CHECK(y_axis.z == doctest::Approx(1.0f).epsilon(1e-5));

    CHECK(z_axis.x == doctest::Approx(1.0f).epsilon(1e-5));
    CHECK(z_axis.y == doctest::Approx(0.0f).epsilon(1e-5));
    CHECK(z_axis.z == doctest::Approx(0.0f).epsilon(1e-5));
}

TEST_CASE("Forward kinematics updates root visual transform") {
    auto root = std::make_shared<urdf::LinkNode>(urdf::Link("root"), nullptr);
    root->w_T_l = MatrixTranslate(1.0f, 2.0f, 3.0f);

    urdf::Visual visual;
    visual.origin = urdf::Origin(Vector3{0.25f, 0.5f, 0.75f}, Vector3{0.0f, 0.0f, 0.0f});
    root->link.visual.push_back(visual);
    root->visual_models.push_back(Model{});

    urdf::Robot::forwardKinematics(root);

    CHECK(root->visual_models[0].transform.m12 == doctest::Approx(1.25f));
    CHECK(root->visual_models[0].transform.m13 == doctest::Approx(2.5f));
    CHECK(root->visual_models[0].transform.m14 == doctest::Approx(3.75f));
}

TEST_CASE("Forward kinematics keeps Diablo wheel assemblies on correct sides") {
    auto root = std::make_shared<urdf::LinkNode>(urdf::Link("base_link"), nullptr);

    auto left_motor = std::make_shared<urdf::LinkNode>(urdf::Link("left_motor"), nullptr);
    auto left_j4 = std::make_shared<urdf::JointNode>(
        urdf::Joint("left_j4", "base_link", "left_motor", "continuous"), root, left_motor);
    left_j4->joint.origin = urdf::Origin(Vector3{0.0f, 0.18755f, 0.0f},
                                         Vector3{1.5708f, 0.13433f, -3.1416f});
    left_motor->parent = left_j4;
    root->children.push_back(left_j4);

    auto left_leg1 = std::make_shared<urdf::LinkNode>(urdf::Link("left_leg1"), nullptr);
    auto left_j1 = std::make_shared<urdf::JointNode>(
        urdf::Joint("left_j1", "left_motor", "left_leg1", "continuous"), left_motor, left_leg1);
    left_j1->joint.origin =
        urdf::Origin(Vector3{0.0f, 0.0f, 0.0f}, Vector3{0.0f, 0.0f, -2.8729f});
    left_leg1->parent = left_j1;
    left_motor->children.push_back(left_j1);

    auto left_leg2 = std::make_shared<urdf::LinkNode>(urdf::Link("left_leg2"), nullptr);
    auto left_j2 = std::make_shared<urdf::JointNode>(
        urdf::Joint("left_j2", "left_leg1", "left_leg2", "continuous"), left_leg1, left_leg2);
    left_j2->joint.origin =
        urdf::Origin(Vector3{0.14f, 0.0f, 0.0f}, Vector3{0.0f, 0.0f, 2.8729f});
    left_leg2->parent = left_j2;
    left_leg1->children.push_back(left_j2);

    auto left_wheel = std::make_shared<urdf::LinkNode>(urdf::Link("left_wheel"), nullptr);
    auto left_j3 = std::make_shared<urdf::JointNode>(
        urdf::Joint("left_j3", "left_leg2", "left_wheel", "continuous"), left_leg2, left_wheel);
    left_j3->joint.origin =
        urdf::Origin(Vector3{0.14f, 0.0f, 0.0537f}, Vector3{0.0f, 0.0f, 0.13433f});
    left_wheel->parent = left_j3;
    left_leg2->children.push_back(left_j3);

    auto right_motor = std::make_shared<urdf::LinkNode>(urdf::Link("right_motor"), nullptr);
    auto right_j4 = std::make_shared<urdf::JointNode>(
        urdf::Joint("right_j4", "base_link", "right_motor", "continuous"), root, right_motor);
    right_j4->joint.origin = urdf::Origin(Vector3{0.0f, -0.18755f, 0.0f},
                                          Vector3{1.5708f, 0.13433f, 3.1416f});
    right_motor->parent = right_j4;
    root->children.push_back(right_j4);

    auto right_leg1 = std::make_shared<urdf::LinkNode>(urdf::Link("right_leg1"), nullptr);
    auto right_j1 = std::make_shared<urdf::JointNode>(
        urdf::Joint("right_j1", "right_motor", "right_leg1", "continuous"), right_motor,
        right_leg1);
    right_j1->joint.origin =
        urdf::Origin(Vector3{0.0f, 0.0f, 0.0f}, Vector3{0.0f, 0.0f, -2.8729f});
    right_leg1->parent = right_j1;
    right_motor->children.push_back(right_j1);

    auto right_leg2 = std::make_shared<urdf::LinkNode>(urdf::Link("right_leg2"), nullptr);
    auto right_j2 = std::make_shared<urdf::JointNode>(
        urdf::Joint("right_j2", "right_leg1", "right_leg2", "continuous"), right_leg1,
        right_leg2);
    right_j2->joint.origin =
        urdf::Origin(Vector3{0.14f, 0.0f, 0.0f}, Vector3{0.0f, 0.0f, 2.8729f});
    right_leg2->parent = right_j2;
    right_leg1->children.push_back(right_j2);

    auto right_wheel = std::make_shared<urdf::LinkNode>(urdf::Link("right_wheel"), nullptr);
    auto right_j3 = std::make_shared<urdf::JointNode>(
        urdf::Joint("right_j3", "right_leg2", "right_wheel", "continuous"), right_leg2,
        right_wheel);
    right_j3->joint.origin =
        urdf::Origin(Vector3{0.14f, 0.0f, -0.0537f}, Vector3{0.0f, 0.0f, -3.0073f});
    right_wheel->parent = right_j3;
    right_leg2->children.push_back(right_j3);

    urdf::Robot::forwardKinematics(root);

    CHECK(left_wheel->w_T_l.m12 == doctest::Approx(0.0f).epsilon(1e-4));
    CHECK(left_wheel->w_T_l.m13 == doctest::Approx(0.2412f).epsilon(1e-4));
    CHECK(left_wheel->w_T_l.m14 == doctest::Approx(-0.0375f).epsilon(1e-4));

    CHECK(right_wheel->w_T_l.m12 == doctest::Approx(0.0f).epsilon(1e-4));
    CHECK(right_wheel->w_T_l.m13 == doctest::Approx(-0.2412f).epsilon(1e-4));
    CHECK(right_wheel->w_T_l.m14 == doctest::Approx(-0.0375f).epsilon(1e-4));
}
