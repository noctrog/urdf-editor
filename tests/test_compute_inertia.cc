#include <doctest/doctest.h>
#include <robot.h>

TEST_CASE("computeInertia: box") {
    float mass = 2.0f;
    auto box = std::make_shared<urdf::Box>(Vector3{3.0f, 4.0f, 5.0f});

    urdf::Inertia inertia = urdf::computeInertia(mass, box);

    // Ixx = m/12 * (y^2 + z^2) = 2/12 * (16 + 25) = 2/12 * 41
    float k = mass / 12.0f;
    CHECK(inertia.ixx == doctest::Approx(k * (16.0f + 25.0f)));
    // Iyy = m/12 * (x^2 + z^2) = 2/12 * (9 + 25)
    CHECK(inertia.iyy == doctest::Approx(k * (9.0f + 25.0f)));
    // Izz = m/12 * (x^2 + y^2) = 2/12 * (9 + 16)
    CHECK(inertia.izz == doctest::Approx(k * (9.0f + 16.0f)));

    CHECK(inertia.ixy == doctest::Approx(0.0f));
    CHECK(inertia.ixz == doctest::Approx(0.0f));
    CHECK(inertia.iyz == doctest::Approx(0.0f));
}

TEST_CASE("computeInertia: unit cube mass 1") {
    float mass = 1.0f;
    auto box = std::make_shared<urdf::Box>(Vector3{1.0f, 1.0f, 1.0f});

    urdf::Inertia inertia = urdf::computeInertia(mass, box);

    // For a unit cube with mass 1: Ixx = Iyy = Izz = 1/12 * (1+1) = 1/6
    float expected = 1.0f / 6.0f;
    CHECK(inertia.ixx == doctest::Approx(expected));
    CHECK(inertia.iyy == doctest::Approx(expected));
    CHECK(inertia.izz == doctest::Approx(expected));
}

TEST_CASE("computeInertia: cylinder") {
    float mass = 3.0f;
    auto cyl = std::make_shared<urdf::Cylinder>();
    cyl->radius = 0.5f;
    cyl->length = 2.0f;

    urdf::Inertia inertia = urdf::computeInertia(mass, cyl);

    float r2 = 0.25f;
    float h2 = 4.0f;
    // Ixx = Iyy = m/12 * (3*r^2 + h^2)
    float ixx_expected = mass / 12.0f * (3.0f * r2 + h2);
    // Izz = m * r^2 / 2
    float izz_expected = mass * r2 / 2.0f;

    CHECK(inertia.ixx == doctest::Approx(ixx_expected));
    CHECK(inertia.iyy == doctest::Approx(ixx_expected));
    CHECK(inertia.izz == doctest::Approx(izz_expected));

    CHECK(inertia.ixy == doctest::Approx(0.0f));
    CHECK(inertia.ixz == doctest::Approx(0.0f));
    CHECK(inertia.iyz == doctest::Approx(0.0f));
}

TEST_CASE("computeInertia: sphere") {
    float mass = 5.0f;
    auto sphere = std::make_shared<urdf::Sphere>(2.0f);

    urdf::Inertia inertia = urdf::computeInertia(mass, sphere);

    // I = 2/5 * m * r^2 = 2/5 * 5 * 4 = 8
    float expected = 2.0f / 5.0f * mass * 4.0f;
    CHECK(inertia.ixx == doctest::Approx(expected));
    CHECK(inertia.iyy == doctest::Approx(expected));
    CHECK(inertia.izz == doctest::Approx(expected));

    CHECK(inertia.ixy == doctest::Approx(0.0f));
    CHECK(inertia.ixz == doctest::Approx(0.0f));
    CHECK(inertia.iyz == doctest::Approx(0.0f));
}

TEST_CASE("computeInertia: unit sphere mass 1") {
    float mass = 1.0f;
    auto sphere = std::make_shared<urdf::Sphere>(1.0f);

    urdf::Inertia inertia = urdf::computeInertia(mass, sphere);

    float expected = 0.4f;  // 2/5 * 1 * 1
    CHECK(inertia.ixx == doctest::Approx(expected));
    CHECK(inertia.iyy == doctest::Approx(expected));
    CHECK(inertia.izz == doctest::Approx(expected));
}

TEST_CASE("computeInertia: mesh returns zeros") {
    float mass = 10.0f;
    auto mesh = std::make_shared<urdf::Mesh>("robot.stl");

    urdf::Inertia inertia = urdf::computeInertia(mass, mesh);

    CHECK(inertia.ixx == doctest::Approx(0.0f));
    CHECK(inertia.iyy == doctest::Approx(0.0f));
    CHECK(inertia.izz == doctest::Approx(0.0f));
    CHECK(inertia.ixy == doctest::Approx(0.0f));
    CHECK(inertia.ixz == doctest::Approx(0.0f));
    CHECK(inertia.iyz == doctest::Approx(0.0f));
}

TEST_CASE("computeInertia: zero mass gives zero inertia") {
    auto box = std::make_shared<urdf::Box>(Vector3{1.0f, 2.0f, 3.0f});

    urdf::Inertia inertia = urdf::computeInertia(0.0f, box);

    CHECK(inertia.ixx == doctest::Approx(0.0f));
    CHECK(inertia.iyy == doctest::Approx(0.0f));
    CHECK(inertia.izz == doctest::Approx(0.0f));
}

TEST_CASE("computeInertia: box symmetry") {
    // A cube should have equal inertia on all axes
    float mass = 6.0f;
    float side = 2.0f;
    auto cube = std::make_shared<urdf::Box>(Vector3{side, side, side});

    urdf::Inertia inertia = urdf::computeInertia(mass, cube);

    CHECK(inertia.ixx == doctest::Approx(inertia.iyy));
    CHECK(inertia.iyy == doctest::Approx(inertia.izz));
}

TEST_CASE("computeInertia: cylinder Izz < Ixx for tall cylinder") {
    // For a tall thin cylinder, axial moment (Izz) should be less than transverse (Ixx)
    float mass = 1.0f;
    auto cyl = std::make_shared<urdf::Cylinder>();
    cyl->radius = 0.1f;
    cyl->length = 5.0f;

    urdf::Inertia inertia = urdf::computeInertia(mass, cyl);

    CHECK(inertia.izz < inertia.ixx);
    CHECK(inertia.ixx == doctest::Approx(inertia.iyy));
}
