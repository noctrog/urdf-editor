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
