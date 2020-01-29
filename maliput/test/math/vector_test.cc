#include "maliput/math/vector.h"

#include <gtest/gtest.h>

namespace maliput {
namespace math {
namespace {

GTEST_TEST(VectorTest, Constructors) {
  {  // 2- dimension vector.
    EXPECT_EQ(Vector2(0., 0.), Vector2());
    EXPECT_EQ(Vector2(5., 98.), Vector2({5., 98.}));
    EXPECT_EQ(Vector2(1., 2.), Vector2(std::array<double, 2>{1., 2.}));
    {
      const Vector2 kDut{3., 4.};
      Vector2 dut = kDut;
      EXPECT_EQ(kDut, dut);
    }
    {
      Vector2 dut = Vector2(45., 9.);
      EXPECT_EQ(dut, Vector2(45., 9.));
    }
  }
  {  // 3- dimension vector.
    EXPECT_EQ(Vector3(0., 0., 0.), Vector3());
    EXPECT_EQ(Vector3(5., 98., -35.), Vector3({5., 98., -35.}));
    EXPECT_EQ(Vector3(1., 2., 3.), Vector3(std::array<double, 3>{1., 2., 3.}));
  }
  {
    const Vector3 kDut{3., 4., 1.};
    Vector3 dut = kDut;
    EXPECT_EQ(kDut, dut);
  }
  {
    Vector3 dut = Vector3(45., 9., 35.);
    EXPECT_EQ(dut, Vector3(45., 9., 35.));
  }
  {  // 4- dimension vector.
    EXPECT_EQ(Vector4(0., 0., 0., 0.), Vector4());
    EXPECT_EQ(Vector4(5., 98., -35., 56.), Vector4({5., 98., -35., 56.}));
    EXPECT_EQ(Vector4(1., 2., 3., 4.), Vector4(std::array<double, 4>{1., 2., 3., 4.}));
  }
  {
    const Vector4 kDut{3., 4., 1., 10.};
    Vector4 dut = kDut;
    EXPECT_EQ(kDut, dut);
  }
  {
    Vector4 dut = Vector4(45., 9., 35., 27.);
    EXPECT_EQ(dut, Vector4(45., 9., 35., 27.));
  }
}

GTEST_TEST(VectorTest, PublicMethods) {
  {  // 2- dimension vector.
    const Vector2 kDut{3., 4.};
    EXPECT_EQ(kDut.x(), 3.);
    EXPECT_EQ(kDut.y(), 4.);
    EXPECT_EQ(kDut.norm(), 5.);
    EXPECT_EQ(kDut.normalized(), Vector2(3. / 5., 4. / 5.));
    Vector2 dut{3., 4.};
    dut.normalize();
    EXPECT_EQ(dut, Vector2(3. / 5., 4. / 5.));
    dut.x() = 5.;
    dut.y() = 10.;
    EXPECT_EQ(dut, Vector2(5., 10.));
  }
  {  // 3- dimension vector.
    const Vector3 kDut{2., 3., 6.};
    EXPECT_EQ(kDut.x(), 2.);
    EXPECT_EQ(kDut.y(), 3.);
    EXPECT_EQ(kDut.z(), 6.);
    EXPECT_EQ(kDut.norm(), 7.);
    EXPECT_EQ(kDut.normalized(), Vector3(2. / 7., 3. / 7., 6. / 7.));
    Vector3 dut{2., 3., 6.};
    dut.normalize();
    EXPECT_EQ(dut, Vector3(2. / 7., 3. / 7., 6. / 7.));
    dut.x() = 5.;
    dut.y() = 10.;
    dut.z() = 15.;
    EXPECT_EQ(dut, Vector3(5., 10., 15.));
  }
  {  // 4- dimension vector.
    const Vector4 kDut{1., 2., 3., 4.};
    EXPECT_EQ(kDut.x(), 1.);
    EXPECT_EQ(kDut.y(), 2.);
    EXPECT_EQ(kDut.z(), 3.);
    EXPECT_EQ(kDut.w(), 4.);
    const double kNorm = kDut.norm();
    EXPECT_EQ(kNorm, 5.477225575051661);
    EXPECT_EQ(kDut.normalized(), Vector4(1. / kNorm, 2. / kNorm, 3. / kNorm, 4. / kNorm));
    Vector4 dut{1., 2., 3., 4.};
    dut.normalize();
    EXPECT_EQ(dut, Vector4(1. / kNorm, 2. / kNorm, 3. / kNorm, 4. / kNorm));
    dut.x() = 5.;
    dut.y() = 10.;
    dut.z() = 15.;
    dut.w() = 20.;
    EXPECT_EQ(dut, Vector4(5., 10., 15., 20.));
  }
}

GTEST_TEST(VectorTest, Operators) {
  {  // 2- dimension vector.
    const Vector2 kDut{3., 4.};
    Vector2 dut = kDut;
    EXPECT_EQ(kDut, dut);
    dut = Vector2{1., 5.};
    EXPECT_EQ(dut, Vector2(1., 5.));
    dut = kDut;
    EXPECT_EQ(kDut, dut);
    EXPECT_EQ(kDut[0], 3.);
    EXPECT_EQ(kDut[1], 4.);
    dut[0] = 33;
    dut[1] = 44;
    EXPECT_EQ(dut[0], 33.);
    EXPECT_EQ(dut[1], 44.);
    EXPECT_NE(kDut, dut);
    EXPECT_EQ(kDut + dut, Vector2(36., 48.));
    EXPECT_EQ(dut - kDut, Vector2(30., 40.));
    EXPECT_EQ(kDut.dot(dut), 275.);
    EXPECT_EQ(kDut * 2, Vector2(6., 8.));
    EXPECT_EQ(2 * kDut, Vector2(6., 8.));
    EXPECT_EQ(kDut / 2, Vector2(1.5, 2.));
    std::stringstream ss;
    ss << kDut;
    EXPECT_EQ(ss.str(), "{3, 4}");
  }
  {  // 3- dimension vector.
    const Vector3 kDut{3., 4., 5.};
    Vector3 dut = kDut;
    EXPECT_EQ(kDut, dut);
    dut = Vector3{1., 5., 25.};
    EXPECT_EQ(dut, Vector3(1., 5., 25.));
    dut = kDut;
    EXPECT_EQ(kDut, dut);
    EXPECT_EQ(kDut[0], 3.);
    EXPECT_EQ(kDut[1], 4.);
    EXPECT_EQ(kDut[2], 5.);
    dut[0] = 33;
    dut[1] = 44;
    dut[2] = 55;
    EXPECT_EQ(dut[0], 33.);
    EXPECT_EQ(dut[1], 44.);
    EXPECT_EQ(dut[2], 55.);
    EXPECT_NE(kDut, dut);
    EXPECT_EQ(kDut + dut, Vector3(36., 48., 60.));
    EXPECT_EQ(dut - kDut, Vector3(30., 40., 50.));
    EXPECT_EQ(kDut.dot(dut), 550.);
    EXPECT_EQ(kDut * 2, Vector3(6., 8., 10.));
    EXPECT_EQ(2 * kDut, Vector3(6., 8., 10.));
    EXPECT_EQ(kDut / 2, Vector3(1.5, 2., 2.5));
    std::stringstream ss;
    ss << kDut;
    EXPECT_EQ(ss.str(), "{3, 4, 5}");
  }
  {  // 4- dimension vector.
    const Vector4 kDut{3., 4., 5., 6.};
    Vector4 dut = kDut;
    EXPECT_EQ(kDut, dut);
    dut = Vector4{1., 5., 25., -55.};
    EXPECT_EQ(dut, Vector4(1., 5., 25., -55.));
    dut = kDut;
    EXPECT_EQ(kDut, dut);
    EXPECT_EQ(kDut[0], 3.);
    EXPECT_EQ(kDut[1], 4.);
    EXPECT_EQ(kDut[2], 5.);
    EXPECT_EQ(kDut[3], 6.);
    dut[0] = 33;
    dut[1] = 44;
    dut[2] = 55;
    dut[3] = 66;
    EXPECT_EQ(dut[0], 33.);
    EXPECT_EQ(dut[1], 44.);
    EXPECT_EQ(dut[2], 55.);
    EXPECT_EQ(dut[3], 66.);
    EXPECT_NE(kDut, dut);
    EXPECT_EQ(kDut + dut, Vector4(36., 48., 60., 72.));
    EXPECT_EQ(dut - kDut, Vector4(30., 40., 50., 60.));
    EXPECT_EQ(kDut.dot(dut), 946.);
    EXPECT_EQ(kDut * 2, Vector4(6., 8., 10., 12.));
    EXPECT_EQ(2 * kDut, Vector4(6., 8., 10., 12.));
    EXPECT_EQ(kDut / 2, Vector4(1.5, 2., 2.5, 3.));
    std::stringstream ss;
    ss << kDut;
    EXPECT_EQ(ss.str(), "{3, 4, 5, 6}");
  }
}
GTEST_TEST(VectorTest, StaticMethods) {
  {  // 2- dimension vector.
    EXPECT_EQ(Vector2::Zero(), Vector2(0., 0.));
    EXPECT_EQ(Vector2::Ones(), Vector2(1., 1.));
    EXPECT_EQ(Vector2::UnitX(), Vector2(1., 0.));
    EXPECT_EQ(Vector2::UnitY(), Vector2(0., 1.));
  }
  {  // 3- dimension vector.
    EXPECT_EQ(Vector3::Zero(), Vector3(0., 0., 0.));
    EXPECT_EQ(Vector3::Ones(), Vector3(1., 1., 1.));
    EXPECT_EQ(Vector3::UnitX(), Vector3(1., 0., 0.));
    EXPECT_EQ(Vector3::UnitY(), Vector3(0., 1., 0.));
    EXPECT_EQ(Vector3::UnitZ(), Vector3(0., 0., 1.));
  }
  {  // 4- dimension vector.
    EXPECT_EQ(Vector4::Zero(), Vector4(0., 0., 0., 0.));
    EXPECT_EQ(Vector4::Ones(), Vector4(1., 1., 1., 1.));
    EXPECT_EQ(Vector4::UnitX(), Vector4(1., 0., 0., 0.));
    EXPECT_EQ(Vector4::UnitY(), Vector4(0., 1., 0., 0.));
    EXPECT_EQ(Vector4::UnitZ(), Vector4(0., 0., 1., 0.));
    EXPECT_EQ(Vector4::UnitW(), Vector4(0., 0., 0., 1.));
  }
}

}  // namespace
}  // namespace math
}  // namespace maliput
