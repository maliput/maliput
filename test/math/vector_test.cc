// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "maliput/math/vector.h"

#include <gtest/gtest.h>

namespace maliput {
namespace math {
namespace {

GTEST_TEST(VectorTest, Constructors) {
  {  // N dimension vector
    EXPECT_EQ(Vector<2>({0., 0.}), Vector<2>());
    EXPECT_EQ(Vector<2>({5., 98.}), Vector<2>({5., 98.}));
    EXPECT_EQ(Vector<2>({1., 2.}), Vector<2>(std::array<double, 2>{1., 2.}));
    EXPECT_EQ(static_cast<std::size_t>(2), Vector<2>::kDimension);
  }
  {  // 2- dimension vector.
    EXPECT_EQ(Vector2(0., 0.), Vector2());
    EXPECT_EQ(Vector2(5., 98.), Vector2({5., 98.}));
    EXPECT_EQ(Vector2(1., 2.), Vector2(std::array<double, 2>{1., 2.}));
    EXPECT_EQ(static_cast<std::size_t>(2), Vector2::kDimension);
    {
      const Vector2 kExpected{3., 4.};
      const Vector2 kDut = kExpected;
      EXPECT_EQ(kDut, kExpected);
    }
    {
      const Vector2 kDut = Vector2(45., 9.);
      EXPECT_EQ(kDut, Vector2(45., 9.));
    }
    EXPECT_EQ(Vector2(Vector<2>({2., 7.})).y(), 7.);
  }
  {  // 3- dimension vector.
    EXPECT_EQ(Vector3(0., 0., 0.), Vector3());
    EXPECT_EQ(Vector3(5., 98., -35.), Vector3({5., 98., -35.}));
    EXPECT_EQ(Vector3(1., 2., 3.), Vector3(std::array<double, 3>{1., 2., 3.}));
    EXPECT_EQ(static_cast<std::size_t>(3), Vector3::kDimension);
    {
      const Vector3 kExpected{3., 4., 1.};
      const Vector3 kDut = kExpected;
      EXPECT_EQ(kDut, kExpected);
    }
    {
      const Vector3 kDut = Vector3(45., 9., 35.);
      EXPECT_EQ(kDut, Vector3(45., 9., 35.));
    }
    EXPECT_EQ(Vector3(Vector<3>({2., 7., 14.})).z(), 14.);
  }
  {  // 4- dimension vector.
    EXPECT_EQ(Vector4(0., 0., 0., 0.), Vector4());
    EXPECT_EQ(Vector4(5., 98., -35., 56.), Vector4({5., 98., -35., 56.}));
    EXPECT_EQ(Vector4(1., 2., 3., 4.), Vector4(std::array<double, 4>{1., 2., 3., 4.}));
    EXPECT_EQ(static_cast<std::size_t>(4), Vector4::kDimension);
    {
      const Vector4 kExpected{3., 4., 1., 10.};
      const Vector4 kDut = kExpected;
      EXPECT_EQ(kDut, kExpected);
    }
    {
      const Vector4 kDut = Vector4(45., 9., 35., 27.);
      EXPECT_EQ(kDut, Vector4(45., 9., 35., 27.));
    }
    EXPECT_EQ(Vector4(Vector<4>({2., 7., 15., 5.})).w(), 5.);
  }
}

GTEST_TEST(VectorTest, PublicMethods) {
  {  // N dimension vector
    const Vector<4> kDut{1., 2., 3., 4.};
    EXPECT_EQ(kDut.reduce(1), Vector<3>({1., 3., 4.}));
  }
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
    EXPECT_EQ("{5, 10}", dut.to_str());
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
    EXPECT_EQ("{5, 10, 15}", dut.to_str());
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
    EXPECT_EQ("{5, 10, 15, 20}", dut.to_str());
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
    dut += Vector2(10., 20.);
    EXPECT_EQ(dut, Vector2(43., 64.));
    {
      std::stringstream ss;
      ss << kDut;
      EXPECT_EQ(ss.str(), "{3, 4}");
    }
    {
      std::stringstream ss;
      ss << Vector2(1., 1.);
      EXPECT_EQ(ss.str(), "{1, 1}");
    }
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
    dut += Vector3(10., 20., 30.);
    EXPECT_EQ(dut, Vector3(43., 64., 85.));
    {
      std::stringstream ss;
      ss << kDut;
      EXPECT_EQ(ss.str(), "{3, 4, 5}");
    }
    {
      std::stringstream ss;
      ss << Vector3(1., 1., 1.);
      EXPECT_EQ(ss.str(), "{1, 1, 1}");
    }
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
    dut += Vector4(10., 20., 30., 40.);
    EXPECT_EQ(dut, Vector4(43., 64., 85., 106.));
    {
      std::stringstream ss;
      ss << kDut;
      EXPECT_EQ(ss.str(), "{3, 4, 5, 6}");
    }
    {
      std::stringstream ss;
      ss << Vector4(1., 1., 1., 1.);
      EXPECT_EQ(ss.str(), "{1, 1, 1, 1}");
    }
  }
}

// The following test can be reproduced in Python with the following snippet:
//
// ```
// import numpy as np
//
// a = [1, 2, 3]
// b = [4, 5, 6]
//
// np.cross(a, a)
// np.cross(a, b)
// ```
GTEST_TEST(Vector3, CorssProduct) {
  const double kTolerance{1e-12};
  const Vector3 a(1., 2., 3.);
  const Vector3 b(4., 5., 6.);

  Vector3 dut = a.cross(a);
  EXPECT_NEAR(dut.x(), 0., kTolerance);
  EXPECT_NEAR(dut.x(), 0., kTolerance);
  EXPECT_NEAR(dut.x(), 0., kTolerance);

  dut = a.cross(b);
  EXPECT_NEAR(dut.x(), -3., kTolerance);
  EXPECT_NEAR(dut.y(), 6., kTolerance);
  EXPECT_NEAR(dut.z(), -3., kTolerance);
}

GTEST_TEST(VectorTest, StaticMethods) {
  {  // 2- dimension vector.
    EXPECT_EQ(Vector2::Zero(), Vector2(0., 0.));
    EXPECT_EQ(Vector2::Ones(), Vector2(1., 1.));
    EXPECT_EQ(Vector2::UnitX(), Vector2(1., 0.));
    EXPECT_EQ(Vector2::UnitY(), Vector2(0., 1.));
    EXPECT_EQ(Vector2(1., 2.), Vector2::FromStr("{1., 2.}"));
  }
  {  // 3- dimension vector.
    EXPECT_EQ(Vector3::Zero(), Vector3(0., 0., 0.));
    EXPECT_EQ(Vector3::Ones(), Vector3(1., 1., 1.));
    EXPECT_EQ(Vector3::UnitX(), Vector3(1., 0., 0.));
    EXPECT_EQ(Vector3::UnitY(), Vector3(0., 1., 0.));
    EXPECT_EQ(Vector3::UnitZ(), Vector3(0., 0., 1.));
    EXPECT_EQ(Vector3(1., 2., 3.), Vector3::FromStr("{1., 2., 3.}"));
  }
  {  // 4- dimension vector.
    EXPECT_EQ(Vector4::Zero(), Vector4(0., 0., 0., 0.));
    EXPECT_EQ(Vector4::Ones(), Vector4(1., 1., 1., 1.));
    EXPECT_EQ(Vector4::UnitX(), Vector4(1., 0., 0., 0.));
    EXPECT_EQ(Vector4::UnitY(), Vector4(0., 1., 0., 0.));
    EXPECT_EQ(Vector4::UnitZ(), Vector4(0., 0., 1., 0.));
    EXPECT_EQ(Vector4::UnitW(), Vector4(0., 0., 0., 1.));
    EXPECT_EQ(Vector4(1., 2., 3., 4.), Vector4::FromStr("{1., 2., 3., 4.}"));
  }
}

GTEST_TEST(VectorTest, SerializationRoundTrip) {
  {  // 2- dimension vector.
    EXPECT_EQ(Vector2(1., 2.), Vector2::FromStr(Vector2(1., 2.).to_str()));
  }
  {  // 3- dimension vector.
    EXPECT_EQ(Vector3(1., 2., 3.), Vector3::FromStr(Vector3(1., 2., 3.).to_str()));
  }
  {  // 4- dimension vector.
    EXPECT_EQ(Vector4(1., 2., 3., 4.), Vector4::FromStr(Vector4(1., 2., 3., 4.).to_str()));
  }
}

}  // namespace
}  // namespace math
}  // namespace maliput
