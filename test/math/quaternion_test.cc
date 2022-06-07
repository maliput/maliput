// Some tests in this file have been inspired by
// https://bitbucket.org/ignitionrobotics/ign-math/src/default/src/Quaternion_TEST.cc
//
// ignition-math's license follows:
//
//                                 Apache License
//                           Version 2.0, January 2004
//                        http://www.apache.org/licenses/
//
//   TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION
//
//   1. Definitions.
//
//      "License" shall mean the terms and conditions for use, reproduction,
//      and distribution as defined by Sections 1 through 9 of this document.
//
//      "Licensor" shall mean the copyright owner or entity authorized by
//      the copyright owner that is granting the License.
//
//      "Legal Entity" shall mean the union of the acting entity and all
//      other entities that control, are controlled by, or are under common
//      control with that entity. For the purposes of this definition,
//      "control" means (i) the power, direct or indirect, to cause the
//      direction or management of such entity, whether by contract or
//      otherwise, or (ii) ownership of fifty percent (50%) or more of the
//      outstanding shares, or (iii) beneficial ownership of such entity.
//
//      "You" (or "Your") shall mean an individual or Legal Entity
//      exercising permissions granted by this License.
//
//      "Source" form shall mean the preferred form for making modifications,
//      including but not limited to software source code, documentation
//      source, and configuration files.
//
//      "Object" form shall mean any form resulting from mechanical
//      transformation or translation of a Source form, including but
//      not limited to compiled object code, generated documentation,
//      and conversions to other media types.
//
//      "Work" shall mean the work of authorship, whether in Source or
//      Object form, made available under the License, as indicated by a
//      copyright notice that is included in or attached to the work
//      (an example is provided in the Appendix below).
//
//      "Derivative Works" shall mean any work, whether in Source or Object
//      form, that is based on (or derived from) the Work and for which the
//      editorial revisions, annotations, elaborations, or other modifications
//      represent, as a whole, an original work of authorship. For the purposes
//      of this License, Derivative Works shall not include works that remain
//      separable from, or merely link (or bind by name) to the interfaces of,
//      the Work and Derivative Works thereof.
//
//      "Contribution" shall mean any work of authorship, including
//      the original version of the Work and any modifications or additions
//      to that Work or Derivative Works thereof, that is intentionally
//      submitted to Licensor for inclusion in the Work by the copyright owner
//      or by an individual or Legal Entity authorized to submit on behalf of
//      the copyright owner. For the purposes of this definition, "submitted"
//      means any form of electronic, verbal, or written communication sent
//      to the Licensor or its representatives, including but not limited to
//      communication on electronic mailing lists, source code control systems,
//      and issue tracking systems that are managed by, or on behalf of, the
//      Licensor for the purpose of discussing and improving the Work, but
//      excluding communication that is conspicuously marked or otherwise
//      designated in writing by the copyright owner as "Not a Contribution."
//
//      "Contributor" shall mean Licensor and any individual or Legal Entity
//      on behalf of whom a Contribution has been received by Licensor and
//      subsequently incorporated within the Work.
//
//   2. Grant of Copyright License. Subject to the terms and conditions of
//      this License, each Contributor hereby grants to You a perpetual,
//      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
//      copyright license to reproduce, prepare Derivative Works of,
//      publicly display, publicly perform, sublicense, and distribute the
//      Work and such Derivative Works in Source or Object form.
//
//   3. Grant of Patent License. Subject to the terms and conditions of
//      this License, each Contributor hereby grants to You a perpetual,
//      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
//      (except as stated in this section) patent license to make, have made,
//      use, offer to sell, sell, import, and otherwise transfer the Work,
//      where such license applies only to those patent claims licensable
//      by such Contributor that are necessarily infringed by their
//      Contribution(s) alone or by combination of their Contribution(s)
//      with the Work to which such Contribution(s) was submitted. If You
//      institute patent litigation against any entity (including a
//      cross-claim or counterclaim in a lawsuit) alleging that the Work
//      or a Contribution incorporated within the Work constitutes direct
//      or contributory patent infringement, then any patent licenses
//      granted to You under this License for that Work shall terminate
//      as of the date such litigation is filed.
//
//   4. Redistribution. You may reproduce and distribute copies of the
//      Work or Derivative Works thereof in any medium, with or without
//      modifications, and in Source or Object form, provided that You
//      meet the following conditions:
//
//      (a) You must give any other recipients of the Work or
//          Derivative Works a copy of this License; and
//
//      (b) You must cause any modified files to carry prominent notices
//          stating that You changed the files; and
//
//      (c) You must retain, in the Source form of any Derivative Works
//          that You distribute, all copyright, patent, trademark, and
//          attribution notices from the Source form of the Work,
//          excluding those notices that do not pertain to any part of
//          the Derivative Works; and
//
//      (d) If the Work includes a "NOTICE" text file as part of its
//          distribution, then any Derivative Works that You distribute must
//          include a readable copy of the attribution notices contained
//          within such NOTICE file, excluding those notices that do not
//          pertain to any part of the Derivative Works, in at least one
//          of the following places: within a NOTICE text file distributed
//          as part of the Derivative Works; within the Source form or
//          documentation, if provided along with the Derivative Works; or,
//          within a display generated by the Derivative Works, if and
//          wherever such third-party notices normally appear. The contents
//          of the NOTICE file are for informational purposes only and
//          do not modify the License. You may add Your own attribution
//          notices within Derivative Works that You distribute, alongside
//          or as an addendum to the NOTICE text from the Work, provided
//          that such additional attribution notices cannot be construed
//          as modifying the License.
//
//      You may add Your own copyright statement to Your modifications and
//      may provide additional or different license terms and conditions
//      for use, reproduction, or distribution of Your modifications, or
//      for any such Derivative Works as a whole, provided Your use,
//      reproduction, and distribution of the Work otherwise complies with
//      the conditions stated in this License.
//
//   5. Submission of Contributions. Unless You explicitly state otherwise,
//      any Contribution intentionally submitted for inclusion in the Work
//      by You to the Licensor shall be under the terms and conditions of
//      this License, without any additional terms or conditions.
//      Notwithstanding the above, nothing herein shall supersede or modify
//      the terms of any separate license agreement you may have executed
//      with Licensor regarding such Contributions.
//
//   6. Trademarks. This License does not grant permission to use the trade
//      names, trademarks, service marks, or product names of the Licensor,
//      except as required for reasonable and customary use in describing the
//      origin of the Work and reproducing the content of the NOTICE file.
//
//   7. Disclaimer of Warranty. Unless required by applicable law or
//      agreed to in writing, Licensor provides the Work (and each
//      Contributor provides its Contributions) on an "AS IS" BASIS,
//      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
//      implied, including, without limitation, any warranties or conditions
//      of TITLE, NON-INFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A
//      PARTICULAR PURPOSE. You are solely responsible for determining the
//      appropriateness of using or redistributing the Work and assume any
//      risks associated with Your exercise of permissions under this License.
//
//   8. Limitation of Liability. In no event and under no legal theory,
//      whether in tort (including negligence), contract, or otherwise,
//      unless required by applicable law (such as deliberate and grossly
//      negligent acts) or agreed to in writing, shall any Contributor be
//      liable to You for damages, including any direct, indirect, special,
//      incidental, or consequential damages of any character arising as a
//      result of this License or out of the use or inability to use the
//      Work (including but not limited to damages for loss of goodwill,
//      work stoppage, computer failure or malfunction, or any and all
//      other commercial damages or losses), even if such Contributor
//      has been advised of the possibility of such damages.
//
//   9. Accepting Warranty or Additional Liability. While redistributing
//      the Work or Derivative Works thereof, You may choose to offer,
//      and charge a fee for, acceptance of support, warranty, indemnity,
//      or other liability obligations and/or rights consistent with this
//      License. However, in accepting such obligations, You may act only
//      on Your own behalf and on Your sole responsibility, not on behalf
//      of any other Contributor, and only if You agree to indemnify,
//      defend, and hold each Contributor harmless for any liability
//      incurred by, or claims asserted against, such Contributor by reason
//      of your accepting any such warranty or additional liability.
//
//   END OF TERMS AND CONDITIONS
//
// -----------------------------------------------------------------------------

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
#include "maliput/math/quaternion.h"

#include <gtest/gtest.h>

#include "maliput/math/matrix.h"
#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"
#include "maliput/test_utilities/maliput_math_compare.h"

namespace maliput {
namespace math {
namespace test {
namespace {

constexpr double kTolerance{1e-15};

GTEST_TEST(Quaternion, DefaultConstructor) {
  const Quaternion dut;
  EXPECT_EQ(dut.w(), 1.);
  EXPECT_EQ(dut.x(), 0.);
  EXPECT_EQ(dut.y(), 0.);
  EXPECT_EQ(dut.z(), 0.);
}

GTEST_TEST(Quaternion, Identity) {
  const Quaternion dut = Quaternion::Identity();
  EXPECT_EQ(dut.w(), 1.);
  EXPECT_EQ(dut.x(), 0.);
  EXPECT_EQ(dut.y(), 0.);
  EXPECT_EQ(dut.z(), 0.);
}

GTEST_TEST(Quaternion, CoefficientsConstructor) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_EQ(dut.w(), kW);
  EXPECT_EQ(dut.x(), kX);
  EXPECT_EQ(dut.y(), kY);
  EXPECT_EQ(dut.z(), kZ);
}

GTEST_TEST(Quaternion, VectorConstructor) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};
  // Default constructor.
  const Quaternion dut(Vector4(kW, kX, kY, kZ));
  EXPECT_EQ(dut.w(), kW);
  EXPECT_EQ(dut.x(), kX);
  EXPECT_EQ(dut.y(), kY);
  EXPECT_EQ(dut.z(), kZ);
}

GTEST_TEST(Quaternion, RotationMatrixRoundTrip) {
  const double kMinAngle{-M_PI / 2.};
  const double kMaxAngle{kMinAngle + M_PI};
  const int kNAngles{5};
  const double kAngleStep{(kMaxAngle - kMinAngle) / static_cast<double>(kNAngles)};

  for (double roll = kMinAngle; roll <= kMaxAngle; roll += kAngleStep) {
    for (double pitch = kMinAngle; pitch <= kMaxAngle; pitch += kAngleStep) {
      for (double yaw = kMinAngle; yaw <= kMaxAngle; yaw += kAngleStep) {
        const Matrix3 kRotationMatrix = RollPitchYaw(roll, pitch, yaw).ToMatrix();
        const Quaternion dut(kRotationMatrix);
        EXPECT_TRUE(CompareMatrices(kRotationMatrix, dut.ToRotationMatrix(), kTolerance));
      }
    }
  }
}

GTEST_TEST(Quaternion, GettersAndSetters) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_EQ(dut.w(), kW);
  EXPECT_EQ(dut.x(), kX);
  EXPECT_EQ(dut.y(), kY);
  EXPECT_EQ(dut.z(), kZ);

  Quaternion mutable_dut(kW, kX, kY, kZ);
  EXPECT_EQ(mutable_dut.w(), kW);
  EXPECT_EQ(mutable_dut.x(), kX);
  EXPECT_EQ(mutable_dut.y(), kY);
  EXPECT_EQ(mutable_dut.z(), kZ);

  mutable_dut.w() = mutable_dut.w() * 2.;
  mutable_dut.x() = mutable_dut.x() * 2.;
  mutable_dut.y() = mutable_dut.y() * 2.;
  mutable_dut.z() = mutable_dut.z() * 2.;
  EXPECT_EQ(mutable_dut.w(), 2. * kW);
  EXPECT_EQ(mutable_dut.x(), 2. * kX);
  EXPECT_EQ(mutable_dut.y(), 2. * kY);
  EXPECT_EQ(mutable_dut.z(), 2. * kZ);
}

GTEST_TEST(Quaternion, VectorAndCoefficients) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_EQ(dut.w(), kW);
  EXPECT_EQ(dut.x(), kX);
  EXPECT_EQ(dut.y(), kY);
  EXPECT_EQ(dut.z(), kZ);

  EXPECT_TRUE(CompareVectors(Vector3(kX, kY, kZ), dut.vec(), 0. /* tolerance */));
  EXPECT_TRUE(CompareVectors(Vector4(kW, kX, kY, kZ), dut.coeffs(), 0. /* tolerance */));
}

GTEST_TEST(Quaternion, SetIdentity) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};

  Quaternion dut(kW, kX, kY, kZ);
  EXPECT_EQ(dut.w(), kW);
  EXPECT_EQ(dut.x(), kX);
  EXPECT_EQ(dut.y(), kY);
  EXPECT_EQ(dut.z(), kZ);

  dut.set_identity();
  EXPECT_EQ(dut.w(), 1.);
  EXPECT_EQ(dut.x(), 0.);
  EXPECT_EQ(dut.y(), 0.);
  EXPECT_EQ(dut.z(), 0.);
}

// Constants below were obtained with the following Python script:
//
// ```
// import math as m
//
// w = 0.884
// x = 0.306
// y = 0.177
// z = 0.306
//
// squared_norm = w*w + x*x + y*y + z*z
// norm = m.sqrt(squared_norm)
// scaled_squared_norm = 3*3*w*w + 3*3*x*x + 3*3*y*y + 3*3*z*z
// scaled_norm = m.sqrt(scaled_squared_norm)
// ```
GTEST_TEST(Quaternion, Norm) {
  EXPECT_NEAR(Quaternion::Identity().norm(), 1., kTolerance);

  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};
  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_NEAR(1.0000284995938866, dut.norm(), kTolerance);

  const Quaternion scaled_dut(3. * kW, 3. * kX, 3. * kY, 3. * kZ);
  EXPECT_NEAR(3.00008549878166, scaled_dut.norm(), kTolerance);
}

GTEST_TEST(Quaternion, SquaredNorm) {
  EXPECT_NEAR(Quaternion::Identity().squared_norm(), 1., kTolerance);

  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_NEAR(1.000057, dut.squared_norm(), kTolerance);

  const Quaternion scaled_dut(3. * kW, 3. * kX, 3. * kY, 3. * kZ);
  EXPECT_NEAR(9.000513000000002, scaled_dut.squared_norm(), kTolerance);
}

// Constants below were obtained with the following Python script:
//
// ```
// import math as m
//
// w = 1.
// x = 2.
// y = 3.
// z = 4.
//
// norm = m.sqrt(w*w + x*x + y*y + z*z)
// normalized_w = w / norm
// normalized_x = x / norm
// normalized_y = y / norm
// normalized_z = z / norm
// ```
GTEST_TEST(Quaternion, NormalizeAndNormalized) {
  const double kW{1.};
  const double kX{2.};
  const double kY{3.};
  const double kZ{4.};

  {
    Quaternion dut(kW, kX, kY, kZ);
    dut.normalize();
    EXPECT_NEAR(1., dut.norm(), kTolerance);
    EXPECT_TRUE(CompareVectors(Vector4(0.18257418583505536, 0.3651483716701107, 0.5477225575051661, 0.7302967433402214),
                               dut.coeffs(), kTolerance));
  }
  {
    const Quaternion dut(kW, kX, kY, kZ);
    const Quaternion normalized_quaternion = dut.normalized();
    EXPECT_NEAR(1., normalized_quaternion.norm(), kTolerance);
    EXPECT_TRUE(CompareVectors(Vector4(0.18257418583505536, 0.3651483716701107, 0.5477225575051661, 0.7302967433402214),
                               normalized_quaternion.coeffs(), kTolerance));
  }
}

GTEST_TEST(Quaternion, DotProduct) {
  const double kA{1.};
  const double kB{2.};
  const double kC{3.};
  const double kD{4.};

  const Quaternion dut(kA, kB, kC, kD);
  EXPECT_NEAR(30., dut.dot(dut), kTolerance);
  EXPECT_NEAR(30., dut.squared_norm(), kTolerance);
  EXPECT_NEAR(20., dut.dot(Quaternion(kD, kC, kB, kA)), kTolerance);
}

GTEST_TEST(Quaternion, AngularDistance) {
  // On the same plane.
  {
    const Quaternion q1(M_PI / 2., Vector3(1., 0., 0.));
    const Quaternion q2(M_PI / 4., Vector3(1., 0., 0.));
    EXPECT_NEAR(q1.AngularDistance(q2), M_PI / 4., kTolerance);
  }
  // The conjugate rotation must be twice the path of the original.
  {
    const Quaternion q1(M_PI / 3., Vector3(1., 0., 0.));
    EXPECT_NEAR(q1.AngularDistance(q1.conjugate()), 2 * M_PI / 3., kTolerance);
  }
  // Distance to the same quaternion must be zero.
  {
    const Quaternion q1(M_PI / 3., Vector3(1., 0., 0.));
    EXPECT_NEAR(q1.AngularDistance(q1), 0., kTolerance);
  }
}

GTEST_TEST(Quaternion, EqualTo) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};
  const double kAnyOtherValue{0.};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_TRUE(dut == dut);
  EXPECT_TRUE(dut == Quaternion(kW, kX, kY, kZ));
  EXPECT_FALSE(dut == Quaternion(kAnyOtherValue, kX, kY, kZ));
  EXPECT_FALSE(dut == Quaternion(kW, kAnyOtherValue, kY, kZ));
  EXPECT_FALSE(dut == Quaternion(kW, kX, kAnyOtherValue, kZ));
  EXPECT_FALSE(dut == Quaternion(kW, kX, kY, kAnyOtherValue));
}

GTEST_TEST(Quaternion, NotEqualTo) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};
  const double kAnyOtherValue{0.};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_FALSE(dut != dut);
  EXPECT_FALSE(dut != Quaternion(kW, kX, kY, kZ));
  EXPECT_TRUE(dut != Quaternion(kAnyOtherValue, kX, kY, kZ));
  EXPECT_TRUE(dut != Quaternion(kW, kAnyOtherValue, kY, kZ));
  EXPECT_TRUE(dut != Quaternion(kW, kX, kAnyOtherValue, kZ));
  EXPECT_TRUE(dut != Quaternion(kW, kX, kY, kAnyOtherValue));
}

// Let $q$ be a quaternion, then its inverse is $ conj(q) / |q|^2 $. The
// following constants were derived in a python script using the previous
// definition.
//
// ```
// w = 0.884
// x = 0.306
// y = 0.177
// z = 0.306
//
// squared_norm = w*w + x*x + y*y + z*z
// w / squared_norm
// -x / squared_norm
// -y / squared_norm
// -z / squared_norm
// ```
GTEST_TEST(Quaternion, Inverse) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_TRUE(
      CompareVectors(Vector4(0.8839496148719523, -0.30598255899413734, -0.1769899115750402, -0.30598255899413734),
                     dut.Inverse().coeffs(), kTolerance));
  // Zero quaternion is handled separately up to Quaternion's tolerance.
  EXPECT_TRUE(CompareVectors(Vector4(0., 0., 0., 0.),
                             Quaternion(Quaternion::kTolerance / 2., 0., 0., 0.).Inverse().coeffs(), kTolerance));
  EXPECT_TRUE(CompareVectors(Vector4(0., 0., 0., 0.), Quaternion(Quaternion::kTolerance, 0., 0., 0.).Inverse().coeffs(),
                             kTolerance));
}

GTEST_TEST(Quaternion, Conjugate) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};
  const Quaternion dut = Quaternion(kW, kX, kY, kZ).conjugate();
  EXPECT_EQ(kW, dut.w());
  EXPECT_EQ(-kX, dut.x());
  EXPECT_EQ(-kY, dut.y());
  EXPECT_EQ(-kZ, dut.z());
}

GTEST_TEST(Quaternion, IsApprox) {
  const double kAlmostEqualPrecision{1e-3};
  const Vector3 kUnitX(1., 0., 0);
  const Vector3 kUnitY(0., 1., 0);

  {
    const Quaternion q1(0., Vector3(1., 0., 0.));
    const Quaternion q2(0., Vector3(0., 1., 0.));
    EXPECT_TRUE(q1.IsApprox(q2, kAlmostEqualPrecision));
  }
  {
    const Quaternion q1(0., kUnitX);
    // Adds a very tiny rotation around the axis.
    const Quaternion q2(1e-4, kUnitX);
    EXPECT_TRUE(q1.IsApprox(q2, kAlmostEqualPrecision));
  }
  {
    const Quaternion q1(0., kUnitX);
    const Quaternion q2(M_PI / 3., (kUnitX + kUnitY).normalized());
    EXPECT_FALSE(q1.IsApprox(q2, kAlmostEqualPrecision));
  }
}

GTEST_TEST(Quaternion, MultiplicationBetweenQuaternions) {
  const Vector3 kUnitX(1., 0., 0.);
  const Vector3 kUnitY(0., 1., 0.);
  const Vector3 kUnitZ(0., 0., 1.);
  const Vector3 kAnyDirection(1., 2., 3.);

  const Quaternion q1 = Quaternion::FromTwoVectors(kUnitX, kUnitY);
  const Quaternion q2 = Quaternion::FromTwoVectors(kUnitY, kUnitX);
  EXPECT_TRUE(CompareVectors(Quaternion::Identity().coeffs(), (q1 * q2).coeffs(), kTolerance));

  const Quaternion q3 = Quaternion::FromTwoVectors(kUnitZ, kAnyDirection);
  const Quaternion q4 = Quaternion::FromTwoVectors(kAnyDirection, kUnitZ);
  EXPECT_TRUE(CompareVectors(Quaternion::Identity().coeffs(), (q3 * q4).coeffs(), kTolerance));
}

GTEST_TEST(Quaternion, MultiplicationAndAssingmentBetweenQuaternions) {
  const Vector3 kUnitX(1., 0., 0.);
  const Vector3 kUnitY(0., 1., 0.);
  const Vector3 kUnitZ(0., 0., 1.);
  const Vector3 kAnyDirection(1., 2., 3.);

  const Quaternion q1 = Quaternion::FromTwoVectors(kUnitX, kUnitY);
  Quaternion q2 = Quaternion::FromTwoVectors(kUnitY, kUnitX);
  q2 *= q1;
  EXPECT_TRUE(CompareVectors(Quaternion::Identity().coeffs(), q2.coeffs(), kTolerance));

  const Quaternion q3 = Quaternion::FromTwoVectors(kUnitZ, kAnyDirection);
  Quaternion q4 = Quaternion::FromTwoVectors(kAnyDirection, kUnitZ);
  q4 *= q3;
  EXPECT_TRUE(CompareVectors(Quaternion::Identity().coeffs(), q4.coeffs(), kTolerance));
}

GTEST_TEST(Quaternion, TransformVector) {
  const Vector3 v(-1., 1., 1.);
  const Quaternion dut(1., 0., 1., 0.);
  const Vector3 kExpectedResult(3., 1., 1.);

  EXPECT_TRUE(CompareVectors(kExpectedResult, dut.TransformVector(v), kTolerance));
}

GTEST_TEST(Quaternion, MultiplicationByVector) {
  const Vector3 v(-1., 1., 1.);
  const Quaternion dut(1., 0., 1., 0.);
  const Vector3 kExpectedResult(3., 1., 1.);

  EXPECT_TRUE(CompareVectors(kExpectedResult, dut * v, kTolerance));
}

GTEST_TEST(Quaternion, Slerp) {
  const Quaternion q1(RollPitchYaw(0.1, 1.2, 2.3).ToMatrix());
  const Quaternion q2(RollPitchYaw(1.2, 2.3, -3.4).ToMatrix());

  const Quaternion q3 = q1.Slerp(1., q2);

  // Degrading tolerance in this test on purpose because of the overhead in the
  // computation and the expected result.
  EXPECT_TRUE(CompareVectors(Vector4(0.554528, -0.717339, 0.32579, 0.267925), q3.coeffs(), 1e-6 /* tolerance */));
}

GTEST_TEST(Quaternion, Serialization) {
  const Quaternion dut(1., 2., 3., 4.);
  std::stringstream ss;
  ss << dut;
  EXPECT_EQ(ss.str(), "(w: 1, x: 2, y: 3, z: 4)");
}

GTEST_TEST(Quaternion, FromTwoVectors) {
  const Vector3 kUnitX(1., 0., 0.);
  const Vector3 kUnitY(0., 1., 0.);
  const Vector3 kUnitZ(0., 0., 1.);

  const double kW = std::sqrt(2.) / 2.;
  const double kU = std::sqrt(2.) / 2.;

  {  // X to Y.
    const Quaternion dut = Quaternion::FromTwoVectors(kUnitX, kUnitY);
    EXPECT_TRUE(CompareVectors(Vector4(kW, 0., 0., kU), dut.coeffs(), kTolerance));
  }
  {
    Quaternion dut;
    dut.SetFromTwoVectors(kUnitX, kUnitY);
    EXPECT_TRUE(CompareVectors(Vector4(kW, 0., 0., kU), dut.coeffs(), kTolerance));
  }
  {  // Y to Z.
    const Quaternion dut = Quaternion::FromTwoVectors(kUnitY, kUnitZ);
    EXPECT_TRUE(CompareVectors(Vector4(kW, kU, 0., 0.), dut.coeffs(), kTolerance));
  }
  {
    Quaternion dut;
    dut.SetFromTwoVectors(kUnitY, kUnitZ);
    EXPECT_TRUE(CompareVectors(Vector4(kW, kU, 0., 0.), dut.coeffs(), kTolerance));
  }
  {  // Z to X.
    const Quaternion dut = Quaternion::FromTwoVectors(kUnitZ, kUnitX);
    EXPECT_TRUE(CompareVectors(Vector4(kW, 0., kU, 0.), dut.coeffs(), kTolerance));
  }
  {
    Quaternion dut;
    dut.SetFromTwoVectors(kUnitZ, kUnitX);
    EXPECT_TRUE(CompareVectors(Vector4(kW, 0., kU, 0.), dut.coeffs(), kTolerance));
  }
}

}  // namespace
}  // namespace test
}  // namespace math
}  // namespace maliput
