#include "maliput/math/matrix.h"

#include <gtest/gtest.h>

namespace maliput {
namespace math {
namespace {

GTEST_TEST(MatrixTest, Constructors) {
  {  // 2- dimension matrix.
    EXPECT_EQ(Matrix<2>{}, Matrix<2>({0., 0., 0., 0.}));
    EXPECT_EQ(Matrix<2>({1., 2., 3., 4.}), Matrix<2>({{1., 2.}, {3., 4.}}));
    EXPECT_EQ(Matrix<2>({1., 2., 3., 4.}), Matrix<2>(std::array<Vector<2>, 2>{Vector<2>{1., 2.}, Vector<2>{3., 4.}}));
    {
      const Matrix<2> kExpectedDut{{1., 12}, {8., 5.}};
      const Matrix<2> kDut = kExpectedDut;
      EXPECT_EQ(kDut, kExpectedDut);
    }
    {
      const Matrix<2> kDut = Matrix<2>{{1., 12}, {8., 5.}};
      EXPECT_EQ(kDut, Matrix<2>({{1., 12}, {8., 5.}}));
    }
  }
  {  // 3- dimension matrix.
    EXPECT_EQ(Matrix<3>{}, Matrix<3>({0., 0., 0., 0., 0., 0., 0., 0., 0.}));
    EXPECT_EQ(Matrix<3>({1., 2., 3., 4., 5., 6., 7., 8., 9.}), Matrix<3>({{1., 2., 3.}, {4., 5., 6.}, {7., 8., 9.}}));
    EXPECT_EQ(Matrix<3>({1., 2., 3., 4., 5., 6., 7., 8., 9.}),
              Matrix<3>(std::array<Vector<3>, 3>{Vector<3>{1., 2., 3.}, Vector<3>{4., 5., 6.}, Vector<3>{7., 8., 9.}}));
    {
      const Matrix<3> kExpectedDut{{1., 12., 3.}, {8., 5., 3.}, {6., 14., 9.}};
      const Matrix<3> kDut = kExpectedDut;
      EXPECT_EQ(kDut, kExpectedDut);
    }
    {
      const Matrix<3> kDut = Matrix<3>{{1., 12., 3.}, {8., 5., 3.}, {6., 14., 9.}};
      EXPECT_EQ(kDut, Matrix<3>({{1., 12., 3.}, {8., 5., 3.}, {6., 14., 9.}}));
    }
  }
  {  // 4- dimension matrix.
    EXPECT_EQ(Matrix<4>{}, Matrix<4>({0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}));
    EXPECT_EQ(Matrix<4>({1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 11., 12., 13., 14., 15., 16.}),
              Matrix<4>({{1., 2., 3., 4.}, {5., 6., 7., 8.}, {9., 10., 11., 12.}, {13., 14., 15., 16.}}));
    EXPECT_EQ(Matrix<4>({1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 11., 12., 13., 14., 15., 16.}),
              Matrix<4>(std::array<Vector<4>, 4>{Vector<4>{1., 2., 3., 4.}, Vector<4>{5., 6., 7., 8.},
                                                 Vector<4>{9., 10., 11., 12.}, Vector<4>{13., 14., 15., 16.}}));
    {
      const Matrix<4> kExpectedDut{{1., 12., 3., 2.}, {8., 5., 3., 7.}, {6., 14., 9., 25.}, {13., 4., 7., 8.}};
      const Matrix<4> kDut = kExpectedDut;
      EXPECT_EQ(kDut, kExpectedDut);
    }
    {
      const Matrix<4> kDut = Matrix<4>{{1., 12., 3., 2.}, {8., 5., 3., 7.}, {6., 14., 9., 25.}, {13., 4., 7., 8.}};
      EXPECT_EQ(kDut, Matrix<4>({{1., 12., 3., 2.}, {8., 5., 3., 7.}, {6., 14., 9., 25.}, {13., 4., 7., 8.}}));
    }
  }
}

GTEST_TEST(MatrixTest, PublicMethods) {
  {  // 2- dimension matrix.
    const Matrix<2> kDut{{1., 12}, {8., 5.}};
    EXPECT_EQ(kDut.row(0), Vector<2>({1., 12.}));
    EXPECT_EQ(kDut.row(1), Vector<2>({8., 5.}));
    EXPECT_EQ(kDut.col(0), Vector<2>({1., 8.}));
    EXPECT_EQ(kDut.col(1), Vector<2>({12., 5.}));
    EXPECT_EQ(kDut.transpose(), Matrix<2>({{1., 8.}, {12., 5.}}));
    EXPECT_EQ(kDut.determinant(), -91.);
    EXPECT_FALSE(kDut.is_singular());
    EXPECT_TRUE(Matrix<2>({2., 2., 4., 4.}).is_singular());
    EXPECT_EQ(kDut.cofactor(0, 1), -8.);
    EXPECT_EQ(kDut.cofactor(), Matrix<2>({{5., -8.}, {-12., 1.}}));
    EXPECT_EQ(kDut * kDut.inverse(), Matrix<2>::Identity());
    const Matrix<2> kResult = kDut * kDut.inverse();
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        EXPECT_NEAR(kResult[i][j], Matrix<2>::Identity()[i][j], 10e-6);
      }
    }
  }
  {  // 3- dimension matrix.
    const Matrix<3> kDut{{1., 12., 3.}, {8., 5., 3.}, {6., 14., 9.}};
    EXPECT_EQ(kDut.row(0), Vector<3>({1., 12., 3.}));
    EXPECT_EQ(kDut.row(1), Vector<3>({8., 5., 3.}));
    EXPECT_EQ(kDut.row(2), Vector<3>({6., 14., 9.}));
    EXPECT_EQ(kDut.col(0), Vector<3>({1., 8., 6.}));
    EXPECT_EQ(kDut.col(1), Vector<3>({12., 5., 14.}));
    EXPECT_EQ(kDut.col(2), Vector<3>({3., 3., 9.}));
    EXPECT_EQ(kDut.transpose(), Matrix<3>({{1., 8., 6.}, {12., 5., 14.}, {3., 3., 9.}}));
    EXPECT_EQ(kDut.determinant(), -399.);
    EXPECT_FALSE(kDut.is_singular());
    EXPECT_TRUE(Matrix<3>({1., 2., 3., 4., 5., 6., 7., 8., 9.}).is_singular());
    EXPECT_EQ(kDut.cofactor(1, 1), -9.);
    EXPECT_EQ(kDut.reduce(1, 1), Matrix<2>({{1., 3.}, {6., 9.}}));
    EXPECT_EQ(kDut.cofactor(), Matrix<3>({{3., -54., 82.}, {-66., -9., 58.}, {21., 21., -91}}));
    const Matrix<3> kResult = kDut * kDut.inverse();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        EXPECT_NEAR(kResult[i][j], Matrix<3>::Identity()[i][j], 10e-6);
      }
    }
  }
  {  // 4- dimension matrix.
    const Matrix<4> kDut{{1., 12., 3., 2.}, {8., 5., 3., 7.}, {6., 14., 9., 25.}, {13., 4., 7., 8.}};
    EXPECT_EQ(kDut.row(0), Vector<4>({1., 12., 3., 2.}));
    EXPECT_EQ(kDut.row(1), Vector<4>({8., 5., 3., 7.}));
    EXPECT_EQ(kDut.row(2), Vector<4>({6., 14., 9., 25.}));
    EXPECT_EQ(kDut.row(3), Vector<4>({13., 4., 7., 8.}));
    EXPECT_EQ(kDut.col(0), Vector<4>({1., 8., 6., 13.}));
    EXPECT_EQ(kDut.col(1), Vector<4>({12., 5., 14., 4.}));
    EXPECT_EQ(kDut.col(2), Vector<4>({3., 3., 9., 7.}));
    EXPECT_EQ(kDut.col(3), Vector<4>({2., 7., 25, 8.}));
    EXPECT_EQ(kDut.transpose(), Matrix<4>({{1., 8., 6., 13.}, {12., 5., 14., 4.}, {3., 3., 9., 7.}, {2., 7., 25, 8.}}));
    EXPECT_EQ(kDut.determinant(), 6430.);
    EXPECT_FALSE(kDut.is_singular());
    EXPECT_TRUE(Matrix<4>({1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 11., 12., 13., 14., 15., 16.}).is_singular());
    EXPECT_EQ(kDut.cofactor(2, 1), -90.);
    EXPECT_EQ(kDut.reduce(1, 2), Matrix<3>({{1., 12., 2.}, {6., 14., 25.}, {13., 4., 8.}}));
    EXPECT_EQ(kDut.cofactor(), Matrix<4>({{-117., 518., 375., -397.},
                                          {1148., 578., -3020., 488},
                                          {-290., -90., 270., 280.},
                                          {-69., -354., 1705., -399.}}));
    EXPECT_EQ(kDut.cofactor().transpose(), kDut.adjoint());
    const Matrix<4> kResult = kDut * kDut.inverse();
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        EXPECT_NEAR(kResult[i][j], Matrix<4>::Identity()[i][j], 10e-6);
      }
    }
  }
}

GTEST_TEST(MatrixTest, Operators) {
  {  // 2- dimension matrix.
    const Matrix<2> kDut{{1., 12.}, {8., 5.}};
    Matrix<2> dut = kDut;
    EXPECT_EQ(kDut, dut);
    dut = Matrix<2>{{35., 1.}, {56., 98.}};
    EXPECT_EQ(dut, Matrix<2>({{35., 1.}, {56., 98.}}));
    dut = kDut;
    EXPECT_EQ(kDut, dut);
    EXPECT_EQ(kDut[0][1], 12.);
    EXPECT_EQ(kDut[1][1], 5.);
    dut[0][0] = 3;
    dut[0][1] = 6;
    dut[1][0] = 4;
    dut[1][1] = 7;
    EXPECT_EQ(dut[0][1], 6.);
    EXPECT_EQ(dut[1][0], 4.);
    EXPECT_NE(kDut, dut);
    EXPECT_EQ(kDut + dut, Matrix<2>({{4., 18.}, {12., 12.}}));
    EXPECT_EQ(kDut - dut, Matrix<2>({{-2., 6.}, {4., -2.}}));
    EXPECT_EQ(kDut * dut, Matrix<2>({{51., 90.}, {44., 83.}}));
    EXPECT_EQ(kDut * 2, Matrix<2>({{2., 24.}, {16., 10.}}));
    EXPECT_EQ(2 * kDut, Matrix<2>({{2., 24.}, {16., 10.}}));
    std::stringstream ss;
    ss << kDut;
    EXPECT_EQ(ss.str(), "{{1, 12},\n {8, 5}}");
  }
  {  // 3- dimension matrix.
    const Matrix<3> kDut{{1., 12., 3.}, {8., 5., 3.}, {6., 14., 9.}};
    Matrix<3> dut = kDut;
    EXPECT_EQ(kDut, dut);
    dut = Matrix<3>{{4., 18., 5.}, {12., 12., 4.}, {6., 22., 18.}};
    EXPECT_EQ(dut, Matrix<3>({{4., 18., 5.}, {12., 12., 4.}, {6., 22., 18.}}));
    dut = kDut;
    EXPECT_EQ(kDut, dut);
    EXPECT_EQ(kDut[0][2], 3.);
    EXPECT_EQ(kDut[1][1], 5.);
    dut[0][0] = 3;
    dut[0][1] = 6;
    dut[0][2] = 2;
    dut[1][0] = 4;
    dut[1][1] = 7;
    dut[1][2] = 1;
    dut[2][0] = 0;
    dut[2][1] = 8;
    dut[2][2] = 9;
    EXPECT_EQ(dut[0][1], 6.);
    EXPECT_EQ(dut[2][0], 0.);
    EXPECT_NE(kDut, dut);
    EXPECT_EQ(kDut + dut, Matrix<3>({{4., 18., 5.}, {12., 12., 4.}, {6., 22., 18.}}));
    EXPECT_EQ(kDut - dut, Matrix<3>({{-2., 6., 1.}, {4., -2., 2.}, {6., 6., 0.}}));
    EXPECT_EQ(kDut * dut, Matrix<3>({{51., 114., 41.}, {44., 107., 48.}, {74., 206., 107.}}));
    EXPECT_EQ(kDut * 2, Matrix<3>({{2., 24., 6.}, {16., 10., 6.}, {12., 28., 18.}}));
    EXPECT_EQ(2 * kDut, Matrix<3>({{2., 24., 6.}, {16., 10., 6.}, {12., 28., 18.}}));
    std::stringstream ss;
    ss << kDut;
    EXPECT_EQ(ss.str(), "{{1, 12, 3},\n {8, 5, 3},\n {6, 14, 9}}");
  }
  {  // 4- dimension matrix.
    const Matrix<4> kDut{{1., 12., 3., 2.}, {8., 5., 3., 7.}, {6., 14., 9., 25.}, {13., 4., 7., 8.}};
    Matrix<4> dut = kDut;
    EXPECT_EQ(kDut, dut);
    dut = Matrix<4>{{4., 18., 5., 6.}, {12., 12., 4., 12.}, {6., 22., 18., 38.}, {26., 7., 14., 13.}};
    EXPECT_EQ(dut, Matrix<4>({{4., 18., 5., 6.}, {12., 12., 4., 12.}, {6., 22., 18., 38.}, {26., 7., 14., 13.}}));
    dut = kDut;
    EXPECT_EQ(kDut, dut);
    EXPECT_EQ(kDut[0][2], 3.);
    EXPECT_EQ(kDut[1][3], 7.);
    dut[0][0] = 3;
    dut[0][1] = 6;
    dut[0][2] = 2;
    dut[0][3] = 4;
    dut[1][0] = 4;
    dut[1][1] = 7;
    dut[1][2] = 1;
    dut[1][3] = 5;
    dut[2][0] = 0;
    dut[2][1] = 8;
    dut[2][2] = 9;
    dut[2][3] = 13;
    dut[3][0] = 13;
    dut[3][1] = 3;
    dut[3][2] = 7;
    dut[3][3] = 5;
    EXPECT_EQ(dut[0][1], 6.);
    EXPECT_EQ(dut[3][2], 7.);
    EXPECT_NE(kDut, dut);
    EXPECT_EQ(kDut + dut,
              Matrix<4>({{4., 18., 5., 6.}, {12., 12., 4., 12.}, {6., 22., 18., 38.}, {26., 7., 14., 13.}}));
    EXPECT_EQ(kDut - dut, Matrix<4>({{-2., 6., 1., -2.}, {4., -2., 2., 2.}, {6., 6., 0., 12.}, {0., 1., 0., 3.}}));
    EXPECT_EQ(
        kDut * dut,
        Matrix<4>(
            {{77., 120., 55., 113.}, {135., 128., 97., 131.}, {399., 281., 282., 336.}, {159., 186., 149., 203.}}));
    EXPECT_EQ(kDut * 2, Matrix<4>({{2., 24., 6., 4.}, {16., 10., 6., 14.}, {12., 28., 18., 50.}, {26., 8., 14., 16.}}));
    EXPECT_EQ(2 * kDut, Matrix<4>({{2., 24., 6., 4.}, {16., 10., 6., 14.}, {12., 28., 18., 50.}, {26., 8., 14., 16.}}));
    std::stringstream ss;
    ss << kDut;
    EXPECT_EQ(ss.str(), "{{1, 12, 3, 2},\n {8, 5, 3, 7},\n {6, 14, 9, 25},\n {13, 4, 7, 8}}");
  }
}
GTEST_TEST(MatrixTest, StaticMethods) {
  {  // 2- dimension matrix.
    EXPECT_EQ(Matrix<2>::Identity(), Matrix<2>({{1., 0.}, {0., 1.}}));
  }
  {  // 3- dimension matrix.
    EXPECT_EQ(Matrix<3>::Identity(), Matrix<3>({{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}}));
  }
  {  // 4- dimension matrix.
    EXPECT_EQ(Matrix<4>::Identity(),
              Matrix<4>({{1., 0., 0., 0.}, {0., 1., 0., 0.}, {0., 0., 1., 0.}, {0., 0., 0., 1.}}));
  }
}

GTEST_TEST(MatrixTest, ExternalMethods) {
  Matrix<3> kDut{1., 2., 3., 4., 5., 6., 7., 8., 9};
  EXPECT_EQ((kDut * Vector3({1., 2., 3.})).z(), 50.);
}

}  // namespace
}  // namespace math
}  // namespace maliput
