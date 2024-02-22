// BSD 3-Clause License
//
// Copyright (c) 2024, Woven Planet. All rights reserved.
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
#include "maliput/math/fresnel.h"

#include <cmath>

namespace maliput {
namespace math {
namespace {

// The code in this unnamed namespace has been copied and modified from:
// https://github.com/pageldev/libOpenDRIVE/blob/master/src/Geometries/Spiral/odrSpiral.cpp
// Commit version: https://github.com/pageldev/libOpenDRIVE/commit/30c1b221cff5e86111b9fb66b53fc8a7967edf68
// License note in the file follows:
//
// /* ===================================================
//  *  file:       odrSpiral.c
//  * ---------------------------------------------------
//  *  purpose:	free method for computing spirals
//  *              in OpenDRIVE applications
//  * ---------------------------------------------------
//  *  using methods of CEPHES library
//  * ---------------------------------------------------
//  *  first edit:	09.03.2010 by M. Dupuis @ VIRES GmbH
//  *  last mod.:  09.03.2010 by M. Dupuis @ VIRES GmbH
//  * ===================================================
//     Copyright 2010 VIRES Simulationstechnologie GmbH
//
//     Licensed under the Apache License, Version 2.0 (the "License");
//     you may not use this file except in compliance with the License.
//     You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
//     Unless required by applicable law or agreed to in writing, software
//     distributed under the License is distributed on an "AS IS" BASIS,
//     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//     See the License for the specific language governing permissions and
//     limitations under the License.
//
//
//     NOTE:
//     The methods have been realized using the CEPHES library
//
//         http://www.netlib.org/cephes/
//
//     and do neither constitute the only nor the exclusive way of implementing
//     spirals for OpenDRIVE applications. Their sole purpose is to facilitate
//     the interpretation of OpenDRIVE spiral data.
//  */
//
// The following is a list of changes introduced in the functions:
// - Used explicit std version of the functions that belonged to the C math library.
// - Changed the name of constants to match our own formatting.
// - Moved coefficients within the fresnel functions instead of having static constants withing a global scope.
// - Accomodated variables, added const and other C++ related cosmetic changes.
// - Removed static to the functions, instead used unnamed namespaces.
// - Changed formatting.

double polevl(double x, const double* coef, int n) {
  const double* p = coef;
  double ans = *p++;
  int i = n;

  do {
    ans = ans * x + *p++;
  } while (--i);

  return ans;
}

double p1evl(double x, const double* coef, int n) {
  const double* p = coef;
  double ans = x + *p++;
  int i = n - 1;

  do {
    ans = ans * x + *p++;
  } while (--i);

  return ans;
}

void fresnel(double xxa, double* ssa, double* cca) {
  /* S(x) for small x */
  static double kSn[6] = {
      -2.99181919401019853726E3, 7.08840045257738576863E5,   -6.29741486205862506537E7,
      2.54890880573376359104E9,  -4.42979518059697779103E10, 3.18016297876567817986E11,
  };
  static double kSd[6] = {
      /* 1.00000000000000000000E0,*/
      2.81376268889994315696E2, 4.55847810806532581675E4,  5.17343888770096400730E6,
      4.19320245898111231129E8, 2.24411795645340920940E10, 6.07366389490084639049E11,
  };

  /* C(x) for small x */
  static double kCn[6] = {
      -4.98843114573573548651E-8, 9.50428062829859605134E-6,  -6.45191435683965050962E-4,
      1.88843319396703850064E-2,  -2.05525900955013891793E-1, 9.99999999999999998822E-1,
  };
  static double kCd[7] = {
      3.99982968972495980367E-12, 9.15439215774657478799E-10, 1.25001862479598821474E-7, 1.22262789024179030997E-5,
      8.68029542941784300606E-4,  4.12142090722199792936E-2,  1.00000000000000000118E0,
  };

  /* Auxiliary function f(x) */
  static double kFn[10] = {
      4.21543555043677546506E-1,  1.43407919780758885261E-1,  1.15220955073585758835E-2,  3.45017939782574027900E-4,
      4.63613749287867322088E-6,  3.05568983790257605827E-8,  1.02304514164907233465E-10, 1.72010743268161828879E-13,
      1.34283276233062758925E-16, 3.76329711269987889006E-20,
  };
  static double kFd[10] = {
      /*  1.00000000000000000000E0,*/
      7.51586398353378947175E-1,  1.16888925859191382142E-1,  6.44051526508858611005E-3,  1.55934409164153020873E-4,
      1.84627567348930545870E-6,  1.12699224763999035261E-8,  3.60140029589371370404E-11, 5.88754533621578410010E-14,
      4.52001434074129701496E-17, 1.25443237090011264384E-20,
  };

  /* Auxiliary function g(x) */
  static double kGn[11] = {
      5.04442073643383265887E-1,  1.97102833525523411709E-1,  1.87648584092575249293E-2,  6.84079380915393090172E-4,
      1.15138826111884280931E-5,  9.82852443688422223854E-8,  4.45344415861750144738E-10, 1.08268041139020870318E-12,
      1.37555460633261799868E-15, 8.36354435630677421531E-19, 1.86958710162783235106E-22,
  };
  static double kGd[11] = {
      /*  1.00000000000000000000E0,*/
      1.47495759925128324529E0,   3.37748989120019970451E-1,  2.53603741420338795122E-2,  8.14679107184306179049E-4,
      1.27545075667729118702E-5,  1.04314589657571990585E-7,  4.60680728146520428211E-10, 1.10273215066240270757E-12,
      1.38796531259578871258E-15, 8.39158816283118707363E-19, 1.86958710162783236342E-22,
  };

  const double x = std::fabs(xxa);
  const double x2 = x * x;

  // Done for small values of the argument xxa.
  auto impl_small_x = [&]() -> void {
    const double t = x2 * x2;
    *cca = x * x2 * polevl(t, kSn, 5) / p1evl(t, kSd, 6);
    *ssa = x * polevl(t, kCn, 5) / polevl(t, kCd, 6);
  };
  // Done for medium sized values of the argument xxa.
  auto impl_medium_x = [&]() -> void {
    double t = M_PI * x2;
    const double u = 1.0 / (t * t);
    const double t_inv = 1.0 / t;
    const double f_u = 1.0 - u * polevl(u, kFn, 9) / p1evl(u, kFd, 10);
    const double g_u = t_inv * polevl(u, kGn, 10) / p1evl(u, kGd, 11);

    t = M_PI * 0.5 * x2;
    const double c_t = std::cos(t);
    const double s_t = std::sin(t);

    t = M_PI * x;
    *ssa = 0.5 + (f_u * s_t - g_u * c_t) / t;
    *cca = 0.5 - (f_u * c_t + g_u * s_t) / t;
  };

  // Range and solution selection.
  if (x < 2.5625) {
    impl_small_x();
  } else if (x > 36974.0) {
    *cca = 0.5;
    *ssa = 0.5;
  } else {
    impl_medium_x();
  }

  // Fix the quadrant of the solution.
  if (xxa < 0.0) {
    *cca = -*cca;
    *ssa = -*ssa;
  }
}

}  // namespace

Vector2 ComputeFresnelCosineAndSine(double t) {
  double x{};
  double y{};
  fresnel(t, &y, &x);
  return Vector2{x, y};
}

double FresnelSpiralHeading(double t, double k_dot) { return 0.5 * t * t * k_dot; }

double FresnelSpiralHeadingDot(double t, double k_dot) { return t * k_dot; }

}  // namespace math
}  // namespace maliput
