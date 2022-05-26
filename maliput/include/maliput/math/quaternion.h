#pragma once
// Code in this file is inspired by:
// https://gitlab.com/libeigen/eigen/-/blob/master/Eigen/src/Geometry/Quaternion.h
//
// and https://bitbucket.org/ignitionrobotics/ign-math/src/default/include/ignition/math/Quaternion.hh
//
// ----------------------------------------------------------------------------
// Eigen's license follows:
//
// Mozilla Public License Version 2.0
// ==================================
//
// 1. Definitions
// --------------
//
// 1.1. "Contributor"
//     means each individual or legal entity that creates, contributes to
//     the creation of, or owns Covered Software.
//
// 1.2. "Contributor Version"
//     means the combination of the Contributions of others (if any) used
//     by a Contributor and that particular Contributor's Contribution.
//
// 1.3. "Contribution"
//     means Covered Software of a particular Contributor.
//
// 1.4. "Covered Software"
//     means Source Code Form to which the initial Contributor has attached
//     the notice in Exhibit A, the Executable Form of such Source Code
//     Form, and Modifications of such Source Code Form, in each case
//     including portions thereof.
//
// 1.5. "Incompatible With Secondary Licenses"
//     means
//
//     (a) that the initial Contributor has attached the notice described
//         in Exhibit B to the Covered Software; or
//
//     (b) that the Covered Software was made available under the terms of
//         version 1.1 or earlier of the License, but not also under the
//         terms of a Secondary License.
//
// 1.6. "Executable Form"
//     means any form of the work other than Source Code Form.
//
// 1.7. "Larger Work"
//     means a work that combines Covered Software with other material, in
//     a separate file or files, that is not Covered Software.
//
// 1.8. "License"
//     means this document.
//
// 1.9. "Licensable"
//     means having the right to grant, to the maximum extent possible,
//     whether at the time of the initial grant or subsequently, any and
//     all of the rights conveyed by this License.
//
// 1.10. "Modifications"
//     means any of the following:
//
//     (a) any file in Source Code Form that results from an addition to,
//         deletion from, or modification of the contents of Covered
//         Software; or
//
//     (b) any new file in Source Code Form that contains any Covered
//         Software.
//
// 1.11. "Patent Claims" of a Contributor
//     means any patent claim(s), including without limitation, method,
//     process, and apparatus claims, in any patent Licensable by such
//     Contributor that would be infringed, but for the grant of the
//     License, by the making, using, selling, offering for sale, having
//     made, import, or transfer of either its Contributions or its
//     Contributor Version.
//
// 1.12. "Secondary License"
//     means either the GNU General Public License, Version 2.0, the GNU
//     Lesser General Public License, Version 2.1, the GNU Affero General
//     Public License, Version 3.0, or any later versions of those
//     licenses.
//
// 1.13. "Source Code Form"
//     means the form of the work preferred for making modifications.
//
// 1.14. "You" (or "Your")
//     means an individual or a legal entity exercising rights under this
//     License. For legal entities, "You" includes any entity that
//     controls, is controlled by, or is under common control with You. For
//     purposes of this definition, "control" means (a) the power, direct
//     or indirect, to cause the direction or management of such entity,
//     whether by contract or otherwise, or (b) ownership of more than
//     fifty percent (50%) of the outstanding shares or beneficial
//     ownership of such entity.
//
// 2. License Grants and Conditions
// --------------------------------
//
// 2.1. Grants
//
// Each Contributor hereby grants You a world-wide, royalty-free,
// non-exclusive license:
//
// (a) under intellectual property rights (other than patent or trademark)
//     Licensable by such Contributor to use, reproduce, make available,
//     modify, display, perform, distribute, and otherwise exploit its
//     Contributions, either on an unmodified basis, with Modifications, or
//     as part of a Larger Work; and
//
// (b) under Patent Claims of such Contributor to make, use, sell, offer
//     for sale, have made, import, and otherwise transfer either its
//     Contributions or its Contributor Version.
//
// 2.2. Effective Date
//
// The licenses granted in Section 2.1 with respect to any Contribution
// become effective for each Contribution on the date the Contributor first
// distributes such Contribution.
//
// 2.3. Limitations on Grant Scope
//
// The licenses granted in this Section 2 are the only rights granted under
// this License. No additional rights or licenses will be implied from the
// distribution or licensing of Covered Software under this License.
// Notwithstanding Section 2.1(b) above, no patent license is granted by a
// Contributor:
//
// (a) for any code that a Contributor has removed from Covered Software;
//     or
//
// (b) for infringements caused by: (i) Your and any other third party's
//     modifications of Covered Software, or (ii) the combination of its
//     Contributions with other software (except as part of its Contributor
//     Version); or
//
// (c) under Patent Claims infringed by Covered Software in the absence of
//     its Contributions.
//
// This License does not grant any rights in the trademarks, service marks,
// or logos of any Contributor (except as may be necessary to comply with
// the notice requirements in Section 3.4).
//
// 2.4. Subsequent Licenses
//
// No Contributor makes additional grants as a result of Your choice to
// distribute the Covered Software under a subsequent version of this
// License (see Section 10.2) or under the terms of a Secondary License (if
// permitted under the terms of Section 3.3).
//
// 2.5. Representation
//
// Each Contributor represents that the Contributor believes its
// Contributions are its original creation(s) or it has sufficient rights
// to grant the rights to its Contributions conveyed by this License.
//
// 2.6. Fair Use
//
// This License is not intended to limit any rights You have under
// applicable copyright doctrines of fair use, fair dealing, or other
// equivalents.
//
// 2.7. Conditions
//
// Sections 3.1, 3.2, 3.3, and 3.4 are conditions of the licenses granted
// in Section 2.1.
//
// 3. Responsibilities
// -------------------
//
// 3.1. Distribution of Source Form
//
// All distribution of Covered Software in Source Code Form, including any
// Modifications that You create or to which You contribute, must be under
// the terms of this License. You must inform recipients that the Source
// Code Form of the Covered Software is governed by the terms of this
// License, and how they can obtain a copy of this License. You may not
// attempt to alter or restrict the recipients' rights in the Source Code
// Form.
//
// 3.2. Distribution of Executable Form
//
// If You distribute Covered Software in Executable Form then:
//
// (a) such Covered Software must also be made available in Source Code
//     Form, as described in Section 3.1, and You must inform recipients of
//     the Executable Form how they can obtain a copy of such Source Code
//     Form by reasonable means in a timely manner, at a charge no more
//     than the cost of distribution to the recipient; and
//
// (b) You may distribute such Executable Form under the terms of this
//     License, or sublicense it under different terms, provided that the
//     license for the Executable Form does not attempt to limit or alter
//     the recipients' rights in the Source Code Form under this License.
//
// 3.3. Distribution of a Larger Work
//
// You may create and distribute a Larger Work under terms of Your choice,
// provided that You also comply with the requirements of this License for
// the Covered Software. If the Larger Work is a combination of Covered
// Software with a work governed by one or more Secondary Licenses, and the
// Covered Software is not Incompatible With Secondary Licenses, this
// License permits You to additionally distribute such Covered Software
// under the terms of such Secondary License(s), so that the recipient of
// the Larger Work may, at their option, further distribute the Covered
// Software under the terms of either this License or such Secondary
// License(s).
//
// 3.4. Notices
//
// You may not remove or alter the substance of any license notices
// (including copyright notices, patent notices, disclaimers of warranty,
// or limitations of liability) contained within the Source Code Form of
// the Covered Software, except that You may alter any license notices to
// the extent required to remedy known factual inaccuracies.
//
// 3.5. Application of Additional Terms
//
// You may choose to offer, and to charge a fee for, warranty, support,
// indemnity or liability obligations to one or more recipients of Covered
// Software. However, You may do so only on Your own behalf, and not on
// behalf of any Contributor. You must make it absolutely clear that any
// such warranty, support, indemnity, or liability obligation is offered by
// You alone, and You hereby agree to indemnify every Contributor for any
// liability incurred by such Contributor as a result of warranty, support,
// indemnity or liability terms You offer. You may include additional
// disclaimers of warranty and limitations of liability specific to any
// jurisdiction.
//
// 4. Inability to Comply Due to Statute or Regulation
// ---------------------------------------------------
//
// If it is impossible for You to comply with any of the terms of this
// License with respect to some or all of the Covered Software due to
// statute, judicial order, or regulation then You must: (a) comply with
// the terms of this License to the maximum extent possible; and (b)
// describe the limitations and the code they affect. Such description must
// be placed in a text file included with all distributions of the Covered
// Software under this License. Except to the extent prohibited by statute
// or regulation, such description must be sufficiently detailed for a
// recipient of ordinary skill to be able to understand it.
//
// 5. Termination
// --------------
//
// 5.1. The rights granted under this License will terminate automatically
// if You fail to comply with any of its terms. However, if You become
// compliant, then the rights granted under this License from a particular
// Contributor are reinstated (a) provisionally, unless and until such
// Contributor explicitly and finally terminates Your grants, and (b) on an
// ongoing basis, if such Contributor fails to notify You of the
// non-compliance by some reasonable means prior to 60 days after You have
// come back into compliance. Moreover, Your grants from a particular
// Contributor are reinstated on an ongoing basis if such Contributor
// notifies You of the non-compliance by some reasonable means, this is the
// first time You have received notice of non-compliance with this License
// from such Contributor, and You become compliant prior to 30 days after
// Your receipt of the notice.
//
// 5.2. If You initiate litigation against any entity by asserting a patent
// infringement claim (excluding declaratory judgment actions,
// counter-claims, and cross-claims) alleging that a Contributor Version
// directly or indirectly infringes any patent, then the rights granted to
// You by any and all Contributors for the Covered Software under Section
// 2.1 of this License shall terminate.
//
// 5.3. In the event of termination under Sections 5.1 or 5.2 above, all
// end user license agreements (excluding distributors and resellers) which
// have been validly granted by You or Your distributors under this License
// prior to termination shall survive termination.
//
// ************************************************************************
// *                                                                      *
// *  6. Disclaimer of Warranty                                           *
// *  -------------------------                                           *
// *                                                                      *
// *  Covered Software is provided under this License on an "as is"       *
// *  basis, without warranty of any kind, either expressed, implied, or  *
// *  statutory, including, without limitation, warranties that the       *
// *  Covered Software is free of defects, merchantable, fit for a        *
// *  particular purpose or non-infringing. The entire risk as to the     *
// *  quality and performance of the Covered Software is with You.        *
// *  Should any Covered Software prove defective in any respect, You     *
// *  (not any Contributor) assume the cost of any necessary servicing,   *
// *  repair, or correction. This disclaimer of warranty constitutes an   *
// *  essential part of this License. No use of any Covered Software is   *
// *  authorized under this License except under this disclaimer.         *
// *                                                                      *
// ************************************************************************
//
// ************************************************************************
// *                                                                      *
// *  7. Limitation of Liability                                          *
// *  --------------------------                                          *
// *                                                                      *
// *  Under no circumstances and under no legal theory, whether tort      *
// *  (including negligence), contract, or otherwise, shall any           *
// *  Contributor, or anyone who distributes Covered Software as          *
// *  permitted above, be liable to You for any direct, indirect,         *
// *  special, incidental, or consequential damages of any character      *
// *  including, without limitation, damages for lost profits, loss of    *
// *  goodwill, work stoppage, computer failure or malfunction, or any    *
// *  and all other commercial damages or losses, even if such party      *
// *  shall have been informed of the possibility of such damages. This   *
// *  limitation of liability shall not apply to liability for death or   *
// *  personal injury resulting from such party's negligence to the       *
// *  extent applicable law prohibits such limitation. Some               *
// *  jurisdictions do not allow the exclusion or limitation of           *
// *  incidental or consequential damages, so this exclusion and          *
// *  limitation may not apply to You.                                    *
// *                                                                      *
// ************************************************************************
//
// 8. Litigation
// -------------
//
// Any litigation relating to this License may be brought only in the
// courts of a jurisdiction where the defendant maintains its principal
// place of business and such litigation shall be governed by laws of that
// jurisdiction, without reference to its conflict-of-law provisions.
// Nothing in this Section shall prevent a party's ability to bring
// cross-claims or counter-claims.
//
// 9. Miscellaneous
// ----------------
//
// This License represents the complete agreement concerning the subject
// matter hereof. If any provision of this License is held to be
// unenforceable, such provision shall be reformed only to the extent
// necessary to make it enforceable. Any law or regulation which provides
// that the language of a contract shall be construed against the drafter
// shall not be used to construe this License against a Contributor.
//
// 10. Versions of the License
// ---------------------------
//
// 10.1. New Versions
//
// Mozilla Foundation is the license steward. Except as provided in Section
// 10.3, no one other than the license steward has the right to modify or
// publish new versions of this License. Each version will be given a
// distinguishing version number.
//
// 10.2. Effect of New Versions
//
// You may distribute the Covered Software under the terms of the version
// of the License under which You originally received the Covered Software,
// or under the terms of any subsequent version published by the license
// steward.
//
// 10.3. Modified Versions
//
// If you create software not governed by this License, and you want to
// create a new license for such software, you may create and use a
// modified version of this License if you rename the license and remove
// any references to the name of the license steward (except to note that
// such modified license differs from this License).
//
// 10.4. Distributing Source Code Form that is Incompatible With Secondary
// Licenses
//
// If You choose to distribute Source Code Form that is Incompatible With
// Secondary Licenses under the terms of this version of the License, the
// notice described in Exhibit B of this License must be attached.
//
// Exhibit A - Source Code Form License Notice
// -------------------------------------------
//
//   This Source Code Form is subject to the terms of the Mozilla Public
//   License, v. 2.0. If a copy of the MPL was not distributed with this
//   file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// If it is not possible or desirable to put the notice in a particular
// file, then You may include the notice in a location (such as a LICENSE
// file in a relevant directory) where a recipient would be likely to look
// for such a notice.
//
// You may add additional accurate notices of copyright ownership.
//
// Exhibit B - "Incompatible With Secondary Licenses" Notice
// ---------------------------------------------------------
//
//   This Source Code Form is "Incompatible With Secondary Licenses", as
//   defined by the Mozilla Public License, v. 2.0.
//
// ----------------------------------------------------------------------------
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
// ----------------------------------------------------------------------------

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

#include <cmath>
#include <ostream>

#include "maliput/common/maliput_copyable.h"
#include "maliput/math/matrix.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace math {

/// A Quaternion representation.
class Quaternion {
 public:
  static constexpr double kTolerance{1e-15};

  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Quaternion);

  /// Constructs a Quaternion initialized to \f$ 1+0i+0j+0k \f$.
  Quaternion() : w_(1.), x_(0.), y_(0.), z_(0.) {}

  /// Constructs a Quaternion and initializes it to \f$ w+xi+yj+zk \f$.
  ///
  /// @param w The real \f$ w \f$ coefficient.
  /// @param x The internal \f$ x \f$ coefficient.
  /// @param y The internal \f$ y \f$ coefficient.
  /// @param z The internal \f$ z \f$s coefficient.
  Quaternion(double w, double x, double y, double z) : w_(w), x_(x), y_(y), z_(z) {}

  /// Constructs a Quaternion expressed by `angle` rotation around `axis` vector.
  ///
  /// @param angle A scalar representing an angle in radians.
  /// @param axis A 3D vector representing the rotation angle.
  Quaternion(double angle, const Vector3& axis);

  /// Constructs a Quaternion from a 4D vector matching all its components.
  ///
  /// @param coeffs A 4D vector whose first element is \f$ w \f$, second element
  /// is \f$ x \f$, third element is \f$ y \f$ and fourth element is \f$ z \f$.
  explicit Quaternion(const Vector4& coeffs) : w_(coeffs[0]), x_(coeffs[1]), y_(coeffs[2]), z_(coeffs[3]) {}

  /// Constructs a Quaternion from a 3D rotation matrix.
  ///
  /// @param rotation_matrix A 3x3 matrix whose elements represent a rotation
  /// matrix.
  /// @see
  /// https://bitbucket.org/ignitionrobotics/ign-math/src/ac54456ab8c431a59fbecfa1cf4b7c2b6f96cb33/include/ignition/math/Quaternion.hh#lines-461:503
  explicit Quaternion(const Matrix3& rotation_matrix);

  /// \addtogroup constcoeffgettersquaternion Constant coefficient getters of Quaternion.
  /// {@
  double w() const { return w_; }
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  /// }@

  /// \addtogroup mutablecoeffgettersquaternion Mutable coefficient getters of Quaternion.
  /// {@
  double& w() { return w_; }
  double& x() { return x_; }
  double& y() { return y_; }
  double& z() { return z_; }
  /// }@

  /// @return This quaternion vector: \f$ [x, y, z] \f$.
  Vector3 vec() const { return Vector3(x_, y_, z_); }

  /// @return This quaternion coefficients: \f$ [w, x, y, z] \f$.
  Vector4 coeffs() const { return Vector4(w_, x_, y_, z_); }

  /// @return A quaternion initialized with \f$ [1, 0, 0, 0] \f$.
  static Quaternion Identity();

  /// @return A quaternion initialized from two vectors.
  /// @see
  /// https://bitbucket.org/ignitionrobotics/ign-math/src/ac54456ab8c431a59fbecfa1cf4b7c2b6f96cb33/include/ignition/math/Quaternion.hh#lines-505:582
  static Quaternion FromTwoVectors(const Vector3& a, const Vector3& b);

  /// Sets the identity quaternion values as if it was initialized with
  /// \f$ [1, 0, 0, 0] \f$.
  ///
  /// @return `*this`.
  Quaternion& set_identity() {
    w_ = 1.;
    x_ = 0.;
    y_ = 0.;
    z_ = 0.;
    return *this;
  }

  /// @return The squared norm of this quaternion.
  double squared_norm() const { return x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_; }

  /// @return The norm of this quaternion.
  double norm() const { return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_); }

  /// Normalizes (`norm()` == 1) this quaternion coefficients.
  ///
  /// When the `norm()` < `kTolerance`, the result is equivalent to call
  /// `set_identity()`.
  void normalize() {
    const double s = norm();
    if (s < kTolerance) {
      set_identity();
    } else {
      w_ /= s;
      x_ /= s;
      y_ /= s;
      z_ /= s;
    }
  }

  /// @return `*this` quaternion with normalized coefficients.
  Quaternion normalized() const {
    Quaternion normalized_q(*this);
    normalized_q.normalize();
    return normalized_q;
  }

  /// Executes the dot product between this quaternion coefficients and `other`.
  ///
  /// @param other A Quaternion.
  /// @return The dot product between this quaternion and `other`.
  double dot(const Quaternion& other) const { return coeffs().dot(other.coeffs()); }

  /// Computes the angular distance between the rotation of this quaternion and
  /// `other`.
  ///
  /// @param other A Quaternion.
  /// @return The angle in radians between `this` rotation and `other`'s
  /// rotation.
  double AngularDistance(const Quaternion& other) const;

  /// @return A rotation Matrix3 equivalent to this normalized quaternion
  /// rotation.
  Matrix3 ToRotationMatrix() const;

  /// Sets this quaternion with the coefficients that makes `a` transform into
  /// `b`.
  ///
  /// @param a A 3D vector.
  /// @param b A 3D vector.
  /// @see
  /// https://bitbucket.org/ignitionrobotics/ign-math/src/ac54456ab8c431a59fbecfa1cf4b7c2b6f96cb33/include/ignition/math/Quaternion.hh#lines-505:582
  void SetFromTwoVectors(const Vector3& a, const Vector3& b);

  /// Product operator overload.
  ///
  /// @param q A Quaternion.
  /// @return A quaternion that results from \f$ `this` * `q` \f$.
  /// @see
  /// https://bitbucket.org/ignitionrobotics/ign-math/src/ac54456ab8c431a59fbecfa1cf4b7c2b6f96cb33/include/ignition/math/Quaternion.hh#lines-638:648
  Quaternion operator*(const Quaternion& q) const;

  /// Product and assignment operator overload.
  ///
  /// @param q A Quaternion.
  /// @return A mutable reference to `*this` initialized with \f$ `this` * `q` \f$.
  /// @see
  /// https://bitbucket.org/ignitionrobotics/ign-math/src/ac54456ab8c431a59fbecfa1cf4b7c2b6f96cb33/include/ignition/math/Quaternion.hh#lines-638:648
  Quaternion& operator*=(const Quaternion& q);

  /// Forwards the call to `TransformVector(v)`.
  Vector3 operator*(const Vector3& v) const;

  /// Operator equal to overload.
  ///
  /// Performs a coefficient wise comparison.
  ///
  /// @param other A quaternion to compare to `this` quaternion.
  /// @return true When `this`' and `other`'s coefficients are equal.
  bool operator==(const Quaternion& other) const;

  /// Operator not equal to overload.
  ///
  /// Performs a coefficient wise comparison.
  ///
  /// @param other A quaternion to compare to `this` quaternion.
  /// @return true When `this`' and `other`'s coefficients are not equal.
  bool operator!=(const Quaternion& other) const { return !operator==(other); }

  /// @return A Quaternion that results from computing the inverse rotation.
  Quaternion Inverse() const;

  /// @return A Quaternion that results from conjugating `this`.
  Quaternion conjugate() const { return Quaternion(w_, -x_, -y_, -z_); }

  /// @return The spherical linear interpolation between the two quaternions
  /// `*this` and `other` at the parameter `t`.
  ///
  /// @param t The interpolation parameter. It must be in [0; 1].
  /// @param other A quaternion.
  Quaternion Slerp(double t, const Quaternion& other) const;

  /// Evaluates if this quaternion is almost equal to `other` up to `precision`
  /// tolerance.
  ///
  /// `this` quaternion is approximately equal to `other` when the squared norm
  /// of the vector difference from both quaternion's coefficients is smaller
  /// than the minimum squared norm between `this` and `other` scaled with the
  /// squared precision.
  ///
  /// @param other A Quaternion to compare with.
  /// @param precision The tolerance.
  /// @return true When this quaternion is almost equal to `other` up to
  /// `precision` tolerance.
  bool IsApprox(const Quaternion& other, double precision) const;

  /// Applies this quaternion transformation to `v` 3D vector.
  ///
  /// @param v A 3D vector.
  /// @return A transformed 3D vector.
  Vector3 TransformVector(const Vector3& v) const;

 private:
  double w_{};
  double x_{};
  double y_{};
  double z_{};
};

/// Serialization operator overload.
///
/// @param os The output stream to serialize `q` into.
/// @param q A Quaternion.
/// @return `os`.
std::ostream& operator<<(std::ostream& os, const Quaternion& q);

}  // namespace math
}  // namespace maliput
