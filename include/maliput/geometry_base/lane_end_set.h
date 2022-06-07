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
#pragma once

#include <vector>

#include "maliput/api/lane_data.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace geometry_base {

/// geometry_base's implementation of api::LaneEndSet.
class LaneEndSet : public api::LaneEndSet {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(LaneEndSet);

  /// Constructs an empty LaneEndSet.
  LaneEndSet() = default;

  /// Adds an api::LaneEnd to the set.
  ///
  /// @throws maliput::common::assertion_error if `end.lane` is nullptr.
  void Add(const api::LaneEnd& end) {
    // TODO(maddog@tri.global)  This assertion belongs in LaneEnd itself.
    MALIPUT_THROW_UNLESS(end.lane != nullptr);
    ends_.push_back(end);
  }

  ~LaneEndSet() override = default;

 private:
  int do_size() const override { return ends_.size(); }

  const api::LaneEnd& do_get(int index) const override { return ends_.at(index); }

  std::vector<api::LaneEnd> ends_;
};

}  // namespace geometry_base
}  // namespace maliput
