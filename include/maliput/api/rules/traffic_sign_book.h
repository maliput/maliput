// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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

#include "maliput/api/lane.h"
#include "maliput/api/rules/traffic_sign.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for providing the mapping from TrafficSign::Id to
/// TrafficSign.
///
/// This follows the same pattern as TrafficLightBook. Backend implementations
/// are responsible for populating the book with the signs present in their
/// road network data sources.
class TrafficSignBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(TrafficSignBook)

  virtual ~TrafficSignBook() = default;

  /// Returns all TrafficSigns in this book.
  std::vector<const TrafficSign*> TrafficSigns() const { return DoTrafficSigns(); }

  /// Gets the specified TrafficSign. Returns nullptr if @p id is unrecognized.
  const TrafficSign* GetTrafficSign(const TrafficSign::Id& id) const { return DoGetTrafficSign(id); }

  /// Returns all TrafficSigns whose related_lanes() includes @p lane_id.
  ///
  /// Returns an empty vector if no signs are associated with the given lane.
  std::vector<const TrafficSign*> FindByLane(const LaneId& lane_id) const { return DoFindByLane(lane_id); }

  /// Returns all TrafficSigns whose type matches @p type.
  ///
  /// For example, `FindByType(TrafficSignType::kStop)` returns all stop signs.
  /// Returns an empty vector if no signs match.
  std::vector<const TrafficSign*> FindByType(const TrafficSignType& type) const { return DoFindByType(type); }

 protected:
  TrafficSignBook() = default;

 private:
  virtual const TrafficSign* DoGetTrafficSign(const TrafficSign::Id& id) const = 0;

  virtual std::vector<const TrafficSign*> DoTrafficSigns() const = 0;

  virtual std::vector<const TrafficSign*> DoFindByLane(const LaneId& lane_id) const = 0;

  virtual std::vector<const TrafficSign*> DoFindByType(const TrafficSignType& type) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
