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

#include <memory>
#include <string>

#include "maliput/api/intersection_book.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/phase_ring_book.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/base/manual_phase_provider.h"

namespace maliput {

/// Instantiates and returns an api::IntersectionBook instance based on the
/// specified @p input document.
///
/// @param input The YAML Intersections document.
/// @param road_rulebook The book containing the road rules.
/// @param phase_ring_book The book containing the phase rings.
/// @param phase_provider The phase provider. Adds PhaseRings and sets their
/// initial states.
///
/// @return The newly created api::IntersectionBook instance.
std::unique_ptr<api::IntersectionBook> LoadIntersectionBook(const std::string& input,
                                                            const api::rules::RoadRulebook& road_rulebook,
                                                            const api::rules::PhaseRingBook& phase_ring_book,
                                                            const api::RoadGeometry* road_geometry,
                                                            ManualPhaseProvider* phase_provider);

/// Instantiates and returns an api::IntersectionBook instance based on the
/// specified @p filename.
///
/// @param filename The YAML file that contains a Intersections document.
/// @param road_rulebook The book containing the road rules.
/// @param phase_ring_book The book containing the phase rings.
/// @param phase_provider The phase provider. Adds PhaseRings and sets their
/// initial states.
///
/// @return The newly created api::IntersectionBook instance.
std::unique_ptr<api::IntersectionBook> LoadIntersectionBookFromFile(const std::string& filename,
                                                                    const api::rules::RoadRulebook& road_rulebook,
                                                                    const api::rules::PhaseRingBook& phase_ring_book,
                                                                    const api::RoadGeometry* road_geometry,
                                                                    ManualPhaseProvider* phase_provider);

}  // namespace maliput
