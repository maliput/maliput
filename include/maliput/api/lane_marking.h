// BSD 3-Clause License
//
// Copyright (c) 2025-2026, Woven by Toyota. All rights reserved.
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

#include <optional>
#include <string>
#include <vector>

namespace maliput {
namespace api {

/// @defgroup lane_marking Lane Marking API
/// @brief Data structures for describing lane boundary markings.
///
/// Lane markings describe the visual and physical characteristics of the
/// markings at lane boundaries. This includes painted lines, physical curbs,
/// and other delineation features.
///
/// The design is influenced by OpenDRIVE road marking specification and
/// OSI (Open Simulation Interface) lane boundary concepts.
/// @{

/// Describes the type/pattern of a lane marking line.
///
/// These types represent common road marking patterns found in road networks.
/// The naming follows OpenDRIVE conventions where applicable.
enum class LaneMarkingType {
  kUnknown = 0,   ///< Unknown or unspecified marking type.
  kNone,          ///< No visible marking present.
  kSolid,         ///< Continuous solid line.
  kBroken,        ///< Dashed/broken line pattern.
  kSolidSolid,    ///< Double solid lines (two parallel solid lines).
  kSolidBroken,   ///< Solid line on the inner side, broken on the outer side.
  kBrokenSolid,   ///< Broken line on the inner side, solid on the outer side.
  kBrokenBroken,  ///< Double broken lines (two parallel dashed lines).
  kBottsDots,     ///< Botts' dots (raised pavement markers).
  kGrass,         ///< Grass edge (natural boundary, no painted marking).
  kCurb,          ///< Physical curb boundary.
  kEdge,          ///< Edge line (typically at road boundary).
};

/// Describes the visual weight/thickness category of the marking.
///
/// Weight affects the visual prominence of the marking. Bold markings are
/// typically used to emphasize important boundaries.
enum class LaneMarkingWeight {
  kUnknown = 0,  ///< Unknown or unspecified weight.
  kStandard,     ///< Standard/normal width marking.
  kBold,         ///< Wide/bold marking (typically 2x standard width).
};

/// Describes the color of the lane marking.
///
/// Colors have different meanings depending on the country's traffic regulations.
/// For example:
/// - White: Typically separates traffic flowing in the same direction.
/// - Yellow: Typically separates traffic flowing in opposite directions (in some countries).
/// - Orange: Often used for temporary markings (e.g., construction zones).
enum class LaneMarkingColor {
  kUnknown = 0,  ///< Unknown or unspecified color.
  kWhite,        ///< White marking.
  kYellow,       ///< Yellow marking.
  kOrange,       ///< Orange marking (often temporary).
  kRed,          ///< Red marking.
  kBlue,         ///< Blue marking (e.g., handicap zones).
  kGreen,        ///< Green marking (e.g., bike lanes in some regions).
  kViolet,       ///< Violet/purple marking.
};

/// Describes the lane change permission indicated by the marking.
///
/// This represents the legal interpretation of the marking regarding
/// lane changes. The permission is typically determined by the marking
/// type (e.g., solid vs. broken lines).
///
/// The Rule API may provide more detailed lane change rules that
/// complement this marking-based permission. The recommendation is to
/// rely on the Rule API for comprehensive lane change logic, using
/// this marking information as supplementary context if needed.
enum class LaneChangePermission {
  kUnknown = 0,  ///< Unknown or unspecified permission.
  kAllowed,      ///< Lane change permitted (crossing the marking is allowed).
  kToLeft,       ///< Lane change only permitted towards the left (increasing r).
  kToRight,      ///< Lane change only permitted towards the right (decreasing r).
  kProhibited,   ///< Lane change prohibited (crossing the marking is not allowed).
};

/// Describes a single line within a lane marking.
///
/// Complex markings (like double lines) can be composed of multiple lines.
/// Each line has its own pattern, width, and optional color.
///
/// For broken/dashed lines:
/// - `length` is the length of the visible (painted) segment.
/// - `space` is the length of the gap between painted segments.
/// For solid lines:
/// - Both `length` and `space` should be 0 (their values are ignored).
///
/// The line type (solid vs. broken) is inferred from these fields:
/// - `space == 0` indicates a solid line.
/// - `space > 0` indicates a broken/dashed line.
///
/// Example: A standard broken white line might have:
/// - length = 3.0 m (painted segment)
/// - space = 9.0 m (gap)
/// - width = 0.15 m
struct LaneMarkingLine {
  /// Length of the visible (painted) part of each dash [m].
  /// For solid lines, this should be 0 (value is ignored).
  double length{0.};

  /// Length of the gap between visible parts [m].
  /// For solid lines, this should be 0.
  double space{0.};

  /// Width of this line [m].
  double width{0.};

  /// Lateral offset from the lane boundary [m].
  /// Positive values offset in the positive r-direction (towards the lane's
  /// left edge when facing the positive s-direction).
  /// This allows positioning multiple lines relative to each other.
  double r_offset{0.};

  /// Color of this specific line.
  /// If set to kUnknown, the parent LaneMarking's color should be used.
  LaneMarkingColor color{LaneMarkingColor::kUnknown};

  /// Compares two LaneMarkingLine instances for equality.
  bool operator==(const LaneMarkingLine& other) const {
    return length == other.length && space == other.space && width == other.width && r_offset == other.r_offset &&
           color == other.color;
  }

  /// Compares two LaneMarkingLine instances for inequality.
  bool operator!=(const LaneMarkingLine& other) const { return !(*this == other); }
};

/// Describes the complete lane marking at a lane boundary.
///
/// A LaneMarking describes all properties of the road marking at a lane's
/// boundary. The marking can vary along the lane's s-coordinate, so this
/// structure represents the marking for a specific s-range.
///
/// **Simple markings:** For single-line markings (e.g., a solid white edge line),
/// the `type`, `color`, `width`, and `weight` fields are sufficient. The `lines`
/// vector can be left empty.
///
/// **Complex markings:** For compound markings (e.g., double lines with different
/// patterns like `kSolidBroken`), the `lines` vector provides detailed information
/// about each component line. When `lines` is non-empty, the individual line
/// definitions take precedence for geometry and per-line colors. The top-level
/// `type`, `color`, and `width` fields remain useful as summary/fallback values.
struct LaneMarking {
  /// The type/pattern of the marking.
  LaneMarkingType type{LaneMarkingType::kUnknown};

  /// The visual weight of the marking.
  LaneMarkingWeight weight{LaneMarkingWeight::kUnknown};

  /// The primary color of the marking.
  /// For multi-colored markings, individual line colors can be specified
  /// in the `lines` vector.
  LaneMarkingColor color{LaneMarkingColor::kUnknown};

  /// Total width of the marking [m].
  /// For compound markings (like double lines), this is the total width
  /// including the space between lines.
  double width{0.};

  /// Height of the marking above the road surface [m].
  /// This is relevant for raised markings like Botts' dots or curbs.
  /// For painted markings, this is typically very small (a few millimeters).
  double height{0.};

  /// Lane change permission indicated by this marking.
  LaneChangePermission lane_change{LaneChangePermission::kUnknown};

  /// Material identifier for the marking.
  /// This is application-specific and can indicate properties like
  /// "standard", "thermoplastic", "reflective", "temporary", etc.
  std::string material{};

  /// Detailed line definitions for compound markings.
  /// For simple markings, this can be empty and `type`/`width`/`color` are used.
  /// For complex markings (double lines, etc.), this provides the detail of
  /// each individual line component. When non-empty, these definitions take
  /// precedence for geometry and per-line colors over the top-level fields.
  /// The line type (solid vs. broken) is determined by each line's `space` field:
  /// `space == 0` indicates solid, `space > 0` indicates broken.
  std::vector<LaneMarkingLine> lines{};

  /// Compares two LaneMarking instances for equality.
  bool operator==(const LaneMarking& other) const {
    return type == other.type && weight == other.weight && color == other.color && width == other.width &&
           height == other.height && lane_change == other.lane_change && material == other.material &&
           lines == other.lines;
  }

  /// Compares two LaneMarking instances for inequality.
  bool operator!=(const LaneMarking& other) const { return !(*this == other); }
};

/// Result of querying lane marking at a specific position or range.
///
/// This structure pairs a LaneMarking with the s-range over which it is valid.
/// Lane markings can change along the lane (e.g., solid to broken at an
/// intersection approach), so the validity range is essential.
///
/// The range uses half-open interval semantics: `[s_start, s_end)`.
struct LaneMarkingResult {
  /// The lane marking description.
  LaneMarking marking{};

  /// Start s-coordinate where this marking begins [m].
  /// This is relative to the lane's s-coordinate system (0 at lane start).
  double s_start{0.};

  /// End s-coordinate where this marking ends [m].
  /// This is relative to the lane's s-coordinate system.
  double s_end{0.};

  /// Compares two LaneMarkingResult instances for equality.
  bool operator==(const LaneMarkingResult& other) const {
    return marking == other.marking && s_start == other.s_start && s_end == other.s_end;
  }

  /// Compares two LaneMarkingResult instances for inequality.
  bool operator!=(const LaneMarkingResult& other) const { return !(*this == other); }
};

/// @}

}  // namespace api
}  // namespace maliput
