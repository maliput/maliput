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
#include "maliput/api/lane_boundary.h"

#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/lane_marking.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace {

/// Mock implementation of LaneBoundary for testing.
class MockLaneBoundary : public LaneBoundary {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockLaneBoundary)

  /// Constructs a MockLaneBoundary.
  ///
  /// @param id The unique identifier for this boundary.
  /// @param segment The segment to which this boundary belongs (may be nullptr).
  /// @param index The index of this boundary within the segment.
  /// @param lane_to_left The lane to the left (may be nullptr).
  /// @param lane_to_right The lane to the right (may be nullptr).
  /// @param markings The lane markings for this boundary.
  MockLaneBoundary(const LaneBoundary::Id& id, const Segment* segment, int index, const Lane* lane_to_left,
                   const Lane* lane_to_right, const std::vector<LaneMarkingResult>& markings)
      : id_(id),
        segment_(segment),
        index_(index),
        lane_to_left_(lane_to_left),
        lane_to_right_(lane_to_right),
        markings_(markings) {}

 private:
  Id do_id() const override { return id_; }
  const Segment* do_segment() const override { return segment_; }
  int do_index() const override { return index_; }
  const Lane* do_lane_to_left() const override { return lane_to_left_; }
  const Lane* do_lane_to_right() const override { return lane_to_right_; }

  std::optional<LaneMarkingResult> DoGetMarking(double s) const override {
    for (const auto& result : markings_) {
      if (s >= result.s_start && s < result.s_end) {
        return result;
      }
    }
    return std::nullopt;
  }

  std::vector<LaneMarkingResult> DoGetMarkings() const override { return markings_; }

  std::vector<LaneMarkingResult> DoGetMarkings(double s_start, double s_end) const override {
    std::vector<LaneMarkingResult> result;
    for (const auto& marking : markings_) {
      // Check if the marking range overlaps with the query range.
      if (marking.s_start < s_end && marking.s_end > s_start) {
        result.push_back(marking);
      }
    }
    return result;
  }

  const Id id_;
  const Segment* segment_;
  const int index_;
  const Lane* lane_to_left_;
  const Lane* lane_to_right_;
  const std::vector<LaneMarkingResult> markings_;
};

// Test LaneBoundary::Id construction and accessors.
GTEST_TEST(LaneBoundaryIdTest, Construction) {
  const LaneBoundary::Id dut("boundary_0");
  EXPECT_EQ(dut.string(), "boundary_0");
}

GTEST_TEST(LaneBoundaryIdTest, Equality) {
  const LaneBoundary::Id id1("boundary_0");
  const LaneBoundary::Id id2("boundary_0");
  const LaneBoundary::Id id3("boundary_1");

  EXPECT_TRUE(id1 == id2);
  EXPECT_FALSE(id1 != id2);
  EXPECT_FALSE(id1 == id3);
  EXPECT_TRUE(id1 != id3);
}

// Test fixture for LaneBoundary tests.
class LaneBoundaryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create sample lane markings for the boundary.
    LaneMarking solid_white;
    solid_white.type = LaneMarkingType::kSolid;
    solid_white.color = LaneMarkingColor::kWhite;
    solid_white.weight = LaneMarkingWeight::kStandard;
    solid_white.width = 0.15;

    LaneMarking broken_white;
    broken_white.type = LaneMarkingType::kBroken;
    broken_white.color = LaneMarkingColor::kWhite;
    broken_white.weight = LaneMarkingWeight::kStandard;
    broken_white.width = 0.15;

    markings_ = {
        {solid_white, 0.0, 50.0},    // Solid white from s=0 to s=50
        {broken_white, 50.0, 100.0}  // Broken white from s=50 to s=100
    };
  }

  std::vector<LaneMarkingResult> markings_;
};

TEST_F(LaneBoundaryTest, IdAccessor) {
  const LaneBoundary::Id expected_id("test_boundary");
  MockLaneBoundary dut(expected_id, nullptr, 0, nullptr, nullptr, {});

  EXPECT_EQ(dut.id(), expected_id);
}

TEST_F(LaneBoundaryTest, SegmentAccessor) {
  // LaneBoundary with no segment (nullptr).
  MockLaneBoundary dut_no_segment(LaneBoundary::Id("boundary_0"), nullptr, 0, nullptr, nullptr, {});
  EXPECT_EQ(dut_no_segment.segment(), nullptr);
}

TEST_F(LaneBoundaryTest, IndexAccessor) {
  MockLaneBoundary dut(LaneBoundary::Id("boundary_2"), nullptr, 2, nullptr, nullptr, {});
  EXPECT_EQ(dut.index(), 2);
}

TEST_F(LaneBoundaryTest, LaneToLeftAccessor) {
  // Test with nullptr (leftmost boundary case).
  MockLaneBoundary dut_leftmost(LaneBoundary::Id("boundary_left"), nullptr, 0, nullptr, nullptr, {});
  EXPECT_EQ(dut_leftmost.lane_to_left(), nullptr);
}

TEST_F(LaneBoundaryTest, LaneToRightAccessor) {
  // Test with nullptr (rightmost boundary case).
  MockLaneBoundary dut_rightmost(LaneBoundary::Id("boundary_right"), nullptr, 0, nullptr, nullptr, {});
  EXPECT_EQ(dut_rightmost.lane_to_right(), nullptr);
}

TEST_F(LaneBoundaryTest, GetMarkingAtPosition) {
  MockLaneBoundary dut(LaneBoundary::Id("boundary_0"), nullptr, 0, nullptr, nullptr, markings_);

  // Query at s=25 (within solid white region).
  auto marking_at_25 = dut.GetMarking(25.0);
  ASSERT_TRUE(marking_at_25.has_value());
  EXPECT_EQ(marking_at_25->marking.type, LaneMarkingType::kSolid);
  EXPECT_EQ(marking_at_25->marking.color, LaneMarkingColor::kWhite);

  // Query at s=75 (within broken white region).
  auto marking_at_75 = dut.GetMarking(75.0);
  ASSERT_TRUE(marking_at_75.has_value());
  EXPECT_EQ(marking_at_75->marking.type, LaneMarkingType::kBroken);
  EXPECT_EQ(marking_at_75->marking.color, LaneMarkingColor::kWhite);

  // Query at s=50 (boundary between markings - should return broken).
  auto marking_at_50 = dut.GetMarking(50.0);
  ASSERT_TRUE(marking_at_50.has_value());
  EXPECT_EQ(marking_at_50->marking.type, LaneMarkingType::kBroken);
}

TEST_F(LaneBoundaryTest, GetMarkingOutsideRange) {
  MockLaneBoundary dut(LaneBoundary::Id("boundary_0"), nullptr, 0, nullptr, nullptr, markings_);

  // Query at s=100 (at the end, half-open interval [s_start, s_end) means s=100 is outside).
  auto marking_at_100 = dut.GetMarking(100.0);
  EXPECT_FALSE(marking_at_100.has_value());

  // Query at s=-1 (before the start).
  auto marking_at_neg = dut.GetMarking(-1.0);
  EXPECT_FALSE(marking_at_neg.has_value());
}

TEST_F(LaneBoundaryTest, GetMarkingsAll) {
  MockLaneBoundary dut(LaneBoundary::Id("boundary_0"), nullptr, 0, nullptr, nullptr, markings_);

  auto all_markings = dut.GetMarkings();
  ASSERT_EQ(all_markings.size(), 2u);

  EXPECT_EQ(all_markings[0].s_start, 0.0);
  EXPECT_EQ(all_markings[0].s_end, 50.0);
  EXPECT_EQ(all_markings[0].marking.type, LaneMarkingType::kSolid);

  EXPECT_EQ(all_markings[1].s_start, 50.0);
  EXPECT_EQ(all_markings[1].s_end, 100.0);
  EXPECT_EQ(all_markings[1].marking.type, LaneMarkingType::kBroken);
}

TEST_F(LaneBoundaryTest, GetMarkingsEmptyBoundary) {
  MockLaneBoundary dut(LaneBoundary::Id("boundary_empty"), nullptr, 0, nullptr, nullptr, {});

  auto all_markings = dut.GetMarkings();
  EXPECT_TRUE(all_markings.empty());
}

TEST_F(LaneBoundaryTest, GetMarkingsInRange) {
  MockLaneBoundary dut(LaneBoundary::Id("boundary_0"), nullptr, 0, nullptr, nullptr, markings_);

  // Query range that spans both markings.
  auto range_markings = dut.GetMarkings(40.0, 60.0);
  ASSERT_EQ(range_markings.size(), 2u);

  // Query range within first marking only.
  auto first_marking_only = dut.GetMarkings(10.0, 40.0);
  ASSERT_EQ(first_marking_only.size(), 1u);
  EXPECT_EQ(first_marking_only[0].marking.type, LaneMarkingType::kSolid);

  // Query range within second marking only.
  auto second_marking_only = dut.GetMarkings(60.0, 90.0);
  ASSERT_EQ(second_marking_only.size(), 1u);
  EXPECT_EQ(second_marking_only[0].marking.type, LaneMarkingType::kBroken);
}

TEST_F(LaneBoundaryTest, GetMarkingsRangeOutside) {
  MockLaneBoundary dut(LaneBoundary::Id("boundary_0"), nullptr, 0, nullptr, nullptr, markings_);

  // Query range completely outside.
  auto outside_range = dut.GetMarkings(200.0, 300.0);
  EXPECT_TRUE(outside_range.empty());
}

TEST_F(LaneBoundaryTest, GetMarkingsRangeAtBoundary) {
  MockLaneBoundary dut(LaneBoundary::Id("boundary_0"), nullptr, 0, nullptr, nullptr, markings_);

  // Query range that touches at exact boundary (should not include the first marking
  // since it ends at s=50 and we query starting at s=50).
  auto at_boundary = dut.GetMarkings(50.0, 60.0);
  ASSERT_EQ(at_boundary.size(), 1u);
  EXPECT_EQ(at_boundary[0].marking.type, LaneMarkingType::kBroken);
}

// Test LaneMarking struct equality operators.
GTEST_TEST(LaneMarkingTest, Equality) {
  LaneMarking marking1;
  marking1.type = LaneMarkingType::kSolid;
  marking1.color = LaneMarkingColor::kWhite;
  marking1.weight = LaneMarkingWeight::kStandard;
  marking1.width = 0.15;
  marking1.height = 0.0;
  marking1.lane_change = LaneChangePermission::kProhibited;
  marking1.material = "paint";

  LaneMarking marking2 = marking1;  // Copy

  EXPECT_TRUE(marking1 == marking2);
  EXPECT_FALSE(marking1 != marking2);

  // Modify one field.
  marking2.type = LaneMarkingType::kBroken;
  EXPECT_FALSE(marking1 == marking2);
  EXPECT_TRUE(marking1 != marking2);
}

// Test LaneMarkingLine struct equality operators.
GTEST_TEST(LaneMarkingLineTest, Equality) {
  LaneMarkingLine line1;
  line1.length = 3.0;
  line1.space = 9.0;
  line1.width = 0.15;
  line1.r_offset = 0.0;
  line1.color = LaneMarkingColor::kWhite;

  LaneMarkingLine line2 = line1;  // Copy

  EXPECT_TRUE(line1 == line2);
  EXPECT_FALSE(line1 != line2);

  // Modify one field.
  line2.length = 4.0;
  EXPECT_FALSE(line1 == line2);
  EXPECT_TRUE(line1 != line2);
}

// Test LaneMarkingResult struct equality operators.
GTEST_TEST(LaneMarkingResultTest, Equality) {
  LaneMarking marking;
  marking.type = LaneMarkingType::kSolid;

  LaneMarkingResult result1;
  result1.marking = marking;
  result1.s_start = 0.0;
  result1.s_end = 100.0;

  LaneMarkingResult result2 = result1;  // Copy

  EXPECT_TRUE(result1 == result2);
  EXPECT_FALSE(result1 != result2);

  // Modify one field.
  result2.s_start = 10.0;
  EXPECT_FALSE(result1 == result2);
  EXPECT_TRUE(result1 != result2);
}

// Test LaneMarkingType enum values.
GTEST_TEST(LaneMarkingTypeTest, EnumValues) {
  EXPECT_EQ(static_cast<int>(LaneMarkingType::kUnknown), 0);
  EXPECT_NE(LaneMarkingType::kSolid, LaneMarkingType::kBroken);
  EXPECT_NE(LaneMarkingType::kSolidSolid, LaneMarkingType::kSolidBroken);
}

// Test LaneMarkingWeight enum values.
GTEST_TEST(LaneMarkingWeightTest, EnumValues) {
  EXPECT_EQ(static_cast<int>(LaneMarkingWeight::kUnknown), 0);
  EXPECT_NE(LaneMarkingWeight::kStandard, LaneMarkingWeight::kBold);
}

// Test LaneMarkingColor enum values.
GTEST_TEST(LaneMarkingColorTest, EnumValues) {
  EXPECT_EQ(static_cast<int>(LaneMarkingColor::kUnknown), 0);
  EXPECT_NE(LaneMarkingColor::kWhite, LaneMarkingColor::kYellow);
}

// Test LaneChangePermission enum values.
GTEST_TEST(LaneChangePermissionTest, EnumValues) {
  EXPECT_EQ(static_cast<int>(LaneChangePermission::kUnknown), 0);
  EXPECT_NE(LaneChangePermission::kAllowed, LaneChangePermission::kProhibited);
  EXPECT_NE(LaneChangePermission::kToLeft, LaneChangePermission::kToRight);
}

// Test complex marking with multiple lines.
GTEST_TEST(LaneMarkingTest, ComplexMarkingWithLines) {
  LaneMarkingLine solid_line;
  solid_line.length = 0.0;
  solid_line.space = 0.0;  // space=0 indicates solid line
  solid_line.width = 0.15;
  solid_line.r_offset = -0.1;
  solid_line.color = LaneMarkingColor::kWhite;

  LaneMarkingLine broken_line;
  broken_line.length = 3.0;
  broken_line.space = 9.0;  // space>0 indicates broken line
  broken_line.width = 0.15;
  broken_line.r_offset = 0.1;
  broken_line.color = LaneMarkingColor::kWhite;

  LaneMarking solid_broken_marking;
  solid_broken_marking.type = LaneMarkingType::kSolidBroken;
  solid_broken_marking.color = LaneMarkingColor::kWhite;
  solid_broken_marking.weight = LaneMarkingWeight::kStandard;
  solid_broken_marking.width = 0.35;  // Total width including space between lines
  solid_broken_marking.lines = {solid_line, broken_line};

  EXPECT_EQ(solid_broken_marking.lines.size(), 2u);
  EXPECT_EQ(solid_broken_marking.lines[0].space, 0.0);  // Solid line
  EXPECT_GT(solid_broken_marking.lines[1].space, 0.0);  // Broken line
}

}  // namespace
}  // namespace api
}  // namespace maliput
