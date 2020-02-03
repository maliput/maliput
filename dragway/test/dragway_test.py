from __future__ import print_function

import copy
import unittest
import numpy as np

import pydrake.systems.framework as framework
from maliput.api import (
    LanePosition,
    LanePositionResult,
    GeoPosition,
    RoadGeometry,
    RoadGeometryId,
    RoadPosition,
    Rotation
)

from maliput.math import (
    Vector3
)
from dragway import create_dragway


# Instantiate and return a two-lane straight road.
def make_test_dragway(lane_width, length):
    kNumLanes = 2
    kShoulderWidth = 1.
    kHeight = 5.
    kTol = 1e-6

    rg_id = RoadGeometryId("two_lane_road")
    return create_dragway(
        road_id=rg_id, num_lanes=kNumLanes, length=length,
        lane_width=lane_width, shoulder_width=kShoulderWidth,
        maximum_height=kHeight, linear_tolerance=kTol,
        angular_tolerance=kTol)

# Returns a numpay.array from a vector.
def make_np_array(vector):
    return np.array([vector.x(), vector.y(), vector.z()])

# Tests the bindings for the API and backend implementations.
class TestMaliput(unittest.TestCase):
    def test_api(self):
        # Test the containers' constructors and accessors.
        srh = [4., 5., 6.]
        lane_pos = LanePosition(srh[0], srh[1], srh[2])
        self.assertTrue(len(make_np_array(lane_pos.srh())) == 3)
        self.assertTrue(np.allclose(make_np_array(lane_pos.srh()), srh))

        lane_pos_alt = LanePosition(s=4., r=5., h=6.)
        self.assertTrue(np.allclose(make_np_array(lane_pos_alt.srh()), srh))

        xyz = [1., 2., 3.]
        geo_pos = GeoPosition(xyz[0], xyz[1], xyz[2])
        self.assertTrue(len(make_np_array(geo_pos.xyz())) == 3)
        self.assertTrue(np.allclose(make_np_array(geo_pos.xyz()), xyz))

        geo_pos_alt = GeoPosition(x=1., y=2., z=3.)
        self.assertTrue(np.allclose(make_np_array(geo_pos_alt.xyz()), xyz))

        RoadPosition()
        rg = make_test_dragway(lane_width=4., length=100.)
        self.assertEqual(rg.num_junctions(), 1)
        junction = rg.junction(0)
        self.assertEqual(junction.num_segments(), 1)
        segment = junction.segment(0)
        self.assertEqual(segment.num_lanes(), 2)
        lane_0 = segment.lane(0)
        lane_1 = segment.lane(1)
        road_pos = RoadPosition(lane=lane_0, pos=lane_pos)
        self.assertEqual(road_pos.lane.id().string(), lane_0.id().string())
        self.assertTrue(np.allclose(make_np_array(road_pos.pos.srh()), make_np_array(lane_pos.srh())))
        road_pos.lane = lane_1
        self.assertEqual(road_pos.lane.id().string(), lane_1.id().string())
        new_srh = [42., 43., 44.]
        road_pos.pos = LanePosition(s=new_srh[0], r=new_srh[1], h=new_srh[2])
        self.assertTrue(np.allclose(make_np_array(road_pos.pos.srh()), new_srh))
        lane_orientation = lane_0.GetOrientation(lane_pos)
        self.assertTrue(np.allclose(
            lane_orientation.rpy().ToQuaternion().wxyz(),
            lane_orientation.quat().wxyz(),
        ))

        # Check that the getters are read-only.
        with self.assertRaises(TypeError):
            lane_pos.srh()[0] = 0.
        with self.assertRaises(TypeError):
            geo_pos.xyz()[0] = 0.

        # Test RoadGeometryId accessors.
        string = "foo"
        rg_id = RoadGeometryId(string)
        self.assertEqual(rg_id.string(), string)

    def test_dragway(self):
        kLaneWidth = 4.
        kLength = 100.
        rg = make_test_dragway(lane_width=kLaneWidth,
                               length=kLength)
        lane_0 = rg.junction(0).segment(0).lane(0)
        lane_1 = rg.junction(0).segment(0).lane(1)

        # Test that both Lanes have equal and expected lengths.
        self.assertEqual(lane_0.length(), kLength)
        self.assertEqual(lane_0.length(), lane_1.length())

        # Test Lane orientations for consistency.
        lane_start = LanePosition(0., 0., 0.)
        lane_end = LanePosition(lane_0.length(), 0., 0.)
        self.assertTrue(np.allclose(
            lane_0.GetOrientation(lane_start).quat().wxyz(),
            [1., 0., 0., 0.]
        ))
        self.assertTrue(np.allclose(
            lane_0.GetOrientation(lane_start).quat().wxyz(),
            lane_0.GetOrientation(lane_end).quat().wxyz()
        ))
        self.assertTrue(np.allclose(
            lane_1.GetOrientation(lane_start).quat().wxyz(),
            lane_1.GetOrientation(lane_end).quat().wxyz()
        ))
        self.assertTrue(np.allclose(
            lane_0.GetOrientation(lane_end).quat().wxyz(),
            lane_1.GetOrientation(lane_end).quat().wxyz()
        ))

        # Test the Lane <-> Geo space coordinate conversion.
        lane_pos = LanePosition(0., 0., 0.)
        geo_pos_result = lane_0.ToGeoPosition(lane_pos)
        geo_pos_expected = GeoPosition(0., -kLaneWidth / 2., 0.)
        self.assertTrue(np.allclose(make_np_array(geo_pos_result.xyz()),
                                    make_np_array(geo_pos_expected.xyz())))

        geo_pos = GeoPosition(1., kLaneWidth / 2., 3.)
        lane_pos_result = lane_1.ToLanePosition(geo_pos)
        lane_pos_result_expected = \
            LanePositionResult(LanePosition(1., 0., 3.), GeoPosition(0., 0., 0.), 0.)
        self.assertTrue(np.allclose(make_np_array(lane_pos_result.lane_position.srh()),
                                    make_np_array(lane_pos_result_expected.lane_position.srh())))
        self.assertTrue(np.allclose(make_np_array(lane_pos_result.nearest_position.xyz()), make_np_array(geo_pos.xyz())))
        self.assertTrue(lane_pos_result.distance == lane_pos_result_expected.distance)

    # TODO(jadecastro) Add more maliput backends as needed.
