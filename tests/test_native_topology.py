from __future__ import annotations

import unittest

from robot_brain.navigation.topology import TopologyGraph, TopologyLandmark


class NativeTopologyTests(unittest.TestCase):
    def test_routes_through_landmarks_in_same_map(self) -> None:
        graph = TopologyGraph(max_edge_distance_m=3.0)
        graph.rebuild([
            TopologyLandmark("a", "room-a", 0, 0, map_id="office"),
            TopologyLandmark("door", "door", 2, 0, "door", map_id="office"),
            TopologyLandmark("b", "room-b", 4, 0, map_id="office"),
        ])
        route = graph.shortest_path((0, 0), (4, 0), map_id="office")
        self.assertEqual(["door", "b"], [item.landmark_id for item in route])

    def test_does_not_connect_landmarks_across_map_identity(self) -> None:
        graph = TopologyGraph(max_edge_distance_m=10.0)
        graph.rebuild([
            TopologyLandmark("a", "a", 0, 0, map_id="one"),
            TopologyLandmark("b", "b", 1, 0, map_id="two"),
        ])
        self.assertEqual((), graph.shortest_path((0, 0), (1, 0), map_id="one"))

    def test_update_removes_stale_bidirectional_edges(self) -> None:
        graph = TopologyGraph(max_edge_distance_m=2.0)
        graph.add(TopologyLandmark("a", "a", 0, 0))
        graph.add(TopologyLandmark("b", "b", 1, 0))
        graph.add(TopologyLandmark("b", "b", 10, 0))
        self.assertEqual((), graph.shortest_path((0, 0), (10, 0)))
