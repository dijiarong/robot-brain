"""Topological routing over robot-brain spatial landmarks."""
from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
from typing import Iterable


@dataclass(frozen=True)
class TopologyLandmark:
    landmark_id: str
    name: str
    x_m: float
    y_m: float
    kind: str = "landmark"
    frame_id: str = "map"
    map_id: str | None = None

    def __post_init__(self) -> None:
        if not self.landmark_id or not all(math.isfinite(v) for v in (self.x_m, self.y_m)):
            raise ValueError("invalid topology landmark")


@dataclass
class TopologyNode:
    landmark: TopologyLandmark
    neighbors: dict[str, float] = field(default_factory=dict)


class TopologyGraph:
    def __init__(self, *, max_edge_distance_m: float = 8.0) -> None:
        if max_edge_distance_m <= 0:
            raise ValueError("topology edge distance must be positive")
        self._max_edge = max_edge_distance_m
        self._nodes: dict[str, TopologyNode] = {}

    def add(self, landmark: TopologyLandmark) -> None:
        self.remove(landmark.landmark_id)
        node = TopologyNode(landmark)
        for existing in self._nodes.values():
            if not _same_space(landmark, existing.landmark):
                continue
            distance = math.hypot(
                landmark.x_m-existing.landmark.x_m,
                landmark.y_m-existing.landmark.y_m,
            )
            if distance < self._max_edge:
                node.neighbors[existing.landmark.landmark_id] = distance
                existing.neighbors[landmark.landmark_id] = distance
        self._nodes[landmark.landmark_id] = node

    def remove(self, landmark_id: str) -> None:
        self._nodes.pop(landmark_id, None)
        for node in self._nodes.values():
            node.neighbors.pop(landmark_id, None)

    def rebuild(self, landmarks: Iterable[TopologyLandmark]) -> None:
        self._nodes.clear()
        for landmark in landmarks:
            self.add(landmark)

    def nearest(self, x_m: float, y_m: float, *, frame_id: str = "map",
                map_id: str | None = None) -> TopologyNode | None:
        candidates = [node for node in self._nodes.values()
                      if node.landmark.frame_id == frame_id
                      and (map_id is None or node.landmark.map_id == map_id)]
        return min(candidates, key=lambda node: math.hypot(
            node.landmark.x_m-x_m, node.landmark.y_m-y_m), default=None)

    def shortest_path(self, start_xy: tuple[float, float], goal_xy: tuple[float, float],
                      *, frame_id: str = "map", map_id: str | None = None,
                      include_goal_landmark: bool = True) -> tuple[TopologyLandmark, ...]:
        start = self.nearest(*start_xy, frame_id=frame_id, map_id=map_id)
        goal = self.nearest(*goal_xy, frame_id=frame_id, map_id=map_id)
        if start is None or goal is None or start is goal:
            return ()
        frontier = [(0.0, start.landmark.landmark_id)]
        costs = {start.landmark.landmark_id: 0.0}
        previous: dict[str, str] = {}
        while frontier:
            _, current_id = heapq.heappop(frontier)
            if current_id == goal.landmark.landmark_id:
                ids = [current_id]
                while ids[-1] != start.landmark.landmark_id:
                    ids.append(previous[ids[-1]])
                ids.reverse()
                if not include_goal_landmark:
                    ids = ids[:-1]
                return tuple(self._nodes[node_id].landmark for node_id in ids[1:])
            current = self._nodes[current_id]
            for neighbor_id, distance in current.neighbors.items():
                candidate = costs[current_id] + distance
                if candidate >= costs.get(neighbor_id, math.inf):
                    continue
                costs[neighbor_id] = candidate
                previous[neighbor_id] = current_id
                neighbor = self._nodes[neighbor_id].landmark
                heuristic = math.hypot(neighbor.x_m-goal.landmark.x_m,
                                       neighbor.y_m-goal.landmark.y_m)
                heapq.heappush(frontier, (candidate + heuristic, neighbor_id))
        return ()

    @property
    def landmarks(self) -> tuple[TopologyLandmark, ...]:
        return tuple(node.landmark for node in self._nodes.values())


def _same_space(left: TopologyLandmark, right: TopologyLandmark) -> bool:
    return left.frame_id == right.frame_id and left.map_id == right.map_id
