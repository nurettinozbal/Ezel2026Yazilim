"""ROS'suz, bounded kamera-lidar fusion çekirdeği.

Bu modül canonical topic'lere bağlanmaz. Girdi kontratları acquisition stamp'li
frame'lerdir; ROS ``base_link`` +Y-sol lidar koordinatı stack +sağ lateral
koordinatına burada ve yalnız burada çevrilir.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


HEALTHY_MAX_DT_S = 0.050
DEGRADED_MAX_DT_S = 0.100
_EPS = 1e-12


def finite_float(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def ros_left_to_stack_right(forward_m: Any, lateral_left_m: Any) -> Optional[Tuple[float, float]]:
    """ROS base +Y-left -> stack lateral +right (işaret tersleme)."""

    forward = finite_float(forward_m)
    left = finite_float(lateral_left_m)
    if forward is None or left is None:
        return None
    return forward, -left


def classify_time_delta(camera_stamp: Any, lidar_stamp: Any) -> Tuple[str, Optional[float]]:
    """Return ``(healthy|degraded|reject, |dt| milliseconds)``."""

    camera = finite_float(camera_stamp)
    lidar = finite_float(lidar_stamp)
    if camera is None or lidar is None:
        return "reject", None
    dt_s = abs(camera - lidar)
    dt_ms = dt_s * 1000.0
    if dt_s <= HEALTHY_MAX_DT_S + _EPS:
        return "healthy", dt_ms
    if dt_s <= DEGRADED_MAX_DT_S + _EPS:
        return "degraded", dt_ms
    return "reject", dt_ms


@dataclass(frozen=True)
class ReadinessFlags:
    model_loaded: bool = False
    camera_calibrated: bool = False
    lidar_calibrated: bool = False
    extrinsics_calibrated: bool = False

    def __post_init__(self) -> None:
        if not all(type(value) is bool for value in (
            self.model_loaded,
            self.camera_calibrated,
            self.lidar_calibrated,
            self.extrinsics_calibrated,
        )):
            raise ValueError("readiness flags must be bool")

    @property
    def ready(self) -> bool:
        return all((
            self.model_loaded,
            self.camera_calibrated,
            self.lidar_calibrated,
            self.extrinsics_calibrated,
        ))

    def as_dict(self) -> Dict[str, bool]:
        return {
            "model_loaded": bool(self.model_loaded),
            "camera_calibrated": bool(self.camera_calibrated),
            "lidar_calibrated": bool(self.lidar_calibrated),
            "extrinsics_calibrated": bool(self.extrinsics_calibrated),
            "ready": self.ready,
        }


@dataclass(frozen=True)
class CameraDetection:
    source_id: str
    bearing_right_deg: float
    color: str
    confidence: float
    bbox_norm_x: Optional[float] = None
    bbox_size: Optional[float] = None


@dataclass(frozen=True)
class LidarCluster:
    source_id: str
    forward_m: float
    lateral_right_m: float
    distance_m: float
    bearing_right_deg: float


@dataclass(frozen=True)
class CameraFrame:
    stamp: float
    detections: Tuple[CameraDetection, ...]
    invalid_count: int = 0


@dataclass(frozen=True)
class LidarFrame:
    stamp: float
    clusters: Tuple[LidarCluster, ...]
    invalid_count: int = 0


@dataclass(frozen=True)
class FusedObstacle:
    lidar_id: str
    forward_m: float
    lateral_m: float
    distance_m: float
    color: str
    confidence: float
    hard_obstacle: bool
    source: str
    camera_id: Optional[str] = None
    association_cost_deg: Optional[float] = None
    bbox_norm_x: Optional[float] = None
    bbox_size: Optional[float] = None


@dataclass(frozen=True)
class CameraDiagnostic:
    camera_id: str
    reason: str
    bearing_right_deg: float
    color: str


@dataclass(frozen=True)
class FusionStatus:
    temporal_health: str
    dt_ms: Optional[float]
    camera_stamp: float
    lidar_stamp: float
    readiness: ReadinessFlags
    accepted: bool
    matched_count: int
    ambiguous_camera_count: int
    ambiguous_lidar_count: int
    invalid_camera_count: int
    invalid_lidar_count: int
    clock_rollback_resets: int = 0
    association_overflow: bool = False
    overflow_component_count: int = 0
    overflow_camera_count: int = 0
    overflow_lidar_count: int = 0


@dataclass(frozen=True)
class FusionResult:
    obstacles: Tuple[FusedObstacle, ...]
    camera_diagnostics: Tuple[CameraDiagnostic, ...]
    status: FusionStatus


def sanitize_camera_frame(stamp: Any, detections: Any) -> Optional[CameraFrame]:
    parsed_stamp = finite_float(stamp)
    if parsed_stamp is None or not isinstance(detections, list):
        return None
    valid: List[CameraDetection] = []
    invalid = 0
    for index, raw in enumerate(detections):
        if not isinstance(raw, dict):
            invalid += 1
            continue
        bearing = finite_float(raw.get("bearing_deg", raw.get("bearing_right_deg")))
        confidence = finite_float(raw.get("confidence"))
        color = str(raw.get("color", "")).strip().lower()
        if (
            bearing is None
            or confidence is None
            or not -180.0 <= bearing <= 180.0
            or not 0.0 <= confidence <= 1.0
            or not color
        ):
            invalid += 1
            continue
        source_id = str(raw.get("id", raw.get("source_id", f"camera:{index}")))
        bbox_norm_x = finite_float(raw.get("bbox_norm_x"))
        bbox_size = finite_float(raw.get("bbox_size"))
        if bbox_norm_x is not None and not -1.0 <= bbox_norm_x <= 1.0:
            bbox_norm_x = None
        if bbox_size is not None and bbox_size < 0.0:
            bbox_size = None
        valid.append(CameraDetection(
            source_id, bearing, color, confidence, bbox_norm_x, bbox_size
        ))
    valid.sort(key=lambda d: (d.bearing_right_deg, d.source_id, d.color, -d.confidence))
    return CameraFrame(parsed_stamp, tuple(valid), invalid)


def sanitize_lidar_frame(stamp: Any, clusters: Any) -> Optional[LidarFrame]:
    parsed_stamp = finite_float(stamp)
    if parsed_stamp is None or not isinstance(clusters, list):
        return None
    valid: List[LidarCluster] = []
    invalid = 0
    for index, raw in enumerate(clusters):
        if not isinstance(raw, dict):
            invalid += 1
            continue
        converted = ros_left_to_stack_right(
            raw.get("forward_m"),
            raw.get("lateral_left_m"),
        )
        if converted is None:
            invalid += 1
            continue
        forward, right = converted
        distance = math.hypot(forward, right)
        if not math.isfinite(distance) or distance <= 0.0:
            invalid += 1
            continue
        source_id = str(raw.get("id", raw.get("source_id", f"lidar:{index}")))
        bearing = math.degrees(math.atan2(right, forward))
        valid.append(LidarCluster(source_id, forward, right, distance, bearing))
    valid.sort(key=lambda c: (c.bearing_right_deg, c.distance_m, c.source_id))
    return LidarFrame(parsed_stamp, tuple(valid), invalid)


class OrderedFrameBuffer:
    """Bounded, stamp-ordered, duplicate-stamp replacing frame buffer."""

    def __init__(self, max_size: int = 20) -> None:
        if (
            isinstance(max_size, bool)
            or not isinstance(max_size, int)
            or max_size <= 0
        ):
            raise ValueError("max_size must be a positive integer")
        self.max_size = max_size
        self._frames: List[Any] = []

    def clear(self) -> None:
        self._frames.clear()

    def add(self, frame: Any) -> None:
        self._frames = [existing for existing in self._frames if existing.stamp != frame.stamp]
        self._frames.append(frame)
        self._frames.sort(key=lambda item: item.stamp)
        if len(self._frames) > self.max_size:
            del self._frames[: len(self._frames) - self.max_size]

    def snapshot(self) -> Tuple[Any, ...]:
        return tuple(self._frames)

    def discard_stamp(self, stamp: float) -> None:
        self._frames = [frame for frame in self._frames if frame.stamp != stamp]

    def discard_older_than(self, minimum_stamp: float) -> None:
        self._frames = [frame for frame in self._frames if frame.stamp >= minimum_stamp]


def _angle_error_deg(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def associate_one_to_one(
    cameras: Sequence[CameraDetection],
    lidars: Sequence[LidarCluster],
    max_bearing_error_deg: float = 6.0,
    ambiguity_margin_deg: float = 0.25,
) -> Tuple[List[Tuple[int, int, float]], set[int], set[int]]:
    """Maximum-cardinality, then global minimum-cost deterministic matching.

    Returns matches and ambiguous camera/lidar indices. Ambiguous nodes are
    removed before optimization; no arbitrary color assignment is made.
    """

    gate = finite_float(max_bearing_error_deg)
    margin = finite_float(ambiguity_margin_deg)
    if gate is None or margin is None or gate < 0.0 or margin < 0.0:
        raise ValueError("association thresholds must be finite and >= 0")
    costs: Dict[Tuple[int, int], float] = {}
    for ci, camera in enumerate(cameras):
        for li, lidar in enumerate(lidars):
            cost = _angle_error_deg(camera.bearing_right_deg, lidar.bearing_right_deg)
            if cost <= gate + _EPS:
                costs[(ci, li)] = cost

    camera_id_counts: Dict[str, int] = {}
    lidar_id_counts: Dict[str, int] = {}
    for item in cameras:
        camera_id_counts[item.source_id] = camera_id_counts.get(item.source_id, 0) + 1
    for item in lidars:
        lidar_id_counts[item.source_id] = lidar_id_counts.get(item.source_id, 0) + 1
    ambiguous_cameras: set[int] = {
        index for index, item in enumerate(cameras)
        if camera_id_counts[item.source_id] > 1
    }
    ambiguous_lidars: set[int] = {
        index for index, item in enumerate(lidars)
        if lidar_id_counts[item.source_id] > 1
    }
    eligible_costs = {
        key: value for key, value in costs.items()
        if key[0] not in ambiguous_cameras and key[1] not in ambiguous_lidars
    }
    camera_indices = [i for i in range(len(cameras)) if i not in ambiguous_cameras]
    lidar_indices = [i for i in range(len(lidars)) if i not in ambiguous_lidars]
    base_matches, base_cost = _solve_assignment(
        eligible_costs, camera_indices, lidar_indices
    )

    # Exhaustive global identity possibilities. Her eligible edge zorlanarak,
    # kalan graph optimum çözülür; ayrıca her node graph'tan tamamen
    # çıkarılarak "unmatched" olasılığı test edilir. Base maksimum
    # cardinality ve base+margin maliyet bütçesi içindeki her olasılık partner
    # setine girer. Birden çok identity/unmatched seçeneği ambiguity'dir.
    base_cardinality = len(base_matches)
    budget = base_cost + margin + _EPS
    camera_possibilities: Dict[int, set[Optional[int]]] = {
        ci: set() for ci in camera_indices
    }
    lidar_possibilities: Dict[int, set[Optional[int]]] = {
        li: set() for li in lidar_indices
    }

    for (ci, li), edge_cost in sorted(eligible_costs.items()):
        remaining_cameras = [item for item in camera_indices if item != ci]
        remaining_lidars = [item for item in lidar_indices if item != li]
        remaining, remaining_cost = _solve_assignment(
            eligible_costs, remaining_cameras, remaining_lidars
        )
        if 1 + len(remaining) == base_cardinality and edge_cost + remaining_cost <= budget:
            camera_possibilities[ci].add(li)
            lidar_possibilities[li].add(ci)

    for ci in camera_indices:
        alternative, cost = _solve_assignment(
            eligible_costs,
            [item for item in camera_indices if item != ci],
            lidar_indices,
        )
        if len(alternative) == base_cardinality and cost <= budget:
            camera_possibilities[ci].add(None)
    for li in lidar_indices:
        alternative, cost = _solve_assignment(
            eligible_costs,
            camera_indices,
            [item for item in lidar_indices if item != li],
        )
        if len(alternative) == base_cardinality and cost <= budget:
            lidar_possibilities[li].add(None)

    ambiguous_cameras.update(
        ci for ci, possibilities in camera_possibilities.items()
        if len(possibilities) > 1
    )
    ambiguous_lidars.update(
        li for li, possibilities in lidar_possibilities.items()
        if len(possibilities) > 1
    )

    # Re-solve edilmez: ambiguity node'larını kaldırdıktan sonra yeni bir
    # identity mapping icat etmek yerine deterministic base optimumun yalnız iki
    # ucu da tek-anlamlı edge'leri tutulur.
    final_matches = [
        match for match in base_matches
        if match[0] not in ambiguous_cameras and match[1] not in ambiguous_lidars
    ]
    return final_matches, ambiguous_cameras, ambiguous_lidars


def _solve_assignment(
    costs: Dict[Tuple[int, int], float],
    camera_indices: Sequence[int],
    lidar_indices: Sequence[int],
    forbidden: Optional[set[Tuple[int, int]]] = None,
) -> Tuple[List[Tuple[int, int, float]], float]:
    """Unit-capacity max-cardinality/min-cost residual flow solver."""

    forbidden = forbidden or set()
    cameras = sorted(camera_indices)
    lidars = sorted(lidar_indices)
    camera_node = {identity: index + 1 for index, identity in enumerate(cameras)}
    lidar_base = 1 + len(cameras)
    lidar_node = {identity: lidar_base + index for index, identity in enumerate(lidars)}
    source = 0
    sink = lidar_base + len(lidars)
    graph: List[List[List[int]]] = [[] for _ in range(sink + 1)]

    def add_edge(start: int, end: int, cap: int, cost: int) -> List[int]:
        forward = [end, len(graph[end]), cap, cost]
        reverse = [start, len(graph[start]), 0, -cost]
        graph[start].append(forward)
        graph[end].append(reverse)
        return forward

    for ci in cameras:
        add_edge(source, camera_node[ci], 1, 0)
    match_edges: Dict[Tuple[int, int], List[int]] = {}
    for ci, li in sorted(costs):
        if (ci, li) in forbidden or ci not in camera_node or li not in lidar_node:
            continue
        match_edges[(ci, li)] = add_edge(
            camera_node[ci], lidar_node[li], 1,
            int(round(costs[(ci, li)] * 1_000_000)),
        )
    for li in lidars:
        add_edge(lidar_node[li], sink, 1, 0)

    count = sink + 1
    while True:
        distance: List[Optional[int]] = [None] * count
        previous: List[Optional[Tuple[int, int]]] = [None] * count
        distance[source] = 0
        for _ in range(count - 1):
            changed = False
            for start in range(count):
                if distance[start] is None:
                    continue
                for edge_index, edge in enumerate(graph[start]):
                    end, _reverse, capacity, edge_cost = edge
                    if capacity <= 0:
                        continue
                    proposal = distance[start] + edge_cost
                    if distance[end] is None or proposal < distance[end]:
                        distance[end] = proposal
                        previous[end] = (start, edge_index)
                        changed = True
            if not changed:
                break
        if distance[sink] is None:
            break
        node = sink
        while node != source:
            step = previous[node]
            if step is None:
                raise RuntimeError("incomplete association augment path")
            start, edge_index = step
            edge = graph[start][edge_index]
            edge[2] -= 1
            graph[node][edge[1]][2] += 1
            node = start

    matches: List[Tuple[int, int, float]] = [
        (ci, li, costs[(ci, li)])
        for (ci, li), edge in match_edges.items() if edge[2] == 0
    ]
    matches.sort(key=lambda item: (item[1], item[0]))
    return matches, sum(item[2] for item in matches)


def fuse_frames(
    camera_frame: CameraFrame,
    lidar_frame: LidarFrame,
    readiness: ReadinessFlags,
    *,
    max_bearing_error_deg: float = 6.0,
    ambiguity_margin_deg: float = 0.25,
    max_association_items: int = 16,
    clock_rollback_resets: int = 0,
) -> FusionResult:
    if (
        isinstance(max_association_items, bool)
        or not isinstance(max_association_items, int)
        or max_association_items <= 0
    ):
        raise ValueError("max_association_items must be a positive integer")
    health, dt_ms = classify_time_delta(camera_frame.stamp, lidar_frame.stamp)
    temporally_accepted = health != "reject"
    accepted = temporally_accepted and readiness.ready
    association_overflow = False
    overflow_component_count = 0
    overflow_cameras: set[int] = set()
    overflow_lidars: set[int] = set()
    matches: List[Tuple[int, int, float]] = []
    ambiguous_cameras: set[int] = set()
    ambiguous_lidars: set[int] = set()
    camera_id_counts: Dict[str, int] = {}
    lidar_id_counts: Dict[str, int] = {}
    for camera in camera_frame.detections:
        camera_id_counts[camera.source_id] = camera_id_counts.get(camera.source_id, 0) + 1
    for lidar in lidar_frame.clusters:
        lidar_id_counts[lidar.source_id] = lidar_id_counts.get(lidar.source_id, 0) + 1
    ambiguous_cameras.update(
        ci for ci, camera in enumerate(camera_frame.detections)
        if camera_id_counts[camera.source_id] > 1
    )
    ambiguous_lidars.update(
        li for li, lidar in enumerate(lidar_frame.clusters)
        if lidar_id_counts[lidar.source_id] > 1
    )
    if accepted:
        gate = finite_float(max_bearing_error_deg)
        margin = finite_float(ambiguity_margin_deg)
        if gate is None or margin is None or gate < 0.0 or margin < 0.0:
            raise ValueError("association thresholds must be finite and >= 0")
        candidate_edges: set[Tuple[int, int]] = set()
        for ci, camera in enumerate(camera_frame.detections):
            for li, lidar in enumerate(lidar_frame.clusters):
                if _angle_error_deg(camera.bearing_right_deg, lidar.bearing_right_deg) <= gate + _EPS:
                    candidate_edges.add((ci, li))

        # Duplicate source identities are a whole-frame contract violation,
        # including copies that have no bearing candidate edge. Quarantine all
        # copies before component construction.
        candidate_edges = {
            (ci, li) for ci, li in candidate_edges
            if ci not in ambiguous_cameras and li not in ambiguous_lidars
        }

        camera_neighbors: Dict[int, set[int]] = {}
        lidar_neighbors: Dict[int, set[int]] = {}
        for ci, li in candidate_edges:
            camera_neighbors.setdefault(ci, set()).add(li)
            lidar_neighbors.setdefault(li, set()).add(ci)

        unseen_cameras = set(camera_neighbors)
        components: List[Tuple[List[int], List[int]]] = []
        while unseen_cameras:
            pending_cameras = [min(unseen_cameras)]
            component_cameras: set[int] = set()
            component_lidars: set[int] = set()
            while pending_cameras:
                ci = pending_cameras.pop()
                if ci in component_cameras:
                    continue
                component_cameras.add(ci)
                unseen_cameras.discard(ci)
                for li in camera_neighbors.get(ci, ()):
                    if li in component_lidars:
                        continue
                    component_lidars.add(li)
                    pending_cameras.extend(
                        other for other in lidar_neighbors.get(li, ())
                        if other not in component_cameras
                    )
            components.append((sorted(component_cameras), sorted(component_lidars)))

        for camera_map, lidar_map in components:
            if (
                len(camera_map) > max_association_items
                or len(lidar_map) > max_association_items
            ):
                association_overflow = True
                overflow_component_count += 1
                overflow_cameras.update(camera_map)
                overflow_lidars.update(lidar_map)
                continue
            subset_matches, subset_ambiguous_cameras, subset_ambiguous_lidars = (
                associate_one_to_one(
                    [camera_frame.detections[index] for index in camera_map],
                    [lidar_frame.clusters[index] for index in lidar_map],
                    gate,
                    margin,
                )
            )
            matches.extend(
                (camera_map[ci], lidar_map[li], cost)
                for ci, li, cost in subset_matches
            )
            ambiguous_cameras.update(
                camera_map[index] for index in subset_ambiguous_cameras
            )
            ambiguous_lidars.update(
                lidar_map[index] for index in subset_ambiguous_lidars
            )

    match_by_lidar = {li: (ci, cost) for ci, li, cost in matches}
    matched_cameras = {ci for ci, _, _ in matches}
    obstacles: List[FusedObstacle] = []
    emitted_lidar_ids: set[str] = set()
    for li, lidar in enumerate(lidar_frame.clusters):
        # A duplicated upstream identity is fail-closed and represented once.
        # Sanitization sorting makes the retained metric representative
        # deterministic under input permutations.
        if lidar.source_id in emitted_lidar_ids:
            continue
        emitted_lidar_ids.add(lidar.source_id)
        association = match_by_lidar.get(li)
        if association is None:
            obstacles.append(FusedObstacle(
                lidar.source_id, lidar.forward_m, lidar.lateral_right_m,
                lidar.distance_m, "unknown", 1.0, True, "lidar_unknown",
            ))
            continue
        ci, cost = association
        camera = camera_frame.detections[ci]
        obstacles.append(FusedObstacle(
            lidar.source_id, lidar.forward_m, lidar.lateral_right_m,
            lidar.distance_m, camera.color, camera.confidence, True,
            "camera_lidar_fused", camera.source_id, cost,
            camera.bbox_norm_x, camera.bbox_size,
        ))

    diagnostics: List[CameraDiagnostic] = []
    for ci, camera in enumerate(camera_frame.detections):
        if ci in matched_cameras:
            continue
        if ci in overflow_cameras:
            reason = "association_overflow"
        elif ci in ambiguous_cameras:
            reason = "ambiguous"
        elif not accepted:
            reason = "fusion_not_ready" if health != "reject" else "time_rejected"
        else:
            reason = "unmatched"
        diagnostics.append(CameraDiagnostic(
            camera.source_id, reason, camera.bearing_right_deg, camera.color
        ))

    status = FusionStatus(
        temporal_health=health,
        dt_ms=dt_ms,
        camera_stamp=camera_frame.stamp,
        lidar_stamp=lidar_frame.stamp,
        readiness=readiness,
        accepted=accepted,
        matched_count=len(matches),
        ambiguous_camera_count=len(ambiguous_cameras),
        ambiguous_lidar_count=len(ambiguous_lidars),
        invalid_camera_count=camera_frame.invalid_count,
        invalid_lidar_count=lidar_frame.invalid_count,
        clock_rollback_resets=int(clock_rollback_resets),
        association_overflow=association_overflow,
        overflow_component_count=overflow_component_count,
        overflow_camera_count=len(overflow_cameras),
        overflow_lidar_count=len(overflow_lidars),
    )
    return FusionResult(tuple(obstacles), tuple(diagnostics), status)


class FusionCore:
    """Bounded acquisition and newest temporally-accepted frame fusion."""

    def __init__(self, buffer_size: int = 20, clock_rollback_threshold_s: float = 0.5) -> None:
        threshold = finite_float(clock_rollback_threshold_s)
        if threshold is None or threshold < 0.0:
            raise ValueError("clock_rollback_threshold_s must be finite and >= 0")
        self.camera_buffer = OrderedFrameBuffer(buffer_size)
        self.lidar_buffer = OrderedFrameBuffer(buffer_size)
        self.clock_rollback_threshold_s = threshold
        self._clock_stamp: Optional[float] = None
        self.clock_rollback_resets = 0

    def reset(self) -> None:
        self.camera_buffer.clear()
        self.lidar_buffer.clear()
        self._clock_stamp = None

    def _acquire(self, source: str, frame: Any, buffer: OrderedFrameBuffer) -> bool:
        if frame is None:
            return False
        # Source arrival order clock epoch değildir. Geç gelen frame ordered
        # insert edilir; iki buffer'ı yalnız explicit observe_clock temizler.
        buffer.add(frame)
        return True

    def observe_clock(self, now: Any, *, force_epoch_reset: bool = False) -> bool:
        """Observe ROS clock; return True when an epoch reset is performed."""

        stamp = finite_float(now)
        if stamp is None:
            return False
        rollback = bool(force_epoch_reset) or (
            self._clock_stamp is not None
            and stamp < self._clock_stamp - self.clock_rollback_threshold_s - _EPS
        )
        if rollback:
            self.reset()
            self.clock_rollback_resets += 1
            self._clock_stamp = stamp
            return True
        self._clock_stamp = stamp if self._clock_stamp is None else max(self._clock_stamp, stamp)
        return False

    def ingest_camera(self, stamp: Any, detections: Any) -> bool:
        return self._acquire("camera", sanitize_camera_frame(stamp, detections), self.camera_buffer)

    def ingest_lidar(self, stamp: Any, clusters: Any) -> bool:
        return self._acquire("lidar", sanitize_lidar_frame(stamp, clusters), self.lidar_buffer)

    def fuse_best(
        self,
        readiness: ReadinessFlags,
        *,
        consume: bool = False,
        now: Any = None,
        max_frame_age_s: Any = None,
        **kwargs: Any,
    ) -> Optional[FusionResult]:
        """Fuse newest accepted pair; by default buffers are non-consuming."""

        cameras = self.camera_buffer.snapshot()
        lidars = self.lidar_buffer.snapshot()
        if now is not None or max_frame_age_s is not None:
            current = finite_float(now)
            max_age = finite_float(max_frame_age_s)
            if current is None or max_age is None or max_age < 0.0:
                raise ValueError("now and max_frame_age_s must be finite; age must be >= 0")
            minimum = current - max_age
            self.camera_buffer.discard_older_than(minimum)
            self.lidar_buffer.discard_older_than(minimum)
            cameras = tuple(
                frame for frame in self.camera_buffer.snapshot()
                if -_EPS <= current - frame.stamp <= max_age + _EPS
            )
            lidars = tuple(
                frame for frame in self.lidar_buffer.snapshot()
                if -_EPS <= current - frame.stamp <= max_age + _EPS
            )
        if not cameras or not lidars:
            return None
        candidates = [
            (
                classify_time_delta(camera.stamp, lidar.stamp)[0] != "reject",
                max(camera.stamp, lidar.stamp),
                min(camera.stamp, lidar.stamp),
                -abs(camera.stamp - lidar.stamp),
                ci,
                li,
            )
            for ci, camera in enumerate(cameras)
            for li, lidar in enumerate(lidars)
        ]
        accepted = [candidate for candidate in candidates if candidate[0]]
        selected = max(accepted or candidates)
        _is_accepted, _newest, _oldest, _negative_dt, ci, li = selected
        result = fuse_frames(
            cameras[ci], lidars[li], readiness,
            clock_rollback_resets=self.clock_rollback_resets,
            **kwargs,
        )
        if consume:
            self.camera_buffer.discard_stamp(cameras[ci].stamp)
            self.lidar_buffer.discard_stamp(lidars[li].stamp)
        return result
