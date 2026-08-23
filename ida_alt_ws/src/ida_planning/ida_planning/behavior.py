"""P1/P2 navigation behavior arbitration and final motion envelope.

The local planner, heading alignment, stuck recovery and near-field guard are
allowed to *propose* a command.  This module is the single authority that
selects one behavior for the current tick and applies the non-negotiable final
motion envelope.  It deliberately has no ROS dependency so bag regressions can
exercise the exact production logic.
"""

from dataclasses import dataclass
import math
from typing import Any, Dict, Iterable, Mapping, Optional

from ida_planning.planner import Command


MODE_CRUISE = "CRUISE"
MODE_SLOW = "SLOW"
MODE_ALIGN = "ALIGN"
MODE_RECOVERY = "RECOVERY"
MODE_NEAR_STOP = "NEAR_STOP"
MODE_SAFETY_STOP = "SAFETY_STOP"


_PRIORITY = {
    MODE_CRUISE: 10,
    MODE_SLOW: 20,
    MODE_ALIGN: 30,
    MODE_RECOVERY: 40,
    MODE_NEAR_STOP: 50,
    MODE_SAFETY_STOP: 60,
}


@dataclass(frozen=True)
class NavigationDecision:
    """Final, single-authority navigation decision for one control tick."""

    command: Command
    mode: str
    requested_mode: str
    held: bool = False
    safety_reason: str = ""


class CenterObstacleEscapeController:
    """Break a differential-boat near-field pure-pivot deadlock safely.

    Some boat/SITL mixers produce almost no yaw while forward speed is zero.
    When the near-field guard reports the *same centered close obstacle* for a
    stable interval, this controller permits a short reverse arc.  The maneuver
    is time bounded, has a cooldown, and is refused when a hard obstacle is
    already close behind the boat.
    """

    def __init__(
        self,
        *,
        trigger_s: float,
        reverse_s: float,
        cooldown_s: float,
        reverse_speed_mps: float,
        yaw_deg_s: float,
        rear_stop_m: float,
        forward_commit_s: float = 0.0,
        forward_commit_speed_mps: float = 0.0,
        forward_commit_yaw_deg_s: float = 0.0,
    ) -> None:
        self.trigger_s = float(trigger_s)
        self.reverse_s = float(reverse_s)
        self.cooldown_s = float(cooldown_s)
        self.reverse_speed_mps = float(reverse_speed_mps)
        self.yaw_rate = math.radians(float(yaw_deg_s))
        self.rear_stop_m = float(rear_stop_m)
        self.forward_commit_s = float(forward_commit_s)
        self.forward_commit_speed_mps = float(forward_commit_speed_mps)
        self.forward_commit_yaw_rate = math.radians(
            float(forward_commit_yaw_deg_s)
        )
        values = (
            self.trigger_s,
            self.reverse_s,
            self.cooldown_s,
            self.reverse_speed_mps,
            self.yaw_rate,
            self.rear_stop_m,
            self.forward_commit_s,
            self.forward_commit_speed_mps,
            self.forward_commit_yaw_rate,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("center obstacle escape parameters must be finite")
        if min(values) < 0.0 or self.reverse_s <= 0.0:
            raise ValueError("center obstacle escape parameters must be non-negative")
        self.reset()

    def reset(self) -> None:
        self._center_since: Optional[float] = None
        self._active_until: Optional[float] = None
        self._forward_until: Optional[float] = None
        self._cooldown_until = 0.0
        self._yaw_sign = 1.0
        self._flip_next_attempt = False
        self._last_now: Optional[float] = None

    def _rear_blocked(self, obstacles: Iterable[Dict[str, Any]]) -> bool:
        for obstacle in obstacles:
            if not isinstance(obstacle, dict) or not bool(
                obstacle.get("hard_obstacle", True)
            ):
                continue
            try:
                forward = float(obstacle.get("forward_m", obstacle.get("x", 0.0)))
                lateral = float(obstacle.get("lateral_m", obstacle.get("y", 0.0)))
            except (TypeError, ValueError, OverflowError):
                continue
            if not (math.isfinite(forward) and math.isfinite(lateral)):
                continue
            if forward < -0.05 and math.hypot(forward, lateral) <= self.rear_stop_m:
                return True
        return False

    def apply(
        self,
        command: Command,
        obstacles: Iterable[Dict[str, Any]],
        now: float,
    ) -> tuple[Command, str]:
        now = float(now)
        if not math.isfinite(now):
            self.reset()
            return command, "invalid_time"
        if self._last_now is not None and now < self._last_now:
            self.reset()
        self._last_now = now

        near_stopped = str(command.action or "").startswith("near_field_stop")
        obstacle_snapshot = list(obstacles)
        if self._rear_blocked(obstacle_snapshot):
            self._active_until = None
            self._forward_until = None
            return command, "rear_blocked"

        if self._active_until is not None:
            if now < self._active_until:
                return (
                    Command(
                        -self.reverse_speed_mps,
                        0.0,
                        math.copysign(self.yaw_rate, self._yaw_sign),
                        "near_field_escape_reverse",
                    ),
                    "reverse_active",
                )
            self._active_until = None
            if (
                not near_stopped
                and self.forward_commit_s > 0.0
                and self.forward_commit_speed_mps > 0.0
                and self.forward_commit_yaw_rate > 0.0
            ):
                # A reverse arc changes the bow angle but the old controller
                # immediately handed control back to a straight DWA command.
                # Complete the manoeuvre with the opposite steering sense,
                # like the second half of a three-point turn.  This is entered
                # only when the near-field guard already considers the forward
                # corridor clear; the arbiter's safety envelope still runs
                # afterwards on every tick.
                self._forward_until = now + self.forward_commit_s
                return (
                    Command(
                        self.forward_commit_speed_mps,
                        0.0,
                        -math.copysign(
                            self.forward_commit_yaw_rate, self._yaw_sign
                        ),
                        "near_field_escape_forward_commit",
                    ),
                    "forward_commit_started",
                )
            # The pulse did not create a clear forward corridor. Trying the
            # same side forever recreates the observed reverse/forward sweep;
            # the next bounded attempt explores the other side.
            self._flip_next_attempt = True
            self._center_since = now
            self._cooldown_until = now + self.cooldown_s
            return command, "cooldown"

        if self._forward_until is not None:
            if near_stopped:
                self._forward_until = None
                self._flip_next_attempt = True
                self._center_since = now
                self._cooldown_until = now + self.cooldown_s
                return command, "forward_commit_blocked"
            if now < self._forward_until:
                return (
                    Command(
                        self.forward_commit_speed_mps,
                        0.0,
                        -math.copysign(
                            self.forward_commit_yaw_rate, self._yaw_sign
                        ),
                        "near_field_escape_forward_commit",
                    ),
                    "forward_commit_active",
                )
            self._forward_until = None
            self._center_since = None
            self._flip_next_attempt = False
            self._cooldown_until = now + self.cooldown_s
            return command, "forward_commit_complete"

        if not near_stopped:
            self._center_since = None
            self._flip_next_attempt = False
            return command, "clear"

        if now < self._cooldown_until:
            return command, "cooldown"
        if self._center_since is None:
            self._center_since = now
            return command, "arming"
        if now - self._center_since < self.trigger_s:
            return command, "arming"

        # Preserve the deterministic near-field turn direction throughout the
        # pulse; changing it every tick would recreate the left/right chatter.
        requested_sign = 1.0 if float(command.yaw_rate) >= 0.0 else -1.0
        if self._flip_next_attempt:
            self._yaw_sign = -self._yaw_sign
            self._flip_next_attempt = False
        else:
            self._yaw_sign = requested_sign
        self._active_until = now + self.reverse_s
        return (
            Command(
                -self.reverse_speed_mps,
                0.0,
                math.copysign(self.yaw_rate, self._yaw_sign),
                "near_field_escape_reverse",
            ),
            "reverse_started",
        )


def navigation_mode(command: Command) -> str:
    """Classify a proposal without parsing nested implementation details."""

    action = str(command.action or "").lower()
    if action.startswith("navigation_safety_stop"):
        return MODE_SAFETY_STOP
    if action.startswith("near_field_stop") or action.startswith("hold "):
        return MODE_NEAR_STOP
    if action.startswith("near_field_escape_reverse"):
        return MODE_RECOVERY
    if action.startswith("near_field_escape_forward_commit"):
        return MODE_RECOVERY
    if "recovery" in action:
        return MODE_RECOVERY
    if "align" in action:
        return MODE_ALIGN
    if action.startswith("near_field_slow") or " slow=true" in action:
        return MODE_SLOW
    return MODE_CRUISE


def nearest_forward_hard_obstacle_m(
    obstacles: Iterable[Dict[str, Any]],
    *,
    rear_clearance_m: float,
    lateral_clearance_m: float,
) -> Optional[float]:
    """Return nearest finite hard obstacle in the forward motion envelope."""

    nearest = float("inf")
    for obstacle in obstacles:
        if not isinstance(obstacle, dict):
            continue
        if not bool(obstacle.get("hard_obstacle", True)):
            continue
        try:
            forward = float(obstacle.get("forward_m", obstacle.get("x", 0.0)))
            lateral = float(obstacle.get("lateral_m", obstacle.get("y", 0.0)))
        except (TypeError, ValueError, OverflowError):
            continue
        if not (math.isfinite(forward) and math.isfinite(lateral)):
            continue
        if forward < rear_clearance_m:
            continue
        # The non-negotiable stop applies to the hull collision corridor, not
        # every obstacle inside a radial circle. Two gate buoys can both be
        # <2 m radially while leaving a clear, navigable channel between them.
        if abs(lateral) > lateral_clearance_m:
            continue
        nearest = min(nearest, math.hypot(forward, lateral))
    return nearest if math.isfinite(nearest) else None


def _nearest_rotational_sweep_obstacle(
    obstacles: Iterable[Dict[str, Any]],
    *,
    sweep_radius_m: float,
) -> Optional[tuple[float, float, float]]:
    """Return the nearest hard obstacle inside the hull's pivot sweep.

    A rectangular boat needs more room to rotate than its lateral half-width.
    The ordinary forward corridor therefore cannot protect a buoy that has
    already moved alongside or just behind the stern.  Distance is measured in
    the body frame and the returned tuple is ``(range, forward, lateral)``.
    """

    radius = max(0.0, float(sweep_radius_m))
    if radius <= 0.0:
        return None
    nearest: Optional[tuple[float, float, float]] = None
    for obstacle in obstacles:
        if not isinstance(obstacle, dict):
            continue
        if not bool(obstacle.get("hard_obstacle", True)):
            continue
        try:
            forward = float(obstacle.get("forward_m", obstacle.get("x", 0.0)))
            lateral = float(obstacle.get("lateral_m", obstacle.get("y", 0.0)))
        except (TypeError, ValueError, OverflowError):
            continue
        if not (math.isfinite(forward) and math.isfinite(lateral)):
            continue
        distance = math.hypot(forward, lateral)
        if distance > radius:
            continue
        candidate = (distance, forward, lateral)
        if nearest is None or candidate < nearest:
            nearest = candidate
    return nearest


def navigation_safety_envelope(
    command: Command,
    obstacles: Iterable[Dict[str, Any]],
    *,
    high_yaw_deg_s: float,
    obstacle_stop_m: float,
    rear_clearance_m: float = -0.59,
    lateral_clearance_m: float = 1.0,
    rotational_sweep_radius_m: float = 0.0,
    rear_sweep_escape_speed_mps: float = 0.0,
) -> tuple[Command, str]:
    """Remove forward thrust during high-yaw or close-obstacle maneuvers.

    Yaw is normally preserved so a differential boat can pivot to create a
    safe path.  When a hard obstacle lies inside the rectangular hull's
    rotational sweep, pivoting is forbidden.  A rear-only obstacle is cleared
    with a bounded straight-forward translation; every other sweep conflict
    stops both thrust and yaw.  This function is intentionally the *last*
    command transformation; neither DWA nor recovery may bypass it.
    """

    try:
        vx = float(command.vx)
        vy = float(command.vy)
        yaw_rate = float(command.yaw_rate)
    except (TypeError, ValueError, OverflowError):
        return Command(0.0, 0.0, 0.0, "navigation_safety_stop invalid_command"), "invalid_command"
    if not all(math.isfinite(value) for value in (vx, vy, yaw_rate)):
        return Command(0.0, 0.0, 0.0, "navigation_safety_stop invalid_command"), "invalid_command"
    reasons = []
    yaw_limit = math.radians(max(0.0, float(high_yaw_deg_s)))
    high_yaw = yaw_limit > 0.0 and abs(yaw_rate) >= yaw_limit
    if high_yaw:
        reasons.append("high_yaw")

    nearest = nearest_forward_hard_obstacle_m(
        obstacles,
        rear_clearance_m=float(rear_clearance_m),
        lateral_clearance_m=max(0.0, float(lateral_clearance_m)),
    )
    stop_distance = max(0.0, float(obstacle_stop_m))
    if nearest is not None and nearest <= stop_distance:
        reasons.append(f"obstacle_{nearest:.2f}m")

    # Rosbag regression (2026-08-19): a buoy at approximately
    # forward=-0.84 m/lateral=0.04 m was outside the forward corridor, but an
    # in-place high-yaw turn swept the stern through it.  Deal with that
    # geometry before the legacy high-yaw pivot conversion.
    # Aşağıdaki koruma yalnız ``high_yaw`` ile sınırlı olamaz. P2 DWA'nın
    # normal kaçış dönüşü 20 deg/s, high-yaw eşiği ise 22.5 deg/s idi; bu
    # nedenle 2026-08-22 seed 6722'de tekne 0 m/s ile pivot ederken gövde
    # köşesi p2_yellow_8'e iki tekrarda da sürttü. Dönüş süpürme zarfına giren
    # engeli her gerçek dönüşte denetle; düşük-yaw durumda dönüşü engelden
    # uzağa çevirerek ileri itmeyi sıfırla. Büyük/high-yaw dönüşlerin önceki
    # fail-closed davranışı değişmez.
    turning = abs(yaw_rate) > math.radians(0.1)
    if high_yaw or turning:
        sweep = _nearest_rotational_sweep_obstacle(
            obstacles,
            sweep_radius_m=rotational_sweep_radius_m,
        )
        if sweep is not None:
            distance, forward, lateral = sweep
            escape_speed = max(0.0, float(rear_sweep_escape_speed_mps))
            forward_path_blocked = nearest is not None and nearest <= stop_distance
            if (
                vx >= 0.0
                and forward < 0.0
                and not forward_path_blocked
                and escape_speed > 0.0
            ):
                return (
                    Command(
                        escape_speed if vx <= 0.0 else min(vx, escape_speed),
                        0.0,
                        0.0,
                        f"near_field_slow rear_sweep_escape_{distance:.2f}m",
                    ),
                    f"rear_sweep_escape_{distance:.2f}m",
                )
            # Düşük-yaw dönüşünde süpürme zarfına giren yan/ön engelden erken
            # uzağa pivot et. Koridorun turuncu kapı dubaları 1.3 m zarfının
            # dışındadır; bu kural yalnız gövde köşesinin gerçekten süpüreceği
            # yakın engelleri sahiplenir.
            if not high_yaw and abs(lateral) > 0.05:
                # Body frame'de lateral ve yaw aynı işaretlidir: lateral<0
                # engel için negatif yaw, dubayı burnun üzerinden geçirmek
                # yerine aynı tarafta büyüyen bir bearing ile uzaklaştırır.
                away_yaw = math.copysign(abs(yaw_rate), lateral)
                return (
                    Command(
                        0.0,
                        0.0,
                        away_yaw,
                        f"navigation_safety_stop pivot_away_sweep_{distance:.2f}m",
                    ),
                    f"pivot_away_sweep_{distance:.2f}m",
                )
            return (
                Command(
                    0.0,
                    0.0,
                    0.0,
                    f"navigation_safety_stop rotational_sweep_{distance:.2f}m",
                ),
                f"rotational_sweep_{distance:.2f}m",
            )

    # Reverse escape arcs are a separate, rear-clearance-gated behavior.  The
    # legacy envelope must not convert them back into a stationary pivot.
    # Zero-vx pivots, however, were already checked above for hull sweep.
    if vx <= 0.0:
        return command, ""

    if not reasons:
        return command, ""
    reason = "+".join(reasons)
    return (
        Command(
            0.0,
            0.0,
            yaw_rate,
            f"navigation_safety_stop {reason}",
        ),
        reason,
    )


class NavigationBehaviorArbiter:
    """Stateful, priority-ordered behavior selector with mode dwell.

    Higher-priority requests preempt immediately.  A lower-priority request is
    accepted only after the active behavior's minimum dwell has elapsed.  The
    selected command is then passed through the final safety envelope again,
    so a held recovery command cannot carry unsafe forward thrust into a newly
    observed obstacle.
    """

    def __init__(
        self,
        *,
        min_dwell_s: Optional[Mapping[str, float]] = None,
        high_yaw_deg_s: float = 22.5,
        obstacle_stop_m: float = 2.0,
        rear_clearance_m: float = -0.59,
        forward_accel_mps2: float = 0.0,
        lateral_clearance_m: float = 1.0,
        rotational_sweep_radius_m: float = 0.0,
        rear_sweep_escape_speed_mps: float = 0.0,
    ) -> None:
        dwell = dict(min_dwell_s or {})
        self.min_dwell_s = {
            mode: max(0.0, float(dwell.get(mode, 0.0))) for mode in _PRIORITY
        }
        self.high_yaw_deg_s = float(high_yaw_deg_s)
        self.obstacle_stop_m = float(obstacle_stop_m)
        self.rear_clearance_m = float(rear_clearance_m)
        self.forward_accel_mps2 = float(forward_accel_mps2)
        self.lateral_clearance_m = float(lateral_clearance_m)
        self.rotational_sweep_radius_m = float(rotational_sweep_radius_m)
        self.rear_sweep_escape_speed_mps = float(rear_sweep_escape_speed_mps)
        values = (
            *self.min_dwell_s.values(),
            self.high_yaw_deg_s,
            self.obstacle_stop_m,
            self.rear_clearance_m,
            self.forward_accel_mps2,
            self.lateral_clearance_m,
            self.rotational_sweep_radius_m,
            self.rear_sweep_escape_speed_mps,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("navigation arbiter parameters must be finite")
        if (
            self.high_yaw_deg_s < 0.0
            or self.obstacle_stop_m < 0.0
            or self.forward_accel_mps2 < 0.0
            or self.lateral_clearance_m < 0.0
            or self.rotational_sweep_radius_m < 0.0
            or self.rear_sweep_escape_speed_mps < 0.0
        ):
            raise ValueError("navigation envelope limits must be non-negative")
        self.reset()

    def reset(self) -> None:
        self._active_mode: Optional[str] = None
        self._active_since = 0.0
        self._active_command: Optional[Command] = None
        self._last_output_vx = 0.0
        self._last_output_at: Optional[float] = None

    @property
    def active_mode(self) -> Optional[str]:
        return self._active_mode

    def select(
        self,
        proposal: Command,
        obstacles: Iterable[Dict[str, Any]],
        now: float,
        *,
        allow_dwell: bool = True,
    ) -> NavigationDecision:
        """Select exactly one behavior and return its final safe command."""

        now = float(now)
        if not math.isfinite(now):
            raise ValueError("arbiter time must be finite")
        # ROS/sim clock epoch changes must not extend an old behavior dwell or
        # replay a held recovery command into the new epoch.
        if self._active_mode is not None and now < self._active_since:
            self.reset()
        obstacle_snapshot = list(obstacles)
        requested_command, requested_safety = navigation_safety_envelope(
            proposal,
            obstacle_snapshot,
            high_yaw_deg_s=self.high_yaw_deg_s,
            obstacle_stop_m=self.obstacle_stop_m,
            rear_clearance_m=self.rear_clearance_m,
            lateral_clearance_m=self.lateral_clearance_m,
            rotational_sweep_radius_m=self.rotational_sweep_radius_m,
            rear_sweep_escape_speed_mps=self.rear_sweep_escape_speed_mps,
        )
        requested_mode = navigation_mode(requested_command)

        held = False
        selected_command = requested_command
        selected_mode = requested_mode
        if (
            allow_dwell
            and self._active_mode is not None
            and self._active_command is not None
        ):
            elapsed = max(0.0, now - self._active_since)
            active_dwell = self.min_dwell_s[self._active_mode]
            lower_priority = _PRIORITY[requested_mode] < _PRIORITY[self._active_mode]
            if lower_priority and elapsed < active_dwell:
                held = True
                selected_mode = self._active_mode
                selected_command = Command(
                    float(self._active_command.vx),
                    float(self._active_command.vy),
                    float(self._active_command.yaw_rate),
                    f"{selected_mode.lower()}_dwell",
                )

        # Re-apply after dwell: stale held commands never bypass current safety.
        final_command, final_safety = navigation_safety_envelope(
            selected_command,
            obstacle_snapshot,
            high_yaw_deg_s=self.high_yaw_deg_s,
            obstacle_stop_m=self.obstacle_stop_m,
            rear_clearance_m=self.rear_clearance_m,
            lateral_clearance_m=self.lateral_clearance_m,
            rotational_sweep_radius_m=self.rotational_sweep_radius_m,
            rear_sweep_escape_speed_mps=self.rear_sweep_escape_speed_mps,
        )
        if final_safety:
            selected_mode = MODE_SAFETY_STOP
            held = False

        # Upward speed changes are rate-limited; reductions and safety stops
        # remain immediate. This removes the waypoint transition jump without
        # delaying obstacle braking. A zero value preserves legacy behavior.
        if self.forward_accel_mps2 > 0.0 and float(final_command.vx) > 0.0:
            if self._last_output_at is None or now < self._last_output_at:
                dt = 0.0
            else:
                dt = now - self._last_output_at
            baseline = max(0.0, self._last_output_vx)
            max_vx = baseline + self.forward_accel_mps2 * max(0.0, dt)
            if float(final_command.vx) > max_vx + 1e-9:
                final_command = Command(
                    max_vx,
                    float(final_command.vy),
                    float(final_command.yaw_rate),
                    f"{final_command.action} accel_limited",
                )

        if selected_mode != self._active_mode:
            self._active_mode = selected_mode
            self._active_since = now
        self._active_command = final_command
        self._last_output_vx = float(final_command.vx)
        self._last_output_at = now
        return NavigationDecision(
            final_command,
            selected_mode,
            requested_mode,
            held,
            final_safety or requested_safety,
        )


def motion_expected_for_stuck(decision: NavigationDecision, threshold_mps: float) -> bool:
    """True only when the selected behavior intentionally commands progress."""

    return (
        decision.mode in {MODE_CRUISE, MODE_SLOW, MODE_RECOVERY}
        and float(decision.command.vx) >= max(0.0, float(threshold_mps))
    )
