"""Fail-closed validation for the manually started field stack profile."""

from __future__ import annotations

import math
from typing import Mapping


FIELD_SAFETY_ACK = "FIELD_OPERATOR_READY_NO_AUTO_ARM"
FIELD_ABSOLUTE_MAX_SPEED_MPS = 1.6
FIELD_ABSOLUTE_MAX_YAW_RATE_DEG_S = 50.0

# Saha davranışını değiştiren her parametre bu dosyada bulunmak zorundadır.
# Port, topic, model dosyası/hash'i, SCR adı, log dizini ve güvenlik kapıları
# kurulum kimliğidir; bu listeye bilerek dahil edilmez.
FIELD_TUNING_PARAMETERS = {
    "ida_autonomy": (
        "loop_hz", "waypoint_threshold_m", "waypoint_overshoot_guard_m",
        "max_speed_mps", "max_yaw_rate_deg_s", "yaw_kp_deg_s_per_deg",
        "telemetry_timeout_s", "perception_timeout_s",
        "obstacle_avoid_distance_m", "target_min_confidence",
        "p3_search_yaw_rate_deg_s", "p3_search_scan_degrees",
        "p3_search_initial_hold_s", "p3_search_reposition_m",
        "p3_search_transit_speed_mps", "p3_search_reposition_arrival_m",
        "p3_search_anchor_acceptance_m", "p2_exit_advance_m",
        "p2_exit_advance_speed_mps", "p3_lock_confirm_s",
        "p3_lock_confirm_frames", "p3_target_loss_grace_s",
        "p3_lock_yaw_gain_deg_s",
        "p3_engage_center_max", "p3_engage_distance_m",
        "p3_lock_min_speed_mps", "p3_align_distance_m",
        "p3_approach_speed_mps", "p3_approach_yaw_max_deg_s",
        "p3_engage_speed_mps", "sim_gate_truth_enabled",
        "p2_min_pair_crossings", "p1_max_duration_s", "p2_max_duration_s",
        "p3_hold_timeout_s", "yellow_sighting_threshold",
        "yellow_sighting_decay", "yellow_sighting_range_m",
        "yellow_sighting_forward_m", "yellow_sighting_lateral_m",
        "yellow_sighting_min_confidence", "pair_max_lateral_m",
        "pair_min_lateral_gap_m", "pair_cross_trigger_forward_m",
        "pair_cross_complete_forward_m", "pair_min_confidence",
        "pair_max_age_s", "pair_max_forward_gap_m",
        "pair_max_crossing_gap_s", "wrong_avoid_lateral_m",
        "wrong_avoid_speed_mps", "engage_window_s",
        "failsafe_recovery_enabled", "failsafe_recovery_s", "score_kd1",
        "score_kd2", "score_ed2", "score_ooc_mode", "score_half_width_m",
        "score_sustained_contact_s", "score_contact_radius_m",
        "score_drop_after_s", "score_include_uav_bonus",
        "course_geometry_enter_m", "course_geometry_exit_m", "dwa_enabled",
        "nn_sort_enabled", "costmap_size_m", "costmap_cell_m",
        "costmap_bot_radius_m", "costmap_safety_m", "dwa_recovery_vx",
        "dwa_recovery_yaw", "dwa_w_obstacle", "dwa_w_heading",
        "dwa_w_progress", "dwa_w_speed", "dwa_w_smooth", "dwa_w_unknown",
        "dwa_w_corridor", "dwa_w_avoid", "dwa_vx_steps", "dwa_yaw_steps",
        "dwa_sim_time", "dwa_sim_step", "dwa_accel_max", "dwa_slow_vx",
        "dwa_min_drive_vx", "dwa_min_turn_rate_deg_s",
        "dwa_recovery_latch_ticks", "dwa_align_before_drive_deg",
        "dwa_align_yaw_rate_deg_s", "dwa_align_parkurs",
        "dwa_avoid_direction_latch_ticks", "p2_dwa_max_yaw_rate_deg_s",
        "near_field_stop_m",
        "near_field_slow_m", "near_field_stop_lateral_m",
        "near_field_slow_lateral_m",
        "near_field_pivot_yaw_deg_s", "near_field_slow_speed_mps",
        "near_field_stop_release_m", "near_field_escape_trigger_s",
        "near_field_escape_reverse_s", "near_field_escape_cooldown_s",
        "near_field_escape_reverse_mps", "near_field_escape_yaw_deg_s",
        "near_field_escape_rear_stop_m",
        "near_field_escape_forward_commit_s",
        "near_field_escape_forward_commit_mps",
        "near_field_escape_forward_commit_yaw_deg_s",
        "near_field_heading_align_speed_mps",
        "near_field_heading_align_max_yaw_deg_s", "nav_align_min_dwell_s",
        "nav_recovery_min_dwell_s", "nav_slow_min_dwell_s",
        "nav_final_high_yaw_deg_s", "nav_final_obstacle_stop_m",
        "nav_rotational_sweep_radius_m", "nav_rear_sweep_escape_speed_mps",
        "corridor_goal_parkurs",
        "corridor_bias_parkurs", "corridor_goal_lookahead_min_m",
        "corridor_goal_lateral_gain", "safety_base_m", "safety_max_m",
        "reaction_time_s", "max_decel_mps2", "max_speed_scale",
        "stuck_timeout_s", "stuck_min_dist_m", "stuck_vx_threshold",
    ),
    "ida_command_limiter": (
        "max_vx_mps", "max_vy_mps", "max_yaw_rate_rad_s",
        "bench_pulse_enabled", "bench_pulse_max_speed_mps",
        "bench_pulse_max_duration_s",
    ),
    "ida_mavsdk_bridge": (
        "command_timeout_s", "send_hz", "health_source_timeout_s",
        "pymavlink_baud", "guided_retry_s", "guided_timeout_s",
        "command_retry_s", "guided_health_timeout_s",
        "guided_max_forward_mps", "guided_max_reverse_mps",
        "guided_max_yaw_rate_rad_s", "target_color_poll_hz",
        "mission_download_hz", "mission_raw_enabled",
        "mission_control_poll_hz", "mission_control_ros_ack_timeout_s",
        "param_call_timeout_s", "param_max_retries",
        "param_consecutive_fail_limit", "yki_status_heartbeat_s",
    ),
    "ida_field_camera_driver": (
        "image_width", "image_height", "camera_fps", "fov_deg",
        "focal_length_px", "pixel_format", "output_encoding",
        "camera_frame_id", "auto_exposure", "exposure_time_absolute",
        "white_balance_automatic", "white_balance_temperature", "gain",
        "saturation", "brightness", "contrast", "power_line_frequency",
    ),
    "sllidar_node": (
        "serial_baudrate", "frame_id", "inverted", "angle_compensate",
        "scan_mode", "scan_frequency",
    ),
    "ida_yolo_camera": (
        "confidence_threshold", "iou_threshold", "imgsz", "device",
        "image_width", "image_height", "camera_fps", "publish_hz",
        "fov_deg", "focal_length_px", "target_height_m", "record_video",
    ),
    "ida_yolo_camera_p3": (
        "confidence_threshold", "iou_threshold", "imgsz", "device",
        "image_width", "image_height", "camera_fps", "publish_hz",
        "fov_deg", "focal_length_px", "target_height_m", "record_video",
    ),
    "ida_sllidar_bridge": (
        "angle_offset_deg", "mirror_scan", "front_fov_deg",
        "sensor_forward_offset_m", "sensor_lateral_right_offset_m",
        "lidar_range_m", "min_distance_m", "cluster_gap_m",
        "min_cluster_points", "max_angle_gap_deg", "publish_hz",
        "raw_point_limit",
    ),
    "ida_sensor_fusion": (
        "publish_hz", "camera_timeout_s", "lidar_timeout_s",
        "camera_role_merge_s", "max_bearing_error_deg",
        "ambiguity_margin_deg", "max_association_items", "buffer_size",
        "clock_rollback_threshold_s",
    ),
    "ida_telemetry_logger": ("log_hz", "flush_every_lines"),
    "ida_video_logger": (
        "log_hz", "fps", "fourcc", "frame_width", "frame_height",
        "segment_duration_s", "frame_stall_timeout_s",
    ),
    "ida_map_logger": ("log_hz", "flush_every_lines"),
    "ida_logging_status": ("status_hz",),
}

# Saha profilinden değil, kurulum/bağlantı sözleşmesinden gelmesi gereken
# parametreler. Bu liste de bilerek exhaustivedir; yeni bir node parametresi
# eklendiğinde test onu sınıflandırmadan kabul etmez.
FIELD_DEPLOYMENT_PARAMETERS = {
    "ida_autonomy": (),
    "ida_command_limiter": (),
    "ida_mavsdk_bridge": (
        "dry_run", "system_address", "pymavlink_address",
        "target_color_param", "mission_counts_param", "mission_control_param",
        "companion_system_id", "companion_component_id",
        "yki_status_udp_host", "yki_status_udp_port",
        "vehicle_setup_enabled", "guided_mode_enabled", "motor_command_enabled",
        "motor_output_limits_check_enabled", "expected_mot_thr_min_pct",
        "expected_mot_thr_max_pct", "expected_mot_slewrate_pct_s",
        "motor_output_limits_poll_s",
        "left_motor_servo_channel", "right_motor_servo_channel",
        "vehicle_frame_class", "actuation_source_system_id",
        "actuation_backend", "bench_rc_safety_ack",
        "bench_rc_steering_channel", "bench_rc_throttle_channel",
        "bench_rc_neutral_pwm", "bench_rc_max_delta_pwm",
        "bench_rc_max_forward_mps", "bench_rc_source_system_id",
    ),
    "ida_yolo_camera": (
        "model_path", "model_type", "class_names", "class_name_map",
        "allowed_colors", "secondary_output_topic", "secondary_allowed_colors",
        "camera_index", "camera_topic", "camera_topic_type", "output_topic",
        "processed_image_topic", "record_path",
    ),
    "ida_yolo_camera_p3": (
        "model_path", "model_type", "class_names", "class_name_map",
        "allowed_colors", "secondary_output_topic", "secondary_allowed_colors",
        "camera_index", "camera_topic", "camera_topic_type", "output_topic",
        "processed_image_topic", "record_path",
    ),
    "ida_sllidar_bridge": ("scan_topic", "output_topic", "raw_contract"),
    "ida_sensor_fusion": (
        "camera_topic_p1p2", "camera_topic_p3", "lidar_topic", "shadow_mode",
        "buoy_output_topic", "obstacle_output_topic", "status_output_topic",
        "model_loaded", "camera_calibrated", "lidar_calibrated",
        "extrinsics_calibrated",
    ),
    "ida_telemetry_logger": ("log_dir", "run_name"),
    "ida_video_logger": (
        "log_dir", "run_name", "record_path", "processed_image_topic",
    ),
    "ida_map_logger": ("log_dir", "run_name"),
    "ida_logging_status": ("log_dir", "run_name"),
}


def _boolean(name: str, value: object) -> bool:
    if value not in ("true", "false"):
        raise ValueError(f"{name} must be exactly true or false")
    return value == "true"


def _bounded_float(name: str, value: object, low: float, high: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(parsed) or not low <= parsed <= high:
        raise ValueError(f"{name} must be within [{low}, {high}]")
    return parsed


def validate_field_drive_envelope(profile: Mapping[str, object]) -> dict:
    """Validate the single-source field motion envelope.

    The observed YKI motor percentage is evidence from ``SERVO_OUTPUT_RAW``;
    it is deliberately not converted to m/s. ArduRover owns that nonlinear
    speed-to-PWM control. This contract instead guarantees that every normal
    field motion request is inside one explicit m/s envelope.
    """
    try:
        envelope_section = profile["ida_field_envelope"]
        envelope = (
            envelope_section.get("ros__parameters", envelope_section)
            if isinstance(envelope_section, Mapping)
            else envelope_section
        )
        autonomy = profile["ida_autonomy"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise ValueError("field profile is missing the drive envelope") from exc
    if not isinstance(envelope, Mapping) or not isinstance(autonomy, Mapping):
        raise ValueError("field drive envelope and autonomy parameters must be mappings")

    hard_max = _bounded_float(
        "hard_max_speed_mps",
        envelope.get("hard_max_speed_mps"),
        0.05,
        FIELD_ABSOLUTE_MAX_SPEED_MPS,
    )
    min_drive = _bounded_float(
        "min_navigation_command_mps",
        envelope.get("min_navigation_command_mps"),
        0.05,
        hard_max,
    )
    max_speed = _bounded_float(
        "ida_autonomy.max_speed_mps",
        autonomy.get("max_speed_mps"),
        min_drive,
        hard_max,
    )

    motion_parameters = (
        "dwa_recovery_vx",
        "dwa_slow_vx",
        "dwa_min_drive_vx",
        "wrong_avoid_speed_mps",
        "p3_lock_min_speed_mps",
        "p3_approach_speed_mps",
        "p3_engage_speed_mps",
        "nav_rear_sweep_escape_speed_mps",
    )
    normalized_motion = {
        name: _bounded_float(
            f"ida_autonomy.{name}", autonomy.get(name), min_drive, max_speed
        )
        for name in motion_parameters
    }
    if not math.isclose(
        normalized_motion["dwa_min_drive_vx"], min_drive, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ValueError("dwa_min_drive_vx must equal min_navigation_command_mps")
    # Waypoint/yellow approach uses a 0.44 scale. It must not silently fall
    # below the measured effective field command at the configured max speed.
    if max_speed * 0.44 + 1e-9 < min_drive:
        raise ValueError("waypoint approach floor is below min_navigation_command_mps")
    return {
        "hard_max_speed_mps": hard_max,
        "min_navigation_command_mps": min_drive,
        "max_speed_mps": max_speed,
        **normalized_motion,
    }


def validate_thruster_hardware_profile(profile: Mapping[str, object]) -> dict:
    """Validate the non-actuating hardware/calibration envelope.

    These values document and bound the first water calibration.  They never
    write Pixhawk parameters and never turn the motors; the operator must
    independently verify ``MOT_THR_MAX`` before an armed run.
    """
    try:
        section = profile["ida_thruster_envelope"]
        values = section.get("ros__parameters", section)
    except (KeyError, TypeError) as exc:
        raise ValueError("field profile is missing ida_thruster_envelope") from exc
    if not isinstance(values, Mapping):
        raise ValueError("ida_thruster_envelope must be a mapping")

    def exact_int(name: str, low: int, high: int) -> int:
        value = values.get(name)
        if isinstance(value, bool):
            raise ValueError(f"{name} must be an exact integer")
        try:
            parsed = int(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"{name} must be an exact integer") from exc
        if parsed != value or not low <= parsed <= high:
            raise ValueError(f"{name} must be within [{low}, {high}]")
        return parsed

    thruster_count = exact_int("thruster_count", 2, 2)
    left_channel = exact_int("left_servo_channel", 1, 16)
    right_channel = exact_int("right_servo_channel", 1, 16)
    if left_channel == right_channel:
        raise ValueError("left/right motor channels must differ")
    cells = exact_int("battery_cells", 3, 6)
    servo_min = exact_int("servo_min_pwm", 800, 1500)
    servo_trim = exact_int("servo_trim_pwm", servo_min + 1, 1900)
    servo_max = exact_int("servo_max_pwm", servo_trim + 1, 2200)
    left_reversed = exact_int("left_servo_reversed", 0, 1)
    right_reversed = exact_int("right_servo_reversed", 0, 1)
    left_function = exact_int("required_left_servo_function", 73, 73)
    right_function = exact_int("required_right_servo_function", 74, 74)
    frame_class = exact_int("required_frame_class", 2, 2)
    continuous = exact_int("continuous_pwm_limit", servo_trim + 1, servo_max)
    burst = exact_int("burst_pwm_limit", continuous, servo_max)
    mot_min = exact_int("pixhawk_mot_thr_min_pct", 0, 20)
    mot_max = exact_int("pixhawk_mot_thr_max_pct", 20, 100)
    slew_rate = exact_int("pixhawk_mot_slewrate_pct_s", 1, 100)
    verification_poll_s = _bounded_float(
        "verification_poll_s", values.get("verification_poll_s"), 1.0, 30.0
    )
    burst_pct = exact_int("calibration_burst_throttle_pct", mot_max, 100)
    burst_s = _bounded_float("burst_duration_s", values.get("burst_duration_s"), 0.1, 2.0)
    cruise = _bounded_float(
        "target_actual_cruise_speed_mps",
        values.get("target_actual_cruise_speed_mps"), 0.1, 1.0,
    )
    actual_max = _bounded_float(
        "target_actual_max_speed_mps",
        values.get("target_actual_max_speed_mps"), cruise, 1.0,
    )
    voltage = _bounded_float(
        "battery_nominal_voltage_v", values.get("battery_nominal_voltage_v"), 12.0, 24.0
    )
    capacity = _bounded_float("battery_capacity_ah", values.get("battery_capacity_ah"), 1.0, 30.0)
    thrust = _bounded_float(
        "thruster_max_thrust_kgf_each", values.get("thruster_max_thrust_kgf_each"), 0.1, 8.0
    )
    current = _bounded_float(
        "manufacturer_continuous_current_a_each",
        values.get("manufacturer_continuous_current_a_each"), 1.0, 35.0,
    )
    servo_snapshot_verified = values.get("servo_snapshot_verified")
    if not isinstance(servo_snapshot_verified, bool):
        raise ValueError("servo_snapshot_verified must be a boolean")
    if values.get("autonomy_burst_enabled") is not False:
        raise ValueError("autonomy_burst_enabled must remain false")
    if continuous != servo_trim + round((servo_max - servo_trim) * mot_max / 100.0):
        raise ValueError("continuous PWM must match Pixhawk throttle limit")
    if burst != servo_trim + round((servo_max - servo_trim) * burst_pct / 100.0):
        raise ValueError("burst PWM must match calibration burst percentage")
    return {
        "thruster_count": thruster_count,
        "left_servo_channel": left_channel,
        "right_servo_channel": right_channel,
        "battery_cells": cells,
        "battery_nominal_voltage_v": voltage,
        "battery_capacity_ah": capacity,
        "thruster_max_thrust_kgf_each": thrust,
        "manufacturer_continuous_current_a_each": current,
        "servo_snapshot_verified": servo_snapshot_verified,
        "servo_min_pwm": servo_min,
        "servo_trim_pwm": servo_trim,
        "servo_max_pwm": servo_max,
        "left_servo_reversed": left_reversed,
        "right_servo_reversed": right_reversed,
        "required_left_servo_function": left_function,
        "required_right_servo_function": right_function,
        "required_frame_class": frame_class,
        "continuous_pwm_limit": continuous,
        "burst_pwm_limit": burst,
        "burst_duration_s": burst_s,
        "pixhawk_mot_thr_min_pct": mot_min,
        "pixhawk_mot_thr_max_pct": mot_max,
        "pixhawk_mot_slewrate_pct_s": slew_rate,
        "verification_poll_s": verification_poll_s,
        "calibration_burst_throttle_pct": burst_pct,
        "target_actual_cruise_speed_mps": cruise,
        "target_actual_max_speed_mps": actual_max,
        "autonomy_burst_enabled": False,
    }


def validate_field_tuning_coverage(profile: Mapping[str, object]) -> dict:
    """Fail if a field-tunable value can silently fall back elsewhere."""
    normalized = {}
    for node_name, required_names in FIELD_TUNING_PARAMETERS.items():
        try:
            params = profile[node_name]["ros__parameters"]
        except (KeyError, TypeError) as exc:
            raise ValueError(f"field profile is missing {node_name}") from exc
        if not isinstance(params, Mapping):
            raise ValueError(f"{node_name}.ros__parameters must be a mapping")
        missing = sorted(set(required_names) - set(params))
        if missing:
            raise ValueError(
                f"field profile is missing {node_name} tuning parameters: "
                + ", ".join(missing)
            )
        normalized[node_name] = dict(params)

    autonomy = normalized["ida_autonomy"]
    limiter = normalized["ida_command_limiter"]
    bridge = normalized["ida_mavsdk_bridge"]
    max_speed = float(autonomy["max_speed_mps"])
    max_yaw_rad = math.radians(float(autonomy["max_yaw_rate_deg_s"]))
    for name, value in (
        ("ida_command_limiter.max_vx_mps", limiter["max_vx_mps"]),
        ("ida_mavsdk_bridge.guided_max_forward_mps", bridge["guided_max_forward_mps"]),
        ("ida_mavsdk_bridge.guided_max_reverse_mps", bridge["guided_max_reverse_mps"]),
    ):
        if not math.isclose(float(value), max_speed, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(f"{name} must equal ida_autonomy.max_speed_mps")
    for name, value in (
        ("ida_command_limiter.max_yaw_rate_rad_s", limiter["max_yaw_rate_rad_s"]),
        ("ida_mavsdk_bridge.guided_max_yaw_rate_rad_s", bridge["guided_max_yaw_rate_rad_s"]),
    ):
        if not math.isclose(float(value), max_yaw_rad, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(f"{name} must equal ida_autonomy.max_yaw_rate_deg_s")

    camera = normalized["ida_field_camera_driver"]
    for role in ("ida_yolo_camera", "ida_yolo_camera_p3"):
        detector = normalized[role]
        for camera_name, detector_name in (
            ("image_width", "image_width"),
            ("image_height", "image_height"),
            ("camera_fps", "camera_fps"),
            ("fov_deg", "fov_deg"),
            ("focal_length_px", "focal_length_px"),
        ):
            if not math.isclose(
                float(camera[camera_name]), float(detector[detector_name]),
                rel_tol=0.0, abs_tol=1e-9,
            ):
                raise ValueError(
                    f"{role}.{detector_name} must equal camera driver {camera_name}"
                )
    return normalized


def validate_field_launch_tuning(
    values: Mapping[str, object], profile: Mapping[str, object]
) -> dict:
    """Reject command-line/env attempts to override the central field profile."""
    covered = validate_field_tuning_coverage(profile)
    autonomy = covered["ida_autonomy"]
    thruster = validate_thruster_hardware_profile(profile)
    camera = covered["ida_field_camera_driver"]
    lidar = covered["sllidar_node"]
    expected = {
        "max_speed_mps": float(autonomy["max_speed_mps"]),
        "max_yaw_rate_rad_s": math.radians(float(autonomy["max_yaw_rate_deg_s"])),
        "autonomy_stuck_timeout_s": float(autonomy["stuck_timeout_s"]),
        "field_camera_width": int(camera["image_width"]),
        "field_camera_height": int(camera["image_height"]),
        "field_camera_fps": int(camera["camera_fps"]),
        "camera_fov_deg": float(camera["fov_deg"]),
        "camera_focal_length_px": float(camera["focal_length_px"]),
        "lidar_serial_baudrate": int(lidar["serial_baudrate"]),
        "lidar_scan_mode": str(lidar["scan_mode"]),
        "expected_mot_thr_min_pct": int(thruster["pixhawk_mot_thr_min_pct"]),
        "expected_mot_thr_max_pct": int(thruster["pixhawk_mot_thr_max_pct"]),
        "expected_mot_slewrate_pct_s": int(
            thruster["pixhawk_mot_slewrate_pct_s"]
        ),
        "motor_output_limits_poll_s": float(thruster["verification_poll_s"]),
    }
    if str(values.get("motor_output_limits_check_enabled")) != "true":
        raise ValueError("field motor output limit verification cannot be disabled")
    for name, expected_value in expected.items():
        actual = values.get(name)
        if isinstance(expected_value, str):
            matches = str(actual) == expected_value
        else:
            try:
                actual_number = float(actual)
            except (TypeError, ValueError, OverflowError):
                matches = False
            else:
                matches = math.isfinite(actual_number) and math.isclose(
                    actual_number, float(expected_value), rel_tol=0.0, abs_tol=1e-9
                )
        if not matches:
            raise ValueError(
                f"{name} cannot override field_profile.yaml "
                f"(expected {expected_value!r}, got {actual!r})"
            )
    return expected


def validate_field_profile(values: Mapping[str, object], env: Mapping[str, str]) -> dict:
    """Validate launch gates without performing any vehicle operation."""
    normalized = {
        name: _boolean(name, values.get(name))
        for name in (
            "canonical_takeover_enabled",
            "dry_run",
            "mavlink_router_enabled",
            "vehicle_setup_enabled",
            "guided_mode_enabled",
            "motor_command_enabled",
            "fusion_model_loaded",
            "camera_calibrated",
            "lidar_calibrated",
            "extrinsics_calibrated",
            "field_camera_driver_enabled",
            "lidar_enabled",
            "single_general_model_enabled",
            "yki_debug_enabled",
        )
    }
    debug_url = str(values.get("yki_debug_websocket_url", ""))
    if normalized["yki_debug_enabled"]:
        token = str(env.get("EZEL_JETSON_DEBUG_TOKEN", ""))
        if debug_url != "auto://yki" and (
            not debug_url.startswith(("ws://", "wss://")) or len(debug_url) > 512
        ):
            raise ValueError("yki_debug_websocket_url must be auto://yki or bounded ws/wss")
        if not token or len(token) > 4096 or "\r" in token or "\n" in token:
            raise ValueError("YKİ debug requires a valid separate Jetson token")
    normalized["yki_debug_websocket_url"] = debug_url
    # Saha profili bu değerleri tek YAML'den türetir; launch argümanları yalnız
    # üç tüketiciye (otonomi/limiter/GUIDED) aynı değeri dağıtmak içindir.
    normalized["max_speed_mps"] = _bounded_float(
        "max_speed_mps", values.get("max_speed_mps"), 0.05,
        FIELD_ABSOLUTE_MAX_SPEED_MPS,
    )
    normalized["max_yaw_rate_rad_s"] = _bounded_float(
        "max_yaw_rate_rad_s", values.get("max_yaw_rate_rad_s"), 0.05,
        math.radians(FIELD_ABSOLUTE_MAX_YAW_RATE_DEG_S),
    )

    # Canonical takeover may be inspected in a no-op state, but the launch must
    # never mutate Pixhawk parameters automatically.
    if normalized["vehicle_setup_enabled"]:
        raise ValueError("field stack forbids automatic vehicle setup")

    if normalized["motor_command_enabled"]:
        required = (
            normalized["canonical_takeover_enabled"]
            and not normalized["dry_run"]
            and normalized["mavlink_router_enabled"]
            and normalized["guided_mode_enabled"]
            and normalized["fusion_model_loaded"]
            and normalized["camera_calibrated"]
            and normalized["lidar_calibrated"]
            and normalized["extrinsics_calibrated"]
        )
        if not required:
            raise ValueError("live field motor path requires every ownership/readiness gate")
        if env.get("IDA_FIELD_PHYSICAL_SAFETY_ACK") != FIELD_SAFETY_ACK:
            raise ValueError("live field motor path requires the exact operator acknowledgement")
    return normalized
