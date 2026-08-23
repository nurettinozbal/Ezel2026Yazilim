#!/usr/bin/env bash
# Pure device-path discovery helpers. Sourcing this file never opens a device.

resolve_field_camera_device() {
  local requested="${1:-auto}"
  if [[ -n "$requested" && "$requested" != "auto" ]]; then
    printf '%s\n' "$requested"
    return 0
  fi
  local candidates=()
  shopt -s nullglob
  candidates=(/dev/v4l/by-id/*Arducam*video-index0)
  shopt -u nullglob
  if [[ ${#candidates[@]} -ne 1 ]]; then
    printf 'Arducam auto-discovery expected exactly one device, found %s\n' \
      "${#candidates[@]}" >&2
    return 1
  fi
  printf '%s\n' "${candidates[0]}"
}

resolve_field_serial_device() {
  local role="${1:?role required}" requested="${2:-auto}"
  if [[ "$requested" != "auto" && -e "$requested" ]]; then
    printf '%s\n' "$requested"
    return 0
  fi
  local stable_alias="/dev/idaws_${role}"
  if [[ "$role" == "pixhawk" ]]; then
    stable_alias="/dev/idaws_pixhawk"
  fi
  if [[ "$requested" == "auto" && -e "$stable_alias" ]]; then
    printf '%s\n' "$stable_alias"
    return 0
  fi
  local all=() matches=() item base
  shopt -s nullglob
  all=(/dev/serial/by-id/*)
  shopt -u nullglob
  for item in "${all[@]}"; do
    base="$(basename "$item")"
    case "$role" in
      lidar)
        [[ "$base" =~ (SLAMTEC|Slamtec|CP210|Silicon_Labs) ]] && matches+=("$item")
        ;;
      pixhawk)
        [[ "$base" =~ (ArduPilot|Pixhawk|Cube|PX4|FMU|Holybro) ]] && matches+=("$item")
        ;;
      *) printf 'Unknown serial-device role: %s\n' "$role" >&2; return 2 ;;
    esac
  done
  if [[ ${#matches[@]} -eq 1 ]]; then
    printf '%s\n' "${matches[0]}"
    return 0
  fi
  printf '%s auto-discovery expected exactly one matching serial device, found %s\n' \
    "$role" "${#matches[@]}" >&2
  return 1
}
