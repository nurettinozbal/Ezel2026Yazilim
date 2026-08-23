"""Field-adjustable EZEL GCS backend configuration.

All values can be overridden via environment variables.
If an environment variable is not set, the hardcoded default is used.
This allows field deployment without code changes.
"""

import os


def _bool_env(name: str, default: str = "false") -> bool:
    return os.environ.get(name, default).strip().lower() in {"1", "true", "yes", "on"}

IDA_PORT = os.environ.get("EZEL_IDA_PORT", "/dev/ttyUSB0")
IHA_PORT = os.environ.get("EZEL_IHA_PORT", "/dev/ttyUSB1")
BAUDRATE = int(os.environ.get("EZEL_BAUDRATE", "57600"))

IDA_SYS_ID = int(os.environ.get("EZEL_IDA_SYS_ID", "1"))
IHA_SYS_ID = int(os.environ.get("EZEL_IHA_SYS_ID", "2"))
GCS_SYS_ID = int(os.environ.get("EZEL_GCS_SYS_ID", "255"))
IDA_COMPANION_COMPONENT_ID = int(
    os.environ.get("EZEL_IDA_COMPANION_COMPONENT_ID", "191")
)

TELEMETRY_RATE_HZ = int(os.environ.get("EZEL_TELEMETRY_RATE_HZ", "2"))
HEARTBEAT_TIMEOUT_SECONDS = float(os.environ.get("EZEL_HEARTBEAT_TIMEOUT", "5.0"))
RECONNECT_DELAY_SECONDS = float(os.environ.get("EZEL_RECONNECT_DELAY", "3.0"))
MISSION_TIMEOUT_SECONDS = float(os.environ.get("EZEL_MISSION_TIMEOUT", "20.0"))
COMMAND_ACK_TIMEOUT_SECONDS = float(os.environ.get("EZEL_COMMAND_ACK_TIMEOUT", "3.0"))
STATE_VERIFY_TIMEOUT_SECONDS = float(os.environ.get("EZEL_STATE_VERIFY_TIMEOUT", "2.0"))

AUTO_RETURN_ON_LINK_LOSS = _bool_env("EZEL_AUTO_RETURN_ON_LINK_LOSS", "true")
RETURN_HOME_MODE = os.environ.get("EZEL_RETURN_HOME_MODE", "RTL").strip().upper()

# Safer defaults for a mixed surface/air vehicle emergency.
EMERGENCY_IDA_ACTION = os.environ.get("EZEL_EMERGENCY_IDA_ACTION", "HOLD")
EMERGENCY_IHA_ACTION = os.environ.get("EZEL_EMERGENCY_IHA_ACTION", "RTL")

ALLOWED_ORIGINS = [
    origin.strip()
    for origin in os.environ.get(
        "EZEL_ALLOWED_ORIGINS",
        "http://localhost:5173,http://127.0.0.1:5173",
    ).split(",")
    if origin.strip()
]

# WebSocket telemetry can be observed without a token, but vehicle commands
# are rejected unless a matching token is configured on both backend/frontend.
WS_AUTH_REQUIRED = _bool_env("EZEL_WS_AUTH_REQUIRED", "true")
WS_AUTH_TOKEN = os.environ.get("EZEL_WS_TOKEN", "").strip()

# Lab diagnostics are compiled into the backend but expose no ingest/test surface
# unless explicitly enabled. Field startup leaves this false.
LAB_DEBUG_ENABLED = _bool_env("EZEL_LAB_DEBUG_ENABLED", "false")
JETSON_DEBUG_TOKEN = os.environ.get("EZEL_JETSON_DEBUG_TOKEN", "").strip()
