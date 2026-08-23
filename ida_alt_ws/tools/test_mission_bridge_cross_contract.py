"""Cross-tree YKİ/Jetson mission mailbox contract regression."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "ezel-yazilim_yeni" / "ezel-yazilim" / "arayuz" / "backend"
CONTROL = ROOT / "src" / "ida_control"
for path in (str(BACKEND), str(CONTROL)):
    if path not in sys.path:
        sys.path.insert(0, path)

from ida_control import mission_contract as jetson  # noqa: E402
from links import mavlink_vehicle as yki  # noqa: E402


def test_yki_and_jetson_share_only_scr_user4_to_6() -> None:
    assert yki.IDA_TARGET_COLOR_PARAM == b"SCR_USER4"
    assert yki.IDA_MISSION_COUNTS_PARAM == b"SCR_USER5"
    assert yki.IDA_MISSION_CONTROL_PARAM == b"SCR_USER6"
    assert yki.MISSION_COUNT_RADIX == jetson.MISSION_COUNT_RADIX == 1001
    assert yki.MISSION_MAILBOX_BASE == jetson.MISSION_MAILBOX_BASE == 8_000_000
    assert yki.MISSION_CONTROL_MAX_SEQUENCE == jetson.MISSION_CONTROL_MAX_SEQUENCE


def test_mailbox_transitions_match_exactly() -> None:
    for idle, start in (
        (8_000_000, True),
        (8_000_003, False),
        (8_000_008, True),
        (15_999_996, True),
    ):
        assert yki._next_mission_mailbox(idle, start=start) == jetson.next_mission_control(
            idle, start=start
        )


def test_packed_counts_are_float32_safe_and_round_trip() -> None:
    for p1, p2 in ((1, 0), (4, 1), (1000, 1000)):
        packed = jetson.pack_mission_counts(p1, p2)
        assert packed == p1 * yki.MISSION_COUNT_RADIX + p2
        assert jetson.unpack_mission_counts(float(packed)) == (p1, p2)


def test_legacy_ackless_backend_entrypoint_is_disabled() -> None:
    text = (BACKEND / "telemetri_oku.py").read_text(encoding="utf-8")
    assert 'raise SystemExit(' in text
    assert "eski ve ACK'siz komut yoludur" in text
