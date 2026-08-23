"""Canonical ida_control <-> canonical YKİ wire-contract regression."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest

from ida_control.mission_contract import COLOR_INT_MAP
from ida_control.yki_status_contract import (
    ACTION_CODE_MAP,
    PARKUR_FROM_STATE,
    STATE_CODE_MAP,
)


ROOT = Path(__file__).resolve().parents[3]
YKI_AUTONOMY = (
    ROOT / "ezel-yazilim_yeni" / "ezel-yazilim" / "arayuz" /
    "backend" / "services" / "autonomy_contract.py"
)
YKI_TARGET = YKI_AUTONOMY.with_name("target_manager.py")


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"contract cannot be loaded: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@unittest.skipUnless(YKI_AUTONOMY.is_file() and YKI_TARGET.is_file(), "canonical YKİ tree missing")
class TestYkiCrossContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.autonomy = _load("canonical_yki_autonomy", YKI_AUTONOMY)
        cls.target = _load("canonical_yki_target", YKI_TARGET)

    def test_state_mapping_is_exact(self) -> None:
        self.assertEqual(STATE_CODE_MAP, self.autonomy.STATE_TO_CODE)

    def test_action_and_parkur_codes_are_decodable(self) -> None:
        self.assertEqual(set(ACTION_CODE_MAP.values()), set(self.autonomy.CODE_TO_ACTION_LABEL))
        self.assertEqual(set(PARKUR_FROM_STATE.values()), set(self.autonomy.PARKUR_CODE_TO_LABEL))

    def test_all_compact_fields_fit_mavlink_char10(self) -> None:
        expected = {"AUTO_ST", "AUTO_AC", "PARKUR", "PERC_DET", "PERC_OBS", "LOG_ACT", "LOG_CNT"}
        self.assertEqual(expected, self.autonomy.IDA_AUTONOMY_FIELD_NAMES)
        self.assertTrue(all(1 <= len(name) <= 10 for name in expected | {"TGT_ACK"}))

    def test_target_color_codes_are_exact(self) -> None:
        yki_names = {
            "red": "KIRMIZI", "green": "YEŞİL", "orange": "TURUNCU",
            "black": "SİYAH", "yellow": "SARI",
        }
        expected = {code: yki_names[color] for code, color in COLOR_INT_MAP.items()}
        self.assertEqual(expected, self.target.CODE_TO_COLOR)
        self.assertEqual(set(expected), {1, 2, 3, 4, 5})


if __name__ == "__main__":
    unittest.main()
