"""Çapraz kontrat testi: ROS2 gateway <-> YKİ backend tabloları birebir aynı.

NAMED_VALUE_INT alan adları ve değer kodlaması her iki tarafta aynı olmalı
(çıktı §1 kontrat tablosu). Bu test iki ayrı repodaki dosyaları okuyup
karşılaştırır — tek taraflı değişiklik kontratı bozar ve test kızar.

Jetson:  src/ida_ws_gateway/ida_ws_gateway/mavlink_parser.py
YKİ:     ezel-yazilim_yeni/ezel-yazilim/arayuz/backend/services/autonomy_contract.py
"""

import os
import json
import re
import sys
import unittest

_GATEWAY_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _GATEWAY_ROOT not in sys.path:
    sys.path.insert(0, _GATEWAY_ROOT)
_WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_PLANNING_ROOT = os.path.join(_WORKSPACE_ROOT, "ida_planning")
if _PLANNING_ROOT not in sys.path:
    sys.path.insert(0, _PLANNING_ROOT)

from ida_ws_gateway import mavlink_parser

_VEHICLE_TEST_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "ida_vehicle_test"))
if _VEHICLE_TEST_ROOT not in sys.path:
    sys.path.insert(0, _VEHICLE_TEST_ROOT)
from ida_vehicle_test.contracts import ALLOWED_TESTS, parse_request

# Kanonik YKİ autonomy_contract.py yolu. Eski ezel-yazilim ağacı bilinçli olarak
# kullanılmaz; aksi halde eski kopya false-green test sonucu üretebilir.
_YKI_CONTRACT_PATH = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..", "..", "..",
        "ezel-yazilim_yeni", "ezel-yazilim", "arayuz", "backend", "services", "autonomy_contract.py",
    )
)
_YKI_VEHICLE_TEST_PATH = os.path.abspath(os.path.join(os.path.dirname(_YKI_CONTRACT_PATH), "..", "..", "contracts", "ida_vehicle_test.v1.json"))

# Jetson tarafı literal adları — YKİ ile birebir olmalı (ws_gateway_node ile senkron).
EXPECTED_FIELD_NAMES = {
    "AUTO_ST",
    "AUTO_AC",
    "PARKUR",
    "PERC_DET",
    "PERC_OBS",
    "LOG_ACT",
    "LOG_CNT",
}

# Jetson tarafı sabitler — YKİ autonomy_contract.py ile birebir olmalı.
EXPECTED_STATE_CODES = set(mavlink_parser.STATE_CODE_MAP.values())


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as file:
        return file.read()


def _extract_set_literal(text: str, name: str) -> set:
    match = re.search(rf"{name}\s*=\s*\{{(.*?)\}}", text, re.S)
    if not match:
        return set()
    return {raw for raw in re.findall(r"\"([A-Z0-9_]{1,10})\"", match.group(1))}


def _extract_dict_values(text: str, name: str) -> set:
    """Sözlük literal'indeki int DEĞERLERI çıkarır (örn. STATE_TO_CODE)."""
    match = re.search(rf"{name}\s*=\s*\{{(.*?)\}}", text, re.S)
    if not match:
        return set()
    return {int(value) for value in re.findall(r":\s*(\d+)", match.group(1))}


def _extract_dict_keys(text: str, name: str) -> set:
    """Sözlük literal'indeki int ANAHTARLARI çıkarır (örn. CODE_TO_ACTION_LABEL)."""
    match = re.search(rf"{name}\s*=\s*\{{(.*?)\}}", text, re.S)
    if not match:
        return set()
    return {int(key) for key in re.findall(r"(\d+)\s*:", match.group(1))}


@unittest.skipUnless(os.path.exists(_YKI_CONTRACT_PATH), "YKİ backend yolu yok (repo yan yana değil)")
class TestCrossContract(unittest.TestCase):
    def test_uses_canonical_new_yki_path(self) -> None:
        normalized = _YKI_CONTRACT_PATH.replace("\\", "/")
        self.assertIn("/ezel-yazilim_yeni/ezel-yazilim/arayuz/", normalized)

    def test_field_names_match_both_sides(self) -> None:
        yki_text = _read(_YKI_CONTRACT_PATH)
        yki_names = _extract_set_literal(yki_text, "IDA_AUTONOMY_FIELD_NAMES")
        self.assertTrue(yki_names, "YKİ kontrat dosyasında alan adı bulunamadı")
        self.assertEqual(EXPECTED_FIELD_NAMES, yki_names)

    def test_state_codes_match_both_sides(self) -> None:
        yki_text = _read(_YKI_CONTRACT_PATH)
        yki_codes = _extract_dict_values(yki_text, "STATE_TO_CODE")
        self.assertTrue(yki_codes, "YKİ kontrat dosyasında STATE_TO_CODE bulunamadı")
        self.assertEqual(EXPECTED_STATE_CODES, yki_codes)

    def test_all_field_names_within_char10(self) -> None:
        for name in EXPECTED_FIELD_NAMES:
            self.assertLessEqual(len(name), 10, f"{name} char[10] sınırını aşıyor")

    def test_action_and_parkur_labels_match_both_sides(self) -> None:
        yki_text = _read(_YKI_CONTRACT_PATH)
        # Jetson action kodları (1..8) YKİ CODE_TO_ACTION_LABEL anahtarlarıyla aynı.
        action_codes = set(mavlink_parser.ACTION_CODE_MAP.values())
        yki_action_keys = _extract_dict_keys(yki_text, "CODE_TO_ACTION_LABEL")
        self.assertEqual(action_codes, yki_action_keys)
        # Jetson parkur kodları YKİ PARKUR_CODE_TO_LABEL anahtarlarıyla aynı.
        yki_parkur_keys = _extract_dict_keys(yki_text, "PARKUR_CODE_TO_LABEL")
        self.assertEqual({1, 2, 3}, yki_parkur_keys)

    def test_vehicle_test_manifest_and_real_ros_request_match(self) -> None:
        with open(_YKI_VEHICLE_TEST_PATH, encoding="utf-8") as file:
            manifest = json.load(file)
        self.assertEqual({item["id"] for item in manifest["tests"]}, set(ALLOWED_TESTS))
        basic = next(item for item in manifest["tests"] if item["id"] == "comms")
        request = {"action": "start", "run_id": "cross-contract-1", "seq": 1, "test": basic["id"], "timeout_s": basic["timeout_s"]}
        parsed = parse_request(request)
        self.assertEqual((parsed.action, parsed.test, parsed.seq), ("start", "comms", 1))


if __name__ == "__main__":
    unittest.main()
