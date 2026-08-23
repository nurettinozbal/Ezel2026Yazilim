"""Regression contract: YKI map clicks never synthesize a HOME waypoint."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAP = (
    ROOT / "ezel-yazilim_yeni" / "ezel-yazilim" / "arayuz" / "src"
    / "features" / "MapSystem" / "components" / "MapView.jsx"
)


class MapMissionContractTests(unittest.TestCase):
    def test_first_click_is_exactly_one_real_target_not_vehicle_home(self):
        text = MAP.read_text(encoding="utf-8")
        handler = text.split("function SimpleMapClickHandler()", 1)[1].split(
            "function GeofenceClickHandler()", 1
        )[0]
        self.assertEqual(handler.count("addMissionPoint("), 1)
        self.assertNotIn("ida.lat", handler)
        self.assertNotIn("ida.lon", handler)
        self.assertNotIn("setTimeout", handler)
        self.assertIn("İLK HEDEF #1", text)
        self.assertNotIn("BAŞLANGIÇ (HOME) NOKTASI", text)


if __name__ == "__main__":
    unittest.main()
