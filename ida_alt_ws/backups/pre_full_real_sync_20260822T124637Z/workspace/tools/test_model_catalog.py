"""Regression tests for the one-source field model catalog."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.model_catalog import (
    CatalogError,
    add_profile,
    import_artifact,
    load_catalog,
    resolve_profile,
    select_profile,
    selected_alias,
    shell_lines,
)


def write_catalog(root: Path, artifacts: dict, profiles: dict) -> Path:
    path = root / "model_catalog.json"
    path.write_text(json.dumps({
        "schema_version": 1,
        "default_profile": "general",
        "artifacts": artifacts,
        "profiles": profiles,
    }), encoding="utf-8")
    return path


class ModelCatalogTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        model = self.root / "general.pt"
        model.write_bytes(b"general-model")
        digest = hashlib.sha256(model.read_bytes()).hexdigest()
        self.catalog_path = write_catalog(
            self.root,
            {"general-artifact": {
                "path": "general.pt", "sha256": digest,
                "class_names": ["black", "green", "orange", "red", "yellow"],
                "class_name_map": {},
            }},
            {"general": {
                "p1p2_artifact": "general-artifact",
                "p3_artifact": "general-artifact",
                "p1p2_allowed_colors": ["orange", "yellow"],
                "p3_allowed_colors": ["red", "green", "black"],
                "single_general_model": True,
            }},
        )

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_resolve_returns_one_complete_launch_contract(self) -> None:
        catalog = load_catalog(self.catalog_path)
        fields = shell_lines(resolve_profile(catalog, "general"))
        self.assertEqual(len(fields), 12)
        self.assertEqual(fields[0], "general")
        self.assertEqual(fields[3], "black,green,orange,red,yellow")
        self.assertEqual(fields[11], "true")

    def test_selection_is_atomic_alias_only_and_defaults_safely(self) -> None:
        selection = self.root / "selection"
        catalog = load_catalog(self.catalog_path)
        self.assertEqual(selected_alias(catalog, selection), "general")
        select_profile(catalog, "general", selection)
        self.assertEqual(selection.read_text(encoding="utf-8"), "general\n")
        selection.write_text("../escape\n", encoding="utf-8")
        with self.assertRaises(CatalogError):
            selected_alias(catalog, selection)

    def test_hash_mismatch_and_role_incomplete_profile_fail_closed(self) -> None:
        model = self.root / "general.pt"
        model.write_bytes(b"tampered")
        with self.assertRaises(CatalogError):
            load_catalog(self.catalog_path)

    def test_import_general_model_creates_immediately_selectable_profile(self) -> None:
        incoming = self.root / "incoming.pt"
        incoming.write_bytes(b"new-five-color-model")
        result = import_artifact(
            self.catalog_path, "candidate", incoming,
            "black,green,orange,red,yellow",
        )
        self.assertTrue(result["profile_created"])
        catalog = load_catalog(self.catalog_path)
        self.assertIn("candidate", catalog["profiles"])
        self.assertTrue(catalog["artifacts"]["candidate"]["path"].is_file())

    def test_two_artifact_profile_supports_p3_specific_model(self) -> None:
        incoming = self.root / "p3.pt"
        incoming.write_bytes(b"p3-model")
        import_artifact(self.catalog_path, "p3", incoming, "black_buoy,red_buoy,green_buoy")
        add_profile(self.catalog_path, "mixed", "general-artifact", "p3")
        resolved = resolve_profile(load_catalog(self.catalog_path), "mixed")
        self.assertFalse(resolved["single_general_model"])
        self.assertEqual(
            resolved["p3"]["class_name_map"],
            {"black_buoy": "black", "red_buoy": "red", "green_buoy": "green"},
        )


if __name__ == "__main__":
    unittest.main()
