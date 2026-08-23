#!/usr/bin/env python3
"""Validated, atomic model catalog used by field launch and ``ida_cli``.

The versioned catalog describes model artifacts once.  The machine-local
selection file contains only a profile alias, so field changes never require
editing service environment files or duplicating class/hash declarations.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
import os
import re
import shutil
import tempfile
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping


ALIAS_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,39}$")
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
MODEL_SUFFIXES = {".pt", ".engine", ".onnx"}
CONTRACT_COLORS = {"orange", "yellow", "red", "green", "black"}
P1P2_COLORS = {"orange", "yellow"}
P3_COLORS = {"red", "green", "black"}

_COLOR_ALIASES = {
    "orange": "orange", "turuncu": "orange",
    "yellow": "yellow", "sari": "yellow", "sarı": "yellow",
    "red": "red", "kirmizi": "red", "kırmızı": "red",
    "green": "green", "yesil": "green", "yeşil": "green",
    "black": "black", "siyah": "black",
}


class CatalogError(ValueError):
    pass


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _alias(value: Any, label: str = "alias") -> str:
    result = str(value or "").strip().lower()
    if not ALIAS_RE.fullmatch(result):
        raise CatalogError(f"{label} must match {ALIAS_RE.pattern}")
    return result


def _string_list(value: Any, label: str) -> list[str]:
    if not isinstance(value, list) or not value:
        raise CatalogError(f"{label} must be a non-empty list")
    result = [str(item).strip().lower() for item in value]
    if any(not item or "\n" in item or "," in item for item in result):
        raise CatalogError(f"{label} contains an invalid item")
    if len(set(result)) != len(result):
        raise CatalogError(f"{label} contains duplicates")
    return result


def _mapping(value: Any, names: Iterable[str]) -> Dict[str, str]:
    if value in (None, {}):
        mapping: Dict[str, str] = {}
    elif isinstance(value, dict):
        mapping = {
            str(key).strip().lower(): str(mapped).strip().lower()
            for key, mapped in value.items()
        }
    else:
        raise CatalogError("class_name_map must be an object")
    native = list(names)
    if mapping:
        if set(mapping) != set(native):
            raise CatalogError("class_name_map must cover the native classes exactly")
        outputs = list(mapping.values())
    else:
        outputs = native
    if not set(outputs).issubset(CONTRACT_COLORS) or len(set(outputs)) != len(outputs):
        raise CatalogError("mapped classes must be unique contract colors")
    return mapping


def _contained_model_path(models_root: Path, relative: Any) -> Path:
    rel = Path(str(relative or ""))
    if rel.is_absolute() or ".." in rel.parts or rel.suffix.lower() not in MODEL_SUFFIXES:
        raise CatalogError("model path must be a relative .pt/.engine/.onnx path")
    root = models_root.resolve()
    result = (root / rel).resolve()
    if result == root or root not in result.parents:
        raise CatalogError("model path escapes the models directory")
    return result


def load_catalog(catalog_path: str | Path, *, verify_files: bool = True) -> Dict[str, Any]:
    path = Path(catalog_path)
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CatalogError(f"catalog cannot be read: {path}") from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise CatalogError("unsupported model catalog schema")
    artifacts = raw.get("artifacts")
    profiles = raw.get("profiles")
    if not isinstance(artifacts, dict) or not artifacts:
        raise CatalogError("catalog artifacts must be a non-empty object")
    if not isinstance(profiles, dict) or not profiles:
        raise CatalogError("catalog profiles must be a non-empty object")
    models_root = path.parent
    normalized_artifacts: Dict[str, Any] = {}
    for raw_alias, entry in artifacts.items():
        alias = _alias(raw_alias, "artifact alias")
        if not isinstance(entry, dict):
            raise CatalogError(f"artifact {alias} must be an object")
        model_path = _contained_model_path(models_root, entry.get("path"))
        digest = str(entry.get("sha256", "")).strip().lower()
        if not SHA_RE.fullmatch(digest):
            raise CatalogError(f"artifact {alias} has invalid sha256")
        names = _string_list(entry.get("class_names"), f"{alias}.class_names")
        mapping = _mapping(entry.get("class_name_map", {}), names)
        if verify_files:
            if not model_path.is_file():
                raise CatalogError(f"artifact file is missing: {model_path}")
            actual = _sha256(model_path)
            if actual != digest:
                raise CatalogError(f"artifact hash mismatch: {alias}")
        normalized_artifacts[alias] = {
            "path": model_path,
            "relative_path": str(Path(entry["path"]).as_posix()),
            "sha256": digest,
            "class_names": names,
            "class_name_map": mapping,
            "mapped_colors": [mapping.get(name, name) for name in names],
        }
    normalized_profiles: Dict[str, Any] = {}
    for raw_alias, entry in profiles.items():
        alias = _alias(raw_alias, "profile alias")
        if not isinstance(entry, dict):
            raise CatalogError(f"profile {alias} must be an object")
        p1_name = _alias(entry.get("p1p2_artifact"), "p1p2 artifact")
        p3_name = _alias(entry.get("p3_artifact"), "p3 artifact")
        if p1_name not in normalized_artifacts or p3_name not in normalized_artifacts:
            raise CatalogError(f"profile {alias} references an unknown artifact")
        p1_allowed = _string_list(entry.get("p1p2_allowed_colors"), "p1p2 colors")
        p3_allowed = _string_list(entry.get("p3_allowed_colors"), "p3 colors")
        if set(p1_allowed) != P1P2_COLORS or set(p3_allowed) != P3_COLORS:
            raise CatalogError(f"profile {alias} must cover all competition role colors")
        if not set(p1_allowed).issubset(normalized_artifacts[p1_name]["mapped_colors"]):
            raise CatalogError(f"profile {alias} P1/P2 artifact lacks a required color")
        if not set(p3_allowed).issubset(normalized_artifacts[p3_name]["mapped_colors"]):
            raise CatalogError(f"profile {alias} P3 artifact lacks a required color")
        single = entry.get("single_general_model")
        if not isinstance(single, bool) or (single and p1_name != p3_name):
            raise CatalogError(f"profile {alias} has invalid single_general_model")
        normalized_profiles[alias] = {
            "p1p2_artifact": p1_name,
            "p3_artifact": p3_name,
            "p1p2_allowed_colors": p1_allowed,
            "p3_allowed_colors": p3_allowed,
            "single_general_model": single,
        }
    default = _alias(raw.get("default_profile"), "default profile")
    if default not in normalized_profiles:
        raise CatalogError("default profile does not exist")
    return {
        "path": path,
        "raw": raw,
        "default_profile": default,
        "artifacts": normalized_artifacts,
        "profiles": normalized_profiles,
    }


def selected_alias(catalog: Mapping[str, Any], selection_path: str | Path) -> str:
    path = Path(selection_path)
    if not path.is_file():
        return str(catalog["default_profile"])
    value = _alias(path.read_text(encoding="utf-8").strip(), "selected profile")
    if value not in catalog["profiles"]:
        raise CatalogError(f"selected profile is not in catalog: {value}")
    return value


def resolve_profile(catalog: Mapping[str, Any], alias: str) -> Dict[str, Any]:
    name = _alias(alias, "profile alias")
    if name not in catalog["profiles"]:
        raise CatalogError(f"unknown model profile: {name}")
    profile = catalog["profiles"][name]
    return {
        "profile": name,
        "p1p2": catalog["artifacts"][profile["p1p2_artifact"]],
        "p3": catalog["artifacts"][profile["p3_artifact"]],
        **profile,
    }


def _atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        mode = path.stat().st_mode & 0o777
    except OSError:
        mode = 0o644
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        if hasattr(os, "fchmod"):
            os.fchmod(fd, mode)
        else:  # Windows development/test host
            os.chmod(temporary, mode)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.close(fd)
        except OSError:
            pass
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _write_validated_catalog(path: Path, raw: Mapping[str, Any]) -> None:
    """Validate the complete candidate before atomically replacing catalog."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".json", dir=str(path.parent))
    temporary_path = Path(temporary)
    try:
        try:
            mode = path.stat().st_mode & 0o777
        except OSError:
            mode = 0o644
        if hasattr(os, "fchmod"):
            os.fchmod(fd, mode)
        else:  # Windows development/test host
            os.chmod(temporary_path, mode)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(raw, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        load_catalog(temporary_path, verify_files=True)
        os.replace(temporary_path, path)
    except BaseException:
        try:
            os.close(fd)
        except OSError:
            pass
        try:
            temporary_path.unlink()
        except OSError:
            pass
        raise


def select_profile(catalog: Mapping[str, Any], alias: str, selection_path: str | Path) -> None:
    resolve_profile(catalog, alias)
    _atomic_text(Path(selection_path), f"{alias}\n")


def _native_names(model_path: Path, explicit: str) -> list[str]:
    if explicit.strip():
        return _string_list([part for part in explicit.split(",")], "class names")
    try:
        from ultralytics import YOLO  # type: ignore
        # CLI stdout machine-readable JSON olarak kalmalı; Ultralytics'in
        # model özeti/import mesajı sonucu bozmamalı.
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            names = YOLO(str(model_path)).names
    except Exception as exc:
        raise CatalogError("model classes could not be read; pass --classes") from exc
    if isinstance(names, dict):
        try:
            ordered = [names[index] for index in range(len(names))]
        except (KeyError, TypeError) as exc:
            raise CatalogError("model.names is not a contiguous integer mapping") from exc
    elif isinstance(names, (list, tuple)):
        ordered = list(names)
    else:
        raise CatalogError("model.names has an unsupported shape")
    return _string_list(ordered, "model native classes")


def _auto_map(names: Iterable[str]) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for name in names:
        base = name.lower().removesuffix("_buoy").removesuffix("_duba")
        mapped = _COLOR_ALIASES.get(base)
        if mapped is None:
            raise CatalogError(f"class cannot be mapped to a competition color: {name}")
        result[name] = mapped
    if len(set(result.values())) != len(result):
        raise CatalogError("model classes collapse to duplicate competition colors")
    return {} if all(key == value for key, value in result.items()) else result


def import_artifact(catalog_path: Path, alias: str, source: Path, classes: str) -> Dict[str, Any]:
    name = _alias(alias, "artifact alias")
    if not source.is_file() or source.suffix.lower() not in MODEL_SUFFIXES:
        raise CatalogError("source model must be an existing .pt/.engine/.onnx file")
    catalog = load_catalog(catalog_path, verify_files=True)
    if name in catalog["artifacts"]:
        raise CatalogError(f"artifact already exists: {name}")
    names = _native_names(source, classes)
    mapping = _auto_map(names)
    destination = catalog_path.parent / "operator" / f"{name}{source.suffix.lower()}"
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise CatalogError(f"destination already exists: {destination}")
    shutil.copy2(source, destination)
    digest = _sha256(destination)
    raw = catalog["raw"]
    raw["artifacts"][name] = {
        "path": destination.relative_to(catalog_path.parent).as_posix(),
        "sha256": digest,
        "class_names": names,
        "class_name_map": mapping,
    }
    mapped = {mapping.get(item, item) for item in names}
    if P1P2_COLORS | P3_COLORS <= mapped:
        raw["profiles"][name] = {
            "p1p2_artifact": name,
            "p3_artifact": name,
            "p1p2_allowed_colors": ["orange", "yellow"],
            "p3_allowed_colors": ["red", "green", "black"],
            "single_general_model": True,
        }
    _write_validated_catalog(catalog_path, raw)
    return {"artifact": name, "sha256": digest, "profile_created": name in raw["profiles"]}


def add_profile(catalog_path: Path, alias: str, p1p2: str, p3: str) -> None:
    catalog = load_catalog(catalog_path, verify_files=True)
    name = _alias(alias, "profile alias")
    if name in catalog["profiles"]:
        raise CatalogError(f"profile already exists: {name}")
    p1_name = _alias(p1p2, "P1/P2 artifact")
    p3_name = _alias(p3, "P3 artifact")
    raw = catalog["raw"]
    raw["profiles"][name] = {
        "p1p2_artifact": p1_name,
        "p3_artifact": p3_name,
        "p1p2_allowed_colors": ["orange", "yellow"],
        "p3_allowed_colors": ["red", "green", "black"],
        "single_general_model": p1_name == p3_name,
    }
    _write_validated_catalog(catalog_path, raw)


def shell_lines(resolved: Mapping[str, Any]) -> list[str]:
    p1, p3 = resolved["p1p2"], resolved["p3"]
    return [
        str(resolved["profile"]), str(p1["path"]), str(p1["sha256"]),
        ",".join(p1["class_names"]),
        (json.dumps(p1["class_name_map"], separators=(",", ":")) if p1["class_name_map"] else ""),
        ",".join(resolved["p1p2_allowed_colors"]), str(p3["path"]), str(p3["sha256"]),
        ",".join(p3["class_names"]),
        (json.dumps(p3["class_name_map"], separators=(",", ":")) if p3["class_name_map"] else ""),
        ",".join(resolved["p3_allowed_colors"]),
        "true" if resolved["single_general_model"] else "false",
    ]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--catalog", required=True)
    parser.add_argument("--selection", default="/etc/ida/model-selection")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list")
    resolve = sub.add_parser("resolve")
    resolve.add_argument("--profile", default="")
    select = sub.add_parser("select")
    select.add_argument("profile")
    imported = sub.add_parser("import")
    imported.add_argument("alias")
    imported.add_argument("source")
    imported.add_argument("--classes", default="")
    profile = sub.add_parser("profile")
    profile.add_argument("alias")
    profile.add_argument("p1p2_artifact")
    profile.add_argument("p3_artifact")
    args = parser.parse_args(argv)
    catalog_path = Path(args.catalog)
    try:
        catalog = load_catalog(catalog_path, verify_files=True)
        if args.command == "list":
            chosen = selected_alias(catalog, args.selection)
            print("ARTIFACTS")
            for name, item in sorted(catalog["artifacts"].items()):
                print(f"  {name}: {item['relative_path']} [{','.join(item['mapped_colors'])}]")
            print("PROFILES")
            for name, item in sorted(catalog["profiles"].items()):
                marker = "*" if name == chosen else " "
                print(f"{marker} {name}: P1/P2={item['p1p2_artifact']} P3={item['p3_artifact']}")
        elif args.command == "resolve":
            name = args.profile or selected_alias(catalog, args.selection)
            print("\n".join(shell_lines(resolve_profile(catalog, name))))
        elif args.command == "select":
            select_profile(catalog, args.profile, args.selection)
            print(args.profile)
        elif args.command == "import":
            print(json.dumps(import_artifact(catalog_path, args.alias, Path(args.source), args.classes)))
        elif args.command == "profile":
            add_profile(catalog_path, args.alias, args.p1p2_artifact, args.p3_artifact)
            print(args.alias)
    except CatalogError as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
