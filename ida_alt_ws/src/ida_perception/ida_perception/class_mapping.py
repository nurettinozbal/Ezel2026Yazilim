"""Model sınıf adı -> kontrat rengi eşlemesi (saf Python, ida_perception).

Bu senenin P3 modelleri (yolo_8_m/best.pt, yolo_9_n/best2.pt) İngilizce sınıf
adları döndürür: red/green/black. Autonomy ve kontrat da İngilizce
red/black/green bekler (ida_planning.contracts.VALID_TARGET_COLORS) — eşleme
gerekmez, kimlik yeterli. Bu modül yine de geriye dönük olarak Türkçe sınıf
adlı eski modellerin kontrata çevrilmesi için parametrik eşleme mekanizmasını
sunar; rclpy bağımlılığı yoktur, dev makinesinde ve Jetson'da doğrudan test
edilebilir.
"""

import json
from typing import Dict, Iterable, Optional

# Varsayılan eşleme boş: model sınıf adları İngilizce (red/green/black) ve
# kontrat renkleriyle aynı -> kimlik eşleme. Türkçe sınıf adlı bir model
# gerekirse (geriye dönük) eşleme ``class_name_map`` parametresiyle verilir.
DEFAULT_CLASS_NAME_MAP: Dict[str, str] = {}
ALLOWED_DETECTION_COLORS = {"orange", "yellow", "red", "green", "black"}


def parse_class_name_map(raw: str) -> Dict[str, str]:
    """JSON {model_class: contract_color} dizesini sözlüğe çevirir.

    Boş/hatalı girdi -> boş sözlük (kimlik eşleme). Anahtarlar ve değerler
    küçük harfe normalize edilir (model RED/Red döndürebilir).
    """
    if not raw or not raw.strip():
        return {}
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError:
        return {}
    if not isinstance(parsed, dict):
        return {}
    return {
        str(k).strip().lower(): str(v).strip().lower()
        for k, v in parsed.items()
        if str(k).strip() and str(v).strip()
    }


def map_class_name(class_name: str, mapping: Dict[str, str]) -> Optional[str]:
    """Model sınıf adını kontrat rengine çevirir.

    ``mapping`` boşsa kimlik (olduğu gibi) döner — P1/P2 modeli (turuncu/sarı)
    İngilizce sınıf adları döndürdüğünde eşleme gerekmez. ``mapping`` doluysa
    eşlenemeyen sınıf ``None`` döner (kontrat rengi üretilemediği için yayından
    atlanır).
    """
    name = str(class_name).strip().lower()
    if not mapping:
        return name
    return mapping.get(name)


def parse_class_name_map_strict(
    raw: str,
    native_class_names: Iterable[str],
) -> Dict[str, str]:
    """Validate the startup mapping against the exact native model manifest."""

    native = [str(name).strip().lower() for name in native_class_names]
    if not native or any(not name for name in native) or len(set(native)) != len(native):
        raise ValueError("native class manifest is invalid")
    if not raw or not raw.strip():
        if not set(native).issubset(ALLOWED_DETECTION_COLORS):
            raise ValueError("identity class mapping produces unsupported colors")
        return {}
    try:
        parsed = json.loads(raw)
    except (json.JSONDecodeError, TypeError) as exc:
        raise ValueError("class_name_map must be a JSON object") from exc
    if not isinstance(parsed, dict):
        raise ValueError("class_name_map must be a JSON object")
    mapping = {
        str(key).strip().lower(): str(value).strip().lower()
        for key, value in parsed.items()
    }
    if (
        any(not key or not value for key, value in mapping.items())
        or len(mapping) != len(parsed)
        or set(mapping) != set(native)
        or not set(mapping.values()).issubset(ALLOWED_DETECTION_COLORS)
        or len(set(mapping.values())) != len(mapping)
    ):
        raise ValueError("class_name_map keys/colors do not match the native manifest")
    return mapping
