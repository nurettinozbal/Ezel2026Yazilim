"""Çalıştırma klasörü yardımcıları — saf stdlib (ida_logging).

Her loglayıcı düğümü kendi çalıştırma klasörünü ``run_YYYYmmdd_HHMMSS``
deseniyle oluşturur; böylece birden çok koşu birbirinin dosyalarını ezmez ve
analiz aşamasında koşular zaman damgasından ayırt edilebilir.
"""

import os
import re
import time
from datetime import datetime
from pathlib import Path
from typing import Optional


_SAFE_RUN_NAME = re.compile(r"run_[A-Za-z0-9_-]{1,96}\Z")


def make_run_dir(base_dir: str = "./logs") -> Path:
    """``base_dir`` altında zaman damgalı bir çalıştırma klasörü oluşturur.

    Klasör adı formatı: ``run_YYYYmmdd_HHMMSS``. Klasör yoksa (ve gerekiyorsa
    üst dizinlerle birlikte) ``exist_ok=True`` ile oluşturulur; eşzamanlı
    çalıştırmalarda ad çakışması olursa mevcut klasör kullanılır.

    Args:
        base_dir: Kök log dizini (varsayılan "./logs").

    Returns:
        Oluşturulan/kullanılan dizinin ``Path`` nesnesi.
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(base_dir) / f"run_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def run_dir_or_create(base_dir: str, explicit_dir: Optional[str] = None) -> Path:
    """Belirtilen dizini ya da yeni bir run klasörünü döndürür.

    ``explicit_dir`` boş değilse olduğu gibi (gerekirse oluşturularak)
    kullanılır; aksi halde ``make_run_dir`` ile yeni bir koşu klasörü üretilir.
    Böylece düğüm parametrelerinde sabit bir log dizini verilebilir ya da
    her koşuya otomatik yeni klasör açılabilir.
    """
    if explicit_dir and explicit_dir.strip():
        path = Path(explicit_dir)
        path.mkdir(parents=True, exist_ok=True)
        return path
    return make_run_dir(base_dir)


def resolve_log_path(base_dir: str, filename: str, explicit_dir: Optional[str] = None) -> Path:
    """``run_dir_or_create`` altına bir dosya yolu döndürür (oluşturmaz)."""
    run_dir = run_dir_or_create(base_dir, explicit_dir)
    return run_dir / filename


def make_run_dir_same(base_dir: str, run_dir: str) -> Path:
    """Aynı koşu klasörünü paylaşmak için: mevcut klasörü ``run_dir`` yapar.

    Loglayıcı düğümleri bağımsız çalışır ama aynı koşuya yazmak istenebilir;
    bu yardımcı mevcut/oluşturulan klasörü döndürür (ismi kendisi üretmez).
    """
    if not isinstance(run_dir, str) or not _SAFE_RUN_NAME.fullmatch(run_dir):
        raise ValueError("run_dir must be a safe run_* basename")
    base = Path(base_dir).resolve()
    base.mkdir(parents=True, exist_ok=True)
    path = (base / run_dir).resolve()
    if path.parent != base:
        raise ValueError("run_dir escapes base_dir")
    path.mkdir(parents=True, exist_ok=True)
    return path


def cleanup_empty_run_dirs(base_dir: str, max_age_days: float = 7.0) -> int:
    """Boş ve eski run_* klasörlerini temizler; silinen sayısını döndürür.

    Kullanım dışı yarı kalmış (hiç dosya yazılmamış) koşu klasörlerini temiz
    tutmak için isteğe bağlı bir bakım aracıdır. ``max_age_days`` üstü yaşı
    geçmiş ve içi boş klasörleri siler.
    """
    removed = 0
    base = Path(base_dir)
    if not base.is_dir():
        return 0
    for child in base.iterdir():
        if not child.is_dir() or not child.name.startswith("run_"):
            continue
        try:
            mtime = child.stat().st_mtime
        except OSError:
            continue
        age_days = (time.time() - mtime) / 86400.0
        if age_days > max_age_days and not any(child.iterdir()):
            child.rmdir()
            removed += 1
    return removed
