"""Küçük, bağımlılıksız CSV yazıcı (ida_logging).

``csv_writer.CsvWriter`` sınıfı dict satırları sabit bir sütun sırasıyla
CSV dosyasına yazar. İlk satır her zaman header'dır; satır sırası verilen
``header`` listesine göre sabittir, böylece dosya pandas/Excel ile okunurken
sütun karışmaz. ``flush_every_lines`` dolunca diske flush edilir (kayıt
kaybolmaması için), ``close()`` son flush'ı yapar.
"""

import csv
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


class CsvWriter:
    """Sütun sırası sabit, header'lı CSV dosya yazıcısı."""

    def __init__(
        self,
        path: str | Path,
        header: Iterable[str],
        flush_every_lines: int = 100,
    ) -> None:
        """CSV dosyasını açar ve header satırını yazar.

        Args:
            path: Oluşturulacak CSV dosyasının yolu.
            header: Sütun adları (yazım sırası korunur).
            flush_every_lines: Kaç satırda bir diske flush edileceği.
        """
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.header = list(header)
        self.flush_every_lines = max(1, int(flush_every_lines))
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.header, extrasaction="ignore")
        self._writer.writeheader()
        self._row_count = 0
        self._closed = False

    def add_row(self, row: Dict[str, Any]) -> None:
        """Tek bir satır yazar; header'da olmayan anahtarlar atlanır.

        Eksik anahtarlar boş hücre olarak yazılır; ``None`` değerler de boş
        hücre üretir. Kapatılmış bir yazıcıya yazmaya çalışmak sessizce
        reddedilir (shutdown sonrası timer kazası olmasın).
        """
        if self._closed:
            return
        sanitized: Dict[str, Any] = {}
        for key in self.header:
            value = row.get(key)
            if value is None:
                sanitized[key] = ""
            elif isinstance(value, float):
                # NaN/Inf, CSV'de okunabilirliği bozar; elle kontrol.
                sanitized[key] = "" if _not_finite(value) else value
            else:
                sanitized[key] = value
        self._writer.writerow(sanitized)
        self._row_count += 1
        if self._row_count % self.flush_every_lines == 0:
            self._file.flush()

    def flush(self) -> None:
        """Ara tamponu diske boşaltır (kapanmadan önce de çağrılabilir)."""
        if not self._closed:
            self._file.flush()

    def close(self) -> None:
        """Kalan satırları flush edip dosyayı kapatır; birden çok çağrı güvenli."""
        if self._closed:
            return
        try:
            self._file.flush()
        finally:
            self._file.close()
            self._closed = True

    @property
    def row_count(self) -> int:
        return self._row_count

    @property
    def closed(self) -> bool:
        return self._closed

    def __enter__(self) -> "CsvWriter":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()


def _not_finite(value: float) -> bool:
    """float NaN/Inf tespiti (math yüklenmeden, import maliyeti yok)."""
    return value != value or value in (float("inf"), float("-inf"))


def write_csv_rows(
    path: str | Path,
    header: Iterable[str],
    rows: List[Dict[str, Any]],
    flush_every_lines: int = 100,
) -> Path:
    """Tek seferde bir dizi satırı yazar ve dosyayı kapatır (test/araç fonksiyonu)."""
    writer = CsvWriter(path, header, flush_every_lines=flush_every_lines)
    for row in rows:
        writer.add_row(row)
    writer.close()
    return writer.path
