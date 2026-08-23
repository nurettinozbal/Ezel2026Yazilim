"""Şartname puan hesaplayıcısı (src/_pdf_ozet.txt:895-922).

Formüller birebir uygulanır:

- P1 Geçiş:      (G1/KD1) x 10
- P1 Çarpma:     16 - (16 x Ç1)/KD1,  Ç1 in [0:KD1]; Ç1 > KD1 -> 0. Max 16.
- P1 Parkur dışı: 24 - (24 x PDÇ1)/4, PDÇ1 in [0:4]; PDÇ1 > 4 -> 0. Max 24.
- P2 Geçiş:      (G2/KD2) x 40
- P2 Çarpma:     30 - (30 x Ç2)/(KD2+ED2), Ç2 in [0:KD2+ED2]; aşarsa -> 0. Max 30.
- P2 Parkur dışı: 30 - (30 x PDÇ2)/5, PDÇ2 in [0:5]. Max 30.
- P3:            TS3=0 -> 100; TS3=1 -> 50; TS3=2 -> 5; 2<TS3<=8 -> 2; TS3>8 -> 1.
                 İHA bonusu: +45 (başarılı angajman koşuluyla).

``estimated`` her zaman True işaretlenir: temas eşiği şartnamede tanımsız ve
hakem sayısı araçta bilinmediği için sayılar DAVRANIŞI yönlendirir (itiraz
adayı üretir), resmi puan değildir. Puan hesabı navigasyonu ASLA engellemez.
Tüm bölümler 0..maksimum aralığına clamp'lenir; bölücü sıfırsa ilgili bölüm
0 puan verir (maksimum korunmaz — eksik veriyle puan üretilmez).
"""

import math
from dataclasses import dataclass
from typing import Any, Dict


@dataclass
class ScoreReport:
    """Tek bir parkurun puan dökümü.

    ``parkur`` 1/2/3; ``penalty`` sürücü sinyalidir (yakın duba/engel ->
    yavaşlama). ``sections`` formül kırılımını taşır, ``estimated`` her zaman
    True'dur (resmi hakem puanı değildir).
    """

    parkur: int
    total: float
    penalty: float
    estimated: bool
    sections: Dict[str, Any]


def _bounded(numerator: float, denominator: float, max_score: float) -> float:
    """Genel oran puanı: oran * max_score, [0, max_score] aralığına clamp'lenir.

    ``denominator <= 0`` ise 0.0 döner (bölme hatası olmaz). NaN/Inf girdiler
    önce sıfırlanır; çıktı negatifse 0'a, maksimumun üstüyse maksimuma çekilir.
    """
    if math.isnan(numerator) or math.isinf(numerator) or numerator < 0.0:
        numerator = 0.0
    if math.isnan(denominator) or math.isinf(denominator) or denominator <= 0.0:
        return 0.0
    return max(0.0, min(max_score, (numerator / denominator) * max_score))


def _clamp_score(value: float, max_score: float) -> float:
    """Puan kalemini [0, max_score] aralığına çeker (negatif girdi güvencesi).

    Çarpma/parkur dışı formülleri negatif sayaçla (algı hatası) maksimumun
    üstüne çıkabilir; her kalem kendi maksimumuna tavanlanır.
    """
    if math.isnan(value) or math.isinf(value):
        return 0.0
    return max(0.0, min(max_score, value))


class ScoreCalculator:
    """P1/P2/P3 puanlarını şartname formülleriyle hesaplar.

    Yapılandırma: KD1, KD2, ED2 (şartname değişkenleri) ve P3 İHA bonusu.
    ``report()`` girdi olarak parkur numarası ve ilgili sayaçları alır;
    sayaçlar ContactCounter/OutOfCourseDetector'dan gelir (Ç1/Ç2, PDÇ1/PDÇ2)
    ya da dışarıdan verilir (test).
    """

    def __init__(self, kd1: float, kd2: float, ed2: float, include_uav_bonus: bool = True) -> None:
        self.kd1 = max(0.0, float(kd1))
        self.kd2 = max(0.0, float(kd2))
        self.ed2 = max(0.0, float(ed2))
        self.include_uav_bonus = bool(include_uav_bonus)

    def report(
        self,
        parkur: int,
        g1: float = 0,
        contact1: float = 0,
        ooc1: float = 0,
        g2: float = 0,
        contact2: float = 0,
        ooc2: float = 0,
        ts3: float = 0,
        uav_bonus: bool = False,
    ) -> ScoreReport:
        """Belirtilen parkur için puan dökümü üretir.

        Args:
            parkur: 1, 2 ya da 3.
            g1/g2: farklı karşılıklı kenar duba ikililerinden geçiş sayısı.
            contact1/contact2: çarpılan duba sayısı (Ç1/Ç2).
            ooc1/ooc2: parkur dışına çıkış sayısı (PDÇ1/PDÇ2).
            ts3: TS3 (angaje olunacak hedefe vurmadan önce farklı hedeflere
                temas sayısı).
            uav_bonus: İHA kullanımı bonusu (yalnız parkur 3, angajman
                koşuluyla +45).

        Returns:
            ``ScoreReport`` — toplam, ceza (sürücü sinyali), estimated=True
            ve formül kırılımları.
        """
        if parkur == 1:
            return self._report_p1(g1, contact1, ooc1)
        if parkur == 2:
            return self._report_p2(g2, contact2, ooc2)
        return self._report_p3(ts3, uav_bonus)

    def _report_p1(self, g1: float, contact1: float, ooc1: float) -> ScoreReport:
        # P1 çarpma: 16 - (16 x Ç1)/KD1, Ç1 in [0:KD1]; Ç1 > KD1 -> 0. Max 16.
        c1 = _safe_num(contact1)
        kd1 = self.kd1
        if kd1 <= 0.0 or c1 > kd1:
            contact_score = 0.0
        else:
            contact_score = _clamp_score(16.0 - (16.0 * c1) / kd1, 16.0)
        # P1 parkur dışı: 24 - (24 x PDÇ1)/4, PDÇ1 in [0:4]; > 4 -> 0. Max 24.
        o1 = _safe_num(ooc1)
        if o1 > 4.0:
            ooc_score = 0.0
        else:
            ooc_score = _clamp_score(24.0 - (24.0 * o1) / 4.0, 24.0)
        crossing = _bounded(_safe_num(g1), kd1, 10.0)
        total = crossing + contact_score + ooc_score
        return ScoreReport(
            parkur=1,
            total=round(total, 2),
            penalty=round(contact_score + ooc_score, 2),
            estimated=True,
            sections={
                "g1": round(crossing, 2),
                "contact": round(contact_score, 2),
                "ooc": round(ooc_score, 2),
                "max": 55.0,
            },
        )

    def _report_p2(self, g2: float, contact2: float, ooc2: float) -> ScoreReport:
        # P2 çarpma: 30 - (30 x Ç2)/(KD2+ED2), Ç2 in [0:KD2+ED2]; aşarsa -> 0. Max 30.
        c2 = _safe_num(contact2)
        denom2 = self.kd2 + self.ed2
        if denom2 <= 0.0 or c2 > denom2:
            contact_score = 0.0
        else:
            contact_score = _clamp_score(30.0 - (30.0 * c2) / denom2, 30.0)
        # P2 parkur dışı: 30 - (30 x PDÇ2)/5, PDÇ2 in [0:5]; > 5 -> 0. Max 30.
        o2 = _safe_num(ooc2)
        if o2 > 5.0:
            ooc_score = 0.0
        else:
            ooc_score = _clamp_score(30.0 - (30.0 * o2) / 5.0, 30.0)
        crossing = _bounded(_safe_num(g2), self.kd2, 40.0)
        total = crossing + contact_score + ooc_score
        return ScoreReport(
            parkur=2,
            total=round(total, 2),
            penalty=round(contact_score + ooc_score, 2),
            estimated=True,
            sections={
                "g2": round(crossing, 2),
                "contact": round(contact_score, 2),
                "ooc": round(ooc_score, 2),
                "max": 100.0,
            },
        )

    def _report_p3(self, ts3: float, uav_bonus: bool) -> ScoreReport:
        # TS3 tablosu (src/_pdf_ozet.txt:915-921).
        t = _safe_num(ts3)
        if t <= 0.0:
            engage_score = 100.0
        elif t == 1:
            engage_score = 50.0
        elif t == 2:
            engage_score = 5.0
        elif t <= 8.0:
            engage_score = 2.0
        else:
            engage_score = 1.0
        bonus = 45.0 if (self.include_uav_bonus and uav_bonus) else 0.0
        total = engage_score + bonus
        return ScoreReport(
            parkur=3,
            total=round(total, 2),
            penalty=0.0,
            estimated=True,
            sections={
                "ts3": t,
                "engage": engage_score,
                "uav_bonus": bonus,
                "max": 145.0,
            },
        )


def _safe_num(value: Any) -> float:
    """float'a çevirir; NaN/Inf/çevrilemez değerlerde 0.0 döner."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return 0.0
    if math.isnan(result) or math.isinf(result):
        return 0.0
    return result
