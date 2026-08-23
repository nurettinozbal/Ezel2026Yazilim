"""Puan ceza modülü (çarpma + parkur dışı + puan hesabı).

``ida_planning.scoring`` paketi şartnamedeki (src/_pdf_ozet.txt) çarpma,
parkur dışına çıkma ve puanlama kurallarını saf Python ile modeller:
- ``contact.py``: duba/engel temas sayacı (Ç1/Ç2) — her temas 1 çarpma,
  aynı dubaya >=30 sn sürekli temas 2 çarpma.
- ``out_of_course.py``: parkur dışında kalma sayacı (PDÇ1/PDÇ2) —
  dışarıda >=40 sn kalan takım 2 defa dışarı çıkmış sayılır.
- ``score.py``: şartname formüllerini birebir uygulayan puan hesaplayıcı.

Paket yalnızca ``ida_planning.geo`` ve ``contracts`` kullanır; rclpy ve
autonomy_node'a bağımlı DEĞİLDİR, bu yüzden test_saf.py ile saf-Python
test edilebilir.
"""
