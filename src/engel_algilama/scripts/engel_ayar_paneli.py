#!/usr/bin/env python3
"""
engel_ayar_paneli.py

engel_node'un ayarlarını değiştirmek için Türkçe, açıklamalı bir pencere.
rqt_reconfigure yerine kullanılır.

Pencerede:
  - Solda: canlı karar (DUR / DIKKAT / GUVENLI) ve yukarıdan bakış çizimi
    (yelpaze şu an tam olarak nereye bakıyor, kaç derece, kaç metre).
  - Sağda: her ayar için kaydırma çubuğu + ne işe yaradığının açıklaması.

Bu panel LiDAR'a veya sürücüsüne HİÇBİR ayar göndermez; sadece engel_node'un
parametrelerini değiştirir.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String

from python_qt_binding.QtCore import QPointF, QRectF, Qt, QTimer
from python_qt_binding.QtGui import QBrush, QColor, QFont, QPainter, QPen
from python_qt_binding.QtWidgets import (
    QApplication, QGroupBox, QHBoxLayout, QLabel, QPushButton, QScrollArea,
    QSlider, QVBoxLayout, QWidget,
)

# -----------------------------------------------------------------------------
# Ayar tanımları: (parametre adı, başlık, birim, alt, üst, adım, varsayılan, açıklama)
# Alt/üst değerler engel_node.cpp'deki aralıklarla AYNI olmalı.
# -----------------------------------------------------------------------------
GRUPLAR = [
    ("1) NEREYE BAKILSIN?", [
        ("merkez_aci", "Bakış yönü", "°", -180.0, 180.0, 1.0, 0.0,
         "Yelpazenin baktığı yön. 0 = LiDAR'ın önü (RViz'deki beyaz ok), "
         "+90 = sol, -90 = sağ, 180 = arka."),
        ("gorus_genisligi", "Görüş genişliği", "°", 10.0, 360.0, 1.0, 90.0,
         "Yelpazenin toplam açısı. Örn. 90 → bakış yönünün 45° sağından 45° "
         "soluna kadar. 360 → her yön."),
    ]),
    ("2) HANGİ NOKTALAR YOK SAYILSIN?", [
        ("min_mesafe", "Yakın sınır", "m", 0.0, 3.0, 0.05, 0.3,
         "LiDAR'a bundan yakın noktalar yok sayılır (aracın kendisi, masadaki "
         "eşyalar). Masada test ederken ~1.0 yapın."),
        ("z_min", "Alt sınır (zemin)", "m", -3.0, 0.0, 0.05, -1.2,
         "LiDAR'ın merkezinden bu kadar AŞAĞIDAKİ noktalar yok sayılır. ZEMİNİN "
         "biraz ÜSTÜNDE olmalı, yoksa yer engel sanılır! LiDAR masadaysa (yer ≈ "
         "-0.85 m) → -0.6 yapın. Araçta yerden 1.3 m yüksekteyse → -1.1."),
        ("z_max", "Üst sınır (tavan)", "m", 0.0, 3.0, 0.05, 1.0,
         "LiDAR'ın merkezinden bu kadar YUKARIDAKİ noktalar yok sayılır "
         "(tavan, ağaç dalı, köprü)."),
    ]),
    ("3) NE ZAMAN UYARI VERİLSİN?", [
        ("dur_mesafesi", "DUR mesafesi", "m", 0.3, 10.0, 0.1, 2.0,
         "Engel bundan yakınsa karar DUR olur. (Çizimde ve RViz'de kırmızı yay)"),
        ("dikkat_mesafesi", "DİKKAT mesafesi", "m", 0.5, 20.0, 0.1, 5.0,
         "Engel bundan yakınsa karar DİKKAT olur. Bundan uzak her şey GÜVENLİ "
         "sayılır. (Çizimde ve RViz'de sarı sınır = yelpazenin boyu)"),
    ]),
]

# Ad → tanım sözlüğü (kolay erişim için)
TANIMLAR = {t[0]: t for _, liste in GRUPLAR for t in liste}

RENKLER = {
    "DUR": QColor(220, 40, 40),
    "DIKKAT": QColor(235, 190, 0),
    "GUVENLI": QColor(40, 170, 60),
}


def aciyi_normalize_et(aci):
    """Açıyı -180..180 aralığına getirir."""
    return (aci + 180.0) % 360.0 - 180.0


def aciyi_anlat(aci):
    """-45 → 'sağ 45°', 30 → 'sol 30°', 0 → 'tam ön' gibi okunur metin."""
    a = aciyi_normalize_et(aci)
    if abs(a) < 0.5:
        return "tam ön"
    if abs(abs(a) - 180.0) < 0.5:
        return "tam arka"
    return f"sol {a:.0f}°" if a > 0 else f"sağ {-a:.0f}°"


def ayarlari_kontrol_et(d):
    """engel_node'daki kontrolün aynısı. Sorun yoksa None, varsa sebep döndürür."""
    if d["dur_mesafesi"] >= d["dikkat_mesafesi"]:
        return "DUR mesafesi, DİKKAT mesafesinden küçük olmalı."
    if d["min_mesafe"] >= d["dur_mesafesi"]:
        return "Yakın sınır, DUR mesafesinden küçük olmalı."
    if d["z_min"] >= d["z_max"]:
        return "Alt sınır (zemin), üst sınırdan (tavan) küçük olmalı."
    return None


# -----------------------------------------------------------------------------
# Tek bir ayar satırı: başlık + değer + kaydırma çubuğu + açıklama
# -----------------------------------------------------------------------------
class AyarSatiri(QWidget):
    def __init__(self, tanim, degisince):
        super().__init__()
        self.ad, baslik, self.birim, self.alt, self.ust, self.adim, _, aciklama = tanim
        self.degisince = degisince

        # Qt kaydırma çubukları sadece tam sayı çalışır: değeri adım sayısına çeviriyoruz
        adim_sayisi = int(round((self.ust - self.alt) / self.adim))

        self.baslik = QLabel(f"<b>{baslik}</b>")
        self.deger = QLabel()
        self.deger.setFont(QFont("Monospace", 11, QFont.Bold))
        self.deger.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.cubuk = QSlider(Qt.Horizontal)
        self.cubuk.setRange(0, adim_sayisi)
        self.cubuk.valueChanged.connect(self._cubuk_oynadi)

        alt_yazi = QLabel(f"{self.alt:g}{self.birim}")
        ust_yazi = QLabel(f"{self.ust:g}{self.birim}")
        for y in (alt_yazi, ust_yazi):
            y.setStyleSheet("color: gray; font-size: 9pt;")

        self.aciklama = QLabel(aciklama)
        self.aciklama.setWordWrap(True)
        self.aciklama.setStyleSheet("color: #555; font-size: 9pt;")

        ust_satir = QHBoxLayout()
        ust_satir.addWidget(self.baslik)
        ust_satir.addStretch()
        ust_satir.addWidget(self.deger)

        cubuk_satiri = QHBoxLayout()
        cubuk_satiri.addWidget(alt_yazi)
        cubuk_satiri.addWidget(self.cubuk, 1)
        cubuk_satiri.addWidget(ust_yazi)

        duzen = QVBoxLayout(self)
        duzen.setContentsMargins(4, 6, 4, 6)
        duzen.addLayout(ust_satir)
        duzen.addLayout(cubuk_satiri)
        duzen.addWidget(self.aciklama)

    def deger_al(self):
        v = self.alt + self.cubuk.value() * self.adim
        return round(v, 3)  # 0.1*23 = 2.3000000004 gibi kayan nokta artıklarını temizle

    def deger_ayarla(self, v, sinyal_gonder=False):
        """Çubuğu verilen değere getirir (node'dan okunan değerler için)."""
        self.cubuk.blockSignals(not sinyal_gonder)
        self.cubuk.setValue(int(round((v - self.alt) / self.adim)))
        self.cubuk.blockSignals(False)
        self._yaziyi_guncelle()

    def hatali_goster(self, hatali):
        renk = "#d22" if hatali else "black"
        self.deger.setStyleSheet(f"color: {renk};")

    def _yaziyi_guncelle(self):
        ondalik = 0 if self.adim >= 1 else (1 if self.adim >= 0.1 else 2)
        self.deger.setText(f"{self.deger_al():.{ondalik}f} {self.birim}")

    def _cubuk_oynadi(self, _):
        self._yaziyi_guncelle()
        self.degisince()


# -----------------------------------------------------------------------------
# Yukarıdan bakış çizimi: yelpaze tam olarak nereye bakıyor?
# Ekranda YUKARI = LiDAR'ın önü (x), SOL = LiDAR'ın solu (y).
# -----------------------------------------------------------------------------
class YelpazeCizimi(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(320, 320)
        self.ayar = {t[0]: t[6] for t in TANIMLAR.values()}
        self.karar = None
        self.engel = None  # (x, y) kararı veren nokta, yoksa None

    def guncelle(self, ayar, karar, engel=None):
        self.ayar = dict(ayar)
        self.karar = karar
        self.engel = engel
        self.update()  # yeniden çiz

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor(35, 35, 40))

        a = self.ayar
        merkez, genislik = a["merkez_aci"], a["gorus_genisligi"]
        dikkat = max(a["dikkat_mesafesi"], 0.1)
        dur = min(a["dur_mesafesi"], dikkat)
        yakin = min(a["min_mesafe"], dur)

        cx, cy = w / 2.0, h / 2.0
        R = min(w, h) / 2.0 - 34          # dikkat mesafesinin piksel karşılığı
        olcek = R / dikkat                # piksel / metre

        # Qt açıları: 0 = saat 3 yönü, saat yönünün tersine artar, 1/16 derece birimi.
        # Bizim 0° = yukarı (ön), +açı = sol (saat yönünün tersi) → qt_açı = 90 + bizim açı
        baslangic16 = int((90.0 + merkez - genislik / 2.0) * 16)
        aralik16 = int(genislik * 16)

        def dilim(r_metre, dolgu, kalem=Qt.NoPen):
            r = r_metre * olcek
            kutu = QRectF(cx - r, cy - r, 2 * r, 2 * r)
            p.setPen(kalem)
            p.setBrush(QBrush(dolgu))
            if genislik >= 360.0:
                p.drawEllipse(kutu)
            else:
                p.drawPie(kutu, baslangic16, aralik16)

        # Referans: 360° tam daire (kesik gri) ve eksenler
        p.setPen(QPen(QColor(90, 90, 90), 1, Qt.DashLine))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QRectF(cx - R, cy - R, 2 * R, 2 * R))
        p.drawLine(QPointF(cx, cy - R), QPointF(cx, cy + R))
        p.drawLine(QPointF(cx - R, cy), QPointF(cx + R, cy))

        # Bölgeler: DİKKAT (sarı), DUR (kırmızı), yok sayılan yakın bölge (arka plan rengi)
        dilim(dikkat, QColor(235, 190, 0, 90), QPen(QColor(235, 190, 0), 2))
        dilim(dur, QColor(220, 40, 40, 130), QPen(QColor(220, 40, 40), 2))
        if yakin > 0:
            dilim(yakin, QColor(35, 35, 40), QPen(QColor(120, 120, 120), 1, Qt.DotLine))

        # Yön yazıları
        p.setPen(QColor(200, 200, 200))
        p.setFont(QFont("Sans", 9, QFont.Bold))
        p.drawText(QRectF(cx - 40, cy - R - 28, 80, 18), Qt.AlignCenter, "ÖN (0°)")
        p.drawText(QRectF(cx - 40, cy + R + 8, 80, 18), Qt.AlignCenter, "ARKA (180°)")
        p.drawText(QRectF(2, cy - 9, 30, 18), Qt.AlignCenter, "SOL")
        p.drawText(QRectF(w - 32, cy - 9, 30, 18), Qt.AlignCenter, "SAĞ")

        # LiDAR ve önünü gösteren beyaz ok
        p.setPen(QPen(QColor(255, 255, 255), 2))
        p.drawLine(QPointF(cx, cy), QPointF(cx, cy - 26))
        p.drawLine(QPointF(cx, cy - 26), QPointF(cx - 5, cy - 18))
        p.drawLine(QPointF(cx, cy - 26), QPointF(cx + 5, cy - 18))
        p.setBrush(QBrush(QColor(80, 160, 255)))
        p.drawEllipse(QPointF(cx, cy), 6, 6)

        # Kararı veren nokta: ekranda yukarı = +x (ön), sol = +y (sol)
        if self.engel is not None:
            ex, ey = self.engel
            px, py = cx - ey * olcek, cy - ex * olcek
            renk = RENKLER.get(self.karar, QColor(200, 200, 200))
            p.setPen(QPen(renk, 1, Qt.DashLine))
            p.drawLine(QPointF(cx, cy), QPointF(px, py))
            p.setPen(QPen(QColor(255, 255, 255), 2))
            p.setBrush(QBrush(renk))
            p.drawEllipse(QPointF(px, py), 8, 8)

        # Ölçek bilgisi
        p.setPen(QColor(170, 170, 170))
        p.setFont(QFont("Sans", 8))
        p.drawText(6, h - 8, f"Dış çember = {dikkat:.1f} m")


# -----------------------------------------------------------------------------
# Ana pencere
# -----------------------------------------------------------------------------
class AnaPencere(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Engel Algılama — Ayar Paneli")
        self.resize(1050, 720)

        hedef = node.declare_parameter("hedef_node", "engel_node").value
        self.istemci = AsyncParameterClient(node, hedef)
        self.hedef = hedef
        self.yuklendi = False        # node'dan ilk değerler okundu mu?
        self.okuma_istegi = None
        self.gonderme_istegi = None
        self.son_gonderilen = {}

        self.karar, self.mesafe, self.son_mesaj = None, -1.0, 0.0
        self.engel_noktasi, self.engel_zamani = None, 0.0
        node.create_subscription(String, "/engel/karar", self._karar_geldi, 10)
        node.create_subscription(Float32, "/engel/mesafe", self._mesafe_geldi, 10)
        node.create_subscription(PointStamped, "/engel/engel_noktasi", self._nokta_geldi, 10)

        # ---------------- Sol sütun: karar + çizim ----------------
        self.karar_yazisi = QLabel("Bağlanıyor…")
        self.karar_yazisi.setAlignment(Qt.AlignCenter)
        self.karar_yazisi.setFont(QFont("Sans", 26, QFont.Bold))
        self.karar_yazisi.setMinimumHeight(90)

        # Kararı hangi nokta verdi? (açı + mesafe + yükseklik)
        self.neden_yazisi = QLabel()
        self.neden_yazisi.setAlignment(Qt.AlignCenter)
        self.neden_yazisi.setWordWrap(True)
        self.neden_yazisi.setFont(QFont("Sans", 10))

        self.cizim = YelpazeCizimi()
        self.bolge_yazisi = QLabel()
        self.bolge_yazisi.setAlignment(Qt.AlignCenter)
        self.bolge_yazisi.setWordWrap(True)
        self.bolge_yazisi.setFont(QFont("Sans", 10))

        sol = QVBoxLayout()
        sol.addWidget(self.karar_yazisi)
        sol.addWidget(self.neden_yazisi)
        sol.addWidget(self.cizim, 1)
        sol.addWidget(self.bolge_yazisi)

        # ---------------- Sağ sütun: ayar çubukları ----------------
        self.satirlar = {}
        sag_icerik = QWidget()
        sag = QVBoxLayout(sag_icerik)
        for grup_adi, liste in GRUPLAR:
            kutu = QGroupBox(grup_adi)
            kutu.setStyleSheet("QGroupBox { font-weight: bold; }")
            kutu_duzen = QVBoxLayout(kutu)
            for tanim in liste:
                satir = AyarSatiri(tanim, self._ayar_degisti)
                satir.setEnabled(False)  # node'dan değerler gelene kadar kilitli
                self.satirlar[tanim[0]] = satir
                kutu_duzen.addWidget(satir)
            sag.addWidget(kutu)
        sag.addStretch()

        kaydirma = QScrollArea()
        kaydirma.setWidgetResizable(True)
        kaydirma.setWidget(sag_icerik)

        self.durum = QLabel()
        self.durum.setWordWrap(True)
        varsayilan = QPushButton("Varsayılan değerlere dön")
        varsayilan.clicked.connect(self._varsayilana_don)

        alt = QHBoxLayout()
        alt.addWidget(self.durum, 1)
        alt.addWidget(varsayilan)

        sag_sutun = QVBoxLayout()
        sag_sutun.addWidget(kaydirma, 1)
        sag_sutun.addLayout(alt)

        ana = QHBoxLayout(self)
        ana.addLayout(sol, 5)
        ana.addLayout(sag_sutun, 6)

        # Çubuk oynarken saniyede onlarca istek göndermemek için kısa bekleme
        self.gonder_zamanlayici = QTimer(self)
        self.gonder_zamanlayici.setSingleShot(True)
        self.gonder_zamanlayici.setInterval(150)
        self.gonder_zamanlayici.timeout.connect(self._gonder)

        # ROS mesajlarını işlemek için Qt zamanlayıcısı (her 30 ms)
        self.ros_zamanlayici = QTimer(self)
        self.ros_zamanlayici.timeout.connect(self._ros_isle)
        self.ros_zamanlayici.start(30)

        self._durum_yaz(f"'{self.hedef}' bekleniyor… (engel_node çalışıyor mu?)", "gray")
        self._cizimi_guncelle()

    # ---------------- Yardımcılar ----------------
    def _degerler(self):
        return {ad: s.deger_al() for ad, s in self.satirlar.items()}

    def _durum_yaz(self, metin, renk="black"):
        self.durum.setText(metin)
        self.durum.setStyleSheet(f"color: {renk};")

    def _cizimi_guncelle(self):
        d = self._degerler() if self.yuklendi else {t[0]: t[6] for t in TANIMLAR.values()}
        engel = None
        if self.engel_noktasi is not None and self.mesafe >= 0 \
                and time.time() - self.engel_zamani < 0.5:
            engel = (self.engel_noktasi.x, self.engel_noktasi.y)
        self.cizim.guncelle(d, self.karar, engel)
        g = d["gorus_genisligi"]
        if g >= 360:
            bolge = "İzlenen bölge: <b>her yön (360°)</b>"
        else:
            bas = aciyi_anlat(d["merkez_aci"] - g / 2)
            bit = aciyi_anlat(d["merkez_aci"] + g / 2)
            bolge = (f"İzlenen bölge: <b>{bas}</b> ile <b>{bit}</b> arası "
                     f"(toplam {g:.0f}°, yön: {aciyi_anlat(d['merkez_aci'])})")
        bolge += (f"<br>Mesafe: <b>{d['min_mesafe']:.2f} m</b> ile "
                  f"<b>{d['dikkat_mesafesi']:.1f} m</b> arası · "
                  f"Yükseklik: <b>{d['z_min']:+.2f} m</b> ile <b>{d['z_max']:+.2f} m</b> arası")
        if d["z_max"] - d["z_min"] < 0.5:
            bolge += ("<br><span style='color:#d22'>⚠ Yükseklik aralığı çok dar "
                      f"({(d['z_max'] - d['z_min']) * 100:.0f} cm) — çoğu cisim görülmez!</span>")
        self.bolge_yazisi.setText(bolge)

    # ---------------- ROS ----------------
    def _karar_geldi(self, msg):
        self.karar, self.son_mesaj = msg.data, time.time()

    def _mesafe_geldi(self, msg):
        self.mesafe = msg.data

    def _nokta_geldi(self, msg):
        self.engel_noktasi, self.engel_zamani = msg.point, time.time()

    def _neden_yazisini_guncelle(self):
        """Kararı veren noktayı düz Türkçe ile anlatır."""
        taze = self.engel_noktasi is not None and time.time() - self.engel_zamani < 0.5
        if self.mesafe < 0 or not taze:
            self.neden_yazisi.setText("Görüş alanında engel yok.")
            self.neden_yazisi.setStyleSheet("color: #2a2;")
            return
        p = self.engel_noktasi
        aci = math.degrees(math.atan2(p.y, p.x))
        metin = (f"Kararı veren nokta: <b>{aciyi_anlat(aci)}</b> ({aci:+.0f}°), "
                 f"<b>{self.mesafe:.2f} m</b> uzakta, yükseklik <b>{p.z:+.2f} m</b>")
        # Nokta LiDAR'ın epey altındaysa uyar: büyük ihtimalle yer (zemin) engel sanılıyor
        if p.z < -0.5:
            oneri = p.z + 0.25
            metin += (f"<br><span style='color:#d22'>⚠ Bu nokta LiDAR'ın {-p.z * 100:.0f} cm "
                      f"ALTINDA — büyük ihtimalle ZEMİN. Yer ise 'Alt sınır (zemin)' "
                      f"değerini yaklaşık <b>{oneri:.2f}</b> yapın.</span>")
        self.neden_yazisi.setText(metin)
        self.neden_yazisi.setStyleSheet("color: black;")

    def _ros_isle(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

        # 1) İlk açılışta node'un mevcut değerlerini oku
        if not self.yuklendi and self.okuma_istegi is None and self.istemci.services_are_ready():
            self.okuma_istegi = self.istemci.get_parameters(list(self.satirlar))
        if self.okuma_istegi is not None and self.okuma_istegi.done():
            sonuc = self.okuma_istegi.result()
            self.okuma_istegi = None
            if sonuc is not None and len(sonuc.values) == len(self.satirlar):
                for ad, deger in zip(self.satirlar, sonuc.values):
                    self.satirlar[ad].deger_ayarla(float(parameter_value_to_python(deger)))
                    self.satirlar[ad].setEnabled(True)
                self.son_gonderilen = self._degerler()
                self.yuklendi = True
                self._durum_yaz(f"'{self.hedef}' bağlandı. Çubukları kaydırın, "
                                "değişiklik anında uygulanır.", "green")
                self._cizimi_guncelle()

        # 2) Gönderilen ayarın sonucu geldi mi?
        if self.gonderme_istegi is not None and self.gonderme_istegi.done():
            sonuc = self.gonderme_istegi.result()
            self.gonderme_istegi = None
            hatalar = [r.reason for r in (sonuc.results if sonuc else []) if not r.successful]
            if hatalar:
                self._durum_yaz("Reddedildi: " + "; ".join(hatalar), "#d22")
            else:
                self._durum_yaz("Uygulandı ✓", "green")

        # 3) Kararı veren nokta: yazı + çizim (saniyede ~10 kez yeterli)
        simdi = time.time()
        if simdi - getattr(self, "_son_cizim", 0.0) > 0.1:
            self._son_cizim = simdi
            self._neden_yazisini_guncelle()
            self._cizimi_guncelle()

        # 4) Karar yazısı
        if time.time() - self.son_mesaj > 1.5:
            self.karar_yazisi.setText("VERİ YOK\n(LiDAR / engel_node açık mı?)")
            self.karar_yazisi.setFont(QFont("Sans", 16, QFont.Bold))
            self.karar_yazisi.setStyleSheet("background: #666; color: white; border-radius: 8px;")
        else:
            renk = RENKLER.get(self.karar, QColor(100, 100, 100))
            alt_metin = f"{self.mesafe:.2f} m" if self.mesafe >= 0 else "engel yok"
            self.karar_yazisi.setText(f"{self.karar}\n{alt_metin}")
            self.karar_yazisi.setFont(QFont("Sans", 26, QFont.Bold))
            self.karar_yazisi.setStyleSheet(
                f"background: {renk.name()}; color: white; border-radius: 8px;")

    # ---------------- Çubuk değişince ----------------
    def _ayar_degisti(self):
        self._cizimi_guncelle()
        d = self._degerler()
        hata = ayarlari_kontrol_et(d)
        # Hatalı ilişkideki değerleri kırmızı göster
        for ad, s in self.satirlar.items():
            s.hatali_goster(False)
        if hata:
            ilgili = {"DUR mesafesi, DİKKAT": ("dur_mesafesi", "dikkat_mesafesi"),
                      "Yakın sınır": ("min_mesafe", "dur_mesafesi"),
                      "Alt sınır": ("z_min", "z_max")}
            for anahtar, adlar in ilgili.items():
                if hata.startswith(anahtar):
                    for ad in adlar:
                        self.satirlar[ad].hatali_goster(True)
            self._durum_yaz("⚠ " + hata + " (Düzeltilene kadar gönderilmiyor.)", "#d22")
            self.gonder_zamanlayici.stop()
            return
        self.gonder_zamanlayici.start()  # 150 ms sonra gönder

    def _gonder(self):
        if not self.yuklendi:
            return
        if self.gonderme_istegi is not None:
            self.gonder_zamanlayici.start()  # önceki istek bitmedi, biraz sonra tekrar dene
            return
        d = self._degerler()
        degisenler = [Parameter(ad, Parameter.Type.DOUBLE, float(v))
                      for ad, v in d.items() if self.son_gonderilen.get(ad) != v]
        if not degisenler:
            return
        self.son_gonderilen = d
        self.gonderme_istegi = self.istemci.set_parameters(degisenler)

    def _varsayilana_don(self):
        for ad, s in self.satirlar.items():
            s.deger_ayarla(TANIMLAR[ad][6])
        self._ayar_degisti()


def main():
    rclpy.init(args=sys.argv)
    node = Node("engel_ayar_paneli")
    uygulama = QApplication(sys.argv)
    pencere = AnaPencere(node)
    pencere.show()
    try:
        uygulama.exec_()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
