#!/usr/bin/env python3
"""
engel_ayar_paneli.py

engel_node'un ve nesne_node'un (kutulama alanı) ayarlarını değiştirmek için
Türkçe, açıklamalı bir pencere.
rqt_reconfigure yerine kullanılır.

Pencerede:
  - Solda: canlı karar (DUR / DIKKAT / GUVENLI) ve yukarıdan bakış çizimi
    (yelpaze şu an tam olarak nereye bakıyor, kaç derece, kaç metre).
  - Sağda: her ayar için kaydırma çubuğu + ne işe yaradığının açıklaması.

Bu panel LiDAR'a veya sürücüsüne HİÇBİR ayar göndermez; sadece engel_node ve
nesne_node parametrelerini değiştirir. nesne_node kapalıysa onun çubukları kilitli kalır.
"""

import math
import os
import re
import sys
import time

import yaml

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String

from python_qt_binding.QtCore import QPointF, QRectF, Qt, QTimer
from python_qt_binding.QtGui import QBrush, QColor, QFont, QPainter, QPen
from python_qt_binding.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QGroupBox, QHBoxLayout, QInputDialog,
    QLabel, QMessageBox, QPushButton, QScrollArea, QSlider, QVBoxLayout, QWidget,
)

# -----------------------------------------------------------------------------
# Kayıt dosyaları
#   son_ayarlar.yaml : panelde en son UYGULANAN ayarlar. Her değişiklikte otomatik
#                      yazılır; launch dosyası açılışta bunu yükler.
#   profiller/*.yaml : "Kaydet…" ile isim verilerek saklanan ayarlar (ör. masa, arac)
# Dosyalar ROS parametre dosyası biçimindedir, yani
#   ros2 run engel_algilama engel_node --ros-args --params-file DOSYA.yaml
# ile de doğrudan kullanılabilir.
# -----------------------------------------------------------------------------
KAYIT_KLASORU = os.path.expanduser("~/.ros/engel_algilama")
SON_AYARLAR = os.path.join(KAYIT_KLASORU, "son_ayarlar.yaml")
PROFIL_KLASORU = os.path.join(KAYIT_KLASORU, "profiller")

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
    ("4) GÜRÜLTÜ FİLTRESİ", [
        ("min_nokta_sayisi", "En az nokta sayısı", "nokta", 1, 100, 1, 10,
         "Görüş alanında bundan AZ nokta varsa engel yok sayılır. Toz, yağmur damlası, "
         "yansıma gibi tek tük noktalar karar verdirmez. Ayrıca mesafe, en yakın bu "
         "kadarıncı noktaya göre ölçülür. Yanlış DUR alıyorsanız artırın (20-30); "
         "küçük cisimleri kaçırıyorsanız azaltın. Bir insan 3 m'de yüzlerce nokta verir."),
    ]),
]

# nesne_node (kutu içine alma) ayarları. engel_node'un yelpazesinden AYRIDIR.
# Alt/üst değerler nesne_node.cpp'deki aralıklarla AYNI olmalı.
NESNE_GRUPLARI = [
    ("5) KUTULAMA ALANI (nesneleri kutu içine alma)", [
        ("nesne_merkez_aci", "Kutulama yönü", "°", -180.0, 180.0, 1.0, 0.0,
         "Nesnelerin kutu içine alınacağı alanın baktığı yön. Yukarıdaki engel "
         "yelpazesinden bağımsızdır. 0 = ön, +90 = sol, -90 = sağ, 180 = arka. "
         "(Çizimde ve RViz'de mavi çizgi)"),
        ("nesne_gorus_genisligi", "Kutulama genişliği", "°", 10.0, 360.0, 1.0, 120.0,
         "Kutulama alanının toplam açısı. 360 → her yöndeki nesneler kutulanır."),
        ("menzil", "Kutulama menzili", "m", 1.0, 50.0, 0.5, 20.0,
         "Bundan uzaktaki nesneler kutulanmaz. Uzaktaki küçük parçalar ekranı "
         "kalabalıklaştırıyorsa azaltın."),
    ]),
]

# Hangi ayar hangi node'a ait
NODE_GRUPLARI = {"engel_node": GRUPLAR, "nesne_node": NESNE_GRUPLARI}
AYAR_NODE = {t[0]: node_adi for node_adi, gruplar in NODE_GRUPLARI.items()
             for _, liste in gruplar for t in liste}

# Ad → tanım sözlüğü (kolay erişim için)
TANIMLAR = {t[0]: t for gruplar in NODE_GRUPLARI.values() for _, liste in gruplar for t in liste}

# Tam sayı (int) olan parametreler; geri kalanların hepsi ondalıklı (double)
TAM_SAYILAR = {"min_nokta_sayisi"}


def ros_degeri(ad, deger):
    """Parametreyi ROS'un beklediği tipe çevirir (int veya float)."""
    return int(round(deger)) if ad in TAM_SAYILAR else float(deger)


def ayar_dosyasi_yaz(yol, degerler):
    """Ayarları ROS parametre dosyası biçiminde yazar (her node kendi bölümünde)."""
    os.makedirs(os.path.dirname(yol), exist_ok=True)
    icerik = {}
    for ad, v in degerler.items():
        node_adi = AYAR_NODE[ad]
        icerik.setdefault(node_adi, {"ros__parameters": {}})
        icerik[node_adi]["ros__parameters"][ad] = ros_degeri(ad, v)
    gecici = yol + ".tmp"
    with open(gecici, "w", encoding="utf-8") as f:
        f.write("# engel_algilama ayar dosyasi (engel_ayar_paneli tarafindan yazildi)\n")
        yaml.safe_dump(icerik, f, allow_unicode=True, sort_keys=False)
    os.replace(gecici, yol)  # yarım yazılmış dosya kalmasın diye önce geçiciye yaz


def ayar_dosyasi_oku(yol):
    """
    Ayar dosyasını okur ve kontrol eder.
    Başarılıysa (degerler, None), hatalıysa (None, hata_metni) döndürür.
    """
    try:
        with open(yol, encoding="utf-8") as f:
            veri = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as e:
        return None, f"Dosya okunamadı: {e}"

    # ROS biçimi: {engel_node: {ros__parameters: {...}}, nesne_node: {...}} veya
    # {/**: {...}}; düz sözlük de olur. İki node'un ayar adları çakışmadığı için
    # bölümleri tek sözlükte birleştirebiliriz.
    if not isinstance(veri, dict):
        return None, "Dosya biçimi tanınmadı."
    birlesik, bolum_bulundu = {}, False
    for anahtar in ("/**", "engel_node", "/engel_node", "nesne_node", "/nesne_node"):
        bolum = veri.get(anahtar)
        if isinstance(bolum, dict):
            bolum = bolum.get("ros__parameters", bolum)
            if isinstance(bolum, dict):
                birlesik.update(bolum)
                bolum_bulundu = True
    if bolum_bulundu:
        veri = birlesik

    degerler = {}
    for ad, tanim in TANIMLAR.items():
        if ad not in veri:
            continue  # dosyada olmayan ayar mevcut değerinde kalır
        try:
            v = float(veri[ad])
        except (TypeError, ValueError):
            return None, f"'{ad}' sayı değil: {veri[ad]!r}"
        alt, ust = tanim[3], tanim[4]
        if not alt <= v <= ust:
            return None, f"'{ad}' = {v:g} izin verilen aralığın ({alt:g} .. {ust:g}) dışında."
        degerler[ad] = v
    if not degerler:
        return None, "Dosyada engel_node / nesne_node ayarı bulunamadı."
    return degerler, None

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

        bosluk = " " if len(self.birim) > 1 else ""  # "1 nokta" ama "10°", "3m"
        alt_yazi = QLabel(f"{self.alt:g}{bosluk}{self.birim}")
        ust_yazi = QLabel(f"{self.ust:g}{bosluk}{self.birim}")
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

        # Kutulama alanı (nesne_node): sadece YÖNÜ gösterilir, dış çembere kadar
        # mavi kesik çizgi. (Menzili genelde dikkat mesafesinden çok büyük, ölçeğe sığmaz.)
        n_merkez, n_genislik = a["nesne_merkez_aci"], a["nesne_gorus_genisligi"]
        p.setPen(QPen(QColor(100, 190, 255), 2, Qt.DashLine))
        p.setBrush(Qt.NoBrush)
        dis = QRectF(cx - R - 6, cy - R - 6, 2 * R + 12, 2 * R + 12)
        if n_genislik >= 360.0:
            p.drawEllipse(dis)
        else:
            p.drawPie(dis, int((90.0 + n_merkez - n_genislik / 2.0) * 16), int(n_genislik * 16))

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
# Bir node'a parametre bağlantısı (okuma / gönderme durumu)
# -----------------------------------------------------------------------------
class Baglanti:
    def __init__(self, node, hedef, adlar):
        self.istemci = AsyncParameterClient(node, hedef)
        self.hedef = hedef
        self.adlar = list(adlar)
        self.yuklendi = False      # node'dan ilk değerler okundu mu?
        self.okuma_istegi = None
        self.gonderme_istegi = None
        self.son_gonderilen = {}


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
        nesne_hedef = node.declare_parameter("nesne_node", "nesne_node").value
        self.hedef = hedef
        self.baglantilar = {
            "engel_node": Baglanti(node, hedef, [t[0] for _, l in GRUPLAR for t in l]),
            "nesne_node": Baglanti(node, nesne_hedef,
                                   [t[0] for _, l in NESNE_GRUPLARI for t in l]),
        }

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
        for grup_adi, liste in GRUPLAR + NESNE_GRUPLARI:
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

        # ---------------- Kayıtlı ayarlar bölümü ----------------
        kayit_kutusu = QGroupBox("KAYITLI AYARLAR")
        kayit_kutusu.setStyleSheet("QGroupBox { font-weight: bold; }")
        kayit_duzen = QVBoxLayout(kayit_kutusu)

        bilgi = QLabel("✓ Uyguladığınız ayarlar otomatik kaydedilir ve bir sonraki "
                       "açılışta aynen yüklenir. Farklı ortamlar (masa, araç…) için "
                       "isimli kayıt oluşturabilirsiniz.")
        bilgi.setWordWrap(True)
        bilgi.setStyleSheet("color: #555; font-weight: normal; font-size: 9pt;")

        self.profil_listesi = QComboBox()
        self.profil_listesi.setMinimumWidth(160)
        yukle = QPushButton("Yükle")
        yukle.setToolTip("Seçili kayıttaki ayarları uygula")
        yukle.clicked.connect(self._profil_yukle)
        sil = QPushButton("Sil")
        sil.setToolTip("Seçili kaydı sil")
        sil.clicked.connect(self._profil_sil)

        satir1 = QHBoxLayout()
        satir1.addWidget(QLabel("Kayıt:"))
        satir1.addWidget(self.profil_listesi, 1)
        satir1.addWidget(yukle)
        satir1.addWidget(sil)

        kaydet = QPushButton("💾 Kaydet…")
        kaydet.setToolTip("Şu anki ayarları isim vererek kaydet")
        kaydet.clicked.connect(self._profil_kaydet)
        disa = QPushButton("Dışa aktar…")
        disa.setToolTip("Ayarları istediğiniz bir yere .yaml dosyası olarak kaydet "
                        "(başka bilgisayara taşımak için)")
        disa.clicked.connect(self._disa_aktar)
        ice = QPushButton("İçe aktar…")
        ice.setToolTip("Bir .yaml ayar dosyasını aç ve uygula")
        ice.clicked.connect(self._ice_aktar)
        varsayilan = QPushButton("Varsayılana dön")
        varsayilan.clicked.connect(self._varsayilana_don)

        satir2 = QHBoxLayout()
        for d in (kaydet, disa, ice, varsayilan):
            satir2.addWidget(d)

        kayit_duzen.addWidget(bilgi)
        kayit_duzen.addLayout(satir1)
        kayit_duzen.addLayout(satir2)
        self._profilleri_listele()

        self.durum = QLabel()
        self.durum.setWordWrap(True)
        self.durum.setMinimumHeight(36)

        sag_sutun = QVBoxLayout()
        sag_sutun.addWidget(kayit_kutusu)
        sag_sutun.addWidget(kaydirma, 1)
        sag_sutun.addWidget(self.durum)

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

    def _baglanti(self, ad):
        return self.baglantilar[AYAR_NODE[ad]]

    @property
    def yuklendi(self):
        """En az bir node'a bağlanıldı mı?"""
        return any(b.yuklendi for b in self.baglantilar.values())

    def _yuklu_degerler(self):
        """Sadece bağlanılmış node'ların değerleri (bağlanılmayanın çubukları gerçek değil)."""
        return {ad: s.deger_al() for ad, s in self.satirlar.items()
                if self._baglanti(ad).yuklendi}

    def _otomatik_kaydet(self):
        """son_ayarlar.yaml: bağlanılmamış node'un önceki kayıtlı değerleri korunur."""
        eski, _ = ayar_dosyasi_oku(SON_AYARLAR) if os.path.exists(SON_AYARLAR) else ({}, None)
        degerler = dict(eski or {})
        degerler.update(self._yuklu_degerler())
        ayar_dosyasi_yaz(SON_AYARLAR, degerler)

    def _durum_yaz(self, metin, renk="black"):
        self.durum.setText(metin)
        self.durum.setStyleSheet(f"color: {renk};")

    def _cizimi_guncelle(self):
        # Bağlanılmamış node'un çubukları yerine varsayılanları çiz
        d = {ad: (s.deger_al() if self._baglanti(ad).yuklendi else TANIMLAR[ad][6])
             for ad, s in self.satirlar.items()}
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
        ng = d["nesne_gorus_genisligi"]
        if ng >= 360:
            kutulama = "her yön (360°)"
        else:
            kutulama = (f"{aciyi_anlat(d['nesne_merkez_aci'] - ng / 2)} ile "
                        f"{aciyi_anlat(d['nesne_merkez_aci'] + ng / 2)} arası ({ng:.0f}°)")
        durum = "" if self.baglantilar["nesne_node"].yuklendi else " <i>(nesne_node bekleniyor)</i>"
        bolge += (f"<br><span style='color:#3a8de0'>Kutulama alanı: <b>{kutulama}</b>, "
                  f"<b>{d['menzil']:.1f} m</b>'ye kadar</span>{durum}")
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

        for b in self.baglantilar.values():
            # 1) İlk açılışta node'un mevcut değerlerini oku
            if not b.yuklendi and b.okuma_istegi is None and b.istemci.services_are_ready():
                b.okuma_istegi = b.istemci.get_parameters(b.adlar)
            if b.okuma_istegi is not None and b.okuma_istegi.done():
                sonuc = b.okuma_istegi.result()
                b.okuma_istegi = None
                if sonuc is not None and len(sonuc.values) == len(b.adlar):
                    for ad, deger in zip(b.adlar, sonuc.values):
                        self.satirlar[ad].deger_ayarla(float(parameter_value_to_python(deger)))
                        self.satirlar[ad].setEnabled(True)
                    b.son_gonderilen = {ad: self.satirlar[ad].deger_al() for ad in b.adlar}
                    ilk = not b.yuklendi
                    b.yuklendi = True
                    if ilk:
                        self._durum_yaz(f"'{b.hedef}' bağlandı. Çubukları kaydırın, "
                                        "değişiklik anında uygulanır.", "green")
                    self._cizimi_guncelle()

            # 2) Gönderilen ayarın sonucu geldi mi?
            if b.gonderme_istegi is not None and b.gonderme_istegi.done():
                sonuc = b.gonderme_istegi.result()
                b.gonderme_istegi = None
                hatalar = [r.reason for r in (sonuc.results if sonuc else []) if not r.successful]
                if hatalar:
                    self._durum_yaz(f"Reddedildi ({b.hedef}): " + "; ".join(hatalar), "#d22")
                    # Node'daki gerçek değerleri tekrar okuyup çubukları onlara eşitle
                    b.okuma_istegi = b.istemci.get_parameters(b.adlar)
                else:
                    # Başarılı: bir sonraki açılış için otomatik kaydet
                    try:
                        self._otomatik_kaydet()
                        self._durum_yaz("Uygulandı ✓  (otomatik kaydedildi)", "green")
                    except OSError as e:
                        self._durum_yaz(f"Uygulandı ✓ ama otomatik kayıt başarısız: {e}", "#c60")

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
        d = self._degerler()
        gonderilen_var = False
        for b in self.baglantilar.values():
            if not b.yuklendi:
                continue
            if b.gonderme_istegi is not None:
                self.gonder_zamanlayici.start()  # önceki istek bitmedi, biraz sonra tekrar dene
                continue
            degisenler = []
            for ad in b.adlar:
                if b.son_gonderilen.get(ad) == d[ad]:
                    continue
                tip = Parameter.Type.INTEGER if ad in TAM_SAYILAR else Parameter.Type.DOUBLE
                degisenler.append(Parameter(ad, tip, ros_degeri(ad, d[ad])))
            if degisenler:
                b.son_gonderilen = {ad: d[ad] for ad in b.adlar}
                b.gonderme_istegi = b.istemci.set_parameters(degisenler)
                gonderilen_var = True
        if not gonderilen_var and not self.gonder_zamanlayici.isActive():
            self._durum_yaz("Ayarlar zaten bu değerlerde ✓", "green")

    # ---------------- Kayıtlı ayarlar ----------------
    def _degerleri_uygula(self, degerler, kaynak):
        """Dosyadan okunan değerleri çubuklara koyar ve node'a gönderir."""
        if not self.yuklendi:
            self._durum_yaz("Önce engel_node / nesne_node'a bağlanılmalı (node açık mı?).", "#d22")
            return
        for ad, v in degerler.items():
            self.satirlar[ad].deger_ayarla(v)
        self._ayar_degisti()
        if self.gonder_zamanlayici.isActive():  # geçerliyse gönderim başladı
            self._durum_yaz(f"'{kaynak}' yüklendi, uygulanıyor…", "green")

    def _profilleri_listele(self, secili=None):
        self.profil_listesi.clear()
        if os.path.isdir(PROFIL_KLASORU):
            adlar = sorted(f[:-5] for f in os.listdir(PROFIL_KLASORU) if f.endswith(".yaml"))
            self.profil_listesi.addItems(adlar)
        if self.profil_listesi.count() == 0:
            self.profil_listesi.setPlaceholderText("(henüz kayıt yok)")
        if secili:
            self.profil_listesi.setCurrentText(secili)

    def _profil_yolu(self, ad):
        return os.path.join(PROFIL_KLASORU, ad + ".yaml")

    def _profil_kaydet(self):
        ad, tamam = QInputDialog.getText(
            self, "Ayarları kaydet",
            "Kayıt adı (ör. masa, arac, otopark):", text=self.profil_listesi.currentText())
        if not tamam:
            return
        # Dosya adında sorun çıkarabilecek karakterleri temizle (Türkçe harfler kalır)
        ad = re.sub(r"[^\w\-]+", "_", ad.strip()).strip("_")
        if not ad:
            self._durum_yaz("Geçerli bir isim girin.", "#d22")
            return
        yol = self._profil_yolu(ad)
        if os.path.exists(yol) and QMessageBox.question(
                self, "Üzerine yaz?", f"'{ad}' adında bir kayıt zaten var. Üzerine yazılsın mı?"
        ) != QMessageBox.Yes:
            return
        try:
            ayar_dosyasi_yaz(yol, self._yuklu_degerler())
        except OSError as e:
            self._durum_yaz(f"Kaydedilemedi: {e}", "#d22")
            return
        self._profilleri_listele(secili=ad)
        self._durum_yaz(f"'{ad}' kaydedildi ✓", "green")

    def _profil_yukle(self):
        ad = self.profil_listesi.currentText()
        if not ad:
            self._durum_yaz("Yüklenecek kayıt yok. Önce 'Kaydet…' ile bir kayıt oluşturun.", "#d22")
            return
        degerler, hata = ayar_dosyasi_oku(self._profil_yolu(ad))
        if hata:
            self._durum_yaz(f"'{ad}' yüklenemedi: {hata}", "#d22")
            return
        self._degerleri_uygula(degerler, ad)

    def _profil_sil(self):
        ad = self.profil_listesi.currentText()
        if not ad:
            return
        if QMessageBox.question(self, "Sil?", f"'{ad}' kaydı silinsin mi?") != QMessageBox.Yes:
            return
        try:
            os.remove(self._profil_yolu(ad))
        except OSError as e:
            self._durum_yaz(f"Silinemedi: {e}", "#d22")
            return
        self._profilleri_listele()
        self._durum_yaz(f"'{ad}' silindi.", "gray")

    def _disa_aktar(self):
        yol, _ = QFileDialog.getSaveFileName(
            self, "Ayarları dışa aktar", os.path.expanduser("~/engel_ayarlari.yaml"),
            "YAML ayar dosyası (*.yaml *.yml)")
        if not yol:
            return
        if not yol.endswith((".yaml", ".yml")):
            yol += ".yaml"
        try:
            ayar_dosyasi_yaz(yol, self._yuklu_degerler())
        except OSError as e:
            self._durum_yaz(f"Dışa aktarılamadı: {e}", "#d22")
            return
        self._durum_yaz(f"Dışa aktarıldı: {yol}", "green")

    def _ice_aktar(self):
        yol, _ = QFileDialog.getOpenFileName(
            self, "Ayar dosyası aç", os.path.expanduser("~"),
            "YAML ayar dosyası (*.yaml *.yml)")
        if not yol:
            return
        degerler, hata = ayar_dosyasi_oku(yol)
        if hata:
            self._durum_yaz(f"İçe aktarılamadı: {hata}", "#d22")
            return
        self._degerleri_uygula(degerler, os.path.basename(yol))

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
