#                                                                  /BİSMİLLAH/

import sys
from folium import plugins
import folium 
from PIL import Image
from PyQt5.QtGui import QPixmap,QColor
import os
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel,QMessageBox, QTabWidget, QPushButton, QFrame, QVBoxLayout, QLineEdit, QHBoxLayout,QTextEdit
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl,QTimer,QCoreApplication, pyqtSignal, QObject, QThread
from dronekit import connect,VehicleMode,Command
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink
import threading
import math

#com9 kart
#'udp:127.0.0.1:14550' ip gazebo cart curt
#-35.36321719 149.16516857 avusturalya cart curt
ipdrone = 'udp:127.0.0.1:14550'
ipkmkz ='com9'


class YerKontrolIstasyonu(QMainWindow):
    gui_guncelle = pyqtSignal(str)
    def __init__(self):
        global ipdrone
        global ipkmkz
        global kmkzip

        super().__init__()

        self.gui_guncelle.connect(self.guiye_yaz)

        self.timergmg = QTimer()
        self.timergmg.timeout.connect(self.guncelle_mod)
        self.timergmg.start(1500)  # 1 saniyede bir günceller 

        self.timergmk = QTimer()
        self.timergmk.timeout.connect(self.guncelle_modkmkz)
        self.timergmk.start(1500)  # 1 saniyede bir günceller 
        
        self.timeraarmkmkz = QTimer()
        self.timeraarmkmkz.timeout.connect(self.goster_Aarmkmkzp_durumu)
        self.timeraarmkmkz.start(1500)  # 1 saniyede bir günceller 


        self.timeraarm = QTimer()
        self.timeraarm.timeout.connect(self.goster_Aarm_durumu)
        self.timeraarm.start(1500)  # 1 saniyede bir günceller 

        self.timerpkmk = QTimer()
        self.timerpkmk.timeout.connect(self.guncelle_pilkmkz)
        self.timerpkmk.start(2000)  # 1 saniyede bir günceller


        self.timerpgzlm = QTimer()
        self.timerpgzlm.timeout.connect(self.guncelle_pilgzlm)
        self.timerpgzlm.start(2000)


        self.setWindowTitle("Yer Kontrol İstasyonu")
        self.setGeometry(100, 100, 1200, 800)
        # Ekran çözünürlüğünü kontrol et
        screen = QApplication.primaryScreen()
        size = screen.size()
        width = size.width()
        height = size.height()

        if width < 1920 or height < 1080:
            QMessageBox.critical(self, "HATA", "Ekran çözünürlüğü en az 1920x1080 olmalıdır.")
            sys.exit()

        # Pencere sabit 1920x1080 olsun
        self.setFixedSize(1920, 1080)
        self.setWindowTitle("Yer Kontrol İstasyonu")
        self.setGeometry(0, 0, 1920, 1080)  # Fullscreen değil ama tam çözünürlük

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # Görev kontrol sekmesi
        self.gorev_kontrol_tab = QWidget()
        self.tabs.addTab(self.gorev_kontrol_tab, "Görev Kontrol")

        # görev kontrol Terminal Paneli
        self.terminal_g_paneli = QWidget(self.gorev_kontrol_tab)  # Burada ana pencereye ekleniyor
        self.terminal_g_paneli.setGeometry(1070, 690, 1500, 4800)
        self.terminal_g_paneli.setStyleSheet("background-color: rgb(38, 118, 118); border: 2px solid black;")

        #terminal başlık
        self.terminal_g_baslik = QLabel("       Terminal", self.terminal_g_paneli)
        self.terminal_g_baslik.setStyleSheet("font-weight: 400; font-size: 20px;")
        self.terminal_g_baslik.setGeometry(10,10,190,30)  

        # Terminal Çıktısı (QTextEdit)
        self.terminal_g_output = QTextEdit(self.terminal_g_paneli)
        self.terminal_g_output.setGeometry(10, 50, 880, 400)
        self.terminal_g_output.setReadOnly(True)  
        self.terminal_g_paneli.setStyleSheet("background-color: rgb(38, 118, 118) ")

       #bağlan butonu
        self.button_baglan = QPushButton("BAĞLAN", self.gorev_kontrol_tab)
        self.button_baglan.setFixedSize(250, 100)
        self.button_baglan.move(1660, 12)
        self.button_baglan.setStyleSheet("""
            background-color: rgb(115, 240, 15); 
            border: 2px solid black;          
            border-radius: 20px;                 
            font-size: 12px;
            padding: 10px;                 
        """)
        self.button_baglan.clicked.connect(self.on_button_baglan_clicked)

        # İç sekmeler
        self.gorev_kontrol_alt_tabs = QTabWidget(self.gorev_kontrol_tab)
        self.gorev_kontrol_alt_tabs.setGeometry(1070, 90, 850, 600)

        # Gözlemci ve Kamikaze sekmeleri
        self.gozlemci_paneli_tab = QWidget()
        self.kamikaze_tab = QWidget()

        # Alt sekmeleri ekleyelim
        self.gorev_kontrol_alt_tabs.addTab(self.gozlemci_paneli_tab, "Gözlemci")
        self.gorev_kontrol_alt_tabs.addTab(self.kamikaze_tab, "Kamikaze")
        self.gorev_kontrol_tab.setStyleSheet("background-color: rgb(60, 60, 60); border: 2px solid black;")        

        # Gözlemci Sekmesinin İçeriği
        self.Gozlemci_paneli_tab = QFrame(self.gozlemci_paneli_tab)  # Bu satırı ekleyelim
        self.Gozlemci_paneli_tab.setGeometry(00, 00, 980, 600)
        self.Gozlemci_paneli_tab.setStyleSheet("background-color: rgb(108, 108, 108); border: 2px solid black;")

        # Butonlar kontrl gözlemci
        self.button_Gozlemci_RTLgzlm = QPushButton("RTL", self.Gozlemci_paneli_tab)
        self.button_Gozlemci_RTLgzlm.setFixedSize(200, 100)
        self.button_Gozlemci_RTLgzlm.move(70, 100)
        self.button_Gozlemci_RTLgzlm.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999);   
            border: 2px solid black;          
            border-radius: 20px;                
            font-size: 15px;                       
            padding: 10px;                       
        """)

        self.button_Gozlemci_kesfebaslagzlm = QPushButton("Kesfe basla", self.Gozlemci_paneli_tab)
        self.button_Gozlemci_kesfebaslagzlm.setFixedSize(200, 100)
        self.button_Gozlemci_kesfebaslagzlm.move(70, 250)
        self.button_Gozlemci_kesfebaslagzlm.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999);  
            border: 2px solid black;           
            border-radius: 20px;                   
            font-size: 15px;                       
            padding: 10px;                       
        """)

        self.button_Gozlemci_kesfebaslagzlm.clicked.connect(self.on_button_Gozlemci_kesfebaslagzlm_clicked)


        self.button_Gozlemci_RTLgzlm.clicked.connect(self.on_button_Gozlemci_RTLgzlm_clicked)


        self.button_Gozlemci_kesfebaslagzlm.clicked.connect(self.on_button_Gozlemci_kesfebaslagzlm_clicked)

        #kamikaze sekmesinin içeriği
        self.Kamikaze_paneli_tab = QFrame(self.kamikaze_tab)
        self.Kamikaze_paneli_tab.setGeometry(00, 00, 900, 600)
        self.Kamikaze_paneli_tab.setStyleSheet("background-color: rgb(108, 108, 108); border: 2px solid black;")

        # Butonlar kontrl kamikaze
        """self.button_Kamikaze_kesfebaslakmkz = QPushButton("Kamikaze Başlat", self.Kamikaze_paneli_tab)
        self.button_Kamikaze_kesfebaslakmkz.setFixedSize(200, 100)
        self.button_Kamikaze_kesfebaslakmkz.move(70, 100)
        self.button_Kamikaze_kesfebaslakmkz.setStyleSheet("""
        """ background-color: rgb(73, 132, 160,0.999999); 
            border: 2px solid black;               
            border-radius: 20px;                  
            font-size: 15px;                       
            padding: 10px;                      
        """""")"""
        """self.button_Kamikaze_kesfebaslakmkz.clicked.connect(self.on_button_Kamikaze_kesfebaslakmkz_clicked) """


        self.button_Kamikaze_RTLkmkz = QPushButton("RTL",self.Kamikaze_paneli_tab)
        self.button_Kamikaze_RTLkmkz.setFixedSize(200, 100)
        self.button_Kamikaze_RTLkmkz.move(70, 250)
        self.button_Kamikaze_RTLkmkz.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999); 
            border: 2px solid black;              
            border-radius: 20px;                  
            font-size: 15px;                   
            padding: 10px;                        
        """)
        
        #kmkz butonlara tıklama
        self.button_Kamikaze_RTLkmkz.clicked.connect(self.on_button_Kamikaze_RTLkmkz_clicked)

        # Koordinat Girişi
        self.dusman_coord_input = QLineEdit(self.gorev_kontrol_tab)
        self.dusman_coord_input.setPlaceholderText("Düşman konumunu girin ")
        self.dusman_coord_input.setFixedSize(300, 50)
        self.dusman_coord_input.move(610, 813)  # Koordinat input'unun konumunu ayarlayalım
        self.dusman_coord_input.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999);   
            border: 2px solid black;             
            border-radius: 20px;              
            font-size: 15px;           
            padding: 10px;                         
        """)
        
        # Düşmanı Haritada Göster Butonu
        self.button_dusman_goster = QPushButton("Düşman koordinatlarını Haritada Göster", self.gorev_kontrol_tab)
        self.button_dusman_goster.setFixedSize(300, 90)
        self.button_dusman_goster.move(610, 868)  # Daha uygun bir konum
        self.button_dusman_goster.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999); 
            border: 2px solid black;               
            border-radius: 20px;                  
            font-size: 15px;                   
            padding: 10px;                       
        """)

        # Buton tıklama olayını bağlayalım
        self.button_dusman_goster.clicked.connect(self.on_button_dusman_goster_clicked)

        # Kontrol kısmı için layout
        self.control_layout = QVBoxLayout()
        self.control_layout.setContentsMargins(10, 20, 10, 20)

        # Koordinat Başlıkları
        self.coord_label = QLabel("Koordinatları Girin:",self.gorev_kontrol_tab)
        self.coord_label.setStyleSheet("font-weight: bold; font-size: 16px;")
        self.coord_label.move(20, 790) 
        self.coord_label.setStyleSheet("background-color: rgb(73, 132, 160,0.999999); border: 2px solid black;")
        
        # 1. Nokta Koordinatları Input
        self.coord_1_input = QLineEdit(self.gorev_kontrol_tab)
        self.coord_1_input.setPlaceholderText("İknci konumu girin")
        self.coord_1_input.setFixedSize(300,60)
        self.coord_1_input.move(20, 825)  

        self.coord_1_input.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999); 
            border: 2px solid black;               
            border-radius: 20px;                   
         font-size: 15px;                  
         padding: 10px;                         
        """)

        # 2. Nokta Koordinatları Input
        self.coord_2_input = QLineEdit(self.gorev_kontrol_tab)
        self.coord_2_input.setPlaceholderText("Birinci konumu girin")
        self.coord_2_input.setFixedSize(300,60)
        self.coord_2_input.move(20, 895)  
        self.coord_2_input.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999);   
            border: 2px solid black;           
             border-radius: 20px;                   
            font-size: 15px;                       
            padding: 10px;                         
        """)

        # "Kare Çiz" butonunu ekle
        self.button_ciz = QPushButton("Taranacak alanı işaretle", self.gorev_kontrol_tab)
        self.button_ciz.clicked.connect(self.ciz_kare)
        self.button_ciz.setFixedSize(250,130)
        self.button_ciz.move(350, 825)  
        self.button_ciz.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999);   
            border: 2px solid black;               
            border-radius: 20px;                   
            font-size: 15px;                       
            padding: 10px;                         
        """)

        # Gözlem Sekmesi
        self.gozlem_tab = QWidget()
        self.tabs.addTab(self.gozlem_tab, "Gözlem")

        # gözlemTerminal Paneli
        self.terminal_paneli = QWidget(self.gozlem_tab)  # Burada ana pencereye ekleniyor
        self.terminal_paneli.setGeometry(1110, 530, 1500, 4800)
        self.terminal_paneli.setStyleSheet("background-color: rgb(38, 118, 118); border: 2px solid black;")

        #terminal başlık
        self.terminal_baslik = QLabel("       Terminal", self.terminal_paneli)
        self.terminal_baslik.setStyleSheet("font-weight: 400; font-size: 20px;")
        self.terminal_baslik.setGeometry(10,10,190,30)  

        # Terminal Çıktısı (QTextEdit)
        self.terminal_output = QTextEdit(self.terminal_paneli)
        self.terminal_output.setGeometry(10, 50, 880, 400)
        self.terminal_output.setReadOnly(True)  
        self.terminal_paneli.setStyleSheet("background-color: rgb(38, 118, 118) ")
        
        # Gözlem sekmesinde alt sekmeler
        self.gorev_verme_g_alt_tabs = QTabWidget(self.gozlem_tab)
        self.gorev_verme_g_alt_tabs.setGeometry(1110, 20, 7800, 500)
        self.gozlem_tab.setStyleSheet("background-color: rgb(68, 69, 69); border: 2px solid black;")

        # Gözlemci ve Kamikaze sekmeleri
        self.gozlemci_paneli_g_tab = QWidget()  
        self.kamikaze_g_tab = QWidget()

        # Alt sekmeleri ekleyelim
        self.gorev_verme_g_alt_tabs.addTab(self.gozlemci_paneli_g_tab,"Gözlemci")
        self.gorev_verme_g_alt_tabs.addTab(self.kamikaze_g_tab,"Kamikaze")

        self.kamikaze_g_tab.setStyleSheet("background-color: rgb(100, 150, 120); border: 2px solid black;")
        self.gozlemci_paneli_g_tab.setStyleSheet("background-color: rgb(70, 87, 101); border: 2px solid black;")

        # Gözlemci Sekmesinin İçeriği
        self.Gozlemci_paneli_g_tab = QFrame(self.gozlemci_paneli_g_tab)  
        self.Gozlemci_paneli_g_tab.setGeometry(00, 00, 800, 700)

        self.label_mode = QLabel("    Mode:", self.gozlemci_paneli_g_tab)
        self.label_mode.setFixedSize(200, 100)
        self.label_mode.move(300, 300)
        self.label_mode.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999);   /* Arka plan rengi */
            border: 2px solid black;               /* Kenarlık */
            border-radius: 20px;                   /* Köşe yuvarlatma */
            font-size: 15px;                       /* Yazı tipi boyutu */
            padding: 10px;                         /* Buton içi boşluk */
        """)
        self.timer_mode = QTimer()
        self.timer_mode.timeout.connect(self.guncelle_mod)
        self.timer_mode.start(1000)

        # pil gözlemci gözlem
        self.label_battery = QLabel("Pil: --%", self.gozlemci_paneli_g_tab)
        self.label_battery.setFixedSize(200, 100)
        self.label_battery.move(300, 100)
        self.label_battery.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999);   /* Arka plan rengi */
            border: 2px solid black;               /* Kenarlık */
            border-radius: 20px;                   /* Köşe yuvarlatma */
            font-size: 15px;                       /* Yazı tipi boyutu */
            padding: 10px;                         /* Buton içi boşluk */
        """)
        self.timer_battery = QTimer()
        self.timer_battery.timeout.connect(self.guncelle_pilgzlm)
        self.timer_battery.start(1000)  
        
        # yükseklik gözlemci gözlem
        self.label_yukseklik = QLabel("Yükseklik: 0.00 m", self.gozlemci_paneli_g_tab)
        self.label_yukseklik.setFixedSize(200, 100)
        self.label_yukseklik.move(550, 100)
        self.label_yukseklik.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999);
            border: 2px solid black;
            border-radius: 20px;
            font-size: 15px;
            padding: 10px;
        """)
        self.timer_yukseklik = QTimer()
        self.timer_yukseklik.timeout.connect(self.guncelle_yukseklik)
        self.timer_yukseklik.start(1000)  

   

        # Arm durumu butonunu oluştur
        self.button_Gozlemci_paneli_armdurumugzlm_g_tab = QPushButton("Arm Durumunu Değiştir", self.gozlemci_paneli_g_tab)
        self.button_Gozlemci_paneli_armdurumugzlm_g_tab.setFixedSize(200, 100)
        self.button_Gozlemci_paneli_armdurumugzlm_g_tab.move(50, 100)
        self.button_Gozlemci_paneli_armdurumugzlm_g_tab.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999);   /* Arka plan rengi */
            border: 2px solid black;               /* Kenarlık */
            border-radius: 20px;                   /* Köşe yuvarlatma */
            font-size: 15px;                       /* Yazı tipi boyutu */
            padding: 10px;                         /* Buton içi boşluk */
            """)

        
        self.button_Gozlemci_paneli_armdurumugzlm_g_tab.clicked.connect(self.arm_veya_disarm)

        self.button_Gozlemci_paneli_Aarmdurumugzlm_g_tab = QPushButton("Anlık arm \nDurumu: Bilinmiyor", self.gozlemci_paneli_g_tab)
        self.button_Gozlemci_paneli_Aarmdurumugzlm_g_tab.setFixedSize(200, 100)
        self.button_Gozlemci_paneli_Aarmdurumugzlm_g_tab.move(50, 300)
        self.button_Gozlemci_paneli_Aarmdurumugzlm_g_tab.setStyleSheet("""
            background-color: rgb(73, 132, 160,0.999999);   /* Arka plan rengi */
            border: 2px solid black;               /* Kenarlık */
            border-radius: 20px;                   /* Köşe yuvarlatma */
            font-size: 13px;                       /* Yazı tipi boyutu */
            padding: 10px;                         /* Buton içi boşluk */
        """)

        self.timer_arm = QTimer()
        self.timer_arm.timeout.connect(self.goster_Aarm_durumu)
        self.timer_arm.start(1000)  

        # Butonlar gözlem kamikaze

        #kamikaze Sekmesinin İçeriği
        
        self.Kamikaze_gkmkz_tab = QFrame(self.kamikaze_g_tab)
        self.Kamikaze_gkmkz_tab.setGeometry(00, 00, 900, 500)

        self.button_Gozlemci_paneli_mode_gkmkz_tab = QLabel("Mode",self.Kamikaze_gkmkz_tab)
        self.button_Gozlemci_paneli_mode_gkmkz_tab.setFixedSize(200, 100)
        self.button_Gozlemci_paneli_mode_gkmkz_tab.move(300, 100)
        self.button_Gozlemci_paneli_mode_gkmkz_tab.setStyleSheet("""
            background-color: rgb(73, 160, 120);   /* Arka plan rengi */
            border: 2px solid black;               /* Kenarlık */
            border-radius: 20px;                   /* Köşe yuvarlatma */
            font-size: 15px;                       /* Yazı tipi boyutu */
            padding: 10px;                         /* Buton içi boşluk */
        """)
        self.timer_modekmkz = QTimer()
        self.timer_modekmkz.timeout.connect(self.guncelle_modkmkz)
        self.timer_modekmkz.start(1000)

   # Arm durumu butonunu oluştur
        self.button_Gozlemci_paneli_armdurumugzlm_gkmkz_tab = QPushButton("Arm Durumu değiştir", self.Kamikaze_gkmkz_tab)
        self.button_Gozlemci_paneli_armdurumugzlm_gkmkz_tab.setFixedSize(200, 100)
        self.button_Gozlemci_paneli_armdurumugzlm_gkmkz_tab.move(50, 300)
        self.button_Gozlemci_paneli_armdurumugzlm_gkmkz_tab.setStyleSheet("""
            background-color: rgb(73, 160, 120);   /* Arka plan rengi */
            border: 2px solid black;               /* Kenarlık */
            border-radius: 20px;                   /* Köşe yuvarlatma */
            font-size: 15px;                       /* Yazı tipi boyutu */
            padding: 10px;                         /* Buton içi boşluk */
            """)  
        self.button_Gozlemci_paneli_armdurumugzlm_gkmkz_tab.clicked.connect(self.arm_durumunukmkz_degistir)

        self.button_Gozlemci_paneli_aarmdurumu_gkmkz_tab = QPushButton("Anlık Arm \nDurumu:",self.Kamikaze_gkmkz_tab)
        self.button_Gozlemci_paneli_aarmdurumu_gkmkz_tab.setFixedSize(200, 100)
        self.button_Gozlemci_paneli_aarmdurumu_gkmkz_tab.move(550, 300)
        self.button_Gozlemci_paneli_aarmdurumu_gkmkz_tab.setStyleSheet("""
            background-color: rgb(73, 160, 120);   /* Arka plan rengi */
            border: 2px solid black;               /* Kenarlık */
            border-radius: 20px;                   /* Köşe yuvarlatma */
            font-size: 15px;                       /* Yazı tipi boyutu */
            padding: 10px;                         /* Buton içi boşluk */
        """)
        self.timer_armkmkz = QTimer()
        self.timer_armkmkz.timeout.connect(self.goster_Aarmkmkzp_durumudis)
        self.timer_armkmkz.start(1000)
        
        
        #kamikaze gözlem pil
        self.button_Gozlemci_paneli_pilkmkz_gkmkz_tab = QLabel("Pil", self.Kamikaze_gkmkz_tab)
        self.button_Gozlemci_paneli_pilkmkz_gkmkz_tab.setFixedSize(200, 100)
        self.button_Gozlemci_paneli_pilkmkz_gkmkz_tab.move(300, 300)
        self.button_Gozlemci_paneli_pilkmkz_gkmkz_tab.setStyleSheet("""
            background-color: rgba(73, 160, 120);  /* Saydamlık düzeltildi */
            border: 2px solid black;
            border-radius: 20px;
            font-size: 15px;
            padding: 10px;
        """)

        self.timerkmkz_pil = QTimer()
        self.timerkmkz_pil.timeout.connect(self.guncelle_pilkmkz)
        self.timerkmkz_pil.start(1000)

        try:
            self.kmkz = connect(ipkmkz, wait_ready=True)
        except Exception as e:
            print(f"Bağlantı kurulamadı: {e}")
            self.kmkz = None

        self.labelkmkz_yukseklik = QLabel("Yükseklik: 0.00 m", self.Kamikaze_gkmkz_tab)
        self.labelkmkz_yukseklik.setFixedSize(200, 100)
        self.labelkmkz_yukseklik.move(50, 100)
        self.labelkmkz_yukseklik.setStyleSheet("""
            background-color: rgb(73, 160, 120);
            border: 2px solid black;
            border-radius: 20px;
            font-size: 15px;
            padding: 10px;
        """)
        self.timerkmkz_yukseklik = QTimer()
        self.timerkmkz_yukseklik.timeout.connect(self.guncelle_yukseklikkmkz)
        self.timerkmkz_yukseklik.start(1000)

        #kamikaze gözlem aktarılan konum
        self.button_Gozlemci_paneli_aktarilankonumkkmkz_gkmkz_tab = QLabel("Aktarılan konum",self.Kamikaze_gkmkz_tab)
        self.button_Gozlemci_paneli_aktarilankonumkkmkz_gkmkz_tab.setFixedSize(200, 100)
        self.button_Gozlemci_paneli_aktarilankonumkkmkz_gkmkz_tab.move(550, 100)
        self.button_Gozlemci_paneli_aktarilankonumkkmkz_gkmkz_tab.setStyleSheet("""
            background-color: rgb(73, 160, 120);   /* Arka plan rengi */
            border: 2px solid black;               /* Kenarlık */
            border-radius: 20px;                   /* Köşe yuvarlatma */
            font-size: 15px;                       /* Yazı tipi boyutu */
            padding: 10px;                         /* Buton içi boşluk */
        """)
     # Başlangıçta verileri yükle
        self.dusmanlar = []  # Düşmanlar [(lat, lon), ...]
        self.dusmanlari_yukle()  # Dosyadan düşmanları yükle

        #self.vehicle = connect('com9', wait_ready=True)
        self.map_center = (-35.36321719 ,149.16516857 )  # Varsayılan merkez
        self.zoom_level = 25 # Varsayılan zoom
        self.kare_koordinat = None

        # WebView Ayarları
        self.webView = QWebEngineView(self.gorev_kontrol_tab)
        self.webView.setFixedSize(1000, 750)
        self.webView.move(20, 35)

        self.webView_g = QWebEngineView(self.gozlem_tab)
        self.webView_g.setFixedSize(1050, 900)
        self.webView_g.move(10, 15)  # Yan yana koymak için

        self.map_filename = "map1.html"
        self.map_filename_g = "map2.html"

        # İlk harita oluştur
        self.harita_guncelle()

        # Timer her 5 saniyede haritayı günceller
        self.timer = QTimer()
        self.timer.timeout.connect(self.harita_guncelle)
        self.timer.start(5000)  # 5000 ms = 5 saniye

    def dusmanlari_yukle(self):
        """Dosyadaki düşmanları yükle"""
        self.dusmanlar = []
        try:
            dosya_yolu = self.dusman_coord_input.text().strip()  # QLineEdit'ten dosya yolunu al
            print(f"Dosya yolu: {dosya_yolu}")
            with open(dosya_yolu, "r") as f:
                for line in f:
                    print(f"Satır: {line.strip()}")
                    if "Düşman eklendi" in line:
                        try:
                            coords = line.strip().replace("Düşman eklendi: ", "").strip()
                            print(f"Koordinatlar: {coords}")
                            lat_str, lon_str = coords.split(",")
                            lat, lon = float(lat_str.strip()), float(lon_str.strip())
                            self.dusmanlar.append((lat, lon))
                        except ValueError:
                            print(f"Hatalı koordinat formatı: '{line.strip()}'")
            print("Yüklenen düşmanlar:", self.dusmanlar)
        except FileNotFoundError:
            print("Dosya bulunamadı.")
        except Exception as e:
            print(f"Bir hata oluştu: {e}")







    def harita_guncelle(self):
        """Harita güncelleme fonksiyonu"""
        try:
            # Drone konumunu al
            self.lat = self.vehicle.location.global_frame.lat
            self.lon = self.vehicle.location.global_frame.lon
            drone_konum = (self.lat, self.lon)

            # Haritayı her seferinde sıfırdan oluştur
            self.m = folium.Map(location=self.map_center, zoom_start=self.zoom_level)
            self.m_g = folium.Map(location=self.map_center, zoom_start=self.zoom_level)

            # Drone marker'ını ekle
            folium.Marker(drone_konum, tooltip="Drone", icon=folium.Icon(color="green")).add_to(self.m)
            folium.Marker(drone_konum, tooltip="Drone", icon=folium.Icon(color="green")).add_to(self.m_g)

            # Kare varsa çiz
            if self.kare_koordinat:
                folium.Polygon(
                    self.kare_koordinat,
                    color="blue",
                    weight=2.5,
                    fill=True,
                    fill_opacity=0.5
                ).add_to(self.m)
                folium.Polygon(
                    self.kare_koordinat,
                    color="blue",
                    weight=2.5,
                    fill=True,
                    fill_opacity=0.5
                ).add_to(self.m_g)

            # Düşmanları ekle
            for d_lat, d_lon in self.dusmanlar:
                folium.Marker((d_lat, d_lon), tooltip="Düşman", icon=folium.Icon(color="red")).add_to(self.m)
                folium.Marker((d_lat, d_lon), tooltip="Düşman", icon=folium.Icon(color="red")).add_to(self.m_g)

            # Zigzag rota varsa ekle
            if hasattr(self, 'zigzag_rota') and self.zigzag_rota:
                folium.PolyLine(self.zigzag_rota, color="orange", weight=2.5).add_to(self.m)
                folium.PolyLine(self.zigzag_rota, color="orange", weight=2.5).add_to(self.m_g)

            # Haritayı kaydet
            self.m.save(self.map_filename)
            self.m_g.save(self.map_filename_g)

            # WebView'leri güncelle
            self.webView.setUrl(QUrl.fromLocalFile(os.path.abspath(self.map_filename)))
            self.webView_g.setUrl(QUrl.fromLocalFile(os.path.abspath(self.map_filename_g)))

        except Exception as e:
            print(f"Konum alınamadı: {e}")

            
    def ciz_kare(self):
        try:
            coord_1 = tuple(map(float, self.coord_1_input.text().split(',')))
            coord_2 = tuple(map(float, self.coord_2_input.text().split(',')))
        except ValueError:
            print("Geçersiz koordinatlar. Lütfen 'Lat, Long' formatında girin.")
            return

        lat_min = min(coord_1[0], coord_2[0])
        lat_max = max(coord_1[0], coord_2[0])
        lon_min = min(coord_1[1], coord_2[1])
        lon_max = max(coord_1[1], coord_2[1])

        # Kare koordinatlarını oluştur
        self.kare_koordinat = [
            (lat_min, lon_min),
            (lat_min, lon_max),
            (lat_max, lon_max),
            (lat_max, lon_min)
        ]

        # Zigzag rota üretimi
        rota = []
        metre_basina_lon = 1 / 85200
        adim = 5
        current_lon = lon_min
        yukari = True

        while current_lon <= lon_max:
            if yukari:
                rota.append((lat_min, current_lon))
                rota.append((lat_max, current_lon))
            else:
                rota.append((lat_max, current_lon))
                rota.append((lat_min, current_lon))
            current_lon += metre_basina_lon * adim
            yukari = not yukari

        self.zigzag_rota = rota  # Zigzag rotasını sakla

        # Waypoint'leri yazdır
        for index, waypoint in enumerate(self.zigzag_rota):
            print(f"{index + 1}. Waypoint: Enlem = {waypoint[0]}, Boylam = {waypoint[1]}")

        self.harita_guncelle()

        # Zigzag waypoint'lerini yükle
        self.zigzag_waypoint_yukle()  # Burada otomatik olarak çağrılıyor


    def zigzag_waypoint_yukle(self):
        mesaj = "Zigzag yükleme fonksiyonu başlatıldı."
        print(mesaj)
        self.terminal_output.append(mesaj)
        self.terminal_g_output.append(mesaj)

        if not hasattr(self, "zigzag_rota") or not self.zigzag_rota:
            hata = "Zigzag rotası oluşturulmamış veya boş!"
            print(hata)
            self.terminal_output.append(hata)
            self.terminal_g_output.append(hata)
            return

        try:
            cmds = self.vehicle.commands

            print("Mevcut görevler temizleniyor...")
            self.terminal_output.append("Mevcut görevler temizleniyor...")
            self.terminal_g_output.append("Mevcut görevler temizleniyor...")

            # Mevcut görevleri sil
            cmds.clear()
            time.sleep(1)

            # Yeni waypoint’leri ekle
            for i, (lat, lon) in enumerate(self.zigzag_rota):
                print(f"Eklenen {i}. waypoint: Enlem={lat}, Boylam={lon}")
                wp = Command(
                    0, 0, 0,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 0, 0, 0, 0,
                    lat, lon, 10
                )
                cmds.add(wp)
                time.sleep(0.05)

            # Tüm görevleri yükle
            cmds.upload()
            time.sleep(2)

            # Yüklenen waypoint'leri kontrol et
            cmds.download()
            cmds.wait_ready()

            # Yüklenen waypoint'leri kontrol et ve karşılaştır
            waypoint_list = list(cmds)
            print(f"Beklenen waypoint sayısı: {len(self.zigzag_rota)}")
            print(f"Yüklenen waypoint sayısı: {len(waypoint_list)}")

            if len(waypoint_list) != len(self.zigzag_rota):
                print(f"HATA: Yüklenen waypoint sayısı beklenenden farklı. Lütfen kontrol edin!")
                self.terminal_output.append("HATA: Yüklenen waypoint sayısı beklenenden farklı.")
                self.terminal_g_output.append("HATA: Yüklenen waypoint sayısı beklenenden farklı.")
            else:
                print("Görevler başarıyla yüklendi.")
                self.terminal_output.append("Görevler başarıyla yüklendi.")
                self.terminal_g_output.append("Görevler başarıyla yüklendi.")
            
            # Yüklenen waypoint'lerin detayları
            for i, cmd in enumerate(waypoint_list):
                print(f"{i}: Lat={cmd.x}, Lon={cmd.y}, Alt={cmd.z}, Command={cmd.command}")

            toplam_wp = f"Toplam yüklenen waypoint sayısı: {len(waypoint_list)}"
            print(toplam_wp)
            self.terminal_output.append(toplam_wp)
            self.terminal_g_output.append(toplam_wp)

        except Exception as e:
            hata_mesaj = f"HATA: {e}"
            print(hata_mesaj)
            self.terminal_output.append(hata_mesaj)
            self.terminal_g_output.append(hata_mesaj)









        # görev kontrol gözlemci
    def on_button_Gozlemci_RTLgzlm_clicked(self):
        try:
            # RTL modunu ayarla
            self.vehicle.mode = VehicleMode("RTL")
            time.sleep(1)  # Mod değişimini beklemek için kısa gecikme

 
            if self.vehicle.mode.name == "RTL":
                mesaj = "RTL modu aktif. Drone eve dönüyor."
            else:
                mesaj = "RTL modu ayarlanamadı!"

            print(mesaj)
            self.terminal_output.append(mesaj)
            self.terminal_g_output.append(mesaj)

        except AttributeError as e:
            print(f"Hata: {e}. 'vehicle' nesnesi tanımlı mı?")
        except Exception as e:
            print(f"Beklenmedik bir hata oluştu: {e}")
            
                

    def on_button_Kamikaze_RTLkmkz_clicked(self):
        try:
          
            self.kmkz.mode = VehicleMode("RTL")
            time.sleep(1) 

            if self.kmkz.mode.name == "RTL":
                mesaj = "RTL modu aktif. Drone eve dönüyor."
            else:
                mesaj = "RTL modu ayarlanamadı!"

            print(mesaj)
            self.terminal_output.append(mesaj)
            self.terminal_g_output.append(mesaj)

        except AttributeError as e:
            print(f"Hata: {e}. 'vehicle' nesnesi tanımlı mı?")
        except Exception as e:
            print(f"Beklenmedik bir hata oluştu: {e}")



    def on_button_Gozlemci_paneli_armdurumugzlm_g_clicked(self):
        try:
            self.vehicle.armed = True

            timeout = 5
            start_time = time.time()
            while not self.vehicle.armed and time.time() - start_time < timeout:
                time.sleep(0.5)

            if self.vehicle.armed:
                mesaj = "Drone arm edildi (uçuşa hazır)."
            else:
                mesaj = "Drone arm edilemedi! (şartlar sağlanmıyor olabilir)"

            print(mesaj)
            self.terminal_output.append(mesaj)
            self.terminal_g_output.append(mesaj)

        except AttributeError as e:
            print(f"Hata: {e}. 'self.vehicle' düzgün başlatılmamış olabilir.")
        except Exception as e:
            print(f"Beklenmedik bir hata oluştu: {e}")

    def guncelle_mod(self):
        try:
            current_mode = self.vehicle.mode.name
            self.label_mode.setText(f"Mode: {current_mode}")
        except AttributeError:
            self.label_mode.setText("Mode: Bilinmiyor")

    def guncelle_pilgzlm(self):
        try:
            pil_seviyesi = self.vehicle.battery.level
            battery_info = f"Pil: {pil_seviyesi}%"
        except AttributeError:
            battery_info = "Pil bilgisi alınamadı"
        self.label_battery.setText(battery_info)

    def guncelle_yukseklik(self):
        try:
            altitude = self.vehicle.location.global_relative_frame.alt
            yukseklik_yazisi = f"Yükseklik: {altitude:.3f} m"
        except AttributeError:
            yukseklik_yazisi = "Yükseklik alınamadı"
        self.label_yukseklik.setText(yukseklik_yazisi)

       #gözlem kamikaze
    def on_button_Gozlemci_paneli_aktarilankonumkkmkz_gkmkz_clicked(self):
       aktrlknm="Kamikazenin gideceği konum =xxxxx"
       print(aktrlknm)
       self.terminal_output.append(aktrlknm)
       self.terminal_g_output.append(aktrlknm)

    def guncelle_pilkmkz(self):
        if not self.kmkz:
            self.button_Gozlemci_paneli_pilkmkz_gkmkz_tab.setText("Pil değeri alınamadı")
            return
        try:
            pilkmkz_seviyesi = self.kmkz.battery.level
            batterykmkz_info = f"Pil: {pilkmkz_seviyesi}%"
        except AttributeError:
            batterykmkz_info = "Pil bilgisi alınamadı"

        self.button_Gozlemci_paneli_pilkmkz_gkmkz_tab.setText(batterykmkz_info)

    def guncelle_modkmkz(self):
        if not self.kmkz:
            self.button_Gozlemci_paneli_mode_gkmkz_tab.setText("Mode: Bilinmiyor")
            return
        try:
            current_modekmkz = self.kmkz.mode.name
            self.button_Gozlemci_paneli_mode_gkmkz_tab.setText(f"Mode: {current_modekmkz}")
        except AttributeError:
            self.button_Gozlemci_paneli_mode_gkmkz_tab.setText("Mode: Bilinmiyor")






    """def  arm_durumunukmkz_degistir (self):
            try:
                aarm_durumu = self.kmkz.armed
                if aarm_durumu:
                    durum_yazisi = "Anlık Arm Durumu \nKamikaze: Aktif"
                else:
                    durum_yazisi = "Anlık Arm Durumu \nKamikaze: Pasif"      
            except AttributeError:
                durum_yazisi = "Anlık Arm Durumu \nKamikaze: Hata"
            mevcut_yazi = self.button_Gozlemci_paneli_aarmdurumu_gkmkz_tab.text()

            if mevcut_yazi != durum_yazisi:
                self.button_Gozlemci_paneli_aarmdurumu_gkmkz_tab.setText(durum_yazisi)
                self.terminal_output.append(durum_yazisi)
                self.terminal_g_output.append(durum_yazisi)

        def  goster_Aarmkmkzp_durumudis (self):
            try:
                kmkzip = self.kmkz

                if kmkzip.armed:

                    kmkzip.armed = False
                    while kmkzip.armed:
                        print(" kamikaze Disarm için bekleniyor...")
                        self.terminal_output.append("kamikaze Disarm için bekleniyor...")
                        self.terminal_g_output.append("kamikaze Disarm için bekleniyor...")
                        time.sleep(1)

                    print("kamikaze disarm olmuştur.")
                    self.terminal_output.append(" kamikaze  disarm olmuştur.")
                    self.terminal_g_output.append(" kamikaze disarm olmuştur.")
                
                else:

                    self.arm_durumunukmkz_degistir()

            except Exception as e:
                hata = f"HATA (ARM/VEYA DISARM): {e}"
                print(hata)
                time.sleep(5000)
                
        def dis_arm_durumunu_degistir_thread(self):
            threading.Thread(target=self.arm_durumunukmkz_degistir).start()
    

        def goster_Aarmkmkzp_durumu (self):
            try:
                kmkzip = self.kmkz
                while not kmkzip.is_armable:
                    print(" kamikaze Arm için gerekli şartlar sağlanmadı")
                    self.terminal_output.append(" kmaikzae Arm için gerekli şartlar sağlanmadı")
                    self.terminal_g_output.append(" kamikaze Arm için gerekli şartlar sağlanmadı")
                    time.sleep(2)

                print(" kamikaze şu anda arm edilebilir")  
                self.terminal_output.append('kamikaze şu anda arm edilebilir')
                self.terminal_g_output.append('kamikazae şu anda arm edilebilir')

                kmkzip.mode = VehicleMode("GUIDED")
                while kmkzip.mode.name != 'GUIDED':
                    print('Guided kamikaze moduna geçiş yapılıyor...')
                    self.terminal_output.append('Guided kmaikaze moduna geçiş yapılıyor...')
                    self.terminal_g_output.append('Guided kamikaze moduna geçiş yapılıyor...')
                    time.sleep(2)

                print("Guided kamikaze moduna geçiş yapıldı")
                self.terminal_output.append("Guided  kamikaze moduna geçiş yapıldı")
                self.terminal_g_output.append("Guided  kmaikaze moduna geçiş yapıldı")

                kmkzip.armed = True
                while not kmkzip.armed:
                    print("Arm kamikaze için bekleniyor...")
                    self.terminal_output.append("Arm  akmaikaze için bekleniyor...")
                    self.terminal_g_output.append("Arm kamikaze için bekleniyor...")
                    time.sleep(2)

                print("kamikaze arm olmuştur")
                self.terminal_output.append("kamikaze arm olmuştur")
                self.terminal_g_output.append("kamikaze arm olmuştur")
                
            except Exception as e:
                hata = f"HATA (ARM): {e}"
                print(hata)


        def arm_durumunu_degistirkmkz_thread(self):
            threading.Thread(target=self.goster_Aarmkmkzp_durumu ).start()"""







    def guncelle_yukseklikkmkz(self):
        if not self.kmkz:
            self.labelkmkz_yukseklik.setText("Yükseklik alinamadı")
            return
        try:
            altitude = self.kmkz.location.global_relative_frame.alt
            yukseklikkmkz_yazisi = f"Yükseklik: {altitude:.3f} m"
        except AttributeError:
            yukseklikkmkz_yazisi = "Yükseklik alınamadı"
        
        self.labelkmkz_yukseklik.setText(yukseklikkmkz_yazisi)

    def arm_durumunu_degistirkmkz_thread(self):
        threading.Thread(target=self.goster_Aarmkmkzp_durumu ).start()

    def guncelle_yukseklikkmkz(self):
        if not self.kmkz:
            self.labelkmkz_yukseklik.setText("Yükseklik alinamadı")
            return
        try:
            altitude = self.kmkz.location.global_relative_frame.alt
            yukseklikkmkz_yazisi = f"Yükseklik: {altitude:.3f} m"
        except AttributeError:
            yukseklikkmkz_yazisi = "Yükseklik alınamadı"
        
        self.labelkmkz_yukseklik.setText(yukseklikkmkz_yazisi)

    def on_button_Gozlemci_paneli_armdurumu_gkmkz_clicked(self):
       armdrm="Kamikaze arm durumu =xxxxx"
       print(armdrm)
       self.terminal_output.append(armdrm)
       self.terminal_g_output.append(armdrm)

    def on_button_baglan_clicked(self):
        try:
            self.vehicle = connect(ipdrone ,wait_ready=True)
            self.vehicle.airspeed=20
            bgln = "Drone bağlandı!"
            print(bgln)
            self.terminal_output.append(bgln) 
            self.terminal_g_output.append(bgln)
        except Exception as e:
            error_message = f"Bağlantı hatası: {str(e)}"
            print(error_message)
            self.terminal_output.append(error_message)  
            self.terminal_g_output.append(error_message) 

    def on_button_dusman_goster_clicked(self):
        try:
            text = self.dusman_coord_input.text().strip()
            lat_lon = text.split(",")
            if len(lat_lon) != 2:
                raise ValueError("Koordinatlar 'lat,lon' formatında olmalı.")
            dusman_coord = tuple(map(float, lat_lon))


            dosya_yolu = self.dusman_coord_input.text()
            with open(dosya_yolu, "a") as f:
                f.write(f"Düşman eklendi: {dusman_coord[0]}, {dusman_coord[1]}\n")

            
            self.harita_guncelle()

        except ValueError:
            print("Geçersiz koordinatlar. Lütfen 'Lat, Long' formatında girin.")
        except Exception as e:
            print(f"Bir hata oluştu: {e}")


            self.m.save(self.map_filename)
            self.m_g.save(self.map_filename_g)

            self.webView.setUrl(QUrl.fromLocalFile(os.path.abspath(self.map_filename)))
            self.webView_g.setUrl(QUrl.fromLocalFile(os.path.abspath(self.map_filename_g)))

        except ValueError:
            print("Geçersiz koordinatlar. Lütfen 'Lat, Long' formatında girin.")






    def goster_Aarm_durumu(self):
        durum_GZLM_yazisi = "Anlık Arm Durumu \nGözlemci: Bilinmiyor" 
        try:
            Aarm_durumu = self.vehicle.armed
            if Aarm_durumu:
                durum_GZLM_yazisi = "Anlık Arm Durumu \nGözlemci: Aktif"
            else:
                durum_GZLM_yazisi = "Anlık Arm Durumu \nGözlemci: Pasif"
        except AttributeError:
            durum_GZLM_yazisi = "Anlık Arm Durumu \nGözlemci: Hata"

        # Butonun mevcut yazısını al
        mevcut_yazi = self.button_Gozlemci_paneli_Aarmdurumugzlm_g_tab.text()
        # Eğer yazı değiştiyse, butonu ve terminali güncelle
        if mevcut_yazi != durum_GZLM_yazisi:
            self.button_Gozlemci_paneli_Aarmdurumugzlm_g_tab.setText(durum_GZLM_yazisi)
            self.terminal_output.append(durum_GZLM_yazisi)
            self.terminal_g_output.append(durum_GZLM_yazisi)

    def arm_veya_disarm(self):
        try:
            vehicle = self.vehicle

            if vehicle.armed:
                # Eğer zaten arm ise -> disarm
                print("Drone disarm ediliyor...")
                self.terminal_output.append("Drone disarm ediliyor...")
                self.terminal_g_output.append("Drone disarm ediliyor...")

                vehicle.armed = False
                while vehicle.armed:
                    print("Disarm için bekleniyor...")
                    self.terminal_output.append("Disarm için bekleniyor...")
                    self.terminal_g_output.append("Disarm için bekleniyor...")
                    time.sleep(1)

                print("Drone disarm olmuştur.")
                self.terminal_output.append("Drone disarm olmuştur.")
                self.terminal_g_output.append("Drone disarm olmuştur.")
            
            else:
                # Eğer disarm ise -> arm (mevcut fonksiyonu çağır)
                self.arm_durumunu_degistir()

        except Exception as e:
            hata = f"HATA (ARM/VEYA DISARM): {e}"
            print(hata)
            self.terminal_output.append(hata)
            self.terminal_g_output.append(hata)
            
    def dis_arm_durumunu_degistirgzlm_thread(self):
        threading.Thread(target=self.arm_veya_disarm).start()


    def arm_durumunu_degistir(self):
        try:
            vehicle = self.vehicle 
            while not vehicle.is_armable:
                print("Arm için gerekli şartlar sağlanmadı")
                self.terminal_output.append("Arm için gerekli şartlar sağlanmadı")
                self.terminal_g_output.append("Arm için gerekli şartlar sağlanmadı")
                time.sleep(2)

            print("İHA şu anda arm edilebilir")  
            self.terminal_output.append('İHA şu anda arm edilebilir')
            self.terminal_g_output.append('İHA şu anda arm edilebilir')

            vehicle.mode = VehicleMode("GUIDED")
            while vehicle.mode.name != 'GUIDED':
                print('Guided moduna geçiş yapılıyor...')
                self.terminal_output.append('Guided moduna geçiş yapılıyor...')
                self.terminal_g_output.append('Guided moduna geçiş yapılıyor...')
                time.sleep(2)

            print("Guided moduna geçiş yapıldı")
            self.terminal_output.append("Guided moduna geçiş yapıldı")
            self.terminal_g_output.append("Guided moduna geçiş yapıldı")

            vehicle.armed = True
            while not vehicle.armed:
                print("Arm için bekleniyor...")
                self.terminal_output.append("Arm için bekleniyor...")
                self.terminal_g_output.append("Arm için bekleniyor...")
                time.sleep(2)

            print("Drone arm olmuştur")
            self.terminal_output.append("Drone arm olmuştur")
            self.terminal_g_output.append("Drone arm olmuştur")
            
        except Exception as e:
            hata = f"HATA (ARM): {e}"
            print(hata)
            self.terminal_output.append(hata)
            self.terminal_g_output.append(hata)

    def arm_durumunu_degistir_thread(self):
        threading.Thread(target=self.arm_durumunu_degistir).start()


    def on_button_Gozlemci_kesfebaslagzlm_clicked(self):
        try:
            baslat_msg = "Keşif görevi başlatılıyor: Uçuş modu AUTO'ya geçiriliyor..."
            print(baslat_msg)
            self.terminal_output.append(baslat_msg)
            self.terminal_g_output.append(baslat_msg)

            self.vehicle.commands.next = 0
            self.vehicle.commands.upload()
            self.vehicle.simple_takeoff(15)
            self.vehicle.mode = VehicleMode("AUTO")
            time.sleep(2)

            if self.vehicle.mode.name == "AUTO":
                basladi = "Drone şu an görev uçuşunda (AUTO mod)."
                print(basladi)
                self.terminal_output.append(basladi)
                self.terminal_g_output.append(basladi)

                # Görevi izleme ayrı thread'de çalışacak
                threading.Thread(target=self.gorev_takibi, daemon=True).start()

        except Exception as e:
            hata = f"HATA (keşfe başla): {e}"
            print(hata)
            self.terminal_output.append(hata)
            self.terminal_g_output.append(hata)
            
    def guiye_yaz(self, mesaj):
            self.terminal_output.append(mesaj)
            self.terminal_g_output.append(mesaj)

    def gorev_takibi(self):
        try:
            toplam_wp = len(self.zigzag_rota)
            while self.vehicle.commands.next < toplam_wp:
                print(f"Aktif WP: {self.vehicle.commands.next}/{toplam_wp}")
                time.sleep(2)
                QCoreApplication.processEvents()

            print("Görev tamamlandı, RTL'ye geçiliyor...")
            self.vehicle.mode = VehicleMode("RTL")

            mesaj = "Görev tamamlandı. Tüm waypoint'ler uçuldu."
            print(mesaj)
            self.gui_guncelle.emit(mesaj)  # GUI'ye sinyal gönderiliyor

        except Exception as e:
            print(f"Hata oluştu: {e}")



if __name__ == "__main__":
    app = QApplication(sys.argv)
    pencere = YerKontrolIstasyonu()
    pencere.show()
    sys.exit(app.exec_())