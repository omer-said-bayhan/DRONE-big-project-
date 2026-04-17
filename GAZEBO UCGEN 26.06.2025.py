#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
GÃ¶rev tabanlÄ± Gazebo drone kÄ±rmÄ±zÄ± Ã¼Ã§gen takip sistemi
- Auto modda rota takibi
- Hedef bulunca Guided moda geÃ§iÅŸ
- Hedef ortalama ve koordinat alma
- Auto moda geri dÃ¶nÃ¼ÅŸ ve rota devamÄ±
"""

from __future__ import print_function, division
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import socket 

YKI_IP = "localhost" #196.155 - 102.224 
PORT = 12345

def send_location(location):
    lat, lon, alt = location
    message = f"{lat},{lon},{alt}"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(message.encode(), (YKI_IP, PORT))
    sock.close()

# === GLOBAL DEÄIÅKENLER ===
ENABLE_TRIANGLE_DETECTION = True
fsh = None
iha = None
bridge = CvBridge()

# Durum deÄŸiÅŸkenleri
saved_location = None
last_triangle_center = None
last_target_coords = None
TRIGGERED = False
TRIANGLE_DETECTED = False
emergency_stopped = False
target_locked = False  # Hedef kilitlenme durumu
last_known_position = None  # Son bilinen hedef pozisyonu
last_known_area = None  # Son bilinen hedef alanÄ±

# MOD KONTROL DEÄÄ°ÅKENLERÄ°
original_mode = None  # Orijinal AUTO modu
guided_mode_active = False  # Guided mod aktif mi?
mission_paused = False  # GÃ¶rev duraklatÄ±ldÄ± mÄ±?

# Kontrol parametreleri
KV = 0.008                # HÄ±z kazancÄ±nÄ± artÄ±rdÄ±k
YAW_STEP = 2             # DÃ¶nÃ¼ÅŸ aÃ§Ä±sÄ±nÄ± kÃ¼Ã§Ã¼lttÃ¼k
PIXEL_THRESHOLD = 20     # Merkez toleransÄ±nÄ± artÄ±rdÄ±k
MOVE_INTERVAL = 0.12     # Hareket aralÄ±ÄŸÄ±nÄ± kÄ±salttÄ±k

# Tracking parametreleri
DETECTION_CONFIDENCE = 3  # KaÃ§ frame Ã¼st Ã¼ste tespit gerekli
LOSS_TOLERANCE = 15      # KaÃ§ frame kayÄ±p tolere edilir
AREA_TOLERANCE = 0.3     # Alan deÄŸiÅŸim toleransÄ± (%30)
POSITION_TOLERANCE = 50  # Pozisyon deÄŸiÅŸim toleransÄ± (piksel)

# Zamanlama ve sayaÃ§lar
last_move_time = 0.0
detection_counter = 0
loss_counter = 0
stable_counter = 0
STABLE_THRESHOLD = 8     # Koordinat kaydetme iÃ§in gereken kararlÄ± frame sayÄ±sÄ±

# Adaptif tespit parametreleri
current_min_area = 300   # Dinamik minimum alan
area_history = []        # Alan geÃ§miÅŸi
position_history = []    # Pozisyon geÃ§miÅŸi

# === DRONE BAÄLANTI VE KONTROL FONKSÄ°YONLARI ===

def connect_vehicle(connection_str='udp:127.0.0.1:14551'):
    """Drone'a baÄŸlanma fonksiyonu"""
    global iha
    try:
        print("Drone'a baglaniliyor: %s" % connection_str)
        iha = connect(connection_str, wait_ready=True)
        iha.airspeed=4
        rospy.loginfo('Drone baglantisi kuruldu: %s', connection_str)
        print("Drone versiyonu: %s" % str(iha.version))
        return True
    except Exception as e:
        rospy.logerr('Drone baglanti hatasi: %s', str(e))
        print("BaÄŸlantÄ± hatasÄ±: %s" % str(e))
        return False

# === MOD KONTROL FONKSÄ°YONLARI ===

def switch_to_guided():
    """AUTO modundan GUIDED moduna geÃ§iÅŸ"""
    global original_mode, guided_mode_active, mission_paused
    
    if not iha:
        return False
    
    try:
        # Mevcut modu kaydet
        original_mode = iha.mode.name
        print("ğŸ”„ Mod deÄŸiÅŸimi: %s -> GUIDED" % original_mode)
        
        # GUIDED moduna geÃ§
        iha.mode = VehicleMode("GUIDED")
        
        # Mod deÄŸiÅŸimini bekle
        timeout = 5
        start_time = time.time()
        while iha.mode.name != "GUIDED" and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if iha.mode.name == "GUIDED":
            guided_mode_active = True
            mission_paused = True
            print("âœ… GUIDED modu aktif - GÃ¶rev duraklatÄ±ldÄ±")
            return True
        else:
            print("âŒ GUIDED moduna geÃ§ilemedi")
            return False
            
    except Exception as e:
        rospy.logerr('Guided mod gecis hatasi: %s', str(e))
        return False

def switch_to_auto():
    """GUIDED modundan AUTO moduna geri dÃ¶nÃ¼ÅŸ"""
    global original_mode, guided_mode_active, mission_paused
    
    if not iha:
        return False
    
    try:
        print("ğŸ”„ Mod deÄŸiÅŸimi: GUIDED -> AUTO")
        
        # AUTO moduna geÃ§
        iha.mode = VehicleMode("AUTO")
        
        # Mod deÄŸiÅŸimini bekle
        timeout = 5
        start_time = time.time()
        while iha.mode.name != "AUTO" and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if iha.mode.name == "AUTO":
            guided_mode_active = False
            mission_paused = False
            print("âœ… AUTO modu aktif - GÃ¶rev devam ediyor")
            return True
        else:
            print("âŒ AUTO moduna geÃ§ilemedi")
            return False
            
    except Exception as e:
        rospy.logerr('Auto mod gecis hatasi: %s', str(e))
        return False

# === HAREKET KONTROL FONKSÄ°YONLARI ===

def send_ned_velocity(vx, vy, vz):
    """NED koordinat sisteminde hÄ±z komutu gÃ¶nderir"""
    if not iha or not guided_mode_active:
        return
    
    try:
        # HÄ±z limitlerini uygula
        max_speed = 0.6  # Biraz artÄ±rdÄ±k
        vx = max(-max_speed, min(max_speed, vx))
        vy = max(-max_speed, min(max_speed, vy))
        vz = max(-max_speed, min(max_speed, vz))
        
        msg = iha.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)
        iha.send_mavlink(msg)
        iha.flush()
    except Exception as e:
        rospy.logerr('Hiz komutu hatasi: %s', str(e))

def condition_yaw(heading, relative=True, direction=1, rate=8):
    """Yaw (dÃ¶nÃ¼ÅŸ) komutu gÃ¶nderir"""
    if not iha or not guided_mode_active:
        return
    
    try:
        is_rel = 1 if relative else 0
        rate = max(1, min(25, rate))
        heading = max(1, min(360, abs(heading)))
        
        msg = iha.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading, rate, direction, is_rel,
            0, 0, 0)
        iha.send_mavlink(msg)
        iha.flush()
    except Exception as e:
        rospy.logerr('Yaw komutu hatasi: %s', str(e))

def emergency_stop():
    """Drone'u anÄ±nda durdurur (sadece GUIDED modda)"""
    if not iha or not guided_mode_active:
        return
    
    try:
        send_ned_velocity(0, 0, 0)
        
        msg = iha.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111111000,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0)
        iha.send_mavlink(msg)
        iha.flush()
        
        print("*** GUIDED MODDA DURDURMA KOMUTU GONDERILDI ***")
        rospy.loginfo("Emergency stop command sent in GUIDED mode")
        
    except Exception as e:
        rospy.logerr('Acil durdurma hatasi: %s', str(e))

# === GELÄ°ÅMÄ°Å GÃ–RÃœNTÃœ Ä°ÅLEME FONKSÄ°YONLARI ===

def is_valid_detection(contour, center, area):
    """Tespitin geÃ§erli olup olmadÄ±ÄŸÄ±nÄ± kontrol eder"""
    global last_known_position, last_known_area, target_locked
    
    if contour is None or center is None or area is None:
        return False
    
    # Ä°lk tespit
    if not target_locked:
        return area > current_min_area
    
    # Hedef kilitlenmiÅŸse daha esnek kriterler
    if last_known_position is not None and last_known_area is not None:
        cx, cy = center
        last_cx, last_cy = last_known_position
        
        # Pozisyon kontrolÃ¼
        position_distance = ((cx - last_cx)**2 + (cy - last_cy)**2)**0.5
        if position_distance > POSITION_TOLERANCE:
            return False
        
        # Alan kontrolÃ¼
        area_ratio = abs(area - last_known_area) / last_known_area
        if area_ratio > AREA_TOLERANCE:
            return False
    
    return True

def detect_red_triangle_enhanced(frame):
    """GeliÅŸmiÅŸ kÄ±rmÄ±zÄ± Ã¼Ã§gen tespit fonksiyonu"""
    global current_min_area, target_locked

    if frame is None or frame.size == 0:
        return None, None, None

    try:
        h, w = frame.shape[:2]
        if h == 0 or w == 0:
            return None, None, None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if target_locked:
            lower1, upper1 = np.array([0, 100, 50]), np.array([15, 255, 255])
            lower2, upper2 = np.array([155, 100, 50]), np.array([180, 255, 255])
        else:
            lower1, upper1 = np.array([0, 120, 70]), np.array([10, 255, 255])
            lower2, upper2 = np.array([160, 120, 70]), np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        if target_locked:
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        else:
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        if cv2.__version__.startswith('3'):
            _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        best_center = None
        best_area = 0

        for contour in contours:
            if contour is not None and len(contour) >= 3:
                area = cv2.contourArea(contour)

                if area > current_min_area:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        if is_valid_detection(contour, (cx, cy), area):
                            epsilon = 0.02 * cv2.arcLength(contour, True)
                            approx = cv2.approxPolyDP(contour, epsilon, True)

                            if target_locked or len(approx) == 3:
                                if area > best_area:
                                    best_area = area
                                    best_contour = contour
                                    best_center = (cx, cy)

        # SADECE GEÃ‡ERLÄ° ÃœÃ‡GEN VARKEN DÃ–N
        if best_contour is not None and best_center is not None and best_area > 0:
            return best_contour, best_center, best_area
        else:
            return None, None, None

    except Exception as e:
        rospy.logerr('Gelismis ucgen tespit hatasi: %s', str(e))
        return None, None, None

def update_tracking_history(center, area):
    """Tracking geÃ§miÅŸini gÃ¼nceller"""
    global area_history, position_history, current_min_area
    
    if center is not None and area is not None:
        # Pozisyon geÃ§miÅŸi
        position_history.append(center)
        if len(position_history) > 10:
            position_history.pop(0)
        
        # Alan geÃ§miÅŸi
        area_history.append(area)
        if len(area_history) > 10:
            area_history.pop(0)
        
        # Dinamik minimum alan gÃ¼ncelle
        if len(area_history) >= 3:
            avg_area = sum(area_history) / len(area_history)
            current_min_area = max(200, int(avg_area * 0.5))

# === YARDIMCI FONKSÄ°YONLAR ===

def save_coordinates():
    global send_location
    """GPS koordinatlarÄ±nÄ± kaydet"""
    try:
        if iha and hasattr(iha, 'location') and iha.location.global_relative_frame:
            location = iha.location.global_relative_frame
            saved_location = (location.lat, location.lon, location.alt)
            print("=== KOORDINATLAR KAYDEDILDI ===")
            print("Lat: %.7f" % saved_location[0])
            print("Lon: %.7f" % saved_location[1]) 
            print("Alt: %.2f m" % saved_location[2])
            rospy.loginfo("GPS Koordinatlari kaydedildi: Lat=%.7f, Lon=%.7f, Alt=%.2f" % saved_location)
            send_location(saved_location)
            print("GÖNDERİLDİ")
            return saved_location
        else:
            print("GPS verisi alinamadi!")
            return None
    except Exception as e:
        rospy.logwarn('Koordinat kayit hatasi: %s', str(e))
        return None

def cleanup():
    """Temizlik fonksiyonu"""
    try:
        if iha and guided_mode_active:
            send_ned_velocity(0, 0, 0)
        cv2.destroyAllWindows()
        if iha:
            print("Drone baglantisi kapatiliyor...")
            iha.close()
        rospy.loginfo('Dugum temizlendi.')
    except Exception as e:
        rospy.logerr('Temizlik hatasi: %s', str(e))

# === ANA CALLBACK FONKSÄ°YONU ===
def image_callback(msg):
    global ENABLE_TRIANGLE_DETECTION, fsh, last_target_coords, last_triangle_center
    global last_move_time, TRIGGERED, TRIANGLE_DETECTED, detection_counter, stable_counter
    global emergency_stopped, target_locked, last_known_position, last_known_area, loss_counter
    global guided_mode_active, mission_paused

    try:
        frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    except (CvBridgeError, Exception) as e:
        rospy.logerr('Goruntu donusturme hatasi: %s', str(e))
        return

    if frame is None or frame.size == 0:
        return

    display = frame.copy()
    h, w = frame.shape[:2]
    cx_frame, cy_frame = w // 2, h // 2
    cv2.circle(display, (cx_frame, cy_frame), 5, (255, 0, 0), -1)

    # === ÜÇGEN TAKİP MODU ===
    if ENABLE_TRIANGLE_DETECTION:
        triangle_contour, triangle_center, triangle_area = detect_red_triangle_enhanced(frame)

        if triangle_contour is not None and len(triangle_contour) >= 3 and triangle_center is not None and triangle_area > 0:
            
            # ✅ AYNІ ÜÇGEN KONTROLÜ - DAHA ESNEK
            skip_same_target = False
            if last_triangle_center and TRIGGERED:  # Sadece hedef kilitliyken kontrol et
                dx = triangle_center[0] - last_triangle_center[0]
                dy = triangle_center[1] - last_triangle_center[1]
                dist_px = (dx**2 + dy**2)**0.5

                if dist_px < 40:  # Aynı hedef kontrolü
                    skip_same_target = True
                    print("⚠️ Aynı hedef tekrar görünüyor, işlem atlanıyor (mesafe: %.1f px)" % dist_px)
                    # Görsel feedback ver ama işlem yapma
                    cv2.drawContours(display, [triangle_contour], -1, (0, 100, 255), 2)  # Turuncu renk
                    cv2.circle(display, triangle_center, 7, (0, 100, 255), -1)
                    cv2.putText(display, "AYNΙ HEDEF - ATLANΙYOR", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)

            # ✅ SADECE YENİ HEDEFLER İÇİN İŞLEM YAP
            if not skip_same_target:
                cx, cy = triangle_center
                update_tracking_history(triangle_center, triangle_area)
                last_known_position = triangle_center
                last_known_area = triangle_area

                # Normal görsel feedback
                cv2.drawContours(display, [triangle_contour], -1, (0, 255, 255), 2)
                cv2.circle(display, (cx, cy), 7, (0, 255, 0), -1)
                cv2.line(display, (cx, cy), (cx_frame, cy_frame), (0, 255, 0), 2)

                offset_x = cx - cx_frame
                offset_y = cy - cy_frame

                # Tetikleme kontrolü
                if not TRIGGERED:
                    if is_valid_detection(triangle_contour, triangle_center, triangle_area):
                        detection_counter += 1
                        print("🔍 Geçerli tespit - counter: %d/%d" % (detection_counter, DETECTION_CONFIDENCE))
                    else:
                        detection_counter = max(0, detection_counter - 1)
                        print("❌ Geçersiz tespit - counter düşürüldü: %d" % detection_counter)

                    if detection_counter >= DETECTION_CONFIDENCE:
                        # ✅ HEDEF TESPİT EDİLDİ - GUIDED MODA GEÇ
                        if is_valid_detection(triangle_contour, triangle_center, triangle_area):
                            print("🎯 Gerçek hedef tespit edildi!")
                            
                            # AUTO'dan GUIDED'a geç
                            if switch_to_guided():
                                print("🔄 GUIDED mod aktif - Hedef ortalama başlıyor")
                                emergency_stop()  # GUIDED modda durdur
                                TRIGGERED = True
                                TRIANGLE_DETECTED = True
                                emergency_stopped = True
                                target_locked = True
                                loss_counter = 0
                                stable_counter = 0
                            else:
                                print("❌ GUIDED moda geçilemedi - tespit iptal")
                                detection_counter = 0
                        else:
                            print("⚠️ Güven skoru yüksek ama hedef geçersiz - counter sıfırlanıyor!")
                            detection_counter = 0

                # Hedef ortalama (sadece GUIDED modda)
                if emergency_stopped and target_locked and guided_mode_active:
                    current_time = time.time()
                    if current_time - last_move_time > MOVE_INTERVAL:
                        if abs(offset_x) > 8 or abs(offset_y) > 8:
                            vel_x = -offset_y * KV
                            vel_y = offset_x * KV
                            send_ned_velocity(vel_x, vel_y, 0)
                            last_move_time = current_time

                    if abs(offset_x) > PIXEL_THRESHOLD:
                        direction = 1 if offset_x > 0 else -1
                        condition_yaw(YAW_STEP, True, direction, 8)

                # Merkez kontrol ve koordinat kaydetme
                distance_from_center = ((cx - cx_frame)**2 + (cy - cy_frame)**2)**0.5
                if distance_from_center < 25 and guided_mode_active:
                    stable_counter += 1
                    if stable_counter >= STABLE_THRESHOLD:
                        saved_coords = save_coordinates()
                        if saved_coords:
                            stable_counter = 0
                            send_ned_velocity(0, 0, 0)
                            fsh = True
                            last_target_coords = saved_coords
                            last_triangle_center = triangle_center       # Görüntü merkezini kaydet
                            
                            print("✅ Hedef merkeze alındı - Koordinat kaydedildi")
                            print("🔄 AUTO moduna geçiş başlıyor...")
                            
                            # GUIDED'dan AUTO'ya geri dön
                            if switch_to_auto():
                                ENABLE_TRIANGLE_DETECTION = False            # Sistemi kapat
                                print("✅ AUTO mod aktif - Rota devam ediyor")
                            else:
                                print("❌ AUTO moda geçilemedi!")
                else:
                    stable_counter = max(0, stable_counter - 1)

            # Durum göstergesi (hem aynı hedef hem yeni hedef için)
            if guided_mode_active:
                cv2.putText(display, "GUIDED MOD - HEDEF ORTALANIYOR", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            else:
                cv2.putText(display, "HEDEF KİLİTLİ", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                
        else:
            # ✅ DÜZELTME: Hedef bulunamadığında counter'ı azalt
            if not TRIGGERED:
                detection_counter = max(0, detection_counter - 1)
                if detection_counter > 0:
                    print("📉 Hedef yok - counter azaltıldı: %d" % detection_counter)
            
            if target_locked and guided_mode_active:
                loss_counter += 1
                if loss_counter > LOSS_TOLERANCE:
                    print("🔄 Hedef tamamen kaybedildi - AUTO moda dönülüyor")
                    
                    # GUIDED'dan AUTO'ya geri dön
                    if switch_to_auto():
                        target_locked = False
                        TRIGGERED = False
                        TRIANGLE_DETECTED = False
                        emergency_stopped = False
                        detection_counter = 0 
                        loss_counter = 0
                        stable_counter = 0
                        last_known_position = None
                        last_known_area = None
                        print("✅ AUTO mod aktif - Hedef kaybı nedeniyle rota devam ediyor")
                    else:
                        print("❌ AUTO moda geçilemedi!")
                else:
                    cv2.putText(display, "GUIDED MOD - HEDEF GEÇİCİ KAYIP", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
            else:
                if iha and iha.mode.name == "AUTO":
                    cv2.putText(display, "AUTO MOD - KIRMIZI ÜÇGEN ARANIYOR", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                else:
                    cv2.putText(display, "KIRMIZI ÜÇGEN ARANIYOR...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    # === TAKİP KAPALIYSA - GPS TABANLI YENİ HEDEF ARAMA ===
    else:
        # ✅ DÜZELTME: Takip kapalıyken tüm detection değişkenlerini sıfırla
        if detection_counter > 0:
            print("🔄 Takip kapalı - detection_counter sıfırlanıyor (eski: %d)" % detection_counter)
            detection_counter = 0
            
        if TRIGGERED or emergency_stopped or target_locked:
            print("🔄 Takip kapalı - tüm hedef durumları sıfırlanıyor")
            TRIGGERED = False
            emergency_stopped = False
            target_locked = False
            loss_counter = 0
            stable_counter = 0

        if iha and iha.location.global_relative_frame and last_target_coords:
            current_lat = iha.location.global_relative_frame.lat
            current_lon = iha.location.global_relative_frame.lon

            d_lat = (current_lat - last_target_coords[0]) * 111320
            d_lon = (current_lon - last_target_coords[1]) * 111320 * np.cos(np.radians(current_lat))
            gps_dist = (d_lat**2 + d_lon**2)**0.5


            if gps_dist > 12:
                print("🔍 Yeterince uzaklaşıldı - üçgen arama tekrar aktif!")
                ENABLE_TRIANGLE_DETECTION = True
                # Tüm durumları temizle
                TRIGGERED = False
                TRIANGLE_DETECTED = False
                emergency_stopped = False
                detection_counter = 0
                loss_counter = 0
                stable_counter = 0
                last_triangle_center = None

        if iha and iha.mode.name == "AUTO":
            cv2.putText(display, "AUTO MOD - Takip devre disi", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        else:
            cv2.putText(display, "Takip devre disi - goruntu aktif", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

    # === GÖRÜNTÜ HER ZAMAN GÖSTERİLİR ===
    cv2.imshow('Görev Tabanlı Kırmızı Üçgen Takip', display)
    cv2.waitKey(1)
# === ANA FONKSÄ°YON ===

def main():
    """Ana fonksiyon"""
    global iha
    try:
        rospy.init_node('mission_red_triangle_tracker', anonymous=True)
        rospy.loginfo('GÃ¶rev tabanlÄ± ROS node baÅŸlatÄ±ldÄ±')
        
        print("ğŸ¯ GÃ¶rev TabanlÄ± KÄ±rmÄ±zÄ± ÃœÃ§gen Takip Sistemi BaÅŸlatÄ±lÄ±yor...")
        
        if not connect_vehicle():
            rospy.logerr("Drone baglantisi kurulamadi!")
            return
        
        print("ğŸ“· GÃ¶rev tabanlÄ± kamera sistemi aktifleÅŸtiriliyor...")
        rospy.Subscriber('/drone/camera1/image_raw', Image, image_callback)
        rospy.loginfo('GÃ¶rev tabanlÄ± subscriber aktif.')
        
        print("ğŸ¯ AUTO modda rota takibi ve hedef arama baÅŸlatÄ±ldÄ±!")
        print("- Hedef bulunca: AUTO -> GUIDED (ortalama)")
        print("- Koordinat alÄ±ndÄ±ktan sonra: GUIDED -> AUTO (rota devam)")
        print("Ã‡Ä±kÄ±ÅŸ iÃ§in Ctrl+C basÄ±n")
        rospy.spin()
        
    except KeyboardInterrupt:
        print("\nâ¹ï¸  Program durduruldu (Ctrl+C)")
        rospy.loginfo('Kapatiliyor...')

    except Exception as e:
            rospy.logerr('Ana fonksiyon hatasi: %s', str(e))
            print("Kritik hata: %s" % str(e))
    finally:
        cleanup()

if __name__ == '__main__':
    main()