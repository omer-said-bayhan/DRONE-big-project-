from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# Bağlantı
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# Parametreler
targetbitis_lat, targetbitis_lon = 40.99928912943858, 29.051764684751525  # Hedef iniş koordinatları
targetbitis_alt = 50  # Hedef irtifa (iniş yapılacak yer)
buraya_inmeyen_maldir = 0.5  # İniş yüksekliği

dalma_lat, dalma_lon = 40.997127432469696, 29.05779194988268  # Dalma noktası
dalma_alt = 10  # Dalma yüksekliği
dalma_start_distance = 50  # Dalma başlama mesafesi
altitude_after_dive = 50  # Dalış sonrası çıkılacak yükseklik

cmds = vehicle.commands
cmds.clear()
cmds.wait_ready()

vehicle.airspeed = 50  # Sabit kanat için uygun hız, 50 m/s gibi bir değer uygun olabilir.

# Kalkış ve başlangıç
vehicle.mode = VehicleMode("GUIDED")
while vehicle.mode.name != "GUIDED":
    print("GUIDED moda geçiliyor...")
    time.sleep(1)

while not vehicle.is_armable:
    print("Araç arm edilemiyor...")
    time.sleep(1)

vehicle.armed = True
while not vehicle.armed:
    print("Araç arm ediliyor...")
    time.sleep(1)

# Kalkış
vehicle.mode = VehicleMode("TAKEOFF")
while vehicle.mode.name != "TAKEOFF":
    print("TAKEOFF moduna geçiliyor...")
    time.sleep(1)

vehicle.simple_takeoff(targetbitis_alt)

# Hedef irtifaya ulaşıldı mı kontrol et
while True:
    alt = vehicle.location.global_relative_frame.alt
    print(f"Yükseklik: {alt:.2f}")
    if alt >= targetbitis_alt * 0.95:
        print("Hedef irtifaya ulaşıldı.")
        break
    time.sleep(1)

vehicle.mode = VehicleMode("AUTO")
while vehicle.mode.name != "AUTO":
    print("AUTO moduna geçiliyor...")
    time.sleep(1)

# Dalma için yaklaşma mesafesini bekleme
def distance_to_target(lat1, lon1, lat2, lon2):
    """ İki koordinat arasındaki yatay mesafeyi (metre cinsinden) hesaplar """
    R = 6371000  # Dünya'nın yarıçapı (metre)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c  # Yatay mesafe (metre cinsinden)

def start_dive():
    print("Dalış başlatılıyor...")
    # Dalma komutunu doğru şekilde oluşturuyoruz
    cmd = Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
        0, 0, 0, 0,
        dalma_lat, dalma_lon, dalma_alt
    )
    cmds.add(cmd)
    cmds.upload()  # Komutu gönder

# Dalma sonrası çıkış (1 metreden çıkma)
def ascend_after_dive():
    print(f"Dalış tamamlandı, {altitude_after_dive} metreye çıkılıyor...")
    # Çıkış komutunu gönderiyoruz
    cmd = Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
        0, 0, 0, 0,
        dalma_lat, dalma_lon, altitude_after_dive
    )
    cmds.add(cmd)
    cmds.upload()

# Hedefe yaklaşma mesafesini kontrol et
while True:
    lat, lon = vehicle.location.global_frame.lat, vehicle.location.global_frame.lon
    dist = distance_to_target(lat, lon, dalma_lat, dalma_lon)
    print(f"Hedefe olan yatay mesafe: {dist:.2f} metre, mevcut yükseklik: {vehicle.location.global_relative_frame.alt:.2f}")
    
    if dist <= dalma_start_distance:
        print("Hedefe yaklaşmak tamam, dalış başlatılıyor...")
        start_dive()
        break
    time.sleep(1)

# Dalış sonrası çıkış
while vehicle.location.global_relative_frame.alt > dalma_alt + 0.5:
    print(f"Dalış yapılmış, mevcut yükseklik: {vehicle.location.global_relative_frame.alt:.2f}")
    if vehicle.location.global_relative_frame.alt <= dalma_alt + 0.5:
        ascend_after_dive()
        break
    time.sleep(1)

# İniş için hız azaltma ve kontrol
vehicle.airspeed = 10  # İniş sırasında hız yavaşlatılabilir

# Hedefe iniş yap
while vehicle.location.global_relative_frame.alt > 0.5:  # Yükseklik sıfırın altına inene kadar bekle
    print(f"İniş yapılıyor, mevcut yükseklik: {vehicle.location.global_relative_frame.alt:.2f}")
    time.sleep(1)

print("İniş tamamlandı. Araç tamamen yerde.")

vehicle.armed = False
vehicle.close()
print("Bağlantı kapatıldı.")
