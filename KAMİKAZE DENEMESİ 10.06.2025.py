
#                                                                  /BİSMİLLAH/
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time
import math

vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

vehicle.airspeed = 50

takeoff_alt = 50
dalma_lat, dalma_lon = 40.990719512689836, 29.077427208182286


home_location = vehicle.location.global_frame
takeoff_lat = home_location.lat + 0.00009
takeoff_lon = home_location.lon





kmkzcikis_lon = 30 / (111320 * math.cos(math.radians(takeoff_lat)))
kmkzcikis_lat = dalma_lat
kmkzcikis_lon = dalma_lon + kmkzcikis_lon

cmds = vehicle.commands
cmds.clear()
cmds.wait_ready()

cmds.add(Command(0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 1, 15, 0, 0, 0,
    takeoff_lon, takeoff_lat, takeoff_alt
))


cmds.add(Command(0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 1, 0, 0, 0, 0,
    dalma_lat, dalma_lon, takeoff_alt
))


cmds.add(Command(0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_SCRIPT_TIME,
    0, 1,
    18, 
    0, 10, 45,
    0, 0,0
))

# **Eklenen Bölüm**: 10 metre ilerisi
# 10 metreyi ileriyi bulmak için, enlem ve boylamda küçük bir değişiklik yapıyoruz (yaklaşık 0.00009 derece)
guncel_lat = dalma_lat + (50 / 111320)  # 10 metreyi lat/lon dönüşümüne çevirdik
cmds.add(Command(0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 1, 0, 0, 0, 0,
    guncel_lat, dalma_lon, takeoff_alt
))

# **Eklenen Bölüm**: 50 metre sağa kayma
# 50 metreyi sağa kaydırmak için, boylamda yaklaşık olarak bir değişiklik yapıyoruz
guncel_lon = dalma_lon + (50 / (111320 * math.cos(math.radians(guncel_lat))))  # sağa kayma için lon değişikliği
cmds.add(Command(0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 1, 0, 0, 0, 0,
    guncel_lat, guncel_lon, takeoff_alt
))





cmds.add(Command(0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 1, 0, 0, 0, 0,
    home_location.lat, home_location.lon, 0
))


cmds.upload()
cmds.download()
cmds.wait_ready()
mission_list = list(cmds)



vehicle.mode = VehicleMode("STABILIZE")
while vehicle.mode.name != "STABILIZE":
    print("STABILIZE moduna geçiliyor...")
    time.sleep(1)

while not vehicle.is_armable:
    print("Araç arm için hazır değil...")
    time.sleep(1)

print("Throttle açılıyor...") 
vehicle.armed = True
while not vehicle.armed:
    print("Araç arm ediliyor...")
    time.sleep(1)


print("Görev başlıyor...")
vehicle.channels.overrides['3'] = 1800   
time.sleep(5)
vehicle.channels.overrides['3'] = None

vehicle.mode = VehicleMode("AUTO")
while vehicle.mode.name != "AUTO":
    print("AUTO moduna geçiliyor...")
    time.sleep(1)

print("Görev başlıyor...")
try:
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Yükseklik: {alt:.2f} m")
        time.sleep(1)
except KeyboardInterrupt:
    print("Takip durduruldu.")
    vehicle.mode=VehicleMode("GUIDED")

