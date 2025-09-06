from rplidar import RPLidar

PORT = "/dev/ttyUSB0"   # ajuste se for outro
for BAUD in (115200, 256000):
    print("\n=== Tentando baud:", BAUD, "===")
    try:
        lidar = RPLidar(PORT, baudrate=BAUD, timeout=3)
        info = lidar.get_info()
        health = lidar.get_health()
        print("INFO:", info)
        print("HEALTH:", health)
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("OK com baud", BAUD)
        break
    except Exception as e:
        print("Falhou:", e)
