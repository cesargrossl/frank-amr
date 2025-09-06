from rplidar import RPLidar

lidar = RPLidar('/dev/ttyUSB0')
info = lidar.get_info()
health = lidar.get_health()
print('Info:', info)
print('Health:', health)
lidar.stop()
lidar.disconnect()
