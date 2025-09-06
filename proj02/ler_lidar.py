import argparse
# ...
def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)  # mude para 256000 se preciso
    return ap.parse_args()

def main():
    args = parse_args()
    setup_gpio()
    lidar = None
    try:
        lidar = RPLidar(args.port, baudrate=args.baud, timeout=3)
        lidar.start_motor()
        time.sleep(0.5)
        # resto do loop...
