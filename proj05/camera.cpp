sudo apt update
sudo apt install -y libcamera-apps libcamera-dev g++ cmake pkg-config \
                    libopencv-dev gstreamer1.0-tools gstreamer1.0-libcamera \
                    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
                    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly



libcamera-hello --list-cameras
# ou
rpicam-hello --list-cameras



libcamera-hello -t 0      # ou rpicam-hello -t 0 (preview contínuo)
