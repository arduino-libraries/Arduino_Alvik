:floppy_disk: `alvik_ros2_firmware`
===================================

By compiling and uploading the firmware contained in this repository all Arduino [Alvik](https://store.arduino.cc/products/alvik) internal sensors and actuators can be accessed via default ROS2 topics.

### How-to-build/upload
```bash
cd ~/Arduino/libraries
git clone https://github.com/arduino-libraries/Arduino_Alvik && cd Arduino_Alvik
arduino-cli core install arduino:esp32
arduino-cli compile --fqbn arduino:esp32:nano_nora  -u -p /dev/ttyACM0
```

### How-to-run Micro-ROS local agent
```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```
