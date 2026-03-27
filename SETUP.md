# Hardware Setup

Overview:

Laptop:
- Runs QGC, mission control node (ModeManager), and broadcoasts hotspot thru wifi card

Raspberry Pi:
- autoconnects to laptop hotspot
- connected via UART to pixhawk (runs PX4 uORB to ROS topics middleware)
- also connects USB-A to USB-C to pixhawk (runs py mavproxy to we can connect to QGC through UDP over hotspot)



STEPS:
1. make sure pixhawks are running middleware client
```
uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600 -n <namespace>
```
write to /etc/extras.txt on the pixhawk sd card so that it runs this on startup everything


`uxrce_dds_client stop`
``uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600 -n <namespace>``

the `<namespace>` is prepended to the bridged topics. For drone 1, namespace should be `px4_1`

2.


MicroXRCEAgent serial --dev /dev/serial0 -b 921600

mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 --out udp:10.42.0.1:14550
mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 --out udp:<LAPTOP_IP>:14550
