# Raspberry Pi Runtime Export

Runtime files for v0.4.2-pre.1.

Main launcher:

~/pfe_raspi_ws/run_raspi_runtime.sh

Main ROS topics:

- /raspi/link_latency_ms
- /system/log
- /force/newton
- /force/raw
- /force/voltage
- /force/limit_n
- /force/log
- /camera/log

Camera stream:

- http://100.112.77.4:8081/stream.mjpg
- http://100.112.77.4:8081/snapshot.jpg

Camera profile:

- OV5647 CSI
- 640x480
- RAW Bayer GB10
- target 20 FPS
- JPEG quality 55
