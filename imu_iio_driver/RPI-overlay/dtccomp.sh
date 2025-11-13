#!/bin/bash
dtc -@ -I dts -O dtb -o epson-imu.dtbo epson-imu-overlay.dts
sudo cp epson-imu.dtbo /boot/firmware/overlays/

