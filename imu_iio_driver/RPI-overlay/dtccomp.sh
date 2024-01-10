#!/bin/bash
dtc -@ -I dts -O dtb -o epsonImu.dtbo epsonImu-overylay.dts
sudo cp epsonImu.dtbo /boot/overlays/.
sudo cp epsonImu.dtbo /boot/`uname -r`/overlays/.
