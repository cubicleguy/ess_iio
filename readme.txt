-----------------------------------------------
Epson IMU IIO Linux Device Driver and Streamer 
-----------------------------------------------
Version 1.20 - January 24, 2022
-----------------------------------------------

PREREQUISITES
=============
1. Epson IMU device, Raspbian Pi and microSD card
2. Epson IMU device connected with Raspberry Pi board through SPI.

Raspbian Pi 3 B+                                Epson IMU
----------------                                ---------
J8.01 (+3V3)                                    +3V3
J8.18 (GPIO24)(GPIO_GEN5)                       DRDY [GPIO1]
J8.15 (GPIO22)(GPIO_GEN3)                       #RST 
J8.19 (GPIO10)(SPI_MOSI)                        SDI 
J8.21 (GPIO09)(SPI_MISO)                        SDO
J8.23 (GPIO11)(SPI_SCLK)                        SCK
J8.24 (GPIO08)(SPI_CE0_N)                       CS
J8.39 (GND)                                     GND


PREPARE SD CARD
===============
1. Image your microSD card with Raspbian Pi by this utility:
   https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/
2. Copy epson_iio_V1.20.tar.gz into the SD card with a new written Raspberry Pi image, in the boot partition.


COMPLETE RPi LINUX OS INSTALLATION
==================================
1. Insert the SD card into a Raspberry Pi board and power it up.
2. Complete installation of RPi OS as it suggested (change your localization, time zone, password and do update) and restart.


PROJECT DIRECTORY
=================
1. Untar epson_iio.tar.gz into your projects directory (i.e. /home/pi/projects):
       mkdir /home/pi/projects       
       tar -xf /boot/epson_iio_V1.20.tar.gz -C projects
       cd projects/


CLONE RPI LINUX KERNEL
======================
(source for more details: https://www.raspberrypi.org/documentation/linux/kernel/building.md)
1. Install Git and the build dependencies:
       sudo apt-get install -y build-essential bc bison flex libssl-dev make libncurses-dev

2. Get the sources, which will take some time:
       git clone --depth=1 --branch=rpi-5.10.y https://github.com/raspberrypi/linux

   *imu_iio_driver and epson-iiostream were tested with kernel 5.10 but may work with newer kernels as well.


INSTALL EPSON IMU_IIO_DRIVER INTO THE BUILD TREE
================================================
1. Copy the folder "epson" into linux directory:
       cp -r imu_iio_driver/epson linux/drivers/iio/imu/

2. Add the following line into the linux/drivers/iio/imu/Kconfig (before "endmenu"):
       source "drivers/iio/imu/epson/Kconfig"

3. Add the following line to the linux/drivers/iio/imu/Makefile:
       obj-y += epson/
   

KERNEL CONFIGURATION
====================
1. Make default build configuration as per your Raspberry Pi version:
   a). Raspberry Pi 1, Pi Zero, Pi Zero W
        cd linux/
        KERNEL=kernel
        make bcmrpi_defconfig   

   b). Raspberry Pi 2, Pi 3, Pi 3+
        cd linux/
        KERNEL=kernel7
        make bcm2709_defconfig

   c). Raspberry Pi 4
        cd linux/
        KERNEL=kernel7l
        make bcm2711_defconfig


2. In the directory /home/pi/projects/linux/ run the command

        make menuconfig

        Select
                Device Drivers --->

        Select
                Industrial I/O support --->

        Select
                Inertial measurement units --->

        Select
                Epson IMU Support --->


        The Epson IMU specific menu allows selecting various options.

        <M> Epson IMUs SPI driver
        [ ]   32-bit Sample Size

2. Save and exit.


ADJUST YOUR LOCALVERSION
========================
1. Open .config in /home/pi/projects/linux/ directory and change the default LOCALVERSION to somewhat you desire.
   In this example 'e' is added (nano .config):       
       #CONFIG_LOCALVERSION="-v7"
       CONFIG_LOCALVERSION="-v7e"


BUILD KERNEL AND DRIVER
=======================
1. Run line by line (after a command completed) in /home/pi/projects/linux/ directory:
       make -j4 zImage modules dtbs 
       sudo make modules_install

2. After modules_install completed the last line reports kernel version compiled: 
       DEPMOD 5.10.81-v7e+

3. Use this version as a folder name for the kernel and drivers installation:
       sudo mkdir /boot/5.10.81-v7e+
       sudo mkdir /boot/5.10.81-v7e+/overlays

4. Copy the kernel and modules:
       sudo cp arch/arm/boot/dts/*.dtb /boot/5.10.81-v7e+/
       sudo cp arch/arm/boot/dts/overlays/*.dtb* /boot/5.10.81-v7e+/overlays/
       sudo cp arch/arm/boot/dts/overlays/README /boot/5.10.81-v7e+/overlays/
       sudo cp arch/arm/boot/zImage /boot/5.10.81-v7e+/$KERNEL.img
       sudo cp /boot/cmdline.txt /boot/5.10.81-v7e+/

5. Point at your custom kernel and modules to boot it from in config.txt file. If something goes 
   wrong with your custom build you can boot from an original one by commenting out the point.
   Add the point to your build into /boot/config.txt at the end (sudo nano /boot/config.txt):
       #point at custom kernel and modules
       os_prefix=5.10.81-v7e+/
       kernel=kernel7.img

   Save it and exit
6. Restart RPi
       sudo reboot

7. After rebooting make sure you did boot with your kernel:
       uname -r
       5.10.81-v7e+


COMPILE DEVICE TREE SOURCE AND LOAD THE OVERLAY
===============================================
1. Go to imu_iio_driver/RPI-overlay directory:
       cd /home/pi/projects/imu_iio_driver/RPI-overlay/

2. Compile epsonImu-overylay.dts (edit it if needed) and copy the output to your boot overlays:
   a). by these commands:
       dtc -@ -I dts -O dtb -o epsonImu.dtbo epsonImu-overylay.dts
       sudo cp epsonImu.dtbo /boot/`uname -r`/overlays/.
       sudo cp epsonImu.dtbo /boot/overlays/.

   b). or by script:    
       ./dtccomp.sh

3. Load the overlay: 
       cd scripts/

   a). by the command:
       sudo dtoverlay -v epsonImu

   b). by the script:
       ./load.sh
4. Make sure the driver loaded by dmesg command:
[  110.435779] epsonimu_probe: IMU driver probe
[  110.435806] epson_imus spi0.0: DRDY is GPIO 24
[  110.435812] iio iio:device0: epson_init(), DRDY uses gpio 24, irq 167
[  110.435823] epson_imus spi0.0: RST is GPIO 22
[  110.435841] epson_imus spi0.0: Spi max freq is 1000000
[  110.436273] iio iio:device0: SPI speed 500000
[  111.369462] iio iio:device0: Product ID epsonG370PDF1
[  111.369478] iio iio:device0: Found a match epsonimu_chips[3]
[  111.369488] iio iio:device0: Model Variant: Compatible with epsonG365PDF0
[  111.500391] epson_imus spi0.0: Self test passed
[  111.598690] iio iio:device0: prod_id epsonG370PDF1 at CS0 (irq 167)
[  111.599100] epson_probe_trigger, irq succeeded



LIBIIO
======
[source for more details: https://wiki.analog.com/resources/tools-software/linux-software/libiio]

1. Install prerequisites:
       sudo apt-get install libxml2 libxml2-dev bison flex libcdk5-dev cmake
       sudo apt-get install libaio-dev libusb-1.0-0-dev libserialport-dev libxml2-dev libavahi-client-dev doxygen graphviz

2. Go to your projects directory, clone libini, make it and install:
       cd /home/pi/projects/
       git clone https://github.com/pcercuei/libini.git
       cd libini
       mkdir build && cd build && cmake ../ && make && sudo make install

3. Go to your projects directory, clone libiio, make it and install:
       cd ../..
       git clone https://github.com/analogdevicesinc/libiio.git
       cd libiio/
       cmake ./
       make all
       sudo make install


EPSON-IIOSTREAM
===============
1. Go to your projects directory:
       cd /home/pi/projects

2. Copy this "epson-iiostream" folder to "libiio":
       cp -r epson-iiostream libiio

3. Go to your projects/libiio directory and build epson-iiostream:
       cd libiio
       mkdir epson_build
       cd epson_build
       cmake ../epson-iiostream
       cmake --build .

4. Run epson-iiostream command examples (Ctrl+c to exit):
       sudo ./epson-iiostream
       sudo ./epson-iiostream -d iio:device0 -t trigger0 -c 10
       sudo ./epson-iiostream -d epsonG354PDH0 -t epsonG354PDH0-dev0



