-------------------------------------------------------------------------
Epson IMU IIO Linux Device Driver and Streamer v1.30 - October 10, 2025
-------------------------------------------------------------------------


REQUIREMENTS
============
1. A supported Epson IMU device (
		G320PDG0	G330PDG0 	G354PDH0	G355QDG0
        G364PDC0	G364PDCA	G365PDC1    G365PDF1	G366PDG0
        G370PDF1 	G370PDS0	G370PDG0	G370PDT0)
		
2. Raspberry Pi (tested with Model 3B+) with Raspberry Pi OS (tested with Bookworm 32-bit and 64-bit)
3. microSD Card (minimum 8GB)


IMU CONNECTION DETAILS
======================
The Epson IMU device must be connected to the Raspberry Pi board through SPI as follows (tested with
Raspberry Pi Model 3B+:

Raspberry Pi 3B+                                Epson IMU
----------------                                ---------
J8.01 (+3V3)                                    +3V3
J8.15 (GPIO22)(GPIO_GEN3)                       DRDY [GPIO1]
J8.18 (GPIO24)(GPIO_GEN5)                       #RST 
J8.19 (GPIO10)(SPI_MOSI)                        SDI 
J8.21 (GPIO09)(SPI_MISO)                        SDO
J8.23 (GPIO11)(SPI_SCLK)                        SCK
J8.24 (GPIO08)(SPI_CE0_N)                       CS
J8.39 (GND)                                     GND


PREPARE SD CARD
===============
1. Prepare the microSD card using the Raspberry Pi Imager utility: https://www.raspberrypi.com/software/
   The suggested release of Raspberry Pi OS is Bookworm (either 32-bit or 64-bit).
2. If the Raspberry Pi will be accessed via a network, ensure to enable ssh during configuration.   
3. Once the Raspberry Pi OS image is written, copy the file epson_iio_v1.30.tar.gz to the microSD card in
   the boot partition (bootfs).


COMPLETE RPI OS INSTALLATION
============================
1. Insert the SD card into a Raspberry Pi board and turn power on. Note that the first boot may take a while.
2. Complete installation of RPi OS as required (change your localization, time zone, password).
3. Perform an update and restart.


PROJECT DIRECTORY
=================
1. Untar the file epson_iio_v1.30.tar.gz into a projects directory (i.e. /home/pi/projects):
    
       mkdir ~/projects
       tar -xf /boot/firmware/epson_iio_v1.30.tar.gz -C ~/projects/
       cd projects/


CLONE RPI LINUX KERNEL
======================
Setup the system with the tools to perform kernel building and then get the kernel source tree. For more details
see, https://www.raspberrypi.org/documentation/linux/kernel/building.md)

1. Install Git and the build dependencies:
       sudo apt-get install -y build-essential bc bison flex libssl-dev make libncurses-dev
2. Get the kernel sources. This step will take some time:
       git clone --depth=1 --branch=rpi-6.12.y https://github.com/raspberrypi/linux

Note: imu_iio_driver and epson-iiostream were tested with kernel 6.12.34+rpt-rpi-v7 (x32) and 6.12.34+rpt-rpi-v8 (x64),
      but may work with other kernels as well.


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
WARNING: These instructions are for kernel configuration of Raspberry Pi 3, and Pi 3+ running the 32‑bit/64-bit Raspberry Pi OS (Bookworm).
For other models or OS versions, see the official Raspberry Pi Kernel Building Guide at
https://www.raspberrypi.org/documentation/linux/kernel/building.md

1. Make default build configuration according to the Raspberry Pi version:
	As an example for Raspberry Pi 3, Pi 3+:
 
        cd linux/
        KERNEL=kernel7
        make bcm2709_defconfig

2. In the folder ~/projects/linux/ run the command

        make menuconfig
        Select
                Device Drivers --->
        Select
                Industrial I/O support --->
        Select
                Inertial measurement units --->
        Select
                Epson IMU Support --->

        The Epson IMU specific menu allows selecting various options such as:

        <M> Epson IMUs SPI driver
        [ ]   32-bit Sample Size

3. Save and exit.


ADJUST YOUR LOCALVERSION
========================
1. Open .config in the folder ~/projects/linux/ and change the default LOCALVERSION to identify the new build.
   The following example adds 'e' to the end of the version using the command "nano .config" (see below)       
       #CONFIG_LOCALVERSION="-v7"
       CONFIG_LOCALVERSION="-v7e"


BUILD KERNEL AND DRIVER
=======================
1. Run the following commands from the ~/projects/linux/ folder:
       make -j6 zImage modules dtbs
       sudo make -j6 modules_install

2. After modules_install completed the last line reports kernel version compiled: 
       DEPMOD  /lib/modules/6.12.41-v7e+

3. Create a backup of the current kernel and install the new kernel image using the following commands:
       sudo cp /boot/firmware/$KERNEL.img /boot/firmware/$KERNEL-backup.img
       sudo cp arch/arm/boot/zImage /boot/firmware/$KERNEL.img
   
4. Copy the overlays and README file:
       sudo cp arch/arm/boot/dts/broadcom/*.dtb /boot/firmware/
       sudo cp arch/arm/boot/dts/overlays/*.dtb* /boot/firmware/overlays/
       sudo cp arch/arm/boot/dts/overlays/README /boot/firmware/overlays/

5. Restart the Raspberry Pi
       sudo reboot

6. After rebooting is complete, ensure the new kernel is running:
       uname -r
       6.12.41-v7e+


COMPILE DEVICE TREE SOURCE AND LOAD THE OVERLAY
===============================================
1. Go to the folder imu_iio_driver/RPI-overlay:
       cd ~/projects/imu_iio_driver/RPI-overlay/

2. Compile epson-imu-overlay.dts (edit if needed) and copy the output to the boot overlays using either:
   a) the following commands:
       dtc -@ -I dts -O dtb -o epson-imu.dtbo epson-imu-overlay.dts
       sudo cp epson-imu.dtbo /boot/firmware/overlays/

   b) or the following script:    
       ./dtccomp.sh

3. Load the overlay by using either: 
   a) the following command:
       sudo dtoverlay -v epson-imu

   b) or the following script:
       ./load.sh


LIBIIO
======
The Epson IMU driver requires the LIBIIO package to be installed. Issue the following commands to make and install LIBIIO
For more details on LIBIIO, see https://wiki.analog.com/resources/tools-software/linux-software/libiio

1. Install the dependencies using the following command:
       sudo apt-get install libxml2 libxml2-dev bison flex libcdk5-dev cmake libaio-dev libusb-1.0-0-dev libserialport-dev libxml2-dev libavahi-client-dev doxygen graphviz libzstd-dev 

2. Go to the projects folder. Clone, make, and install libini using the following commands:
       cd ~/projects/
       git clone https://github.com/pcercuei/libini.git
       cd libini
       mkdir build && cd build && cmake ../ && make && sudo make install

3. Go to the projects folder. Clone, make, and install libiio using the following commands:
       cd ../..
       git clone https://github.com/analogdevicesinc/libiio.git
       cd libiio/
       cmake ./
       make all
       sudo make install


EPSON-IIOSTREAM
===============

1. Install dependencies:
       sudo apt-get install libiio-dev

2. Go to your projects directory:
       cd ~/projects

3. Copy the "epson-iiostream" folder to "libiio":
       cp -r epson-iiostream libiio

4. Go to your projects/libiio directory and build epson-iiostream:
       cd libiio
       mkdir epson_build
       cd epson_build
       cmake ../epson-iiostream
       cmake --build .

5. Run epson-iiostream command examples (Ctrl+c to exit):
       sudo ./epson-iiostream
       sudo ./epson-iiostream -d iio:device0 -t trigger0 -c 10
       sudo ./epson-iiostream -d epsonG354PDH0 -t epsonG354PDH0-dev0

	Note that the above example assumes an Epson G354PDH0 is connected.

6. Run epson-iiostream through iiod
   a) Run iiod on the Raspberry Pi:
       	sudo iiod 

   b) Run epson-iiostream over the network from another computer
      The example below assumes the Raspberry Pi’s IP address is 192.168.1.127:
      	sudo ./epson-iiostream -n 192.168.1.127


ADI IIO OSCILLOSCOPE 
====================
The following commands describe how to run the ADI IIO OSCILLOSCOPE application. For more details, see
https://trupples.github.io/adi-documentation/software/iio-oscilloscope/
1. Run iiod on RPI:
   		sudo iiod 

2. Run ADI IIO OSCILLOSCOPE on a Linux or Windows PC and Select to Manual and type in "URI:" box
           ip:192.168.1.127
3. Click on the "Refresh" button
4. Click on the "Connect" button
5. Select the device in the Device Selection area. For example, an epsonG365PDF1.
6. Check or change the filter and sample rate settings as needed in this window.
7. Switch to another window "Capture". Enables Plot Channels and adjust "Samples" buffer value for the chosen Device's Sample Rate (approximately "Sample Rate" / 2)
8. Click on "Capture/Stop" button.


SYSFS INTERFACE
===============
The SYSFS interface can be used to read and write the device configuration from the command line.
1. type: sudo -i
       
The following examples can be performed:
2. Shows available settings for filter_low_pass_3db_frequency_available:
		cat /sys/bus/iio/devices/iio\:device0/filter_low_pass_3db_frequency_available 
		0.000000 2.000000 4.000000 8.000000 16.000000 32.000000 64.000000 128.000000 32.000050 32.000100 32.000200 32.000400 64.000050 64.000100 64.000200 64.000400 128.000050 128.000100 128.000200 128.000400

3. Set filter_low_pass_3db_frequency to 32.000400 (tap32fc400):
		echo 32.000400 > /sys/bus/iio/devices/iio\:device0/filter_low_pass_3db_frequency

4. Get filter_low_pass_3db_frequency to 32.000400 (tap32fc400):
		cat /sys/bus/iio/devices/iio\:device0/filter_low_pass_3db_frequency
        32.000400
           
5. Get product id:
		cat /sys/bus/iio/devices/iio\:device0/product_id
        G355QDG0
           
 
DEBUGFS INTERFACE
=================
The DEBUGFS interface can be used to dump all the IMU register values
		cat /sys/kernel/debug/iio/iio\:device0/regdump

The following list shows a sample output of register settings.

Window 0
========
Reg Value----|Reg Name--------------|R/W|Dflt
REG[02]=0400 :MODE_CTRL              R/W 0400
REG[04]=0000 :DIAG_STAT               R  0000
REG[06]=0000 :FLAG                    R  0000
REG[08]=0000 :GPIO                   R/W 0200
REG[0a]=0000 :COUNT                   R  0000
REG[0c]=0000 :RANGE_OVER              R  0000
REG[0e]=ffff :TEMP_HIGH               R  FFFF
REG[10]=ffff :TEMP_LOW                R  FFFF
REG[12]=ffff :XGYRO_HIGH              R  FFFF
REG[14]=ffff :XGYRO_LOW               R  FFFF
REG[16]=ffff :YGYRO_HIGH              R  FFFF
REG[18]=ffff :YGYRO_LOW               R  FFFF
REG[1a]=ffff :ZGYRO_HIGH              R  FFFF
REG[1c]=ffff :ZGYRO_LOW               R  FFFF
REG[1e]=ffff :XACCL_HIGH              R  FFFF
REG[20]=ffff :XACCL_LOW               R  FFFF
REG[22]=ffff :YACCL_HIGH              R  FFFF
REG[24]=ffff :YACCL_LOW               R  FFFF
REG[26]=ffff :ZACCL_HIGH              R  FFFF
REG[28]=ffff :ZACCL_LOW               R  FFFF
REG[4c]=5345 :ID                      R  5345
REG[50]=0000 :QTN0_HIGH               R  0000
REG[52]=0000 :QTN0_LOW                R  0000
REG[54]=0000 :QTN1_HIGH               R  0000
REG[56]=0000 :QTN1_LOW                R  0000
REG[58]=0000 :QTN2_HIGH               R  0000
REG[5a]=0000 :QTN2_LOW                R  0000
REG[5c]=0000 :QTN3_HIGH               R  0000
REG[5e]=0000 :QTN3_LOW                R  0000
REG[64]=0000 :XDLTA_HIGH / ANG1_HIGH  R  0000
REG[66]=0000 :XDLTA_LOW  / ANG1_LOW   R  0000
REG[68]=0000 :YDLTA_HIGH / ANG2_HIGH  R  0000
REG[6a]=0000 :YDLTA_LOW  / ANG2_LOW   R  0000
REG[6c]=0000 :ZDLTA_HIGH / ANG3_HIGH  R  0000
REG[6e]=0000 :ZDLTA_LOW  / ANG3_LOW   R  0000
REG[70]=0000 :XDLTV_HIGH              R  0000
REG[72]=0000 :XDLTV_LOW               R  0000
REG[74]=0000 :YDLTV_HIGH              R  0000
REG[76]=0000 :YDLTV_LOW               R  0000
REG[78]=0000 :ZDLTV_HIGH              R  0000
REG[7a]=0000 :ZDLTV_LOW               R  0000
REG[7e]=0001 :WIN_CTRL               R/W 0000

Window 1
========
Reg Value----|Reg Name--------------|R/W|Dflt
REG[00]=fefc :SIG_CTRL               R/W FE00
REG[02]=0006 :MSC_CTRL               R/W 0006
REG[04]=0403 :SMPL_CTRL              R/W 0103
REG[06]=0000 :FILTER_CTRL            R/W 0001
REG[08]=0000 :UART_CTRL              R/W 0000
REG[0a]=0000 :GLOB_CMD               R/W 0000
REG[0c]=f003 :BURST_CTRL1            R/W F006
REG[0e]=0000 :BURST_CTRL2            R/W 0000
REG[10]=0000 :POL_CTRL               R/W 0000
REG[12]=00cc :DLT_CTRL               R/W 00CC
REG[14]=0000 :ATTI_CTRL              R/W 0000
REG[16]=0000 :GLOB_CMD2              R/W 0000
REG[38]=4000 :R_MATRIX_G_M11         R/W 4000
REG[3a]=0000 :R_MATRIX_G_M12         R/W 0000
REG[3c]=0000 :R_MATRIX_G_M13         R/W 0000
REG[3e]=0000 :R_MATRIX_G_M21         R/W 0000
REG[40]=4000 :R_MATRIX_G_M22         R/W 4000
REG[42]=0000 :R_MATRIX_G_M23         R/W 0000
REG[44]=0000 :R_MATRIX_G_M31         R/W 0000
REG[46]=0000 :R_MATRIX_G_M32         R/W 0000
REG[48]=4000 :R_MATRIX_G_M33         R/W 4000
REG[4a]=4000 :R_MATRIX_A_M11         R/W 4000
REG[4c]=0000 :R_MATRIX_A_M12         R/W 0000
REG[4e]=0000 :R_MATRIX_A_M13         R/W 0000
REG[50]=0000 :R_MATRIX_A_M21         R/W 0000
REG[52]=4000 :R_MATRIX_A_M22         R/W 4000
REG[54]=0000 :R_MATRIX_A_M23         R/W 0000
REG[56]=0000 :R_MATRIX_A_M31         R/W 0000
REG[58]=0000 :R_MATRIX_A_M32         R/W 0000
REG[5a]=4000 :R_MATRIX_A_M33         R/W 4000
REG[6a]=3347 :PROD_ID1                R  
REG[6c]=3536 :PROD_ID2                R  
REG[6e]=4450 :PROD_ID3                R  
REG[70]=3146 :PROD_ID4                R  
REG[72]=2816 :VERSION                 R  
REG[74]=3031 :SERIAL_NUM1             R  
REG[76]=3030 :SERIAL_NUM2             R  
REG[78]=3233 :SERIAL_NUM3             R  
REG[7a]=3338 :SERIAL_NUM4             R  
REG[7e]=0001 :WIN_CTRL               R/W 

Dump size:3897, Buffer size:8192


Revision History
================
v1.30 - October 10, 2025
New features added to the IIO Stream tool
Additional IMU devices supported

v1.20 - January 25, 2022
Bug fixes and new features added to the IIO Stream tool

v1.10 - August 13, 2021
Added support for additional IMU devices

v1.00 - December 17, 2020
Initial version of IIO Stream and Linux IIO Driver




