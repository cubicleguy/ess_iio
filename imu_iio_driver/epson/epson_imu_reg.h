//-----------------------------------------------------------------------------
// linux/drivers/iio/imu/epson/epson_imu_reg.h
//
// This is the header for the Epson IMU register specific information.
//
// Copyright(C) SEIKO EPSON CORPORATION 2017,2025. All rights reserved.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <https://www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------

#ifndef __EPSON_IMU_REG_H__
#define __EPSON_IMU_REG_H__

#define EPSON_STALL             20  // Microseconds, TREADRATE = 40 us min, TCYCLERATE @ 0.5MHz = 32 uS,
									// STALL must be atleast (40us - 32us) or 20uS which ever is GREATER (20uS)
#define BURST_STALL1            45  // Microseconds
#define BURST_STALL2            4   // Microseconds
#define DELAY_EPSON_RESET       100 // Milliseconds Reset Pulse Width
#define EPSON_POWER_ON_DELAY    800 // Milliseconds
#define EPSON_FLASH_TEST_DELAY  5   // Milliseconds
#define EPSON_SELF_TEST_DELAY   40  // Milliseconds
#define EPSON_READRATE          40  // Microseconds

#define SPI_MAX (uint32_t)(1000 * 1000)


// SPI Commands Overview
// - defines starting with REG_ are register address values sent at the
//   beginning of a SPI transfer to select a register to read/write
// - defines starting with CMD_R_ values are initial SPI transfers to select a register to read from;
//   another 8 bits of zeroes must be sent before receiving 16 bit response
// - Other CMD values are the write values to configure certain registers
// - NOTE: Register Address Map depends on the active page

// Register Accesses:
// User registers are 8-bit for writes, and 16-bit for reads. The register address
// is 8-bit with the lower 7 bits containing the address, and bit 8 indicating
// whether a read or a write access will be performed. For example, writing to
// REG_MSC_CTRL_LO (W1) = 0x0102
// Set the Register Window to 1 as indicated by the 0x0100 value(upper byte) and the send Addr 0x82
// with the 0x80 indicating we are writing to the register.

// WINDOW_ID 0, first byte indicates the Register Window
#define REG_MODE_CTRL_LO    0x0002  // MODE_CTRL Byte0 (W0)
#define REG_MODE_CTRL_HI    0x0003  // MODE_CTRL Byte1 (W0)
#define REG_DIAG_STAT       0x0004  // DIAG_STAT Byte0 (W0)
#define REG_FLAG            0x0006  // FLAG(ND/EA) (W0)
#define REG_GPIO            0x0008  // GPIO  (W0)
#define REG_COUNT           0x000A  // COUNT (W0)
#define REG_TEMP_HIGH       0x000E  // TEMPC HIGH (W0)
#define REG_TEMP_LOW        0x0010  // TEMPC LOW  (W0)
#define REG_XGYRO_HIGH      0x0012  // XGYRO HIGH (W0)
#define REG_XGYRO_LOW       0x0014  // XGYRO LOW  (W0)
#define REG_YGYRO_HIGH      0x0016  // YGYRO HIGH (W0)
#define REG_YGYRO_LOW       0x0018  // YGYRO LOW  (W0)
#define REG_ZGYRO_HIGH      0x001A  // ZGYRO HIGH (W0)
#define REG_ZGYRO_LOW       0x001C  // ZGYRO LOW  (W0)
#define REG_XACCL_HIGH      0x001E  // XACCL HIGH (W0)
#define REG_XACCL_LOW       0x0020  // XACCL LOW  (W0)
#define REG_YACCL_HIGH      0x0022  // YACCL HIGH (W0)
#define REG_YACCL_LOW       0x0024  // YACCL LOW  (W0)
#define REG_ZACCL_HIGH      0x0026  // ZACCL HIGH (W0)
#define REG_ZACCL_LOW       0x0028  // ZACCL LOW  (W0)

#define REG_XDLTA_HIGH      0x0064  // XDLTA HIGH (W0)
#define REG_XDLTA_LOW       0x0066  // XDLTA LOW  (W0)
#define REG_YDLTA_HIGH      0x0068  // YDLTA HIGH (W0)
#define REG_YDLTA_LOW       0x006A  // YDLTA LOW  (W0)
#define REG_ZDLTA_HIGH      0x006C  // ZDLTA HIGH (W0)
#define REG_ZDLTA_LOW       0x006E  // ZDLTA LOW  (W0)
#define REG_XDLTV_HIGH      0x0070  // XDLTV HIGH (W0)
#define REG_XDLTV_LOW       0x0072  // XDLTV LOW  (W0)
#define REG_YDLTV_HIGH      0x0074  // YDLTV HIGH (W0)
#define REG_YDLTV_LOW       0x0076  // YDLTV LOW  (W0)
#define REG_ZDLTV_HIGH      0x0078  // ZDLTV HIGH (W0)
#define REG_ZDLTV_LOW       0x007A  // ZDLTV LOW  (W0)

// WINDOW_ID 1
#define REG_SIG_CTRL_LO     0x0100  // SIG_CTRL Byte0 (W1)
#define REG_SIG_CTRL_HI     0x0101  // SIG_CTRL Byte1 (W1)
#define REG_MSC_CTRL_LO     0x0102  // MSC_CTRL Byte0 (W1)
#define REG_MSC_CTRL_HI     0x0103  // MSC_CTRL Byte1 (W1)
#define REG_SMPL_CTRL_LO    0x0104  // SMPL_CTRL Byte0 (W1)
#define REG_SMPL_CTRL_HI    0x0105  // SMPL_CTRL Byte1 (W1)
#define REG_FILTER_CTRL_LO  0x0106  // FILTER_CTRL Byte0 (W1)
#define REG_FILTER_CTRL_HI  0x0107  // FILTER_CTRL Byte1 (W1)
#define REG_UART_CTRL_LO    0x0108  // UART_CTRL Byte0 (W1)
#define REG_UART_CTRL_HI    0x0109  // UART_CTRL Byte1 (W1)
#define REG_GLOB_CMD_LO     0x010A  // GLOB_CMD Byte0 (W1)
#define REG_GLOB_CMD_HI     0x010B  // GLOB_CMD Byte1 (W1)
#define REG_BURST_CTRL1_LO  0x010C  // BURST_CTRL1 Byte0 (W1)
#define REG_BURST_CTRL1_HI  0x010D  // BURST_CTRL1 Byte1 (W1)
#define REG_BURST_CTRL2_LO  0x010E  // BURST_CTRL2 Byte0 (W1)
#define REG_BURST_CTRL2_HI  0x010F  // BURST_CTRL2 Byte1 (W1)
#define REG_POL_CTRL_LO     0x0110  // POL_CTRL Byte0 (W1)
#define REG_POL_CTRL_HI     0x0111  // POL_CTRL Byte1 (W1)
#define REG_DLT_CTRL_LO     0x0112  // DLT_CTRL Byte0 (W1)
#define REG_DLT_CTRL_HI     0x0113  // DLT_CTRL Byte1 (W1)

#define REG_PROD_ID1        0x016A  // PROD_ID1(W1)
#define REG_PROD_ID2        0x016C  // PROD_ID2(W1)
#define REG_PROD_ID3        0x016E  // PROD_ID3(W1)
#define REG_PROD_ID4        0x0170  // PROD_ID4(W1)
#define REG_VERSION         0x0172  // VERSION(W1)
#define REG_SERIAL_NUM1     0x0174  // SERIAL_NUM1(W1)
#define REG_SERIAL_NUM2     0x0176  // SERIAL_NUM2(W1)
#define REG_SERIAL_NUM3     0x0178  // SERIAL_NUM3(W1)
#define REG_SERIAL_NUM4     0x017A  // SERIAL_NUM4(W1)
#define REG_WIN_CTRL        0x017E  // WIN_CTRL(W0 or W1)

#define CMD_BURST           0x80    // Write value to Issue Burst Read
#define CMD_EN_NDFLAGS      0x7E    // Write value for SIG_CTRL_HI to Enables new data (ND) flags
									// in FLAG for Gyros, Accelerometers
#define CMD_EN_BRSTDATA_LO  0x03    // Write value for BURST_CTRL1_LO to enable CHKSM, and COUNT
									// bytes in burst mode
#define CMD_EN_BRSTDATA_HI  0x30    // Write value for BURST_CTRL1_HI to enable GYRO, and ACCL
									// registers in burst mode (0xB0 for FLAG as well)
#define CMD_WINDOW0         0x00    // Write value for WIN_CTRL to change to Window 0
#define CMD_WINDOW1         0x01    // Write value for WIN_CTRL to change to Window 1
#define CMD_RSTCNTR_DRDY    0x44    // Write value for MSC_CTRL_LO to enable EXT_SEL to Reset
									// counter and active low DRDY on GPIO1

#define CMD_32BIT           0x30    // Write value for BURST_CTRL2_HI to enable 32 bit mode
									// for gyro and accl data
#define CMD_BEGIN_SAMPLING  0x01    // Write value for MODE_CMD_HI to begin sampling
#define CMD_END_SAMPLING    0x02    // Write value for MODE_CMD_HI to stop sampling
#define CMD_SOFTRESET       0x80    // Write value for GLOB_CMD_LO to issue Software Reset
#define CMD_FLASHTEST       0x08    // Write value for MSC_CTRL_HI to issue Flashtest
#define CMD_SELFTEST        0x04    // Write value for MSC_CTRL_HI to issue Selftest


// DIAG_STAT bitfield
#define EPSON_DIAG_STAT_XGYRO_FAIL  BIT(14)
#define EPSON_DIAG_STAT_YGYRO_FAIL  BIT(13)
#define EPSON_DIAG_STAT_ZGYRO_FAIL  BIT(12)
#define EPSON_DIAG_STAT_ACCL_FAIL   BIT(11)
#define EPSON_DIAG_DLTA_OVF         BIT(9)
#define EPSON_DIAG_DLTV_OVF         BIT(8)
#define EPSON_DIAG_HARD_ERR1        BIT(6)
#define EPSON_DIAG_HARD_ERR0        BIT(5)
#define EPSON_DIAG_SPI_OVF          BIT(4)
#define EPSON_DIAG_UART_OVF         BIT(3)
#define EPSON_DIAG_FLASH_ERR        BIT(2)
#define EPSON_DIAG_STAT_SELF_TEST_ERR   BIT(1)
#define EPSON_DIAG_FLASH_BU_ERR     BIT(0)

// MSC_CTRL bitfield
#define EPSON_MSC_CTRL_FLASH_TEST   BIT(11)
#define EPSON_MSC_CTRL_SELF_TEST    BIT(10)
#define EPSON_MSC_CTRL_EXT_SEL1     BIT(7)
#define EPSON_MSC_CTRL_EXT_SEL0     BIT(6)
#define EPSON_MSC_CTRL_EXT_POL      BIT(5)
#define EPSON_MSC_CTRL_DRDY_ON      BIT(2)
#define EPSON_MSC_CTRL_DRDY_POL     BIT(1)

// GLOB_CMD bitfield
#define EPSON_GLOB_CMD_NOT_READY    BIT(10)
#define EPSON_GLOB_CMD_SOFT_RST     BIT(7)
#define EPSON_GLOB_CMD_FLASH_BACKUP BIT(3)

// Chip Flags to show what features are available in each IMU
#define HAS_PROD_ID             BIT(0)
#define NO_BURST                BIT(1)
#define HAS_SLOW_MODE           BIT(2)
#define HAS_SERIAL_NUMBER       BIT(3)
#define BURST_DIAG_STAT         BIT(4)
#define HAS_FW_VER              BIT(5)

#endif /* __EPSON_IMU_REG_H__ */
