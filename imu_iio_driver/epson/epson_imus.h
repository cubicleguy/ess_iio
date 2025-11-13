//-----------------------------------------------------------------------------
// linux/drivers/iio/imu/epson/epson_imus.h
//
// This is the header for the Epson IMU driver
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

#ifndef __IIO_EPSON_IMUS_H__
#define __IIO_EPSON_IMUS_H__

#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/iio/types.h>
#include "epson_imu_reg.h"      //IMU register specific information

#define EPSON_DRV_VERSION "1.2.0"

#define EPSON_WRITE_REG(reg)    ((0x80 | ((reg) & 0x7F)))
#define EPSON_READ_REG(reg)     ((reg) & 0x7f)
#define EPSON_PAGE_SIZE         0x80
#define WIN_CTRL                0x7E
#define EPSON_BITS_PER_WORD     8

struct epson;


//----------------------------------------------------------------------
// struct epson_data - Epson chip variant specific data
//  - read_delay: SPI delay for read operations in us
//  - write_delay: SPI delay for write operations in us
//  - mode_ctrl_reg: Register address of the MODE_CRTL register
//  - msc_ctrl_reg: Register address of the MSC_CTRL register
//  - diag_stat_reg: Register address of the DIAG_STAT register
//  - glob_cmd_reg: Register address of the GLOB_CMD register
//  - status_error_msgs: Array of error messages
//  - status_error_mask: mask of the status errors
//  - enable_irq(): pointer to the enable_irq() function
//  - has_paging: indicates if the IMU uses register windows
//
//----------------------------------------------------------------------
struct epson_data {
    unsigned int read_delay;
    unsigned int write_delay;

    unsigned int mode_ctrl_reg;
    unsigned int msc_ctrl_reg;
    unsigned int diag_stat_reg;
    unsigned int glob_cmd_reg;

    unsigned int startup_delay;

    const char* const* status_error_msgs;
    unsigned int status_error_mask;

    int (*enable_irq)(struct epson* epson, bool enable);

    bool has_paging;
};


//----------------------------------------------------------------------
// struct epson - Epson device structure
//  - spi: SPI delay for read operations in us
//  - trig: SPI delay for write operations in us
//  - data: Register address of the MSC_CTRL register
//  - spi_max_freq: maximum allowed SPI value
//  - state_lock: Register address of the DIAG_STAT register
//  - msg: Register address of the GLOB_CMD register
//  - xfer: Array of error messages
//  - current_window: mask of the status errors
//  - reset_pin: pointer to the enable_irq() function
//  - buffer: indicates if the IMU uses register windows
//  - tx: byte array used for spi tx messages
//  - rx: byte array used for register reads
//
//----------------------------------------------------------------------
struct epson {
    struct spi_device* spi;
    struct iio_trigger* trig;

    const struct epson_data* data;
    uint32_t spi_max_freq;

    struct mutex state_lock;
    struct spi_message msg;
    struct spi_transfer* xfer;
    unsigned int current_window;
    int reset_pin;
    int proto;
    void* buffer;

    uint8_t tx[10] ____cacheline_aligned;
    uint8_t rx[4];
};

int epson_init(struct epson* epson, struct iio_dev* indio_dev,
    struct spi_device* spi, const struct epson_data* data);
int epson_reset(struct epson* epson);
int epson_write_reg(struct epson* epson, unsigned int reg,
    unsigned int val, unsigned int size);
int epson_read_reg(struct epson* epson, unsigned int reg,
    unsigned int* val, unsigned int size);



//----------------------------------------------------------------------
// epson_write_reg_8()
// Inline function to write single byte to a register
//
// Parameters:
// - epson: pointer to the Epson device structure
// - reg: the address of the register
// - val: the value to write
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static inline int epson_write_reg_8(struct epson* epson, unsigned int reg,
    uint8_t val)
{
    return epson_write_reg(epson, reg, val, 1);
}



//----------------------------------------------------------------------
// epson_write_reg_16()
// Inline function to write 2 bytes to a pair of registers
//
// Parameters:
// - epson: pointer to the Epson device structure
// - reg: the address of the lower register
// - val: the value to write
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static inline int epson_write_reg_16(struct epson* epson, unsigned int reg,
    uint16_t val)
{
    return epson_write_reg(epson, reg, val, 2);
}



//----------------------------------------------------------------------
// epson_write_reg_32()
// Inline function to write 4 bytes to a pair of registers
//
// Parameters:
// - epson: pointer to the Epson device structure
// - reg: the starting address of the registers
// - val: the value to write
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static inline int epson_write_reg_32(struct epson* epson, unsigned int reg,
    uint32_t val)
{
    return epson_write_reg(epson, reg, val, 4);
}



//----------------------------------------------------------------------
// epson_read_reg_16()
// Inline function to read 2 bytes from a 16-bit register
//
// Parameters:
// - epson: pointer to the Epson device structure
// - reg: the address of the register
// - val: the value read from the register
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static inline int epson_read_reg_16(struct epson* epson, unsigned int reg,
    uint16_t* val)
{
    unsigned int tmp;
    int ret;

    ret = epson_read_reg(epson, reg, &tmp, 2);
    *val = tmp;

    return ret;
}



//----------------------------------------------------------------------
// epson_read_reg_32()
// Inline function to read 4 bytes from a 32-bit register
//
// Parameters:
// - epson: pointer to the Epson device structure
// - reg: the address of the register
// - val: the value read from the register
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static inline int epson_read_reg_32(struct epson* epson, unsigned int reg,
    uint32_t* val)
{
    unsigned int tmp;
    int ret;

    ret = epson_read_reg(epson, reg, &tmp, 4);
    *val = tmp;

    return ret;
}

// Function Prototypes
int epson_enable_irq(struct epson* epson, bool enable);
int epson_check_status(struct epson* epson);
int epson_initial_startup(struct epson* epson);
int epson_single_conversion(struct iio_dev* indio_dev,
    const struct iio_chan_spec* chan, unsigned int error_mask, int* val);

#ifdef CONFIG_IIO_EPSON_LIB_BUFFER
int epson_setup_buffer_and_trigger(struct epson* epson,
    struct iio_dev* indio_dev,
    const struct iio_buffer_setup_ops* ops,
    irqreturn_t(*trigger_handler)(int, void*));
void epson_cleanup_buffer_and_trigger(struct epson* epson,
    struct iio_dev* indio_dev);
int epson_probe_trigger(struct epson* epson,
    struct iio_dev* indio_dev);
void epson_remove_trigger(struct epson* epson);

int epsonimu_update_scan_mode(struct iio_dev* indio_dev,
    const unsigned long* scan_mask);
#else  /* CONFIG_IIO_BUFFER */
static inline int epson_setup_buffer_and_trigger(struct epson* epson,
    struct iio_dev* indio_dev,
    const struct iio_buffer_setup_ops* ops,
    irqreturn_t(*trigger_handler)(int, void*))
{
    return 0;
}

static inline void epson_cleanup_buffer_and_trigger(struct epson* epson,
    struct iio_dev* indio_dev)
{
}

static inline int epson_probe_trigger(struct epson* epson,
    struct iio_dev* indio_dev)
{
    return 0;
}

static inline void epson_remove_trigger(struct epson* epson)
{
}

#define epsonimu_update_scan_mode           NULL

#endif /* CONFIG_IIO_BUFFER */

#ifdef CONFIG_DEBUG_FS
int epson_debugfs_reg_access(struct iio_dev* indio_dev,
    unsigned int reg, unsigned int writeval, unsigned int* readval);
#else
#define epson_debugfs_reg_access NULL
#endif

struct epsonimu_state;

#define O_SHOW_AVAIL    1
#define O_FREQ_LIST1    2
#define O_FREQ_LIST2    4


struct epson_lpf_3db_freq {
    uint8_t   reg_bits;
    const char *label;   /* "tap32", "tap32fc200", etc */
    int  taps;
    int  cutoff_hz;      /* 0 means N/A */
    int  options;
};


struct epson_accel_scale {
    uint8_t reg_bits;
    const char *label;   /* Range "8G", "16G", etc */
    int val;
    int val2;
};


struct epson_sample_freq {
    uint8_t reg_bits;
    int hz;
    int micro_hz;
};


struct epson_register_map {
    const char *label;
    uint16_t addr;
    const char *rw;
    const char *default_value;
};



//----------------------------------------------------------------------
// struct epsonimu_chip_info - type specific information
//  - product_id: IMU id
//  - channels: the IMU channels supported
//  - num_channels: the number of scan channels
//  - flags: the flags describing the features
//  - gyro_scale_micro: gyro scale factor
//  - accel_scale_micro: accelerometer scale factor
//  - temp_scale_nano: temperature scale factor
//  - temp_offset: temperature offset
//  - sample_freq_mode: pointer to sample frequency modes
//  - sample_freq_mode_cnt: counter of sample frequency modes
//  - accel_scale_mode: pointer to accel scale modes
//  - accel_scale_mode_cnt: counter of accel scale modes
//  - lpf_3db_freq_mode: pointer to filter modes
//  - lpf_3db_freq_mode_cnt: counter of filter modes
//  - *set_lpf_3db_freq: pointer to set filter function
//  - *get_lpf_3db_freq: pointer to get filter function
//  - *reg_map_window_0: pointer to reg dump W0 data
//  - *reg_map_window_1: pointer to reg dump W1 data
//
//----------------------------------------------------------------------
#define EPSON_FILTER_MAX_LIST_SIZE   20
#define EPSON_SCALE_MAX_LIST_SIZE     5
#define EPSON_DOUT_MAX_LIST_SIZE     20

struct epsonimu_chip_info {
    const char* product_id;
    const struct iio_chan_spec* channels;
    const int num_channels;
    const long flags;
    unsigned int gyro_scale_micro;
    unsigned int accel_scale_micro;
    int temp_scale_nano;
    int temp_offset;
    const struct epson_sample_freq *sample_freq_mode;
    size_t sample_freq_mode_cnt;
    const struct epson_accel_scale *accel_scale_mode;
    size_t accel_scale_mode_cnt;
    const struct epson_lpf_3db_freq *lpf_3db_freq_mode;
    size_t lpf_3db_freq_mode_cnt;
    int (*set_lpf_3db_freq)(struct epsonimu_state* st, int tap, int fc);
    int (*get_lpf_3db_freq)(struct epsonimu_state* st, int* tap, int* fc);
    const struct epson_register_map* reg_map_window_0;
    const struct epson_register_map* reg_map_window_1;
};



//----------------------------------------------------------------------
// struct epsonimu_state - structure for state of IMU
//  - variant: the type specific information of IMU
//  - epson: Epson IMU device
//  - avail_scan_mask: possible scan masks
//
//----------------------------------------------------------------------
struct epsonimu_state {
    struct epsonimu_chip_info* variant;
    struct epson epson;
    unsigned long avail_scan_mask[2];
    int sample_freq_avail[EPSON_DOUT_MAX_LIST_SIZE * 2];
    int accel_scale_avail[EPSON_SCALE_MAX_LIST_SIZE * 2];
    int lpf_3db_freq_avail[EPSON_FILTER_MAX_LIST_SIZE * 2];
    int lpf_3db_freq_ix;
    int sample_freq_ix;
};

//enumeration of the the scan channels
enum {
    SCAN_TEMP,
    SCAN_GYRO_X,
    SCAN_GYRO_Y,
    SCAN_GYRO_Z,
    SCAN_ACC_X,
    SCAN_ACC_Y,
    SCAN_ACC_Z,
    SCAN_COUNT,
    SCAN_TIMESTAMP,
};
#endif  //end of __IIO_EPSON_IMUS_H__
