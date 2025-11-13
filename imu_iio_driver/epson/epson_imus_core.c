//-----------------------------------------------------------------------------
// linux/drivers/iio/imu/epson/epson_imus_core.c
//
// This is the source for the Epson IMU core functions.
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

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/export.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "epson_imus.h"

enum epsonimu_chip_variant {
    EPSON_G320PDG0,
    EPSON_G354PDH0,
    EPSON_G355QDG0,
    EPSON_G364PDC0,
    EPSON_G364PDCA,
    EPSON_G365PDC1,
    EPSON_G365PDF1,
    EPSON_G366PDG0,
    EPSON_G330PDG0,
    EPSON_G370PDF1,
    EPSON_G370PDS0,
    EPSON_G370PDG0,
    EPSON_G370PDT0,
};



//----------------------------------------------------------------------
// epsonimu_get_sample_freq()
// Retrieves the current Epson IMU sampling frequency
//
// Parameters:
// - st: pointer to the state of the Epson device
// - hz: pointer to integer part of the desired sampling frequency in hertz
// - micro_hz: pointer to fractional part of the sampling frequency in microhertz
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epsonimu_get_sample_freq(struct epsonimu_state* st, int* hz, int* micro_hz)
{
    uint16_t t;
    int i;

    int ret = epson_read_reg_16(&st->epson, REG_SMPL_CTRL_LO, &t);
    if (ret < 0)
        return -EINVAL;

    //mask DOUT_RATE
    t = (t & 0x0F00) >> 8;

    for (i = 0; i < st->variant->sample_freq_mode_cnt; i++) {
        if (st->variant->sample_freq_mode[i].reg_bits == t) {
            *hz = st->variant->sample_freq_mode[i].hz;
            *micro_hz = st->variant->sample_freq_mode[i].micro_hz;
            st->sample_freq_ix = i;

            return 0;
        }
    }

    return -EINVAL;
}



//----------------------------------------------------------------------
// epsonimu_set_sample_freq()
// Configures the Epson IMU sampling frequency
// Note: Does not touch the Filter Sel
//
// Parameters:
// - st: pointer to the state of the Epson device
// - hz: integer part of the desired sampling frequency in hertz
// - micro_hz: fractional part of the sampling frequency in microhertz
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epsonimu_set_sample_freq(struct epsonimu_state* st, int hz, int micro_hz)
{
    int i, ret;

    for (i = 0; i < st->variant->sample_freq_mode_cnt; i++) {
        if (hz == st->variant->sample_freq_mode[i].hz &&
            micro_hz == st->variant->sample_freq_mode[i].micro_hz) {

            ret = epson_write_reg_8(&st->epson, REG_SMPL_CTRL_HI,
                            st->variant->sample_freq_mode[i].reg_bits);

            if (ret == 0) {
                st->sample_freq_ix = i;
                return 0;
            }
            break;
        }
    }

    return -EINVAL;
}



//----------------------------------------------------------------------
// epsonimu_set_lpf_3db_freq()
// Configures the filter settings for common Epson IMU 
//
// Parameters:
// - st: pointer to the state of the Epson device
// - tap: number of FIR taps to use
// - fc: 3 dB cutoff frequency for the filter (in Hz)
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epsonimu_set_lpf_3db_freq(struct epsonimu_state* st, int tap, int fc)
{
    uint16_t regval;
    int ret;

    for (int i=0; i < st->variant->lpf_3db_freq_mode_cnt; i++) {

        if (tap == st->variant->lpf_3db_freq_mode[i].taps &&
            fc == st->variant->lpf_3db_freq_mode[i].cutoff_hz) {

            ret = epson_write_reg_8(&st->epson, REG_FILTER_CTRL_LO,
                            st->variant->lpf_3db_freq_mode[i].reg_bits);
            if (ret) {
                return -EINVAL;
            }

            // wait for filter setting complete
            do {
                msleep(10);
                ret = epson_read_reg_16(&st->epson, REG_FILTER_CTRL_LO, &regval);
                if (ret < 0) {
                    return -EINVAL;
                }
            } while (regval & 0x0020);   // mask Filter_stat bit

            if (ret == 0) {
                st->lpf_3db_freq_ix = i;
                return 0;
            }
            break;
        }
    }

    return -EINVAL;
}



//----------------------------------------------------------------------
// epsonimu_get_lpf_3db_freq()
// Retrieves the current filter settings of Epson IMU
//
// Parameters:
// - st: pointer to structure for state of IMU
// - tap: output pointer for the number of FIR taps
// - fc: output pointer for the 3 dB cutoff frequency (in Hz)
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epsonimu_get_lpf_3db_freq(struct epsonimu_state* st, int* tap, int* fc)
{
    uint16_t regval;
    int ret;

    ret = epson_read_reg_16(&st->epson, REG_FILTER_CTRL_LO, &regval);
    if (ret == 0) {
        for (int i=0; i < st->variant->lpf_3db_freq_mode_cnt; i++) {

            if ((regval & 0x001F) == st->variant->lpf_3db_freq_mode[i].reg_bits) {
                *tap  = st->variant->lpf_3db_freq_mode[i].taps;
                *fc = st->variant->lpf_3db_freq_mode[i].cutoff_hz;
                st->lpf_3db_freq_ix = i;
                return 0;
            }
        }
    }

    return -EINVAL;
}



//----------------------------------------------------------------------
// epsonimu_set_lpf_3db_freq_odd()
// Configures the filter settings for Epson IMU G370PDS0
//
// Parameters:
// - st: pointer to the state of the Epson device
// - tap: number of FIR taps to use
// - fc: 3 dB cutoff frequency for the filter (in Hz)
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epsonimu_set_lpf_3db_freq_odd(struct epsonimu_state* st, int tap, int fc)
{
    uint16_t regval;
    int ret;

    const int list1_reg[] = {0x00, 0x08, 0x0B, -1};
    const int list2_reg[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                    0x09, 0x0A, 0x0C, 0x0D, 0x0E, 0x0F, -1};

    int reg_bits = (int)st->variant->sample_freq_mode[st->sample_freq_ix].reg_bits;
    int freq_list = 0;

    for (int i=0; list1_reg[i] >= 0; i++) {
        if (reg_bits == list1_reg[i]) {
            freq_list = O_FREQ_LIST1;
            break;
        }
    }

    if (freq_list == 0) {
        for (int i=0; list2_reg[i] >= 0; i++) {
            if (reg_bits == list2_reg[i]) {
                freq_list = O_FREQ_LIST2;
                break;
            }
        }
    }

    if (freq_list) {
        for (int i=0; i < st->variant->lpf_3db_freq_mode_cnt; i++) {

            if (tap == st->variant->lpf_3db_freq_mode[i].taps &&
                fc == st->variant->lpf_3db_freq_mode[i].cutoff_hz) {
                if (st->variant->lpf_3db_freq_mode[i].options & freq_list) {
                    ret = epson_write_reg_8(&st->epson, REG_FILTER_CTRL_LO,
                                    st->variant->lpf_3db_freq_mode[i].reg_bits);
                    if (ret) {
                        return -EINVAL;
                    }

                    // wait for filter setting complete
                    do {
                        msleep(10);
                        ret = epson_read_reg_16(&st->epson, REG_FILTER_CTRL_LO, &regval);
                        if (ret < 0) {
                            return -EINVAL;
                        }
                    } while (regval & 0x0020);   // mask Filter_stat bit

                    if (ret == 0) {
                        st->lpf_3db_freq_ix = i;
                        return 0;
                    }
                    break;
                }
            }
        }
    }
    return -EINVAL;
}



//----------------------------------------------------------------------
// epsonimu_get_lpf_3db_freq_odd()
// Retrieves the current filter settings of Epson IMU G370PDS0
//
// Parameters:
// - st: pointer to structure for state of IMU
// - tap: output pointer for the number of FIR taps
// - fc: output pointer for the 3 dB cutoff frequency (in Hz)
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epsonimu_get_lpf_3db_freq_odd(struct epsonimu_state* st, int* tap, int* fc)
{
    uint16_t regval;
    int ret;
    const int list1_reg[] = {0x00, 0x08, 0x0B, -1};
    const int list2_reg[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                    0x09, 0x0A, 0x0C, 0x0D, 0x0E, 0x0F, -1};

    int reg_bits = (int)st->variant->sample_freq_mode[st->sample_freq_ix].reg_bits;
    int freq_list = 0;

    for (int i=0; list1_reg[i] >= 0; i++) {
        if (reg_bits == list1_reg[i]) {
            freq_list = O_FREQ_LIST1;
            break;
        }
    }

    if (freq_list == 0) {
        for (int i=0; list2_reg[i] >= 0; i++) {
            if (reg_bits == list2_reg[i]) {
                freq_list = O_FREQ_LIST2;
                break;
            }
        }
    }

    if (freq_list) {
        ret = epson_read_reg_16(&st->epson, REG_FILTER_CTRL_LO, &regval);
        if (ret == 0) {
            for (int i=0; i < st->variant->lpf_3db_freq_mode_cnt; i++) {

                if ((regval & 0x001F) == st->variant->lpf_3db_freq_mode[i].reg_bits) {
                    if (freq_list & st->variant->lpf_3db_freq_mode[i].options) {
                        *tap  = st->variant->lpf_3db_freq_mode[i].taps;
                        *fc = st->variant->lpf_3db_freq_mode[i].cutoff_hz;
                        st->lpf_3db_freq_ix = i;
                        return 0;
                    }
                }
            }
        }
    }

    return -EINVAL;
}



//----------------------------------------------------------------------
// epsonimu_write_raw()
// Performs raw writes to the Epson IMU
//
// Parameters:
// - indio_dev: the IIO device
// - chan: pointer to the channel-specification struct
// - val: primary integer component of the value to write
// - val2: secondary component (for fixed-point fractional values)
// - info: selector for which attribute is being written
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epsonimu_write_raw(struct iio_dev* indio_dev,
    struct iio_chan_spec const* chan, int val, int val2, long info)
{
    struct epsonimu_state* st = iio_priv(indio_dev);
    int ret;

    switch (info) {

    case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
        ret = st->variant->set_lpf_3db_freq(st, val, val2);
        if (ret == 0) {
            dev_info(&indio_dev->dev, "(set)Filter Low Pass 3dB frequency: %s (reg:%xh)\n",
                st->variant->lpf_3db_freq_mode[st->lpf_3db_freq_ix].label,
                st->variant->lpf_3db_freq_mode[st->lpf_3db_freq_ix].reg_bits);
            return 0;
        }
        dev_err(&indio_dev->dev, "Failed to set Filter Low Pass 3dB frequency: %i.%i\n", val, val2);
        return -EINVAL;

    case IIO_CHAN_INFO_SAMP_FREQ:
        ret = epsonimu_set_sample_freq(st, val, val2);
        if (ret == 0) {
            dev_info(&indio_dev->dev, "(set)Sample frequency: %i.%i (reg:%xh)\n",
                st->variant->sample_freq_mode[st->sample_freq_ix].hz,
                st->variant->sample_freq_mode[st->sample_freq_ix].micro_hz,
                st->variant->sample_freq_mode[st->sample_freq_ix].reg_bits );
            return 0;
        }
        dev_err(&indio_dev->dev, "Failed to set Sample frequency: %i.%i\n", val, val2);
        return -EINVAL;

    case IIO_CHAN_INFO_SCALE:
        if (st->variant->accel_scale_mode_cnt > 1) {
            for (int i=0; i < st->variant->accel_scale_mode_cnt; i++) {

                if (val == st->variant->accel_scale_mode[i].val &&
                    val2 == st->variant->accel_scale_mode[i].val2) {

                    ret = epson_write_reg_8(&st->epson, REG_DLT_CTRL_HI, st->variant->accel_scale_mode[i].reg_bits);
                    if (ret == 0) {
                        dev_info(&indio_dev->dev, "(set)Accel Scale Range: %s (reg:%xh)\n", st->variant->accel_scale_mode[i].label,
                        st->variant->accel_scale_mode[i].reg_bits);
                        st->variant->accel_scale_micro = st->variant->accel_scale_mode[i].val2;

                        return 0;
                    }
                    break;
                }
            }
            dev_err(&indio_dev->dev, "Failed to set Accel Scale Range: %i.%i\n", val, val2);
        }
        return -EINVAL;

    default:
        dev_err(&indio_dev->dev, "%s: %li\n\n", __FUNCTION__, info);
        return -EINVAL;
    }
}



//----------------------------------------------------------------------
// epsonimu_read_raw()
// Performs raw reads from the Epson IMU
//
// Parameters:
// - indio_dev: the IIO device instance
// - chan: pointer to the channel-specification struct
// - val:  output parameter for the primary integer value
// - val2: output parameter for the secondary value (e.g. denominator
//            for fractional returns)
// - info: selector for which attribute to access
//
// - Return: one of the IIO_VAL_* codes telling the core how to interpret
//               val/val2 on success, or a negative errno on failure.
//----------------------------------------------------------------------
static int epsonimu_read_raw(struct iio_dev* indio_dev,
    struct iio_chan_spec const* chan, int* val, int* val2, long info)
{
    struct epsonimu_state* st = iio_priv(indio_dev);
    uint16_t val16;
    int ret;

    switch (info) {
    case IIO_CHAN_INFO_RAW:
        ret = epson_single_conversion(indio_dev, chan, 0, val);
        return ret;

    case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
        ret = st->variant->get_lpf_3db_freq(st, val, val2);
        if (ret == 0) {
            dev_info(&indio_dev->dev, "(get)Filter Low Pass 3dB frequency: %s (reg:%xh)\n",
                st->variant->lpf_3db_freq_mode[st->lpf_3db_freq_ix].label,
                st->variant->lpf_3db_freq_mode[st->lpf_3db_freq_ix].reg_bits);
            return IIO_VAL_INT_PLUS_MICRO;
        }
        dev_err(&indio_dev->dev, "Failed to get Filter Low Pass 3dB frequency: %i.%i\n", *val, *val2);
        return -EINVAL;

    case IIO_CHAN_INFO_SCALE:
        switch (chan->type) {
        case IIO_ANGL_VEL:
            *val = 0;
            *val2 = st->variant->gyro_scale_micro;
            return IIO_VAL_INT_PLUS_MICRO;
            
        case IIO_ACCEL:
            if (st->variant->accel_scale_mode_cnt > 1) {
                ret = epson_read_reg_16(&st->epson, REG_DLT_CTRL_LO, &val16);
                if (ret == 0) {
                    for (int i=0; i < st->variant->accel_scale_mode_cnt; i++) {
                        if ((1&(val16>>8)) == st->variant->accel_scale_mode[i].reg_bits) {
                            dev_info(&indio_dev->dev, "(get)Accel Scale Range: %s (reg:%xh)\n",
                            st->variant->accel_scale_mode[i].label,
                            st->variant->accel_scale_mode[i].reg_bits);
                            st->variant->accel_scale_micro = st->variant->accel_scale_mode[i].val2;
                            *val = 0;
                            *val2 = st->variant->accel_scale_micro;
                            return IIO_VAL_INT_PLUS_MICRO;
                        }
                    }
                }
                return -EINVAL;
            }
            *val = 0;
            *val2 = st->variant->accel_scale_micro;
            return IIO_VAL_INT_PLUS_MICRO;

        case IIO_TEMP:
            *val = st->variant->temp_scale_nano / 1000000;
            *val2 = (st->variant->temp_scale_nano % 1000000);
            return IIO_VAL_INT_PLUS_MICRO;
            
        default:
            dev_err(&indio_dev->dev, "%s: chan->type: %i\n", __FUNCTION__, (int)chan->type);
            return -EINVAL;
        }
    case IIO_CHAN_INFO_OFFSET:
        *val = st->variant->temp_offset;
        return IIO_VAL_INT;

    case IIO_CHAN_INFO_SAMP_FREQ:
        ret = epsonimu_get_sample_freq(st, val, val2);
        if (ret == 0) {
            dev_info(&indio_dev->dev, "(get)Sample frequency: %i.%i (reg:%xh)\n",
                st->variant->sample_freq_mode[st->sample_freq_ix].hz,
                st->variant->sample_freq_mode[st->sample_freq_ix].micro_hz,
                st->variant->sample_freq_mode[st->sample_freq_ix].reg_bits);
            return IIO_VAL_INT_PLUS_MICRO;
        }
        dev_err(&indio_dev->dev, "Failed to get Sample frequency: %i.%i\n", *val, *val2);
        return -EINVAL;;

    default:
        dev_err(&indio_dev->dev, "%s: %li\n\n", __FUNCTION__, info);
        return -EINVAL;
    }
}



//----------------------------------------------------------------------
// epsonimu_read_avail()
// Reports the set of allowed values for a channel attribute
//
// Parameters:
// - indio_dev: the IIO device
// - chan: pointer to the channel-specification struct
// - vals: output pointer to an array of permitted integer values
// - type: output selector for how to interpret *vals
// - length: output count of entries in the *vals array
// - mask: selector for which attribute’s list is being requested
//
// Return: IIO_AVAIL_LIST on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epsonimu_read_avail(struct iio_dev *indio_dev,
                               struct iio_chan_spec const *chan,
                               const int **vals, int *type, int *length,
                               long mask)
{
    struct epsonimu_state* st = iio_priv(indio_dev);
    int cnt = 0;

    switch (mask) {

    case IIO_CHAN_INFO_SAMP_FREQ:
        for (int i=0; i < st->variant->sample_freq_mode_cnt; i++ ) {
            st->sample_freq_avail[2*i]   = st->variant->sample_freq_mode[i].hz;
            st->sample_freq_avail[2*i+1] = st->variant->sample_freq_mode[i].micro_hz;
        }
        *vals = st->sample_freq_avail;
        *type = IIO_VAL_INT_PLUS_MICRO;
        *length = st->variant->sample_freq_mode_cnt * 2;
        return IIO_AVAIL_LIST;

    case IIO_CHAN_INFO_SCALE:
        for (int i=0; i < st->variant->accel_scale_mode_cnt; i++) {
            st->accel_scale_avail[2*i]   = st->variant->accel_scale_mode[i].val;
            st->accel_scale_avail[2*i+1] = st->variant->accel_scale_mode[i].val2;
        }
        *vals = st->accel_scale_avail;
        *type = IIO_VAL_INT_PLUS_MICRO;
        *length = st->variant->accel_scale_mode_cnt * 2;
        return IIO_AVAIL_LIST;

    case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
        for (int i=0; i < st->variant->lpf_3db_freq_mode_cnt; i++) {
            if (st->variant->lpf_3db_freq_mode[i].options & 1) {
                st->lpf_3db_freq_avail[2*i]   = st->variant->lpf_3db_freq_mode[i].taps;
                st->lpf_3db_freq_avail[2*i+1] = st->variant->lpf_3db_freq_mode[i].cutoff_hz;
            } else {
                break;
            }
            cnt++;
        }
        *vals = st->lpf_3db_freq_avail;
        *type = IIO_VAL_INT_PLUS_MICRO;
        *length = cnt * 2;
        return IIO_AVAIL_LIST;

    default:
        return -EINVAL;
    }
}



//---------------------------------------------------------------------
// Channel defines
//
//----------------------------------------------------------------------
#define GYRO_CHAN(mod, addr, bits) { \
    .type = IIO_ANGL_VEL, \
    .modified = 1, \
    .channel2 = IIO_MOD_ ## mod, \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
    .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
        BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
    .info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
        BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
    .address = addr, \
    .scan_index = SCAN_GYRO_ ## mod, \
    .scan_type = { \
        .sign = 's', \
        .realbits = (bits), \
        .storagebits = (bits), \
        .shift = 0, \
        .endianness = IIO_BE, \
    }, \
}

#define ACCEL_CHAN(mod, addr, bits) { \
    .type = IIO_ACCEL, \
    .modified = 1, \
    .channel2 = IIO_MOD_ ## mod, \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
    .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
        BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
    .info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
        BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
    .address = (addr), \
    .scan_index = SCAN_ACC_ ## mod, \
    .scan_type = { \
        .sign = 's', \
        .realbits = (bits), \
        .storagebits = (bits), \
        .shift = 0, \
        .endianness = IIO_BE, \
    }, \
}

#define ACCEL_SCALE_DUAL_RANGE_CHAN(mod, addr, bits) { \
    .type = IIO_ACCEL, \
    .modified = 1, \
    .channel2 = IIO_MOD_ ## mod, \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
    .info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_SCALE), \
    .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
        BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
    .info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
        BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
    .address = (addr), \
    .scan_index = SCAN_ACC_ ## mod, \
    .scan_type = { \
        .sign = 's', \
        .realbits = (bits), \
        .storagebits = (bits), \
        .shift = 0, \
        .endianness = IIO_BE, \
    }, \
}

#define TEMP_CHAN(addr, bits) { \
    .type = IIO_TEMP, \
    .indexed = 1, \
    .channel = 0, \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
        BIT(IIO_CHAN_INFO_OFFSET) | \
        BIT(IIO_CHAN_INFO_SCALE), \
    .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
        BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
    .info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
        BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
    .address = (addr), \
    .scan_index = SCAN_TEMP, \
    .scan_type = { \
        .sign = 's', \
        .realbits = (bits), \
        .storagebits = (bits), \
        .shift = 0, \
        .endianness = IIO_BE, \
    }, \
}

#define COUNT_CHAN(addr, bits) { \
    .type = IIO_COUNT, \
    .indexed = 1, \
    .channel = 1, \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
    .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
    .address = (addr), \
    .scan_index = SCAN_COUNT, \
    .scan_type = { \
        .sign = 'u', \
        .realbits = (bits), \
        .storagebits = (bits), \
        .shift = 0, \
        .endianness = IIO_BE, \
    }, \
}



//----------------------------------------------------------------------
// IMU specific channel list
//
//----------------------------------------------------------------------
#ifdef CONFIG_EPSONIMU_SAMPLE_32BIT
//32-bit samples
static const struct iio_chan_spec epson_Temp_Gyro_Accel_Cnt_Time[] = {
    TEMP_CHAN(REG_TEMP_HIGH, 32),
    GYRO_CHAN(X, REG_XGYRO_HIGH, 32),
    GYRO_CHAN(Y, REG_YGYRO_HIGH, 32),
    GYRO_CHAN(Z, REG_ZGYRO_HIGH, 32),
    ACCEL_CHAN(X, REG_XACCL_HIGH, 32),
    ACCEL_CHAN(Y, REG_YACCL_HIGH, 32),
    ACCEL_CHAN(Z, REG_ZACCL_HIGH, 32),
    COUNT_CHAN(REG_COUNT, 16),
    IIO_CHAN_SOFT_TIMESTAMP(SCAN_TIMESTAMP),
};
static const struct iio_chan_spec epson_Temp_Gyro_Accel2Range_Cnt_Time[] = {
    TEMP_CHAN(REG_TEMP_HIGH, 32),
    GYRO_CHAN(X, REG_XGYRO_HIGH, 32),
    GYRO_CHAN(Y, REG_YGYRO_HIGH, 32),
    GYRO_CHAN(Z, REG_ZGYRO_HIGH, 32),
    ACCEL_SCALE_DUAL_RANGE_CHAN(X, REG_XACCL_HIGH, 32),
    ACCEL_SCALE_DUAL_RANGE_CHAN(Y, REG_YACCL_HIGH, 32),
    ACCEL_SCALE_DUAL_RANGE_CHAN(Z, REG_ZACCL_HIGH, 32),
    COUNT_CHAN(REG_COUNT, 16),
    IIO_CHAN_SOFT_TIMESTAMP(SCAN_TIMESTAMP),
};


#else
//16-bit samples
static const struct iio_chan_spec epson_Temp_Gyro_Accel_Cnt_Time[] = {
    TEMP_CHAN(REG_TEMP_HIGH, 16),
    GYRO_CHAN(X, REG_XGYRO_HIGH, 16),
    GYRO_CHAN(Y, REG_YGYRO_HIGH, 16),
    GYRO_CHAN(Z, REG_ZGYRO_HIGH, 16),
    ACCEL_CHAN(X, REG_XACCL_HIGH, 16),
    ACCEL_CHAN(Y, REG_YACCL_HIGH, 16),
    ACCEL_CHAN(Z, REG_ZACCL_HIGH, 16),
    COUNT_CHAN(REG_COUNT, 16),
    IIO_CHAN_SOFT_TIMESTAMP(SCAN_TIMESTAMP),
};
static const struct iio_chan_spec epson_Temp_Gyro_Accel2Range_Cnt_Time[] = {
    TEMP_CHAN(REG_TEMP_HIGH, 16),
    GYRO_CHAN(X, REG_XGYRO_HIGH, 16),
    GYRO_CHAN(Y, REG_YGYRO_HIGH, 16),
    GYRO_CHAN(Z, REG_ZGYRO_HIGH, 16),
    ACCEL_SCALE_DUAL_RANGE_CHAN(X, REG_XACCL_HIGH, 16),
    ACCEL_SCALE_DUAL_RANGE_CHAN(Y, REG_YACCL_HIGH, 16),
    ACCEL_SCALE_DUAL_RANGE_CHAN(Z, REG_ZACCL_HIGH, 16),
    COUNT_CHAN(REG_COUNT, 16),
    IIO_CHAN_SOFT_TIMESTAMP(SCAN_TIMESTAMP),
};
#endif


#define read_only " R "
#define read_write "R/W"
/* G320PDG0, G354PDH0, G364PDC0, G364PDCA */
const struct epson_register_map reg_w0_common[] = {
            {"MODE_CTRL  ", REG_MODE_CTRL_LO,   read_write, "0400"},
            {"DIAG_STAT  ", REG_DIAG_STAT,      read_only,  "0000"},
            {"FLAG       ", REG_FLAG,           read_only,  "0000"},
            {"GPIO       ", REG_GPIO,           read_write, "0200"},
            {"COUNT      ", REG_COUNT,          read_only,  "0000"},
            {"TEMP_HIGH  ", REG_TEMP_HIGH,      read_only,  "FFFF"},
            {"TEMP_LOW   ", REG_TEMP_LOW,       read_only,  "FFFF"},
            {"XGYRO_HIGH ", REG_XGYRO_HIGH,     read_only,  "FFFF"},
            {"XGYRO_LOW  ", REG_XGYRO_LOW,      read_only,  "FFFF"},
            {"YGYRO_HIGH ", REG_YGYRO_HIGH,     read_only,  "FFFF"},
            {"YGYRO_LOW  ", REG_YGYRO_LOW,      read_only,  "FFFF"},
            {"ZGYRO_HIGH ", REG_ZGYRO_HIGH,     read_only,  "FFFF"},
            {"ZGYRO_LOW  ", REG_ZGYRO_LOW,      read_only,  "FFFF"},
            {"XACCL_HIGH ", REG_XACCL_HIGH,     read_only,  "FFFF"},
            {"XACCL_LOW  ", REG_XACCL_LOW,      read_only,  "FFFF"},
            {"YACCL_HIGH ", REG_YACCL_HIGH,     read_only,  "FFFF"},
            {"YACCL_LOW  ", REG_YACCL_LOW,      read_only,  "FFFF"},
            {"ZACCL_HIGH ", REG_ZACCL_HIGH,     read_only,  "FFFF"},
            {"ZACCL_LOW  ", REG_ZACCL_LOW,      read_only,  "FFFF"},
            {"XDLTA_HIGH ", REG_XDLTA_HIGH,     read_only,  "FFFF"},
            {"XDLTA_LOW  ", REG_XDLTA_LOW,      read_only,  "FFFF"},
            {"YDLTA_HIGH ", REG_YDLTA_HIGH,     read_only,  "FFFF"},
            {"YDLTA_LOW  ", REG_YDLTA_LOW,      read_only,  "FFFF"},
            {"ZDLTA_HIGH ", REG_ZDLTA_HIGH,     read_only,  "FFFF"},
            {"ZDLTA_LOW  ", REG_ZDLTA_LOW,      read_only,  "FFFF"},
            {"XDLTV_HIGH ", REG_XDLTV_HIGH,     read_only,  "FFFF"},
            {"XDLTV_LOW  ", REG_XDLTV_LOW,      read_only,  "FFFF"},
            {"YDLTV_HIGH ", REG_YDLTV_HIGH,     read_only,  "FFFF"},
            {"YDLTV_LOW  ", REG_YDLTV_LOW,      read_only,  "FFFF"},
            {"ZDLTV_HIGH ", REG_ZDLTV_HIGH,     read_only,  "FFFF"},
            {"ZDLTV_LOW  ", REG_ZDLTV_LOW,      read_only,  "FFFF"},
            {"WIN_CTRL   ", REG_WIN_CTRL,       read_write, "0000"},
            { NULL,         0,                  NULL,        NULL },
        };
        
const struct epson_register_map reg_w1_common[] = {
            {"SIG_CTRL   ", REG_SIG_CTRL_LO,    read_write, "FE00"},
            {"MSC_CTRL   ", REG_MSC_CTRL_LO,    read_write, "0006"},
            {"SMPL_CTRL  ", REG_SMPL_CTRL_LO,   read_write, "0103"},
            {"FILTER_CTRL", REG_FILTER_CTRL_LO, read_write, "0001"},
            {"UART_CTRL  ", REG_UART_CTRL_LO,   read_write, "0000"},
            {"GLOB_CMD   ", REG_GLOB_CMD_LO,    read_write, "0000"},
            {"BURST_CTRL1", REG_BURST_CTRL1_LO, read_write, "F006"},
            {"BURST_CTRL2", REG_BURST_CTRL2_LO, read_write, "0000"},
            {"POL_CTRL   ", REG_POL_CTRL_LO,    read_write, "0000"},
            {"DLT_CTRL   ", REG_DLT_CTRL_LO,    read_write, "000C"},
            {"PROD_ID1   ", REG_PROD_ID1,       read_only,   NULL },
            {"PROD_ID2   ", REG_PROD_ID2,       read_only,   NULL },
            {"PROD_ID3   ", REG_PROD_ID3,       read_only,   NULL },
            {"PROD_ID4   ", REG_PROD_ID4,       read_only,   NULL },
            {"VERSION    ", REG_VERSION,        read_only,   NULL },
            {"SERIAL_NUM1", REG_SERIAL_NUM1,    read_only,   NULL },
            {"SERIAL_NUM2", REG_SERIAL_NUM2,    read_only,   NULL },
            {"SERIAL_NUM3", REG_SERIAL_NUM3,    read_only,   NULL },
            {"SERIAL_NUM4", REG_SERIAL_NUM4,    read_only,   NULL },
            {"WIN_CTRL   ", REG_WIN_CTRL,       read_write, "0000"},
            { NULL,         0,                  NULL,        NULL },
        };

/* G365PDC1_w1, G365PDF1_w1 */
const struct epson_register_map reg_w1_G365PDxx[] = {
            {"SIG_CTRL              ", REG_SIG_CTRL_LO,    read_write, "FE00"},
            {"MSC_CTRL              ", REG_MSC_CTRL_LO,    read_write, "0006"},
            {"SMPL_CTRL             ", REG_SMPL_CTRL_LO,   read_write, "0103"},
            {"FILTER_CTRL           ", REG_FILTER_CTRL_LO, read_write, "0001"},
            {"UART_CTRL             ", REG_UART_CTRL_LO,   read_write, "0000"},
            {"GLOB_CMD              ", REG_GLOB_CMD_LO,    read_write, "0000"},
            {"BURST_CTRL1           ", REG_BURST_CTRL1_LO, read_write, "F006"},
            {"BURST_CTRL2           ", REG_BURST_CTRL2_LO, read_write, "0000"},
            {"POL_CTRL              ", REG_POL_CTRL_LO,    read_write, "0000"},
            {"DLT_CTRL              ", REG_DLT_CTRL_LO,    read_write, "00CC"},
            {"ATTI_CTRL             ", 0x0114,             read_write, "0000"},
            {"GLOB_CMD2             ", 0x0116,             read_write, "0000"},
            {"R_MATRIX_G_M11        ", 0x0138,             read_write, "4000"},
            {"R_MATRIX_G_M12        ", 0x013A,             read_write, "0000"},
            {"R_MATRIX_G_M13        ", 0x013C,             read_write, "0000"},
            {"R_MATRIX_G_M21        ", 0x013E,             read_write, "0000"},
            {"R_MATRIX_G_M22        ", 0x0140,             read_write, "4000"},
            {"R_MATRIX_G_M23        ", 0x0142,             read_write, "0000"},
            {"R_MATRIX_G_M31        ", 0x0144,             read_write, "0000"},
            {"R_MATRIX_G_M32        ", 0x0146,             read_write, "0000"},
            {"R_MATRIX_G_M33        ", 0x0148,             read_write, "4000"},
            {"R_MATRIX_A_M11        ", 0x014A,             read_write, "4000"},
            {"R_MATRIX_A_M12        ", 0x014C,             read_write, "0000"},
            {"R_MATRIX_A_M13        ", 0x014E,             read_write, "0000"},
            {"R_MATRIX_A_M21        ", 0x0150,             read_write, "0000"},
            {"R_MATRIX_A_M22        ", 0x0152,             read_write, "4000"},
            {"R_MATRIX_A_M23        ", 0x0154,             read_write, "0000"},
            {"R_MATRIX_A_M31        ", 0x0156,             read_write, "0000"},
            {"R_MATRIX_A_M32        ", 0x0158,             read_write, "0000"},
            {"R_MATRIX_A_M33        ", 0x015A,             read_write, "4000"},
            {"PROD_ID1              ", REG_PROD_ID1,       read_only,   NULL },
            {"PROD_ID2              ", REG_PROD_ID2,       read_only,   NULL },
            {"PROD_ID3              ", REG_PROD_ID3,       read_only,   NULL },
            {"PROD_ID4              ", REG_PROD_ID4,       read_only,   NULL },
            {"VERSION               ", REG_VERSION,        read_only,   NULL },
            {"SERIAL_NUM1           ", REG_SERIAL_NUM1,    read_only,   NULL },
            {"SERIAL_NUM2           ", REG_SERIAL_NUM2,    read_only,   NULL },
            {"SERIAL_NUM3           ", REG_SERIAL_NUM3,    read_only,   NULL },
            {"SERIAL_NUM4           ", REG_SERIAL_NUM4,    read_only,   NULL },
            {"WIN_CTRL              ", REG_WIN_CTRL,       read_write,  NULL },
            { NULL,                    0,                  NULL,        NULL },
        };

/* G366PDG0, G330PDG0, G365PDC1_w0, G365PDF1_w0 */
const struct epson_register_map reg_w0_G366PD[] = {
            {"MODE_CTRL             ", REG_MODE_CTRL_LO,   read_write, "0400"},
            {"DIAG_STAT             ", REG_DIAG_STAT,      read_only,  "0000"},
            {"FLAG                  ", REG_FLAG,           read_only,  "0000"},
            {"GPIO                  ", REG_GPIO,           read_write, "0200"},
            {"COUNT                 ", REG_COUNT,          read_only,  "0000"},
            {"RANGE_OVER            ", 0x000C,             read_only,  "0000"},
            {"TEMP_HIGH             ", REG_TEMP_HIGH,      read_only,  "FFFF"},
            {"TEMP_LOW              ", REG_TEMP_LOW,       read_only,  "FFFF"},
            {"XGYRO_HIGH            ", REG_XGYRO_HIGH,     read_only,  "FFFF"},
            {"XGYRO_LOW             ", REG_XGYRO_LOW,      read_only,  "FFFF"},
            {"YGYRO_HIGH            ", REG_YGYRO_HIGH,     read_only,  "FFFF"},
            {"YGYRO_LOW             ", REG_YGYRO_LOW,      read_only,  "FFFF"},
            {"ZGYRO_HIGH            ", REG_ZGYRO_HIGH,     read_only,  "FFFF"},
            {"ZGYRO_LOW             ", REG_ZGYRO_LOW,      read_only,  "FFFF"},
            {"XACCL_HIGH            ", REG_XACCL_HIGH,     read_only,  "FFFF"},
            {"XACCL_LOW             ", REG_XACCL_LOW,      read_only,  "FFFF"},
            {"YACCL_HIGH            ", REG_YACCL_HIGH,     read_only,  "FFFF"},
            {"YACCL_LOW             ", REG_YACCL_LOW,      read_only,  "FFFF"},
            {"ZACCL_HIGH            ", REG_ZACCL_HIGH,     read_only,  "FFFF"},
            {"ZACCL_LOW             ", REG_ZACCL_LOW,      read_only,  "FFFF"},
            {"ID                    ", 0x004C,             read_only,  "5345"},
            {"QTN0_HIGH             ", 0x0050,             read_only,  "0000"},
            {"QTN0_LOW              ", 0x0052,             read_only,  "0000"},
            {"QTN1_HIGH             ", 0x0054,             read_only,  "0000"},
            {"QTN1_LOW              ", 0x0056,             read_only,  "0000"},
            {"QTN2_HIGH             ", 0x0058,             read_only,  "0000"},
            {"QTN2_LOW              ", 0x005A,             read_only,  "0000"},
            {"QTN3_HIGH             ", 0x005C,             read_only,  "0000"},
            {"QTN3_LOW              ", 0x005E,             read_only,  "0000"},
            {"XDLTA_HIGH / ANG1_HIGH", REG_XDLTA_HIGH,     read_only,  "0000"},
            {"XDLTA_LOW  / ANG1_LOW ", REG_XDLTA_LOW,      read_only,  "0000"},
            {"YDLTA_HIGH / ANG2_HIGH", REG_YDLTA_HIGH,     read_only,  "0000"},
            {"YDLTA_LOW  / ANG2_LOW ", REG_YDLTA_LOW,      read_only,  "0000"},
            {"ZDLTA_HIGH / ANG3_HIGH", REG_ZDLTA_HIGH,     read_only,  "0000"},
            {"ZDLTA_LOW  / ANG3_LOW ", REG_ZDLTA_LOW,      read_only,  "0000"},
            {"XDLTV_HIGH            ", REG_XDLTV_HIGH,     read_only,  "0000"},
            {"XDLTV_LOW             ", REG_XDLTV_LOW,      read_only,  "0000"},
            {"YDLTV_HIGH            ", REG_YDLTV_HIGH,     read_only,  "0000"},
            {"YDLTV_LOW             ", REG_YDLTV_LOW,      read_only,  "0000"},
            {"ZDLTV_HIGH            ", REG_ZDLTV_HIGH,     read_only,  "0000"},
            {"ZDLTV_LOW             ", REG_ZDLTV_LOW,      read_only,  "0000"},
            {"WIN_CTRL              ", REG_WIN_CTRL,       read_write, "0000"},
            { NULL,                    0,                  NULL,        NULL },
        };
        
/* G366PDG0, G330PDG0, G370_PDT0_w1, G370PDG0_w1 */
const struct epson_register_map reg_w1_G366PD[] = {
            {"SIG_CTRL              ", REG_SIG_CTRL_LO,    read_write, "FE00"},
            {"MSC_CTRL              ", REG_MSC_CTRL_LO,    read_write, "0006"},
            {"SMPL_CTRL             ", REG_SMPL_CTRL_LO,   read_write, "0103"},
            {"FILTER_CTRL           ", REG_FILTER_CTRL_LO, read_write, "0001"},
            {"UART_CTRL             ", REG_UART_CTRL_LO,   read_write, "0000"},
            {"GLOB_CMD              ", REG_GLOB_CMD_LO,    read_write, "0000"},
            {"BURST_CTRL1           ", REG_BURST_CTRL1_LO, read_write, "F006"},
            {"BURST_CTRL2           ", REG_BURST_CTRL2_LO, read_write, "0000"},
            {"POL_CTRL              ", REG_POL_CTRL_LO,    read_write, "0000"},
            {"GLOB_CMD3             ", 0x0112,             read_write, "00CC"},
            {"ATTI_CTRL             ", 0x0114,             read_write, "0000"},
            {"GLOB_CMD2             ", 0x0116,             read_write, "0000"},
            {"R_MATRIX_M11          ", 0x0138,             read_write, "4000"},
            {"R_MATRIX_M12          ", 0x013A,             read_write, "0000"},
            {"R_MATRIX_M13          ", 0x013C,             read_write, "0000"},
            {"R_MATRIX_M21          ", 0x013E,             read_write, "0000"},
            {"R_MATRIX_M22          ", 0x0140,             read_write, "4000"},
            {"R_MATRIX_M23          ", 0x0142,             read_write, "0000"},
            {"R_MATRIX_M31          ", 0x0144,             read_write, "0000"},
            {"R_MATRIX_M32          ", 0x0146,             read_write, "0000"},
            {"R_MATRIX_M33          ", 0x0148,             read_write, "4000"},
            {"PROD_ID1              ", REG_PROD_ID1,       read_only,   NULL },
            {"PROD_ID2              ", REG_PROD_ID2,       read_only,   NULL },
            {"PROD_ID3              ", REG_PROD_ID3,       read_only,   NULL },
            {"PROD_ID4              ", REG_PROD_ID4,       read_only,   NULL },
            {"VERSION               ", REG_VERSION,        read_only,   NULL },
            {"SERIAL_NUM1           ", REG_SERIAL_NUM1,    read_only,   NULL },
            {"SERIAL_NUM2           ", REG_SERIAL_NUM2,    read_only,   NULL },
            {"SERIAL_NUM3           ", REG_SERIAL_NUM3,    read_only,   NULL },
            {"SERIAL_NUM4           ", REG_SERIAL_NUM4,    read_only,   NULL },
            {"WIN_CTRL              ", REG_WIN_CTRL,       read_write, "0000"},
            { NULL,                    0,                  NULL,        NULL },
        };

/* G355QDG0 */
const struct epson_register_map reg_w0_G355QDG0[] = {
            {"MODE_CTRL   ", REG_MODE_CTRL_LO,   read_write, "0400"},
            {"DIAG_STAT   ", REG_DIAG_STAT,      read_only,  "0000"},
            {"FLAG        ", REG_FLAG,           read_only,  "0000"},
            {"GPIO        ", REG_GPIO,           read_write, "0200"},
            {"COUNT       ", REG_COUNT,          read_only,  "0000"},
            {"RANGE_OVER  ", 0x000C,             read_only,  "0000"},
            {"TEMP_HIGH   ", REG_TEMP_HIGH,      read_only,  "FFFF"},
            {"TEMP_LOW    ", REG_TEMP_LOW,       read_only,  "FFFF"},
            {"XGYRO_HIGH  ", REG_XGYRO_HIGH,     read_only,  "FFFF"},
            {"XGYRO_LOW   ", REG_XGYRO_LOW,      read_only,  "FFFF"},
            {"YGYRO_HIGH  ", REG_YGYRO_HIGH,     read_only,  "FFFF"},
            {"YGYRO_LOW   ", REG_YGYRO_LOW,      read_only,  "FFFF"},
            {"ZGYRO_HIGH  ", REG_ZGYRO_HIGH,     read_only,  "FFFF"},
            {"ZGYRO_LOW   ", REG_ZGYRO_LOW,      read_only,  "FFFF"},
            {"XACCL_HIGH  ", REG_XACCL_HIGH,     read_only,  "FFFF"},
            {"XACCL_LOW   ", REG_XACCL_LOW,      read_only,  "FFFF"},
            {"YACCL_HIGH  ", REG_YACCL_HIGH,     read_only,  "FFFF"},
            {"YACCL_LOW   ", REG_YACCL_LOW,      read_only,  "FFFF"},
            {"ZACCL_HIGH  ", REG_ZACCL_HIGH,     read_only,  "FFFF"},
            {"ZACCL_LOW   ", REG_ZACCL_LOW,      read_only,  "FFFF"},
            {"OB_DIAG_STAT", 0x002A,             read_only,  "0000"},
            {"ID          ", 0x004C,             read_only,  "5345"},
            { NULL,          0,                  NULL,        NULL },
        };
        
const struct epson_register_map reg_w1_G355QDG0[] = {
            {"SIG_CTRL    ", REG_SIG_CTRL_LO,    read_write, "FE00"},
            {"MSC_CTRL    ", REG_MSC_CTRL_LO,    read_write, "0006"},
            {"SMPL_CTRL   ", REG_SMPL_CTRL_LO,   read_write, "0803"},
            {"FILTER_CTRL ", REG_FILTER_CTRL_LO, read_write, "0003"},
            {"UART_CTRL   ", REG_UART_CTRL_LO,   read_write, "0000"},
            {"GLOB_CMD    ", REG_GLOB_CMD_LO,    read_write, "0000"},
            {"BURST_CTRL1 ", REG_BURST_CTRL1_LO, read_write, "F006"},
            {"BURST_CTRL2 ", REG_BURST_CTRL2_LO, read_write, "0000"},
            {"POL_CTRL    ", REG_POL_CTRL_LO,    read_write, "0000"},
            {"GLOB_CMD3   ", 0x0112,             read_write, "00CC"},
            {"PROD_ID1    ", REG_PROD_ID1,       read_only,   NULL },
            {"PROD_ID2    ", REG_PROD_ID2,       read_only,   NULL },
            {"PROD_ID3    ", REG_PROD_ID3,       read_only,   NULL },
            {"PROD_ID4    ", REG_PROD_ID4,       read_only,   NULL },
            {"VERSION     ", REG_VERSION,        read_only,   NULL },
            {"SERIAL_NUM1 ", REG_SERIAL_NUM1,    read_only,   NULL },
            {"SERIAL_NUM2 ", REG_SERIAL_NUM2,    read_only,   NULL },
            {"SERIAL_NUM3 ", REG_SERIAL_NUM3,    read_only,   NULL },
            {"SERIAL_NUM4 ", REG_SERIAL_NUM4,    read_only,   NULL },
            {"WIN_CTRL    ", REG_WIN_CTRL,       read_write, "0000"},
            { NULL,          0,                  NULL,        NULL },
        };

/* G370PDS0, G370PDF1 */
const struct epson_register_map reg_w0_G370[] = {
            {"MODE_CTRL     ", REG_MODE_CTRL_LO,   read_write, "0400"},
            {"DIAG_STAT     ", REG_DIAG_STAT,      read_only,  "0000"},
            {"FLAG          ", REG_FLAG,           read_only,  "0000"},
            {"GPIO          ", REG_GPIO,           read_write, "0200"},
            {"COUNT         ", REG_COUNT,          read_only,  "0000"},
            {"RANGE_OVER    ", 0x000C,             read_only,  "0000"},
            {"TEMP_HIGH     ", REG_TEMP_HIGH,      read_only,  "FFFF"},
            {"TEMP_LOW      ", REG_TEMP_LOW,       read_only,  "FFFF"},
            {"XGYRO_HIGH    ", REG_XGYRO_HIGH,     read_only,  "FFFF"},
            {"XGYRO_LOW     ", REG_XGYRO_LOW,      read_only,  "FFFF"},
            {"YGYRO_HIGH    ", REG_YGYRO_HIGH,     read_only,  "FFFF"},
            {"YGYRO_LOW     ", REG_YGYRO_LOW,      read_only,  "FFFF"},
            {"ZGYRO_HIGH    ", REG_ZGYRO_HIGH,     read_only,  "FFFF"},
            {"ZGYRO_LOW     ", REG_ZGYRO_LOW,      read_only,  "FFFF"},
            {"XACCL_HIGH    ", REG_XACCL_HIGH,     read_only,  "FFFF"},
            {"XACCL_LOW     ", REG_XACCL_LOW,      read_only,  "FFFF"},
            {"YACCL_HIGH    ", REG_YACCL_HIGH,     read_only,  "FFFF"},
            {"YACCL_LOW     ", REG_YACCL_LOW,      read_only,  "FFFF"},
            {"ZACCL_HIGH    ", REG_ZACCL_HIGH,     read_only,  "FFFF"},
            {"ZACCL_LOW     ", REG_ZACCL_LOW,      read_only,  "FFFF"},
            {"RT_DIAG       ", 0x002A,             read_only,  "0000"},
            {"ID            ", 0x004C,             read_only,  "5345"},
            {"XDLTA_HIGH    ", REG_XDLTA_HIGH,     read_only,  "0000"},
            {"XDLTA_LOW     ", REG_XDLTA_LOW,      read_only,  "0000"},
            {"YDLTA_HIGH    ", REG_YDLTA_HIGH,     read_only,  "0000"},
            {"YDLTA_LOW     ", REG_YDLTA_LOW,      read_only,  "0000"},
            {"ZDLTA_HIGH    ", REG_ZDLTA_HIGH,     read_only,  "0000"},
            {"ZDLTA_LOW     ", REG_ZDLTA_LOW,      read_only,  "0000"},
            {"XDLTV_HIGH    ", REG_XDLTV_HIGH,     read_only,  "0000"},
            {"XDLTV_LOW     ", REG_XDLTV_LOW,      read_only,  "0000"},
            {"YDLTV_HIGH    ", REG_YDLTV_HIGH,     read_only,  "0000"},
            {"YDLTV_LOW     ", REG_YDLTV_LOW,      read_only,  "0000"},
            {"ZDLTV_HIGH    ", REG_ZDLTV_HIGH,     read_only,  "0000"},
            {"ZDLTV_LOW     ", REG_ZDLTV_LOW,      read_only,  "0000"},
            {"WIN_CTRL      ", REG_WIN_CTRL,       read_write, "0000"},
            { NULL,            0,                  NULL,        NULL },
        };
        
/* G370PDS0, G370PDF1 */
const struct epson_register_map reg_w1_G370[] = {
            {"SIG_CTRL      ", REG_SIG_CTRL_LO,    read_write, "FE00"},
            {"MSC_CTRL      ", REG_MSC_CTRL_LO,    read_write, "0006"},
            {"SMPL_CTRL     ", REG_SMPL_CTRL_LO,   read_write, "0107"},
            {"FILTER_CTRL   ", REG_FILTER_CTRL_LO, read_write, "0001"},
            {"UART_CTRL     ", REG_UART_CTRL_LO,   read_write, "0000"},
            {"GLOB_CMD      ", REG_GLOB_CMD_LO,    read_write, "0000"},
            {"BURST_CTRL1   ", REG_BURST_CTRL1_LO, read_write, "F006"},
            {"BURST_CTRL2   ", REG_BURST_CTRL2_LO, read_write, "0000"},
            {"POL_CTRL      ", REG_POL_CTRL_LO,    read_write, "0000"},
            {"DLT_CTRL      ", REG_DLT_CTRL_LO,    read_write, "00CC"},
            {"ATTI_CTRL     ", 0x0114,             read_write, "0000"},
            {"GLOB_CMD2     ", 0x0116,             read_write, "0001"},
            {"R_MATRIX_G_M11", 0x0138,             read_write, "4000"},
            {"R_MATRIX_G_M12", 0x013A,             read_write, "0000"},
            {"R_MATRIX_G_M13", 0x013C,             read_write, "0000"},
            {"R_MATRIX_G_M21", 0x013E,             read_write, "0000"},
            {"R_MATRIX_G_M22", 0x0140,             read_write, "4000"},
            {"R_MATRIX_G_M23", 0x0142,             read_write, "0000"},
            {"R_MATRIX_G_M31", 0x0144,             read_write, "0000"},
            {"R_MATRIX_G_M32", 0x0146,             read_write, "0000"},
            {"R_MATRIX_G_M33", 0x0148,             read_write, "4000"},
            {"R_MATRIX_A_M11", 0x014A,             read_write, "4000"},
            {"R_MATRIX_A_M12", 0x014C,             read_write, "0000"},
            {"R_MATRIX_A_M13", 0x014E,             read_write, "0000"},
            {"R_MATRIX_A_M21", 0x0150,             read_write, "0000"},
            {"R_MATRIX_A_M22", 0x0152,             read_write, "4000"},
            {"R_MATRIX_A_M23", 0x0154,             read_write, "0000"},
            {"R_MATRIX_A_M31", 0x0156,             read_write, "0000"},
            {"R_MATRIX_A_M32", 0x0158,             read_write, "0000"},
            {"R_MATRIX_A_M33", 0x015A,             read_write, "4000"},
            {"PROD_ID1      ", REG_PROD_ID1,       read_only,   NULL },
            {"PROD_ID2      ", REG_PROD_ID2,       read_only,   NULL },
            {"PROD_ID3      ", REG_PROD_ID3,       read_only,   NULL },
            {"PROD_ID4      ", REG_PROD_ID4,       read_only,   NULL },
            {"VERSION       ", REG_VERSION,        read_only,   NULL },
            {"SERIAL_NUM1   ", REG_SERIAL_NUM1,    read_only,   NULL },
            {"SERIAL_NUM2   ", REG_SERIAL_NUM2,    read_only,   NULL },
            {"SERIAL_NUM3   ", REG_SERIAL_NUM3,    read_only,   NULL },
            {"SERIAL_NUM4   ", REG_SERIAL_NUM4,    read_only,   NULL },
            {"WIN_CTRL      ", REG_WIN_CTRL,       read_write, "0000"},
            { NULL,            0,                  NULL,        NULL },
        };

/* G370PDG0, G370PDT0 */
const struct epson_register_map reg_w0_G370PD_PDG0_PDT0[] = {
            {"MODE_CTRL     ", REG_MODE_CTRL_LO,   read_write, "0400"},
            {"DIAG_STAT     ", REG_DIAG_STAT,      read_only,  "0000"},
            {"FLAG          ", REG_FLAG,           read_only,  "0000"},
            {"GPIO          ", REG_GPIO,           read_write, "0200"},
            {"COUNT         ", REG_COUNT,          read_only,  "0000"},
            {"RANGE_OVER    ", 0x000C,             read_only,  "0000"},
            {"TEMP_HIGH     ", REG_TEMP_HIGH,      read_only,  "FFFF"},
            {"TEMP_LOW      ", REG_TEMP_LOW,       read_only,  "FFFF"},
            {"XGYRO_HIGH    ", REG_XGYRO_HIGH,     read_only,  "FFFF"},
            {"XGYRO_LOW     ", REG_XGYRO_LOW,      read_only,  "FFFF"},
            {"YGYRO_HIGH    ", REG_YGYRO_HIGH,     read_only,  "FFFF"},
            {"YGYRO_LOW     ", REG_YGYRO_LOW,      read_only,  "FFFF"},
            {"ZGYRO_HIGH    ", REG_ZGYRO_HIGH,     read_only,  "FFFF"},
            {"ZGYRO_LOW     ", REG_ZGYRO_LOW,      read_only,  "FFFF"},
            {"XACCL_HIGH    ", REG_XACCL_HIGH,     read_only,  "FFFF"},
            {"XACCL_LOW     ", REG_XACCL_LOW,      read_only,  "FFFF"},
            {"YACCL_HIGH    ", REG_YACCL_HIGH,     read_only,  "FFFF"},
            {"YACCL_LOW     ", REG_YACCL_LOW,      read_only,  "FFFF"},
            {"ZACCL_HIGH    ", REG_ZACCL_HIGH,     read_only,  "FFFF"},
            {"ZACCL_LOW     ", REG_ZACCL_LOW,      read_only,  "FFFF"},
            {"ID            ", 0x004C,             read_only,  "5345"},
            {"XDLTA_HIGH    ", REG_XDLTA_HIGH,     read_only,  "0000"},
            {"XDLTA_LOW     ", REG_XDLTA_LOW,      read_only,  "0000"},
            {"YDLTA_HIGH    ", REG_YDLTA_HIGH,     read_only,  "0000"},
            {"YDLTA_LOW     ", REG_YDLTA_LOW,      read_only,  "0000"},
            {"ZDLTA_HIGH    ", REG_ZDLTA_HIGH,     read_only,  "0000"},
            {"ZDLTA_LOW     ", REG_ZDLTA_LOW,      read_only,  "0000"},
            {"XDLTV_HIGH    ", REG_XDLTV_HIGH,     read_only,  "0000"},
            {"XDLTV_LOW     ", REG_XDLTV_LOW,      read_only,  "0000"},
            {"YDLTV_HIGH    ", REG_YDLTV_HIGH,     read_only,  "0000"},
            {"YDLTV_LOW     ", REG_YDLTV_LOW,      read_only,  "0000"},
            {"ZDLTV_HIGH    ", REG_ZDLTV_HIGH,     read_only,  "0000"},
            {"ZDLTV_LOW     ", REG_ZDLTV_LOW,      read_only,  "0000"},
            {"WIN_CTRL      ", REG_WIN_CTRL,       read_write, "0000"},
            { NULL,            0,                  NULL,        NULL },
        };

const struct epson_accel_scale accel_scale_two_ranges[] = {
            { 0,  "+/-8G", 0, IIO_G_TO_M_S_2(250) },
            { 1, "+/-16G", 0, IIO_G_TO_M_S_2(500) }
        };

const struct epson_sample_freq common_sample_freq_mode[] = {
            { 0x00,2000, 0      },
            { 0x01,1000, 0      },
            { 0x02, 500, 0      },
            { 0x03, 250, 0      },
            { 0x04, 125, 0      },
            { 0x05,  62, 500000 },
            { 0x06,  31, 250000 },
            { 0x07,  15, 625000 },
            { 0x08, 400, 0      },
            { 0x09, 200, 0      },
            { 0x0A, 100, 0      },
            { 0x0B,  80, 0      },
            { 0x0C,  50, 0      },
            { 0x0D,  40, 0      },
            { 0x0E,  25, 0      },
            { 0x0F,  20, 0      }
        };
const size_t common_sample_freq_mode_cnt = 16;

const struct epson_lpf_3db_freq common_lpf_3db_freq_mode[] = {
            { 0x00, "tap0",          0,   0, 1 },
            { 0x01, "tap2",          2,   0, 1 },
            { 0x02, "tap4",          4,   0, 1 },
            { 0x03, "tap8",          8,   0, 1 },
            { 0x04, "tap16",        16,   0, 1 },
            { 0x05, "tap32",        32,   0, 1 },
            { 0x06, "tap64",        64,   0, 1 },
            { 0x07, "tap128",      128,   0, 1 },
            { 0x08, "tap32fc50",    32,  50, 1 },
            { 0x09, "tap32fc100",   32, 100, 1 },
            { 0x0A, "tap32fc200",   32, 200, 1 },
            { 0x0B, "tap32fc400",   32, 400, 1 },
            { 0x0C, "tap64fc50",    64,  50, 1 },
            { 0x0D, "tap64fc100",   64, 100, 1 },
            { 0x0E, "tap64fc200",   64, 200, 1 },
            { 0x0F, "tap64fc400",   64, 400, 1 },
            { 0x10, "tap128fc50",  128,  50, 1 },
            { 0x11, "tap128fc100", 128, 100, 1 },
            { 0x12, "tap128fc200", 128, 200, 1 },
            { 0x13, "tap128fc400", 128, 400, 1 },
        };
const size_t common_lpf_3db_freq_mode_cnt = 20;



//----------------------------------------------------------------------
// epsonimu_chips[]
// information about the Epson IMUs
//
//----------------------------------------------------------------------
struct epsonimu_chip_info epsonimu_chips[] = {

    [EPSON_G320PDG0] = {
        .product_id = "epsonG320PDG0",
        .channels = epson_Temp_Gyro_Accel_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel_Cnt_Time),
        .flags = HAS_PROD_ID |
            HAS_SERIAL_NUMBER |
            BURST_DIAG_STAT |
            HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(8000), /* 0.008 (deg/s)/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(200), /* 0.2 mG/LSB */
        .accel_scale_mode_cnt = 1,
        .accel_scale_mode = NULL,
        .temp_scale_nano = -3791800, /*-0.0037918 */
        .temp_offset = -2634, /* Output = 2634(0x0A4A) @ +25C */
        .sample_freq_mode = common_sample_freq_mode,
        .sample_freq_mode_cnt = common_sample_freq_mode_cnt,
        .lpf_3db_freq_mode = (struct epson_lpf_3db_freq[]) {
            { 0x00, "tap0",          0,   0, 1 },
            { 0x01, "tap2",          2,   0, 1 },
            { 0x02, "tap4",          4,   0, 1 },
            { 0x03, "tap8",          8,   0, 1 },
            { 0x04, "tap16",        16,   0, 1 },
            { 0x05, "tap32",        32,   0, 1 },
            { 0x06, "tap64",        64,   0, 1 },
            { 0x07, "tap128",      128,   0, 1 },
            { 0x08, "tap32fc25",    32,  25, 1 },
            { 0x09, "tap32fc50",    32,  50, 1 },
            { 0x0A, "tap32fc100",   32, 100, 1 },
            { 0x0B, "tap32fc200",   32, 200, 1 },
            { 0x0C, "tap64fc25",    64,  25, 1 },
            { 0x0D, "tap64fc50",    64,  50, 1 },
            { 0x0E, "tap64fc100",   64, 100, 1 },
            { 0x0F, "tap64fc200",   64, 200, 1 },
            { 0x10, "tap128fc25",  128,  25, 1 },
            { 0x11, "tap128fc50",  128,  50, 1 },
            { 0x12, "tap128fc100", 128, 100, 1 },
            { 0x13, "tap128fc200", 128, 200, 1 },
        },
        .lpf_3db_freq_mode_cnt = 20,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq,
        .reg_map_window_0 = reg_w0_common,
        .reg_map_window_1 = reg_w1_common,
    },
    [EPSON_G354PDH0] = {
        .product_id = "epsonG354PDH0",
        .channels = epson_Temp_Gyro_Accel_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel_Cnt_Time),
        .flags = HAS_PROD_ID |
            HAS_SERIAL_NUMBER |
            BURST_DIAG_STAT |
            HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(16000), /* 0.016 (deg/s)/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(200), /* 0.2 mG/LSB */
        .accel_scale_mode_cnt = 1,
        .accel_scale_mode = NULL,
        .temp_scale_nano = -3791800, /*-0.0037918 */
        .temp_offset = -2634, /* Output = 2634(0x0A4A) @ +25C */
        .sample_freq_mode = common_sample_freq_mode,
        .sample_freq_mode_cnt = common_sample_freq_mode_cnt,
        .lpf_3db_freq_mode = common_lpf_3db_freq_mode,
        .lpf_3db_freq_mode_cnt = common_lpf_3db_freq_mode_cnt,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq,
        .reg_map_window_0 = reg_w0_common,
        .reg_map_window_1 = reg_w1_common,
    },
    [EPSON_G355QDG0] = {
        .product_id = "epsonG355QDG0",
        .channels = epson_Temp_Gyro_Accel2Range_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel2Range_Cnt_Time),
        .flags = HAS_PROD_ID |
             HAS_SERIAL_NUMBER |
             BURST_DIAG_STAT |
             HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(15152), /* 66 LBS/(deg/s) => 1/66 => 0.0151515 (deg/s)/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(500), /* 8G: 4 LSB/mG => 1/4 => 0.250 mG/LSB */ /* 16G: 2 LSB/mG => 1/2 => 0.500 mG/LSB */
        .accel_scale_mode_cnt = 2,
        .accel_scale_mode = accel_scale_two_ranges,
        .temp_scale_nano = 3906250, /* 0.00390625 */
        .temp_offset = 0, /* Output = 0 @ +25C */
        .sample_freq_mode = (struct epson_sample_freq[]) {
            { 0x03, 250, 0      },
            { 0x04, 125, 0      },
            { 0x05,  62, 500000 },
            { 0x06,  31, 250000 },
            { 0x07,  15, 625000 },
            { 0x08, 400, 0      },
            { 0x09, 200, 0      },
            { 0x0A, 100, 0      },
            { 0x0B,  80, 0      },
            { 0x0C,  50, 0      },
            { 0x0D,  40, 0      },
            { 0x0E,  25, 0      },
            { 0x0F,  20, 0      }
        },
        .sample_freq_mode_cnt = 13,
        .lpf_3db_freq_mode = (struct epson_lpf_3db_freq[]) {
            { 0x03, "tap8",          8,   0, 1 },
            { 0x04, "tap16",        16,   0, 1 },
            { 0x05, "tap32",        32,   0, 1 },
            { 0x06, "tap64",        64,   0, 1 },
            { 0x07, "tap128",      128,   0, 1 },

            { 0x08, "tap32fc50",    32,  50, 1 },
            { 0x09, "tap32fc100",   32, 100, 1 },
            { 0x0A, "tap32fc200",   32, 200, 1 },

            { 0x0C, "tap64fc50",    64,  50, 1 },
            { 0x0D, "tap64fc100",   64, 100, 1 },
            { 0x0E, "tap64fc200",   64, 200, 1 },

            { 0x10, "tap128fc50",  128,  50, 1 },
            { 0x11, "tap128fc100", 128, 100, 1 },
            { 0x12, "tap128fc200", 128, 200, 1 },
        },
        .lpf_3db_freq_mode_cnt = 14,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq,
        .reg_map_window_0 = reg_w0_G355QDG0,
        .reg_map_window_1 = reg_w1_G355QDG0,
    },
    [EPSON_G364PDC0] = {
        .product_id = "epsonG364PDC0",
        .channels = epson_Temp_Gyro_Accel_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel_Cnt_Time),
        .flags = HAS_PROD_ID |
            HAS_SERIAL_NUMBER |
            BURST_DIAG_STAT |
            HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(7500), /* 0.0075 (deg/s)/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(125), /* 0.125 mG/LSB */
        .accel_scale_mode_cnt = 1,
        .accel_scale_mode = NULL,
        .temp_scale_nano = -3791800, /* -0.0037918 */
        .temp_offset = -2634, /* Output = 2634(0x0A4A) @ +25C */
        .sample_freq_mode = common_sample_freq_mode,
        .sample_freq_mode_cnt = common_sample_freq_mode_cnt,
        .lpf_3db_freq_mode = common_lpf_3db_freq_mode,
        .lpf_3db_freq_mode_cnt = common_lpf_3db_freq_mode_cnt,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq,
        .reg_map_window_0 = reg_w0_common,
        .reg_map_window_1 = reg_w1_common,
    },
    [EPSON_G364PDCA] = {
        .product_id = "epsonG364PDCA",
        .channels = epson_Temp_Gyro_Accel_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel_Cnt_Time),
        .flags = HAS_PROD_ID |
            HAS_SERIAL_NUMBER |
            BURST_DIAG_STAT |
            HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(3750), /* 0.00375  (deg/s)/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(125), /* 0.125 mG/LSB  */
        .accel_scale_mode_cnt = 1,
        .accel_scale_mode = NULL,
        .temp_scale_nano = -3791800, /* -0.0037918 */
        .temp_offset = -2634, /* Output = 2634(0x0A4A) @ +25C */
        .sample_freq_mode = common_sample_freq_mode,
        .sample_freq_mode_cnt = common_sample_freq_mode_cnt,
        .lpf_3db_freq_mode = common_lpf_3db_freq_mode,
        .lpf_3db_freq_mode_cnt = common_lpf_3db_freq_mode_cnt,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq,
        .reg_map_window_0 = reg_w0_common,
        .reg_map_window_1 = reg_w1_common,
    },
    [EPSON_G365PDC1] = {
        .product_id = "epsonG365PDC1",
        .channels = epson_Temp_Gyro_Accel_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel_Cnt_Time),
        .flags = HAS_PROD_ID |
                HAS_SERIAL_NUMBER |
                BURST_DIAG_STAT |
                HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(15152), /* 66 LBS/(deg/s) => 1/66 => 0.0151515 (deg/s)/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(160), /* 6.25 LSB/mG => 1/6.25 => 0.16 mG/LSB */
        .accel_scale_mode_cnt = 1,
        .accel_scale_mode = NULL,
        .temp_scale_nano = -3791800, /* -0.0037918 */
        .temp_offset = -2634, /* Output = 2634(0x0A4A) @ +25'C */
        .sample_freq_mode = common_sample_freq_mode,
        .sample_freq_mode_cnt = common_sample_freq_mode_cnt,
        .lpf_3db_freq_mode = common_lpf_3db_freq_mode,
        .lpf_3db_freq_mode_cnt = common_lpf_3db_freq_mode_cnt,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq,
        .reg_map_window_0 = reg_w0_G366PD,
        .reg_map_window_1 = reg_w1_G365PDxx,
    },
    [EPSON_G365PDF1] = {
        .product_id = "epsonG365PDF1",
        .channels = epson_Temp_Gyro_Accel_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel_Cnt_Time),
        .flags = HAS_PROD_ID |
                HAS_SERIAL_NUMBER |
                BURST_DIAG_STAT |
                HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(15152), /* 66 LBS/(deg/s) => 1/66 => 0.0151515 (deg/s)/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(400), /* 2.5 LSB/mG => 1/2.5 => 0.400 mG/LSB */
        .accel_scale_mode_cnt = 1,
        .accel_scale_mode = NULL,
        .temp_scale_nano = -3791800, /* -0.0037918 */
        .temp_offset = -2634, /* Output = 2634(0x0A4A) @ +25'C */
        .sample_freq_mode = common_sample_freq_mode,
        .sample_freq_mode_cnt = common_sample_freq_mode_cnt,
        .lpf_3db_freq_mode = common_lpf_3db_freq_mode,
        .lpf_3db_freq_mode_cnt = common_lpf_3db_freq_mode_cnt,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq,
        .reg_map_window_0 = reg_w0_G366PD,
        .reg_map_window_1 = reg_w1_G365PDxx,
    },
    [EPSON_G366PDG0] = {
        .product_id = "epsonG366PDG0",
        .channels = epson_Temp_Gyro_Accel2Range_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel2Range_Cnt_Time),
        .flags = HAS_PROD_ID |
                HAS_SERIAL_NUMBER |
                BURST_DIAG_STAT |
                HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(15152), /* 66 LBS/(deg/s) => 1/66 => 0.0151515 (deg/s)/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(250), /* 8G: 4 LSB/mG => 1/4 => 0.250 mG/LSB */ /* 16G: 2 LSB/mG => 1/2 => 0.500 mG/LSB */
        .accel_scale_mode_cnt = 2,
        .accel_scale_mode = accel_scale_two_ranges,
        .temp_scale_nano = 3906250,   /* 0.00390625 */
        .temp_offset = 0, /* Output = 0 @ +25C */
        .sample_freq_mode = common_sample_freq_mode,
        .sample_freq_mode_cnt = common_sample_freq_mode_cnt,
        .lpf_3db_freq_mode = common_lpf_3db_freq_mode,
        .lpf_3db_freq_mode_cnt = common_lpf_3db_freq_mode_cnt,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq,
        .reg_map_window_0 = reg_w0_G366PD,
        .reg_map_window_1 = reg_w1_G366PD,
    },
    [EPSON_G330PDG0] = {
        .product_id = "epsonG330PDG0",
        .channels = epson_Temp_Gyro_Accel2Range_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel2Range_Cnt_Time),
        .flags = HAS_PROD_ID |
                HAS_SERIAL_NUMBER |
                BURST_DIAG_STAT |
                HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(15152), /* 66 LBS/(deg/s) => 1/66 => 0.0151515 (deg/s)/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(250), /* 8G: 4 LSB/mG => 1/4 => 0.250 mG/LSB */ /* 16G: 2 LSB/mG => 1/2 => 0.500 mG/LSB */
        .accel_scale_mode_cnt = 2,
        .accel_scale_mode = accel_scale_two_ranges,
        .temp_scale_nano = 3906250, /* 0.00390625 */
        .temp_offset = 0, /* Output = 0 @ +25'C */
        .sample_freq_mode = common_sample_freq_mode,
        .sample_freq_mode_cnt = common_sample_freq_mode_cnt,
        .lpf_3db_freq_mode = common_lpf_3db_freq_mode,
        .lpf_3db_freq_mode_cnt = common_lpf_3db_freq_mode_cnt,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq,
        .reg_map_window_0 = reg_w0_G366PD,
        .reg_map_window_1 = reg_w1_G366PD,
    },
    [EPSON_G370PDF1] = {
        .product_id = "epsonG370PDF1",
        .channels = epson_Temp_Gyro_Accel_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel_Cnt_Time),
        .flags = HAS_PROD_ID |
                HAS_SERIAL_NUMBER |
                BURST_DIAG_STAT |
                HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(15152), /* 66 LBS/(deg/s) => 1/66 => 0.0151515 (deg/s)/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(400), /* 2.5 LSB/mG => 1/2.5 => 0.4 mG/LSB */
        .accel_scale_mode_cnt = 1,
        .accel_scale_mode = NULL,
        .temp_scale_nano = -3791800, /* -0.0037918 */
        .temp_offset = -2634, /* Output = 2634(0x0A4A) @ +25'C */
        .sample_freq_mode = common_sample_freq_mode,
        .sample_freq_mode_cnt = common_sample_freq_mode_cnt,
        .lpf_3db_freq_mode = common_lpf_3db_freq_mode,
        .lpf_3db_freq_mode_cnt = common_lpf_3db_freq_mode_cnt,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq,
        .reg_map_window_0 = reg_w0_G370,
        .reg_map_window_1 = reg_w1_G370,
    },
    [EPSON_G370PDS0] = {
        .product_id = "epsonG370PDS0",
        .channels = epson_Temp_Gyro_Accel_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel_Cnt_Time),
        .flags = HAS_PROD_ID |
                HAS_SERIAL_NUMBER |
                BURST_DIAG_STAT |
                HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(6667), /* 150 LBS/(deg/s) => 1/150 => 0.00666666666667 (deg/s)/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(400), /* 2.5 LSB/mG => 1/2.5 => 0.400 mG/LSB */
        .accel_scale_mode_cnt = 1,
        .accel_scale_mode = NULL,
        .temp_scale_nano = -3791800, /* -0.0037918 */
        .temp_offset = -2634, /* Output = 2634(0x0A4A) @ +25'C*/
        .sample_freq_mode = common_sample_freq_mode,
        .sample_freq_mode_cnt = common_sample_freq_mode_cnt,
        .lpf_3db_freq_mode = (struct epson_lpf_3db_freq[]) {
        /*  | reg |    label     | tap| fc | opt < ('111b': show_avail:'01b', O_FREQ_LIST1:'010b', O_FREQ_LIST1:'100b')|   */
            /* common block */
            { 0x00, "tap0",          0,   0, O_SHOW_AVAIL | O_FREQ_LIST1 | O_FREQ_LIST2 },
            { 0x01, "tap2",          2,   0, O_SHOW_AVAIL | O_FREQ_LIST1 | O_FREQ_LIST2 },
            { 0x02, "tap4",          4,   0, O_SHOW_AVAIL | O_FREQ_LIST1 | O_FREQ_LIST2 },
            { 0x03, "tap8",          8,   0, O_SHOW_AVAIL | O_FREQ_LIST1 | O_FREQ_LIST2 },
            { 0x04, "tap16",        16,   0, O_SHOW_AVAIL | O_FREQ_LIST1 | O_FREQ_LIST2 },
            { 0x05, "tap32",        32,   0, O_SHOW_AVAIL | O_FREQ_LIST1 | O_FREQ_LIST2 },
            { 0x06, "tap64",        64,   0, O_SHOW_AVAIL | O_FREQ_LIST1 | O_FREQ_LIST2 },
            { 0x07, "tap128",      128,   0, O_SHOW_AVAIL | O_FREQ_LIST1 | O_FREQ_LIST2 },
            /* show avail block */
            { 0x08, "tap32fc25",    32,  25, O_SHOW_AVAIL | O_FREQ_LIST2 },
            { 0x08, "tap32fc50",    32,  50, O_SHOW_AVAIL | O_FREQ_LIST1 },
            { 0x09, "tap32fc100",   32, 100, O_SHOW_AVAIL | O_FREQ_LIST1 },
            { 0x0A, "tap32fc200",   32, 200, O_SHOW_AVAIL | O_FREQ_LIST1 },
            { 0x0B, "tap32fc400",   32, 400, O_SHOW_AVAIL | O_FREQ_LIST1 },
            { 0x0C, "tap64fc25",    64,  25, O_SHOW_AVAIL | O_FREQ_LIST2 },
            { 0x0C, "tap64fc50",    64,  50, O_SHOW_AVAIL | O_FREQ_LIST1 },
            { 0x0D, "tap64fc100",   64, 100, O_SHOW_AVAIL | O_FREQ_LIST1 },
            { 0x0E, "tap64fc200",   64, 200, O_SHOW_AVAIL | O_FREQ_LIST1 },
            { 0x0F, "tap64fc400",   64, 400, O_SHOW_AVAIL | O_FREQ_LIST1 },
            { 0x10, "tap128fc25",  128,  25, O_SHOW_AVAIL | O_FREQ_LIST2 },
            { 0x10, "tap128fc50",  128,  50, O_SHOW_AVAIL | O_FREQ_LIST1 },
            { 0x11, "tap128fc100", 128, 100, O_SHOW_AVAIL | O_FREQ_LIST1 },
            { 0x12, "tap128fc200", 128, 200, O_SHOW_AVAIL | O_FREQ_LIST1 },
            { 0x13, "tap128fc400", 128, 400, O_SHOW_AVAIL | O_FREQ_LIST1 },
            /* do not show avail block */
            { 0x09, "tap32fc50",    32,  50, O_FREQ_LIST2 },
            { 0x0A, "tap32fc100",   32, 100, O_FREQ_LIST2 },
            { 0x0B, "tap32fc200",   32, 200, O_FREQ_LIST2 },
            { 0x0D, "tap64fc50",    64,  50, O_FREQ_LIST2 },
            { 0x0E, "tap64fc100",   64, 100, O_FREQ_LIST2 },
            { 0x0F, "tap64fc200",   64, 200, O_FREQ_LIST2 },
            { 0x11, "tap128fc50",  128,  50, O_FREQ_LIST2 },
            { 0x12, "tap128fc100", 128, 100, O_FREQ_LIST2 },
            { 0x13, "tap128fc200", 128, 200, O_FREQ_LIST2 },
        },
        .lpf_3db_freq_mode_cnt = 32,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq_odd,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq_odd,
        .reg_map_window_0 = reg_w0_G370,
        .reg_map_window_1 = reg_w1_G370,
    },
    [EPSON_G370PDG0] = {
        .product_id = "epsonG370PDG0",
        .channels = epson_Temp_Gyro_Accel2Range_Cnt_Time, //epson_Temp_Gyro_Accel_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel2Range_Cnt_Time), //(epson_Temp_Gyro_Accel_Cnt_Time),
        .flags = HAS_PROD_ID |
                HAS_SERIAL_NUMBER |
                BURST_DIAG_STAT |
                HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(15152), /* 66 LBS/(deg/s) => 1/66 => 0.0151515 (deg/s)/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(250), /* 8G: 4 LSB/mG => 1/4 => 0.250 mG/LSB */ /* 16G: 2 LSB/mG => 1/2 => 0.500 mG/LSB */
        .accel_scale_mode_cnt = 2,
        .accel_scale_mode = accel_scale_two_ranges,
        .temp_scale_nano = 3906250, /* 0.00390625 */
        .temp_offset = 0, /* Output = 0 @ +25'C */
        .sample_freq_mode = common_sample_freq_mode,
        .sample_freq_mode_cnt = common_sample_freq_mode_cnt,
        .lpf_3db_freq_mode = common_lpf_3db_freq_mode,
        .lpf_3db_freq_mode_cnt = common_lpf_3db_freq_mode_cnt,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq,
        .reg_map_window_0 = reg_w0_G370,
        .reg_map_window_1 = reg_w1_G366PD,
    },
    [EPSON_G370PDT0] = {
        .product_id = "epsonG370PDT0",
        .channels = epson_Temp_Gyro_Accel2Range_Cnt_Time,
        .num_channels = ARRAY_SIZE(epson_Temp_Gyro_Accel2Range_Cnt_Time),
        .flags = HAS_PROD_ID |
                HAS_SERIAL_NUMBER |
                BURST_DIAG_STAT |
                HAS_FW_VER,
        .gyro_scale_micro = IIO_DEGREE_TO_RAD(6667),  /* 150 LBS/('/s) => 1/150 => 0.00666666666667 '/s/LSB */
        .accel_scale_micro = IIO_G_TO_M_S_2(250), /* 8G: 4 LSB/mG => 1/4 => 0.250 mG/LSB */ /* 16G: 2 LSB/mG => 1/2 => 0.500 mG/LSB */
        .accel_scale_mode_cnt = 2,
        .accel_scale_mode = accel_scale_two_ranges,
        .temp_scale_nano = 3906250, /* 0.00390625 */
        .temp_offset = 0, /* Output = 0 @ +25'C */
        .sample_freq_mode = common_sample_freq_mode,
        .sample_freq_mode_cnt = common_sample_freq_mode_cnt,
        .lpf_3db_freq_mode = common_lpf_3db_freq_mode,
        .lpf_3db_freq_mode_cnt = common_lpf_3db_freq_mode_cnt,
        .set_lpf_3db_freq = epsonimu_set_lpf_3db_freq,
        .get_lpf_3db_freq = epsonimu_get_lpf_3db_freq,
        .reg_map_window_0 = reg_w0_G370,
        .reg_map_window_1 = reg_w1_G366PD,
    },
};



//----------------------------------------------------------------------
// product_id_show()
// Reads the product ID from hardware registers,
// formats it as a human‐readable string and writes it into the buffer
//
// Parameters:
// - dev:  pointer to the struct device
// - attr: pointer to the device_attribute (unused)
// - buf:  buffer to fill with the attribute’s text representation
//
// Return: the number of bytes written into buf on success,
//                  or a negative errno code on failure.
//
//----------------------------------------------------------------------
static ssize_t product_id_show(struct device* dev,
    struct device_attribute* attr,
    char* buf)
{
    struct iio_dev* indio_dev = dev_to_iio_dev(dev);
    struct epsonimu_state* st = iio_priv(indio_dev);
    uint16_t product_id[4];
    ssize_t ret, i, len = 0;

    ret = epson_read_reg_16(&st->epson, REG_PROD_ID1, &product_id[0]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_PROD_ID2, &product_id[1]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_PROD_ID3, &product_id[2]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_PROD_ID4, &product_id[3]);
    if (ret < 0)
        return ret;

    for (i = 0; i < 4; i++)
        len += scnprintf(buf + len, PAGE_SIZE - len, "%c%c",
            (product_id[i] & 0x00FF), (product_id[i] & 0xFF00) >> 8);

    len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

    return len;
}
static IIO_DEVICE_ATTR_RO(product_id, 0);



//----------------------------------------------------------------------
// serial_number_show()
// Reads the device’s serial number from hardware registers,
// formats it as a human‐readable string and writes it into the buffer
//
// Parameters:
// - dev:  pointer to the struct device
// - attr: pointer to the device_attribute (unused)
// - buf:  buffer to fill with the serial number text representation
//
// Return: the number of bytes written into buf on success,
//                  or a negative errno code on failure.
//
//----------------------------------------------------------------------
static ssize_t serial_number_show(struct device* dev,
    struct device_attribute* attr,
    char* buf)
{
    struct iio_dev* indio_dev = dev_to_iio_dev(dev);
    struct epsonimu_state* st = iio_priv(indio_dev);
    uint16_t serial_number[4];
    ssize_t ret, i, len = 0;

    ret = epson_read_reg_16(&st->epson, REG_SERIAL_NUM1, &serial_number[0]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_SERIAL_NUM2, &serial_number[1]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_SERIAL_NUM3, &serial_number[2]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_SERIAL_NUM4, &serial_number[3]);
    if (ret < 0)
        return ret;

    for (i = 0; i < 4; i++)
        len += scnprintf(buf + len, PAGE_SIZE - len, "%c%c",
            (serial_number[i] & 0x00FF), (serial_number[i] & 0xFF00) >> 8);

    len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

    return len;
}
static IIO_DEVICE_ATTR_RO(serial_number, 0);



//----------------------------------------------------------------------
// firmware_version_show()
// Reads the device’s firmware version from hardware registers,
// formats it as a human‐readable string and writes it into the buffer
//
// Parameters:
// - dev:  pointer to the struct device
// - attr: pointer to the device_attribute (unused)
// - buf:  buffer to fill with the firmware version text representation
//
// Return: the number of bytes written into buf on success,
//                  or a negative errno code on failure.
//
//----------------------------------------------------------------------
static ssize_t firmware_version_show(struct device* dev,
    struct device_attribute* attr,
    char* buf)
{
    struct iio_dev* indio_dev = dev_to_iio_dev(dev);
    struct epsonimu_state* st = iio_priv(indio_dev);
    uint16_t firmware_version;
    ssize_t ret, len = 0;

    //get firmware version
    ret = epson_read_reg_16(&st->epson, REG_VERSION, &firmware_version);
    if (ret < 0)
        return ret;

    len = scnprintf(buf, PAGE_SIZE, "%x\n", firmware_version);

    return len;
}
static IIO_DEVICE_ATTR_RO(firmware_version, 0);



//----------------------------------------------------------------------
// version_show()
// Retrieves the driver’s version identifier (EPSON_DRV_VERSION)
// appends a newline, and writes it into the buffer.
//
// Parameters:
// - dev:  pointer to the struct device
// - attr: pointer to the device_attribute (unused)
// - buf:  buffer to fill with the driver’s version text representation
//
// Return: the number of bytes written into buf on success,
//                  or a negative errno code on failure.
//
//----------------------------------------------------------------------
static ssize_t version_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    return sysfs_emit(buf, "%s\n", EPSON_DRV_VERSION);
}
static DEVICE_ATTR_RO(version);


static struct attribute* epsonimu_attrs[] = {
    &iio_dev_attr_product_id.dev_attr.attr,
    &iio_dev_attr_serial_number.dev_attr.attr,
    &iio_dev_attr_firmware_version.dev_attr.attr,
    &dev_attr_version.attr,
    NULL,
};

static const struct attribute_group epsonimu_attrs_group = {
    .attrs = epsonimu_attrs,
};

static const struct iio_info epsonimu_info = {
    .read_raw = &epsonimu_read_raw,
    .write_raw = &epsonimu_write_raw,
    .read_avail = epsonimu_read_avail,
    .update_scan_mode = epsonimu_update_scan_mode,
    .attrs = &epsonimu_attrs_group,
    .debugfs_reg_access = epson_debugfs_reg_access,
};

// Precalculate the number of chip entrues in the epsonimu_chips table
#define NumIMUChipsSupported sizeof epsonimu_chips / sizeof epsonimu_chips[0]


//----------------------------------------------------------------------
// epsonimu_setup_spi_and_get_device_id()
// Sets up the SPI port
// Identifies the IMU by reading the Product ID registers, and
// locates its variant in epsonimu_chips array.
// Then performs the initial setup of the IMU by calling epson_initial_startup
// Note: This function must come after the definition of the
// epsonimu_chips table
//
// Parameters:
// - indio_dev: the IIO device
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epsonimu_setup_spi_and_get_device_id(struct iio_dev* indio_dev)
{
    struct epsonimu_state* st = iio_priv(indio_dev);
    int ret;
    uint16_t product_id[4];
    char prodbuf[16];
    size_t len;
    int i;

    // setup the SPI for default settings
    // from the Device Tree Overlay
    st->epson.spi->max_speed_hz = st->epson.spi_max_freq / 2;
    dev_info(&indio_dev->dev, "SPI speed %d\n", st->epson.spi->max_speed_hz);

    st->epson.spi->mode = SPI_MODE_3;
    spi_setup(st->epson.spi);

    // make sure reset happened correctly
    ret = epson_reset(&st->epson);
    if (ret) {
        dev_err(&st->epson.spi->dev, "IMU Not Ready\n");
        return ret;
    }

    // read of all 4 Product ID registers and match to the chip variant structure

    ret = epson_read_reg_16(&st->epson, REG_PROD_ID1, &product_id[0]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_PROD_ID2, &product_id[1]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_PROD_ID3, &product_id[2]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_PROD_ID4, &product_id[3]);
    if (ret < 0)
        return ret;

    len = snprintf(prodbuf, sizeof(prodbuf), "epson%c%c%c%c%c%c%c%c",
        (product_id[0] & 0x00FF), (product_id[0] & 0xFF00) >> 8,
        (product_id[1] & 0x00FF), (product_id[1] & 0xFF00) >> 8,
        (product_id[2] & 0x00FF), (product_id[2] & 0xFF00) >> 8,
        (product_id[3] & 0x00FF), (product_id[3] & 0xFF00) >> 8);
    dev_info(&indio_dev->dev, "Product ID %s\n", prodbuf);

    for (i = 0; i < NumIMUChipsSupported; i++) {
        if (memcmp(epsonimu_chips[i].product_id, prodbuf, 14) == 0) {
            dev_info(&indio_dev->dev, "Found a match epsonimu_chips[%d]\n", i);
            st->variant = &epsonimu_chips[i];
            break;
        }
    }

    if (i >= NumIMUChipsSupported) {
        // if the product_id not found, and proto enabled default
        if (st->epson.proto == true) {
            dev_info(&indio_dev->dev, "No match found, Proto Enabled defaulting to G320\n");
            st->variant = &epsonimu_chips[0];
        }
        // otherwise leave with an error
        else {
            dev_info(&indio_dev->dev, "No match found, Proto not enabled temporarily defaulting to G320\n");
            st->variant = &epsonimu_chips[0];
            return -EINVAL;
        }
    }

    //do some initial startup
    ret = epson_initial_startup(&st->epson);
    if (!ret) {
        //display the Product ID and IRQ used
        dev_info(&indio_dev->dev, "prod_id %s at CS%u (irq %d)\n",
            prodbuf, (unsigned int)*st->epson.spi->chip_select, st->epson.spi->irq);
    }

    return ret;
}



//----------------------------------------------------------------------
// epsonimu_status_error_msgs[]
// defines messages for the possible Epson IMUs errors
//
//----------------------------------------------------------------------
static const char* const epsonimu_status_error_msgs[] = {
    [EPSON_DIAG_STAT_XGYRO_FAIL] = "X-axis gyroscope self-test failure",
    [EPSON_DIAG_STAT_YGYRO_FAIL] = "Y-axis gyroscope self-test failure",
    [EPSON_DIAG_STAT_ZGYRO_FAIL] = "Z-axis gyroscope self-test failure",
    [EPSON_DIAG_STAT_ACCL_FAIL] = "accelerometer self-test failure",
    [EPSON_DIAG_DLTA_OVF] = "Delta Angle Overflow Error",
    [EPSON_DIAG_DLTV_OVF] = "Delta Velocity Overflow Error",
    [EPSON_DIAG_HARD_ERR1] = "Hardware Startup Check Error1",
    [EPSON_DIAG_HARD_ERR0] = "Hardware Startup Check Error0",
    [EPSON_DIAG_SPI_OVF] = "SPI Overflow Error",
    [EPSON_DIAG_UART_OVF] = "UART Overflow Error",
    [EPSON_DIAG_FLASH_ERR] = "Flash Test error",
    [EPSON_DIAG_STAT_SELF_TEST_ERR] = "Self test error",
    [EPSON_DIAG_FLASH_BU_ERR] = "Flash Backup failed",
};



//----------------------------------------------------------------------
// epsonimu_data
// data for the Epson IMUs
//
//----------------------------------------------------------------------
static const struct epson_data epsonimu_data = {
    .mode_ctrl_reg = REG_MODE_CTRL_HI,
    .msc_ctrl_reg = REG_MSC_CTRL_LO,
    .diag_stat_reg = REG_DIAG_STAT,
    .glob_cmd_reg = REG_GLOB_CMD_LO,

    .read_delay = EPSON_READRATE, // Updated per G320 spec
    .write_delay = EPSON_READRATE, // Updated per G320 spec

    .startup_delay = EPSON_POWER_ON_DELAY,

    .status_error_msgs = epsonimu_status_error_msgs,
    .status_error_mask = EPSON_DIAG_STAT_XGYRO_FAIL |
            EPSON_DIAG_STAT_YGYRO_FAIL |
            EPSON_DIAG_STAT_ZGYRO_FAIL |
            EPSON_DIAG_STAT_ACCL_FAIL |
            EPSON_DIAG_DLTA_OVF |
            EPSON_DIAG_DLTV_OVF |
            EPSON_DIAG_HARD_ERR1 |
            EPSON_DIAG_HARD_ERR0 |
            EPSON_DIAG_SPI_OVF |
            EPSON_DIAG_UART_OVF |
            EPSON_DIAG_FLASH_ERR |
            EPSON_DIAG_STAT_SELF_TEST_ERR |
            EPSON_DIAG_FLASH_BU_ERR,
    .has_paging = true,
};



//----------------------------------------------------------------------
// epsonimu_setup_chan_mask()
// Sets up the channel mask for the Epson IMU
// Used by NON_BURST mode
//
// Parameters:
// - st: IMU state
//
//----------------------------------------------------------------------
static void epsonimu_setup_chan_mask(struct epsonimu_state* st)
{
    const struct epsonimu_chip_info* chip_info = st->variant;
    unsigned i;

    for (i = 0; i < chip_info->num_channels; i++) {
        const struct iio_chan_spec* ch = &chip_info->channels[i];

        if (ch->scan_index >= 0 && ch->scan_index != SCAN_TIMESTAMP)
            st->avail_scan_mask[0] |= BIT(ch->scan_index);
    }
}



#ifdef CONFIG_DEBUG_FS
//----------------------------------------------------------------------
// epsonimu_show_serial_number()
// DebugFS routine to display serial number of Epson IMU
//
// Parameters:
// - file:    the open file instance for the Epson IMU character device
// - userbuf: user‐space buffer to fill with the serial number string
// - count:   maximum bytes to copy into userbuf
// - ppos:    pointer to the file offset (loff_t) for sequential reads
//
// Return: the number of bytes written into buf on success,
//                  or a negative errno code on failure.
//
// Example Usage: cat /sys/kernel/debug/iio/iio:device0/serial_number
//
// Notes: The exact usage will depend on the kernel, iio device number, etc.
//        Only available when DEBUG_FS is enabled
//
//----------------------------------------------------------------------
static ssize_t epsonimu_show_serial_number(struct file* file,
    char __user* userbuf, size_t count, loff_t* ppos)
{
    struct epsonimu_state* st = file->private_data;
    uint16_t serial_number[4];
    char buf[16];
    size_t len;
    int ret;

    ret = epson_read_reg_16(&st->epson, REG_SERIAL_NUM1, &serial_number[0]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_SERIAL_NUM2, &serial_number[1]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_SERIAL_NUM3, &serial_number[2]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_SERIAL_NUM4, &serial_number[3]);
    if (ret < 0)
        return ret;

    len = snprintf(buf, sizeof(buf), "sn: %c%c%c%c%c%c%c%c\n",
        (serial_number[0] & 0x00FF), (serial_number[0] & 0xFF00) >> 8,
        (serial_number[1] & 0x00FF), (serial_number[1] & 0xFF00) >> 8,
        (serial_number[2] & 0x00FF), (serial_number[2] & 0xFF00) >> 8,
        (serial_number[3] & 0x00FF), (serial_number[3] & 0xFF00) >> 8);

    return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static const struct file_operations epsonimu_serial_number_fops = {
    .open = simple_open,
    .read = epsonimu_show_serial_number,
    .llseek = default_llseek,
    .owner = THIS_MODULE,
};



//----------------------------------------------------------------------
// epsonimu_show_product_id()
// DebugFS routine to display product ID of Epson IMU
//
// Parameters:
// - file:    the open file instance for the Epson IMU character device
// - userbuf: user‐space buffer to fill with the serial number string
// - count:   maximum bytes to copy into userbuf
// - ppos:    pointer to the file offset (loff_t) for sequential reads
//
// Return: the number of bytes written into buf on success,
//                  or a negative errno code on failure.
//
// Example Usage: cat /sys/kernel/debug/iio/iio:device0/product_id
//
// Notes: The exact usage will depend on the kernel, iio device number, etc.
//        Only available when DEBUG_FS is enabled
//
//----------------------------------------------------------------------
static ssize_t epsonimu_show_product_id(struct file* file,
    char __user* userbuf, size_t count, loff_t* ppos)
{
    struct epsonimu_state* st = file->private_data;
    uint16_t product_id[4];
    char buf[16];
    size_t len;
    int ret;

    ret = epson_read_reg_16(&st->epson, REG_PROD_ID1, &product_id[0]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_PROD_ID2, &product_id[1]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_PROD_ID3, &product_id[2]);
    if (ret < 0)
        return ret;

    ret = epson_read_reg_16(&st->epson, REG_PROD_ID4, &product_id[3]);
    if (ret < 0)
        return ret;

    len = snprintf(buf, sizeof(buf), "ID: %c%c%c%c%c%c%c%c\n",
        (product_id[0] & 0x00FF), (product_id[0] & 0xFF00) >> 8,
        (product_id[1] & 0x00FF), (product_id[1] & 0xFF00) >> 8,
        (product_id[2] & 0x00FF), (product_id[2] & 0xFF00) >> 8,
        (product_id[3] & 0x00FF), (product_id[3] & 0xFF00) >> 8);

    return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static const struct file_operations epsonimu_product_id_fops = {
    .open = simple_open,
    .read = epsonimu_show_product_id,
    .llseek = default_llseek,
    .owner = THIS_MODULE,
};



//----------------------------------------------------------------------
// epsonimu_show_firmware_version()
// DebugFS routine to display firmware version of Epson IMU
//
// Parameters:
// - arg: opaque pointer passed from debugfs_create_file(), must be a
//       struct epsonimu_state*
// - val: pointer to a 64-bit value where the firmware version will be stored
//
// Return: 0 on success, or a negative errno on failure
//
// Example Usage: cat /sys/kernel/debug/iio/iio:device0/firmware_version
//
// Notes: The exact usage will depend on the kernel, iio device number, etc.
//        Only available when DEBUG_FS is enabled
//
//----------------------------------------------------------------------
static int epsonimu_show_firmware_version(void* arg, uint64_t* val)
{
    struct epsonimu_state* st = arg;
    uint16_t firmware_version;
    int ret;

    //get firmware version
    ret = epson_read_reg_16(&st->epson, REG_VERSION, &firmware_version);
    if (ret < 0)
        return ret;

    *val = firmware_version;

    return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(epsonimu_firmware_version_fops,
    epsonimu_show_firmware_version, NULL, "%llx\n");



//----------------------------------------------------------------------
// epsonimu_show_diagnose()
// DebugFS routine to display diagnose flags (if error) of IMU
//
// Parameters:
// - file:    the open file instance for the Epson IMU character device
// - userbuf: user‐space buffer to fill with the serial number string
// - count:   maximum bytes to copy into userbuf
// - ppos:    pointer to the file offset (loff_t) for sequential reads
//
// Return: the number of bytes written into buf on success,
//                  or a negative errno code on failure.
//
// Example Usage: cat /sys/kernel/debug/iio/iio:device0/diagnose
//
// Notes: The exact usage will depend on the kernel, iio device number, etc.
//        Only available when DEBUG_FS is enabled
//
//----------------------------------------------------------------------
static ssize_t epsonimu_show_diagnose(struct file* file,
    char __user* userbuf, size_t count, loff_t* ppos)
{
    struct epsonimu_state* st = file->private_data;

    uint16_t flag_reg;
    char buf[256];
    size_t len = 0;
    size_t lentot = 0;
    int ret;

    //check the Flags register
    ret = epson_read_reg_16(&st->epson, REG_FLAG, &flag_reg);
    if (ret < 0)
        return ret;

    //if Error All flag is set
    if (flag_reg & 0x0001) {
        uint16_t diag_stat_reg;

        //read the diagnostics register
        ret = epson_read_reg_16(&st->epson, REG_DIAG_STAT, &diag_stat_reg);
        if (ret < 0)
            return ret;

        len = snprintf(buf, sizeof(buf), "Diagnostics: Errors detected -- flags = %x\n", diag_stat_reg);
        lentot = lentot + len;

        //add error messages based on the diagnostics flags
        if (diag_stat_reg & EPSON_DIAG_STAT_XGYRO_FAIL) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tX-Gyro Self Test Error\n");
            lentot = lentot + len;
        }
        if (diag_stat_reg & EPSON_DIAG_STAT_YGYRO_FAIL) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tY-Gyro Self Test Error\n");
            lentot = lentot + len;
        }
        if (diag_stat_reg & EPSON_DIAG_STAT_ZGYRO_FAIL) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tZ-Gyro Self Test Error\n");
            lentot = lentot + len;
        }
        if (diag_stat_reg & EPSON_DIAG_STAT_ACCL_FAIL) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tAccelerometer Self Test Error\n");
            lentot = lentot + len;
        }
        if (diag_stat_reg & EPSON_DIAG_DLTA_OVF) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tDelta Angle Overflow\n");
            lentot = lentot + len;
        }
        if (diag_stat_reg & EPSON_DIAG_DLTV_OVF) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tDelta Velocity Overflow\n");
            lentot = lentot + len;
        }
        if (diag_stat_reg & EPSON_DIAG_HARD_ERR1) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tHardware Check Error\n");
            lentot = lentot + len;
        }
        else if (diag_stat_reg & EPSON_DIAG_HARD_ERR0) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tHardware Check Error\n");
            lentot = lentot + len;
        }
        if (diag_stat_reg & EPSON_DIAG_SPI_OVF) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tSPI Overflow\n");
            lentot = lentot + len;
        }
        if (diag_stat_reg & EPSON_DIAG_UART_OVF) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tUART Overflow\n");
            lentot = lentot + len;
        }
        if (diag_stat_reg & EPSON_DIAG_FLASH_ERR) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tFlash Test Error\n");
            lentot = lentot + len;
        }
        if (diag_stat_reg & EPSON_DIAG_STAT_SELF_TEST_ERR) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tSelf Test Error\n");
            lentot = lentot + len;
        }
        if (diag_stat_reg & EPSON_DIAG_FLASH_BU_ERR) {
            len = snprintf(buf + lentot, sizeof(buf) - lentot, "\tFlash Backup Error\n");
            lentot = lentot + len;
        }
    }
    else
        len = snprintf(buf, sizeof(buf), "Diagnostics: No errors detected.\n");

    lentot = lentot + len;

    return simple_read_from_buffer(userbuf, count, ppos, buf, lentot);
}

static const struct file_operations epsonimu_diagnose_fops = {
    .open = simple_open,
    .read = epsonimu_show_diagnose,
    .llseek = default_llseek,
    .owner = THIS_MODULE,
};



//----------------------------------------------------------------------
// epsonimu_show_regdump()
// DebugFS routine to perform Register Dump of IMU
//
// Parameters:
// - file:    the open file instance for the Epson IMU character device
// - userbuf: user‐space buffer to fill with the serial number string
// - count:   maximum bytes to copy into userbuf
// - ppos:    pointer to the file offset (loff_t) for sequential reads
//
// Return: the number of bytes written into buf on success,
//                  or a negative errno code on failure.
//
// Example Usage: cat /sys/kernel/debug/iio/iio:device0/regdump
//
// Notes: The exact usage will depend on the kernel, iio device number, etc.
//        Only available when DEBUG_FS is enabled
//
//----------------------------------------------------------------------
static ssize_t epsonimu_show_regdump(struct file* file,
    char __user* userbuf, size_t count, loff_t* ppos)
{
    struct epsonimu_state* st = file->private_data;

    uint16_t regval;
    char *buf;
    size_t len = 0;
    size_t lentot = 0;
    int ret, i;
    const size_t bufSize = 8192;
    
    buf = kmalloc(bufSize, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    for (int w = 0; w < 2; w++) {
        const struct epson_register_map* reg_map = (w == 0) ? st->variant->reg_map_window_0 : st->variant->reg_map_window_1;
        len = snprintf(buf + lentot, bufSize - lentot, "\nWindow %i\n========\n", w);
        lentot = lentot + len;
        len = snprintf(buf + lentot, bufSize - lentot, "Reg Value----|Reg Name");
        lentot = lentot + len;
        len = strlen(reg_map[0].label);
        len -= strlen("Reg Name");
        memset(buf + lentot, '-', len);
        lentot = lentot + len;
        len = snprintf(buf + lentot, bufSize - lentot, "|R/W|Dflt\n");
        lentot = lentot + len;

        for (i = 0; reg_map[i].label != NULL; i++) {
            ret = epson_read_reg_16(&st->epson, reg_map[i].addr, &regval);
            if (ret < 0)
                break;
            len = snprintf(buf + lentot, bufSize - lentot, "REG[%02x]=%04x :%s %s %s\n",
                    reg_map[i].addr & 0x00ff,
                    regval,
                    reg_map[i].label,
                    reg_map[i].rw,
                    (reg_map[i].default_value == NULL) ? "" : reg_map[i].default_value);
            lentot = lentot + len;
        }
    }

    //end the register dump
    len = snprintf(buf + lentot, bufSize - lentot, "\nDump size:%zu, Buffer size:%zu\n\n", lentot + 27 + 8, bufSize);
    lentot = lentot + len;

    ret = simple_read_from_buffer(userbuf, count, ppos, buf, lentot);
    kfree(buf);

    return ret;
}

static const struct file_operations epsonimu_regdump_fops = {
    .open = simple_open,
    .read = epsonimu_show_regdump,
    .llseek = default_llseek,
    .owner = THIS_MODULE,
};



//----------------------------------------------------------------------
// epsonimu_debugfs_init()
// Initializes the DebugFS routines for the IMU
//
// Parameters:
// - indio_dev: the IIO device
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epsonimu_debugfs_init(struct iio_dev* indio_dev)
{
    struct epsonimu_state* st = iio_priv(indio_dev);
    struct dentry* debugfs_dentry = iio_get_debugfs_dentry(indio_dev);

    if (st->variant->flags & HAS_SERIAL_NUMBER)
        debugfs_create_file("serial_number", 0400,
            debugfs_dentry, st,
            &epsonimu_serial_number_fops);

    if (st->variant->flags & HAS_PROD_ID)
        debugfs_create_file("product_id", 0400,
            debugfs_dentry, st,
            &epsonimu_product_id_fops);

    if (st->variant->flags & HAS_FW_VER)
        debugfs_create_file("firmware_version", 0400,
            debugfs_dentry, st,
            &epsonimu_firmware_version_fops);

    debugfs_create_file("diagnose", 0400,
        debugfs_dentry, st,
        &epsonimu_diagnose_fops);

    debugfs_create_file("regdump", 0400,
        debugfs_dentry, st,
        &epsonimu_regdump_fops);

    return 0;
}
#else
static int epsonimu_debugfs_init(struct iio_dev* indio_dev)
{
    return 0;
}
#endif



//----------------------------------------------------------------------
// epsonimu_probe()
// Probes the Epson IMU spi device
//
// Parameters:
// - spi: pointer to the SPI device instance
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epsonimu_probe(struct spi_device* spi)
{
    struct epsonimu_state* st;
    struct iio_dev* indio_dev;
    int ret;

    indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));

    if (indio_dev == NULL)
        return -ENOMEM;

    dev_info(&indio_dev->dev, "Epson IMU Driver version %s\n", EPSON_DRV_VERSION);

    st = iio_priv(indio_dev);
    //used for removal purposes
    spi_set_drvdata(spi, indio_dev);

    //setup the IIO
    // put some defaults in place that get adjusted after the
    // IMU chip has been ID'd
    st->variant = &epsonimu_chips[spi_get_device_id(spi)->driver_data];
    indio_dev->dev.parent = &spi->dev;
    indio_dev->info = &epsonimu_info;
    indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
    indio_dev->name = spi_get_device_id(spi)->name;
    indio_dev->channels = st->variant->channels;
    indio_dev->num_channels = st->variant->num_channels;

    ret = epson_init(&st->epson, indio_dev, spi, &epsonimu_data);
    if (ret)
        return ret;

    ret = epsonimu_setup_spi_and_get_device_id(indio_dev);
    if (ret)
        return ret;

    // fix some values that are dependent on the product id
    indio_dev->name = st->variant->product_id;
    indio_dev->channels = st->variant->channels;
    indio_dev->num_channels = st->variant->num_channels;
    dev_info(&indio_dev->dev, "Product ID %s\n", indio_dev->name);

    if (!(st->variant->flags & NO_BURST)) {
        epsonimu_setup_chan_mask(st);
        indio_dev->available_scan_masks = st->avail_scan_mask;
    }

    ret = epson_setup_buffer_and_trigger(&st->epson, indio_dev,
        NULL,
        NULL);
    if (ret)
        return ret;

    ret = iio_device_register(indio_dev);
    if (ret)
        goto error_cleanup_buffer;

    epsonimu_debugfs_init(indio_dev);
    return 0;

error_cleanup_buffer:
    epson_cleanup_buffer_and_trigger(&st->epson, indio_dev);
    return ret;
}



//----------------------------------------------------------------------
// epsonimu_remove()
// Removes the Epson IMU spi device
//
// Parameters:
// - spi: pointer to the SPI device instance
//
//----------------------------------------------------------------------
static void epsonimu_remove(struct spi_device* spi)
{
    struct iio_dev* indio_dev = spi_get_drvdata(spi);
    struct epsonimu_state* st = iio_priv(indio_dev);

    iio_device_unregister(indio_dev);
    epson_cleanup_buffer_and_trigger(&st->epson, indio_dev);
}


static const struct spi_device_id epsonimu_id[] = {

    {"epsonG320PDG0", EPSON_G320PDG0},
    {"epsonG354PDH0", EPSON_G354PDH0},
    {"epsonG355QDG0", EPSON_G355QDG0},
    {"epsonG364PDC0", EPSON_G364PDC0},
    {"epsonG364PDCA", EPSON_G364PDCA},
    {"epsonG365PDC1", EPSON_G365PDC1},
    {"epsonG365PDF1", EPSON_G365PDF1},
    {"epsonG366PDG0", EPSON_G366PDG0},
    {"epsonG330PDG0", EPSON_G330PDG0},
    {"epsonG370PDF1", EPSON_G370PDF1},
    {"epsonG370PDS0", EPSON_G370PDS0},
    {"epsonG370PDG0", EPSON_G370PDG0},
    {"epsonG370PDT0", EPSON_G370PDT0},
    {}
};
MODULE_DEVICE_TABLE(spi, epsonimu_id);


static const struct of_device_id epson_imu_of_match[] = {

    { .compatible = "epson,epsonG320PDG0", },
    { .compatible = "epson,epsonG354PDH0", },
    { .compatible = "epson,epsonG355QDG0", },
    { .compatible = "epson,epsonG364PDC0", },
    { .compatible = "epson,epsonG364PDCA", },
    { .compatible = "epson,epsonG365PDC1", },
    { .compatible = "epson,epsonG365PDF1", },
    { .compatible = "epson,epsonG366PDG0", },
    { .compatible = "epson,epsonG330PDG0", },
    { .compatible = "epson,epsonG370PDF1", },
    { .compatible = "epson,epsonG370PDS0", },
    { .compatible = "epson,epsonG370PDG0", },
    { .compatible = "epson,epsonG370PDT0", },
    {}
};
MODULE_DEVICE_TABLE(of, epson_imu_of_match);


static struct spi_driver epsonimu_driver = {

    .id_table = epsonimu_id,
    .probe = epsonimu_probe,
    .remove = epsonimu_remove,
    .driver = {
        .name = "epson_imus",
        .of_match_table = epson_imu_of_match,
    },
};
module_spi_driver(epsonimu_driver);



MODULE_AUTHOR("Dennis Henderson <sensingsystem_support@ea.epson.com>");
MODULE_DESCRIPTION("EPSON IMU SPI driver");
MODULE_VERSION(EPSON_DRV_VERSION);
MODULE_LICENSE("GPL v2");