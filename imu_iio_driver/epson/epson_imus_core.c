//-----------------------------------------------------------------------------
// linux/drivers/iio/imu/epson/epson_imus_core.c
//
// This is the source for the Epson IMU core functions.
//
// Copyright(C) SEIKO EPSON CORPORATION 2017-2021. All rights reserved.
//
// This driver software is distributed as is, without any warranty of any kind,
// either express or implied as further specified in the GNU Public License. This
// software may be used and distributed according to the terms of the GNU Public
// License, version 2. See the file COPYING in the main directory of this archive
// for more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.
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
	EPSONG320,
	EPSONG325,
	EPSONG354,
	EPSONG365,
	EPSONG365PDC1,
	EPSONG364PDCA,
	EPSONG364PDC0,
};


// Samples Per Second register values for some IMU
// G320, G325, G354, G365
const sps_rate dout1[] = {
{2000000,	0x00}, // TAP>=0
{1000000,	0x01}, // TAP>=2
{500000,	0x02}, // TAP>=4
{250000,	0x03}, // TAP>=8
{125000,	0x04}, // TAP>=16
{62500,		0x05}, // TAP>=32
{31250,		0x06}, // TAP>=64
{15625,		0x07}, // TAP>=128
{400000,	0x08}, // TAP>=8
{200000,	0x09}, // TAP>=16
{100000,	0x0A}, // TAP>=32
{80000,		0x0B}, // TAP>=32
{50000,		0x0C}, // TAP>=64
{40000,		0x0D}, // TAP>=64
{25000,		0x0E}, // TAP>=128
{20000,		0x0F} }; // TAP>=128


//----------------------------------------------------------------------
// epsonimu_get_freq()
// Returns the sampling frequency of the Epson IMU
//
// Parameters:
// - st: the state of the Epson device
//
//----------------------------------------------------------------------
static int epsonimu_get_freq(struct epsonimu_state *st)
{
	int sps, ret;
	uint16_t t;
	int i;

	ret = epson_read_reg_16(&st->epson, REG_SMPL_CTRL_LO, &t);
	if (ret < 0)
		return ret;

	//mask DOUT_RATE
	t = (t & 0x0F00) >> 8;

	sps = 1000000;
	for (i=0; i<16; i++){
		if (st->variant->dout[i].value == t) {
			sps = st->variant->dout[i].sps;
			dev_info(&st->epson.spi->dev, "SPS is 0x%02x\n", sps);
			return sps;
		}
	}
	dev_err(&st->epson.spi->dev, "No freq match\n");
	return sps;
}


//----------------------------------------------------------------------
// epsonimu_set_freq()
// Sets the sampling frequency of the Epson IMU
// Note: Does not touch the Filter Sel
//
// Parameters:
// - st: the state of the Epson device
// - freq: the desired sampling frequency
//
//----------------------------------------------------------------------
static int epsonimu_set_freq(struct epsonimu_state *st, unsigned int freq)
{
	uint8_t val;
	int i;
	dev_info(&st->epson.spi->dev, "FREQ requested is %d\n", freq);

	for (i=0; i<16; i++){
		if (st->variant->dout[i].sps == freq) {
			val = st->variant->dout[i].value;
			dev_info(&st->epson.spi->dev, "DOUT is 0x%02x\n", val);
			return epson_write_reg_8(&st->epson, REG_SMPL_CTRL_HI, val);
		}
	}
	dev_err(&st->epson.spi->dev, "No freq match\n");
	return -EINVAL;
}


//----------------------------------------------------------------------
// epsonimu_set_filter()
// Sets the Epson IMU Filter
//
// Parameters:
// - indio_dev: the IIO device
// - val: the desired filter setting
//
//----------------------------------------------------------------------
static int epsonimu_set_filter(struct iio_dev *indio_dev, int val)
{
	struct epsonimu_state *st = iio_priv(indio_dev);
	uint16_t regval;
	int ret;

	//set new filter index
	ret = epson_write_reg_8(&st->epson, REG_FILTER_CTRL_LO, (uint8_t) val);

	//wait for filter setting complete
	do{
		msleep(10);
		ret = epson_read_reg_16(&st->epson, REG_FILTER_CTRL_LO, &regval);
		if (ret < 0)
			return ret;
	} while(regval & 0x0020);	//mask Filter_stat bit

	return ret;
}
static int epsonimu_get_chen(struct epsonimu_state *st, unsigned int ch, int * en)
{
	uint16_t val16;
	int ret;
	
	ret = epson_read_reg_16(&st->epson, REG_BURST_CTRL1_LO, &val16);
	if (ret < 0) {
		return ret;
	}
	switch (ch) {
		case IIO_ACCEL:
			*en = (val16 >> 12)&1;
			ret = IIO_VAL_INT;
		break;
		case IIO_ANGL_VEL:
			*en = (val16 >> 13)&1;
			ret = IIO_VAL_INT;
		break;
		case IIO_TEMP:
			*en = (val16 >> 14)&1;
			ret = IIO_VAL_INT;
		break;
		case IIO_TIMESTAMP:
			return 0;
		case IIO_COUNT:
			*en = (val16 >>  1)&1;
			ret = IIO_VAL_INT;
		break;
		default:
			ret = -EINVAL;
		break;
	}
	
	return ret;
}

static int epsonimu_set_chen(struct epsonimu_state *st, unsigned int ch, int en)
{
	uint16_t val16;
	uint8_t val8, bitn;
	int ret;

	switch (ch) {
		case IIO_ACCEL:
			bitn = 12;
		break;
		case IIO_ANGL_VEL:
			bitn = 13;
		break;
		case IIO_TEMP:
			bitn = 14;
		break;
		case IIO_TIMESTAMP:
			return 0;
		break;
		case IIO_COUNT:
			bitn =  1;
		break;
		default:
		return -EINVAL;
	}
	ret = epson_read_reg_16(&st->epson, REG_BURST_CTRL1_LO, &val16);
	if ( ret < 0 ) 
		return ret;
	if ( en ) val16 |=  (1<<bitn);
	else      val16 &= ~(1<<bitn);
	if ( bitn > 7 ) {
		val8 = (uint8_t)(val16>>8);
		ret =  epson_write_reg_8(&st->epson, REG_BURST_CTRL1_HI, val8);
	} else {
		val8 = (uint8_t)(val16>>0);
		ret =  epson_write_reg_8(&st->epson, REG_BURST_CTRL1_LO, val8);
	}
	
	return ret;
}


//----------------------------------------------------------------------
// epsonimu_write_raw()
// Performs raw writes to the Epson IMU
//
// Parameters:
// - indio_dev: the IIO device
// - chan: channel information
// - val: integer value
// - val2: decimal value
// - info:
//
//----------------------------------------------------------------------
static int epsonimu_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long info)
{
	struct epsonimu_state *st = iio_priv(indio_dev);
	int ret=0, sps;

	switch (info) {
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		//setting based on register bitfield index only
		mutex_lock(&indio_dev->mlock);
		if ((val >= 0) && (val <= 0x14))	//register index valid values
			ret = epsonimu_set_filter(indio_dev, val);
		mutex_unlock(&indio_dev->mlock);
		return ret;
	case IIO_CHAN_INFO_SAMP_FREQ:
		//val and val2 are used to convert a float frequency
		//to an int we can use to set the IMU registers
		sps = val * 1000 + val2 / 1000;

		if (sps <= 0)
			return -EINVAL;

		mutex_lock(&indio_dev->mlock);
		ret = st->variant->set_freq(st, sps);
		mutex_unlock(&indio_dev->mlock);
		return ret;
	case IIO_CHAN_INFO_ENABLE:	
		mutex_lock(&indio_dev->mlock);
		ret = epsonimu_set_chen(st, chan->type, val);
		mutex_unlock(&indio_dev->mlock);
		return ret;
	default:
		return -EINVAL;
	}
}


//----------------------------------------------------------------------
// epsonimu_read_raw()
// Performs raw reads from the Epson IMU
//
// Parameters:
// - indio_dev: the IIO device
// - chan: channel information
// - val: integer value
// - val2: decimal value
// - info:
//
//----------------------------------------------------------------------
static int epsonimu_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct epsonimu_state *st = iio_priv(indio_dev);
	int16_t val16;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = epson_single_conversion(indio_dev, chan, 0, val);
		return ret;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			*val = 0;
			*val2 = st->variant->gyro_scale_micro;
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_ACCEL:
			*val = 0;
			*val2 = st->variant->accel_scale_micro;
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_TEMP:	//for COUNT_CHAN scaled value is invalid
				//see Note about COUNT_CHAN using IIO_TEMP
			*val = st->variant->temp_scale_nano / 1000000;
			*val2 = (st->variant->temp_scale_nano % 1000000);
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		/* currently only temperature */
		*val = st->variant->temp_offset;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		mutex_lock(&indio_dev->mlock);
		// only reporting Filter Select index
		ret = epson_read_reg_16(&st->epson, REG_FILTER_CTRL_LO, &val16);
		if (ret < 0) {
			mutex_unlock(&indio_dev->mlock);
			return ret;
		}
		*val = val16 & 0x001F;	//keep only filter select bits
		//*val2 = 0;	//val2 not needed
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = st->variant->get_freq(st);
		if (ret < 0)
			return ret;
		//val and val2 are used to generate the float frequency
		//output to the subsystem
		*val = ret / 1000;
		*val2 = (ret % 1000) * 1000;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_ENABLE:
		mutex_lock(&indio_dev->mlock);
		ret = epsonimu_get_chen(st, chan->type, val);
		mutex_unlock(&indio_dev->mlock);
		return ret;  
	default:
		return -EINVAL;
	}
}


//----------------------------------------------------------------------
// Channel defines
//
//----------------------------------------------------------------------
#define GYRO_CHAN(mod, addr, bits) { \
	.type = IIO_ANGL_VEL, \
	.modified = 1, \
	.channel2 = IIO_MOD_ ## mod, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),  \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_ENABLE), \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
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
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_ENABLE), \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
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
		BIT(IIO_CHAN_INFO_SCALE) | \
		BIT(IIO_CHAN_INFO_ENABLE), \
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

//COUNT_CHAN currently uses the IIO_COUNT type to define a Count channel.
//This should be updated to use the IIO_TEMP type for linux kernels that
//do not support the additional IIO_COUNT type
//.realbits and .shift settings are for libiio workaround
#define COUNT_CHAN(addr, bits) { \
	.type = IIO_COUNT, \
	.indexed = 1, \
	.channel = 1, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
		BIT(IIO_CHAN_INFO_ENABLE), \
	.address = (addr), \
	.scan_index = SCAN_COUNT, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 16, \
		.storagebits = (bits), \
		.shift = (bits==32) ? 16 : 0, \
		.endianness = IIO_BE, \
	}, \
}


//----------------------------------------------------------------------
// IMU specific channel list
//  
//----------------------------------------------------------------------
#ifdef CONFIG_EPSONIMU_SAMPLE_32BIT	//32-bit samples
static const struct iio_chan_spec epsonGxxx_channels[] = {
	TEMP_CHAN(REG_TEMP_HIGH, 32),
	GYRO_CHAN(X, REG_XGYRO_HIGH, 32),
	GYRO_CHAN(Y, REG_YGYRO_HIGH, 32),
	GYRO_CHAN(Z, REG_ZGYRO_HIGH, 32),
	ACCEL_CHAN(X, REG_XACCL_HIGH, 32),
	ACCEL_CHAN(Y, REG_YACCL_HIGH, 32),
	ACCEL_CHAN(Z, REG_ZACCL_HIGH, 32),
	COUNT_CHAN(REG_COUNT, 32), //< 32 is for libiio workarround
	IIO_CHAN_SOFT_TIMESTAMP(SCAN_TIMESTAMP),
};
#else //16-bit samples
static const struct iio_chan_spec epsonGxxx_channels[] = {
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
#endif

//----------------------------------------------------------------------
// epsonimu_chips[]
// information about the Epson IMUs
//
//----------------------------------------------------------------------
struct epsonimu_chip_info epsonimu_chips[] = {
	[EPSONG320] = {
		.product_id = "epsonG320PDG0",
		.dout = dout1,
		.channels = epsonGxxx_channels,
		.num_channels = ARRAY_SIZE(epsonGxxx_channels),
		.flags = HAS_PROD_ID |
			HAS_SERIAL_NUMBER |
			BURST_DIAG_STAT |
			HAS_FW_VER,
		.gyro_scale_micro = IIO_DEGREE_TO_RAD(8000), /* 0.008 deg/s */
		.accel_scale_micro = IIO_G_TO_M_S_2(200), /* 1/5000 g */
		.temp_scale_nano = -3791800, /* -0.0037918 C */
		.temp_offset = -2634, /* 25 C = 0x00 */  //16-bit offset
		.set_freq = epsonimu_set_freq,
		.get_freq = epsonimu_get_freq,
	},
	[EPSONG325] = {
		.product_id = "epsonG325PDF0",
		.dout = dout1,
		.channels = epsonGxxx_channels,
		.num_channels = ARRAY_SIZE(epsonGxxx_channels),
		.flags = HAS_PROD_ID |
			HAS_SERIAL_NUMBER |
			BURST_DIAG_STAT |
			HAS_FW_VER,
		.gyro_scale_micro = IIO_DEGREE_TO_RAD(50000), /* 0.05 deg/s */
		.accel_scale_micro = IIO_G_TO_M_S_2(1000000/2500), /* 1/2500 g */
		.temp_scale_nano = 7812500, /* 0.0078125 C */
		.temp_offset = 3200, /* 25 C = 0x00 */  //16-bit offset
		.set_freq = epsonimu_set_freq,
		.get_freq = epsonimu_get_freq,
	},
	[EPSONG354] = {
		.product_id = "epsonG354PDH0",
		.dout = dout1,
		.channels = epsonGxxx_channels,
		.num_channels = ARRAY_SIZE(epsonGxxx_channels),
		.flags = HAS_PROD_ID |
			HAS_SERIAL_NUMBER |
			BURST_DIAG_STAT |
			HAS_FW_VER,
		.gyro_scale_micro = IIO_DEGREE_TO_RAD(16000), /* 0.016 deg/s */
		.accel_scale_micro = IIO_G_TO_M_S_2(200), /* 1/5000 g */
		.temp_scale_nano = -3791800, /* -0.0037918 C */
		.temp_offset = -2634, /* 25 C = 0x00 */  //16-bit offset
		.set_freq = epsonimu_set_freq,
		.get_freq = epsonimu_get_freq,
	},
	[EPSONG365] = {
		.product_id = "epsonG365PDF0",
		.dout = dout1,
		.channels = epsonGxxx_channels,
		.num_channels = ARRAY_SIZE(epsonGxxx_channels),
		.flags = HAS_PROD_ID |
			HAS_SERIAL_NUMBER |
			BURST_DIAG_STAT |
			HAS_FW_VER,
		.gyro_scale_micro = IIO_DEGREE_TO_RAD(1000000/66), /* 0.0151515 deg/s */
		.accel_scale_micro = IIO_G_TO_M_S_2(1000000/2500), /* 1/2500 g */
		.temp_scale_nano = -3791800, /* -0.0037918 C */
		.temp_offset = -2634, /* 25 C = 0x00 */  //16-bit offset
		.set_freq = epsonimu_set_freq,
		.get_freq = epsonimu_get_freq,
	},
	[EPSONG365PDC1] = {
		.product_id = "epsonG365PDC1",
		.dout = dout1,
		.channels = epsonGxxx_channels,
		.num_channels = ARRAY_SIZE(epsonGxxx_channels),
		.flags = HAS_PROD_ID |
			HAS_SERIAL_NUMBER |
			BURST_DIAG_STAT |
			HAS_FW_VER,
		.gyro_scale_micro = IIO_DEGREE_TO_RAD(1000000/66), /* 0.0151515 deg/s */
		.accel_scale_micro = IIO_G_TO_M_S_2(1000000/6250), /* 1/6250 g */
		.temp_scale_nano = -3791800, /* -0.0037918 C */
		.temp_offset = -2634, /* 25 C = 0x00 */  //16-bit offset
		.set_freq = epsonimu_set_freq,
		.get_freq = epsonimu_get_freq,
	},
	[EPSONG364PDCA] = {
		.product_id = "epsonG364PDCA",
		.dout = dout1,
		.channels = epsonGxxx_channels,
		.num_channels = ARRAY_SIZE(epsonGxxx_channels),
		.flags = HAS_PROD_ID |
			HAS_SERIAL_NUMBER |
			BURST_DIAG_STAT |
			HAS_FW_VER,
		.gyro_scale_micro = IIO_DEGREE_TO_RAD(3750), /* 0.00375 deg/s */
		.accel_scale_micro = IIO_G_TO_M_S_2(125), /* 0.125 mg */
		.temp_scale_nano = -3791800, /* -0.0037918 C */
		.temp_offset = -2634, /* 25 C = 0x00 */  //16-bit offset
		.set_freq = epsonimu_set_freq,
		.get_freq = epsonimu_get_freq,
	},
	[EPSONG364PDC0] = {
		.product_id = "epsonG364PDC0",
		.dout = dout1,
		.channels = epsonGxxx_channels,
		.num_channels = ARRAY_SIZE(epsonGxxx_channels),
		.flags = HAS_PROD_ID |
			HAS_SERIAL_NUMBER |
			BURST_DIAG_STAT |
			HAS_FW_VER,
		.gyro_scale_micro = IIO_DEGREE_TO_RAD(7500), /* 0.0075 deg/s */
		.accel_scale_micro = IIO_G_TO_M_S_2(125), /* 0.125 mg */
		.temp_scale_nano = -3791800, /* -0.0037918 C */
		.temp_offset = -2634, /* 25 C = 0x00 */  //16-bit offset
		.set_freq = epsonimu_set_freq,
		.get_freq = epsonimu_get_freq,
	},
};


static ssize_t product_id_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct epsonimu_state *st = iio_priv(indio_dev);
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
		    (product_id[i] & 0x00FF), (product_id[i] & 0xFF00) >>8 );

    len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

    return len;
}
static IIO_DEVICE_ATTR_RO(product_id, 0);

static ssize_t serial_number_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct epsonimu_state *st = iio_priv(indio_dev);
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
		    (serial_number[i] & 0x00FF), (serial_number[i] & 0xFF00) >>8 );

    len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

    return len;
}
static IIO_DEVICE_ATTR_RO(serial_number, 0);

static ssize_t firmware_version_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct epsonimu_state *st = iio_priv(indio_dev);
	uint16_t firmware_version;
    ssize_t ret, len = 0;

	//get firmware version
	ret = epson_read_reg_16(&st->epson, REG_VERSION, &firmware_version);
	if (ret < 0)
		return ret;

    len = scnprintf(buf, PAGE_SIZE, "%x\n", firmware_version );

	return len;
}
static IIO_DEVICE_ATTR_RO(firmware_version, 0);

static struct attribute *epsonimu_attrs[] = {
    &iio_dev_attr_product_id.dev_attr.attr,
    &iio_dev_attr_serial_number.dev_attr.attr,
    &iio_dev_attr_firmware_version.dev_attr.attr,
	NULL,
};

static const struct attribute_group epsonimu_attrs_group = {
	.attrs = epsonimu_attrs,
};

static const struct iio_info epsonimu_info = {
	.read_raw = &epsonimu_read_raw,
	.write_raw = &epsonimu_write_raw,
	.update_scan_mode = epsonimu_update_scan_mode,
	.attrs = &epsonimu_attrs_group,
	.debugfs_reg_access = epson_debugfs_reg_access,
};

static const struct spi_device_id epsonimu_id[] = {
	{"epsonG320PDG0", EPSONG320},
	{"epsonG325PDF0", EPSONG325},
	{"epsonG354PDH0", EPSONG354},
	{"epsonG365PDF0", EPSONG365},
	{"epsonG365PDF1", EPSONG365},
	{"epsonG370PDF0", EPSONG365},
	{"epsonG370PDF1", EPSONG365},
	{"epsonG365PDC1", EPSONG365PDC1},
	{"epsonG364PDCA", EPSONG364PDCA},
	{"epsonG364PDC0", EPSONG364PDC0},
	{}
};
// Precalculate the number of chip entries in the epsonimu_id table
#define NumIMUChipsSupported sizeof epsonimu_id / sizeof epsonimu_id[0]

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
//----------------------------------------------------------------------
static int epsonimu_setup_spi_and_get_device_id(struct iio_dev *indio_dev)
{
	struct epsonimu_state *st = iio_priv(indio_dev);
	int ret;
	uint16_t product_id[4];
	char prodbuf[16];
	int len;
	int i;

	// setup the SPI for default settings
	// from the Device Tree Overlay
	st->epson.spi->max_speed_hz = st->epson.spi_max_freq/2;
	dev_info(&indio_dev->dev, "SPI speed %d\n", st->epson.spi->max_speed_hz );

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
		(product_id[0] & 0x00FF), (product_id[0] & 0xFF00) >>8,
		(product_id[1] & 0x00FF), (product_id[1] & 0xFF00) >>8,
		(product_id[2] & 0x00FF), (product_id[2] & 0xFF00) >>8,
		(product_id[3] & 0x00FF), (product_id[3] & 0xFF00) >>8 );
	dev_info(&indio_dev->dev, "Product ID %s\n", prodbuf);
	if (len < 0) {
		return len;
	}

	for (i=0; i<NumIMUChipsSupported; i++){
		if (memcmp(epsonimu_id[i].name, prodbuf, 14) == 0) {
			dev_info(&indio_dev->dev, "Found a match epsonimu_chips[%i]\n", (int)epsonimu_id[i].driver_data);
			st->variant = &epsonimu_chips[(enum epsonimu_chip_variant)epsonimu_id[i].driver_data];
			if (memcmp(st->variant->product_id, prodbuf, 14) == 0) {
				dev_info(&indio_dev->dev, "Model Variant: %s\n", prodbuf);
			}else{
				dev_info(&indio_dev->dev, "Model Variant: Compatible with %s\n", st->variant->product_id);
			}
			break;
		}
	}

	if(i>=NumIMUChipsSupported){
		dev_info(&indio_dev->dev, "No match found\n");
		st->variant = &epsonimu_chips[0];
		return -EINVAL;
	}

	//do some initial startup
	ret = epson_initial_startup(&st->epson);
	if ( ! ret ) {
		//display the Product ID and IRQ used
		dev_info(&indio_dev->dev, "prod_id %s at CS%d (irq %d)\n",
		prodbuf, st->epson.spi->chip_select, st->epson.spi->irq);
	}

	return ret;
}

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

	.has_paging = true,
};

#ifdef CONFIG_DEBUG_FS
//----------------------------------------------------------------------
// epsonimu_show_serial_number()
// DebugFS routine to display serial number of Epson IMU
//
// Example Usage: cat /sys/kernel/debug/iio/iio:device0/serial_number
//
// Notes: The exact usage will depend on the kernel, iio device number, etc.
//        Only available when DEBUG_FS is enabled
//
//----------------------------------------------------------------------
static ssize_t epsonimu_show_serial_number(struct file *file,
	char __user *userbuf, size_t count, loff_t *ppos)
{
	struct epsonimu_state *st = file->private_data;
	uint16_t serial_number[4];
	char buf[16];
	int len;
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
		(serial_number[0] & 0x00FF), (serial_number[0] & 0xFF00) >>8,
		(serial_number[1] & 0x00FF), (serial_number[1] & 0xFF00) >>8,
		(serial_number[2] & 0x00FF), (serial_number[2] & 0xFF00) >>8,
		(serial_number[3] & 0x00FF), (serial_number[3] & 0xFF00) >>8 );

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
// Example Usage: cat /sys/kernel/debug/iio/iio:device0/product_id
//
// Notes: The exact usage will depend on the kernel, iio device number, etc.
//        Only available when DEBUG_FS is enabled
//
//----------------------------------------------------------------------
static ssize_t epsonimu_show_product_id(struct file *file,
	char __user *userbuf, size_t count, loff_t *ppos)
{
	struct epsonimu_state *st = file->private_data;
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
		(product_id[0] & 0x00FF), (product_id[0] & 0xFF00) >>8,
		(product_id[1] & 0x00FF), (product_id[1] & 0xFF00) >>8,
		(product_id[2] & 0x00FF), (product_id[2] & 0xFF00) >>8,
		(product_id[3] & 0x00FF), (product_id[3] & 0xFF00) >>8 );

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
// Example Usage: cat /sys/kernel/debug/iio/iio:device0/firmware_version
//
// Notes: The exact usage will depend on the kernel, iio device number, etc.
//        Only available when DEBUG_FS is enabled
//
//----------------------------------------------------------------------
static int epsonimu_show_firmware_version(void *arg, uint64_t *val)
{
	struct epsonimu_state *st = arg;
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
// Example Usage: cat /sys/kernel/debug/iio/iio:device0/diagnose
//
// Notes: The exact usage will depend on the kernel, iio device number, etc.
//        Only available when DEBUG_FS is enabled
//
//----------------------------------------------------------------------
static ssize_t epsonimu_show_diagnose(struct file *file,
	char __user *userbuf, size_t count, loff_t *ppos)
{
	struct epsonimu_state *st = file->private_data;

	uint16_t flag_reg;
	char buf[256];
	size_t len = 0;
	size_t lentot = 0;
	int ret;

	//check the Flags register
	ret = epson_read_reg_16(&st->epson, REG_FLAG, &flag_reg);
	if (ret < 0)
		return ret;

	//if Error All flag, RO, or RTD is set
	if (flag_reg & 0x0103) {
		if (flag_reg & 0x0001) { // if Error All flag in DIAG_STAT [0x04(W0)]
			uint16_t diag_stat_reg;

			//read the diagnostics register
			ret = epson_read_reg_16(&st->epson, REG_DIAG_STAT, &diag_stat_reg);
			if (ret < 0)
				return ret;

			len = snprintf(buf, sizeof(buf), "Diagnostics: Errors detected -- flags = %x\n", diag_stat_reg );
			lentot = lentot + len;

			//add error messages based on the diagnostics flags
			if (diag_stat_reg & EPSON_DIAG_STAT_XGYRO_FAIL) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tX-Gyro Self Test Error\n");
				lentot = lentot + len;
			}
			if (diag_stat_reg & EPSON_DIAG_STAT_YGYRO_FAIL) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tY-Gyro Self Test Error\n");
				lentot = lentot + len;
			}
			if (diag_stat_reg & EPSON_DIAG_STAT_ZGYRO_FAIL) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tZ-Gyro Self Test Error\n");
				lentot = lentot + len;
			}
			if (diag_stat_reg & EPSON_DIAG_STAT_ACCL_FAIL) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tAccelerometer Self Test Error\n");
				lentot = lentot + len;
			}
			if (diag_stat_reg & EPSON_DIAG_DLTA_OVF) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tDelta Angle Overflow\n");
				lentot = lentot + len;
			}
			if (diag_stat_reg & EPSON_DIAG_DLTV_OVF) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tDelta Velocity Overflow\n");
				lentot = lentot + len;
			}
			if (diag_stat_reg & EPSON_DIAG_HARD_ERR1) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tHardware Check Error\n");
				lentot = lentot + len;
			}
			else if (diag_stat_reg & EPSON_DIAG_HARD_ERR0) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tHardware Check Error\n");
				lentot = lentot + len;
			}
			if (diag_stat_reg & EPSON_DIAG_SPI_OVF) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tSPI Overflow\n");
				lentot = lentot + len;
			}
			if (diag_stat_reg & EPSON_DIAG_UART_OVF) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tUART Overflow\n");
				lentot = lentot + len;
			}
			if (diag_stat_reg & EPSON_DIAG_FLASH_ERR) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tFlash Test Error\n");
				lentot = lentot + len;
			}
			if (diag_stat_reg & EPSON_DIAG_STAT_SELF_TEST_ERR) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tSelf Test Error\n");
				lentot = lentot + len;
			}
			if (diag_stat_reg & EPSON_DIAG_FLASH_BU_ERR) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tFlash Backup Error\n");
				lentot = lentot + len;
			}
		}
		if (flag_reg & 0x0002) {
			uint16_t rt_diag_reg;

			//read the run time diag register
			ret = epson_read_reg_16(&st->epson, REG_RT_DIAG, &rt_diag_reg);
			if (ret < 0)
				return ret;

			len = snprintf(buf+lentot, sizeof(buf)-lentot, "Run Time Diag: Errors detected -- flags = %x\n", rt_diag_reg );
			lentot = lentot + len;

			if (rt_diag_reg & EPSON_RUN_TIME_DIAG_XGYRO) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tRun Time Diag X Gyro\n");
				lentot = lentot + len;
			}
			if (rt_diag_reg & EPSON_RUN_TIME_DIAG_YGYRO) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tRun Time Diag Y Gyro\n");
				lentot = lentot + len;
			}
			if (rt_diag_reg & EPSON_RUN_TIME_DIAG_ZGYRO) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tRun Time Diag Z Gyro\n");
				lentot = lentot + len;
			}
			if (rt_diag_reg & EPSON_RUN_TIME_DIAG_XACCL) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tRun Time Diag X Accl\n");
				lentot = lentot + len;
			}
			if (rt_diag_reg & EPSON_RUN_TIME_DIAG_YACCL) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tRun Time Diag Y Accl\n");
				lentot = lentot + len;
			}
			if (rt_diag_reg & EPSON_RUN_TIME_DIAG_ZACCL) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tRun Time Diag Z Accl\n");
				lentot = lentot + len;
			}
		}
		if (flag_reg & 0x0100) {
			uint16_t range_over_reg;

			//read the range over register
			ret = epson_read_reg_16(&st->epson, REG_RANGE_OVER, &range_over_reg);
			if (ret >= 0)
				return ret;

			len = snprintf(buf+lentot, sizeof(buf)-lentot, "RangeOver: Errors detected -- flags = %x\n", range_over_reg );
			lentot = lentot + len;

			if (range_over_reg & EPSON_RANGE_OVER_XGYRO) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tRangeOver X Gyro\n");
				lentot = lentot + len;
			}
			if (range_over_reg & EPSON_RANGE_OVER_YGYRO) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tRangeOver Y Gyro\n");
				lentot = lentot + len;
			}
			if (range_over_reg & EPSON_RANGE_OVER_ZGYRO) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tRangeOver Z Gyro\n");
				lentot = lentot + len;
			}
			if (range_over_reg & EPSON_RANGE_OVER_XACCL) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tRangeOver X Accl\n");
				lentot = lentot + len;
			}
			if (range_over_reg & EPSON_RANGE_OVER_YACCL) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tRangeOver Y Accl\n");
				lentot = lentot + len;
			}
			if (range_over_reg & EPSON_RANGE_OVER_ZACCL) {
				len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tRangeOver Z Accl\n");
				lentot = lentot + len;
			}
		}

	}
	else
		len = snprintf(buf, sizeof(buf), "Diagnostics: No errors detected.\n" );

	lentot = lentot+len;

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
// Example Usage: cat /sys/kernel/debug/iio/iio:device0/regdump
//
// Notes: The exact usage will depend on the kernel, iio device number, etc.
//        Only available when DEBUG_FS is enabled
//
//----------------------------------------------------------------------
static ssize_t epsonimu_show_regdump(struct file *file,
	char __user *userbuf, size_t count, loff_t *ppos)
{
	struct epsonimu_state *st = file->private_data;

	uint16_t regval;
	char buf[1000];
	size_t len = 0;
	size_t lentot = 0;
	int ret;

	//create a register dump buffer, not pretty but does the job
	//create heading for buffer
	len = snprintf(buf, sizeof(buf), "IMU Register Dump (REG[XX]=????)\n");
	lentot = lentot+len;
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "=================================\n\n");
	lentot = lentot+len;
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "Window 0\n--------\n");
	lentot = lentot+len;

	//dump register values
	ret = epson_read_reg_16(&st->epson, REG_MODE_CTRL_LO, &regval);
	if (ret < 0)
		return ret;

	//Window 0
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[02]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_DIAG_STAT, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[04]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_FLAG, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[06]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_GPIO, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[08]=%04x\n", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_COUNT, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[0A]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_TEMP_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[0E]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_TEMP_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[10]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_XGYRO_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[12]=%04x\n", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_XGYRO_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[14]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_YGYRO_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[16]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_YGYRO_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[18]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_ZGYRO_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[1A]=%04x\n", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_ZGYRO_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[1C]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_XACCL_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[1E]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_XACCL_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[20]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_YACCL_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[22]=%04x\n", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_YACCL_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[24]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_ZACCL_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[26]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_ZACCL_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[28]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_XDLTA_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[64]=%04x\n", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_XDLTA_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[66]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_YDLTA_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[68]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_YDLTA_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[6A]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_ZDLTA_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[6C]=%04x\n", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_ZDLTA_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[6E]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_XDLTV_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[70]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_XDLTV_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[72]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_YDLTV_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[74]=%04x\n", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_YDLTV_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[76]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_ZDLTV_HIGH, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[78]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_ZDLTV_LOW, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[7A]=%04x\n\n", regval);
	lentot = lentot+len;

	//Window 1
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "Window 1\n--------\n");
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_SIG_CTRL_LO, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[00]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_MSC_CTRL_LO, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[02]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_SMPL_CTRL_LO, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[04]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_FILTER_CTRL_LO, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[06]=%04x\n", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_UART_CTRL_LO, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[08]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_GLOB_CMD_LO, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[0A]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_BURST_CTRL1_LO, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[0C]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_BURST_CTRL2_LO, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[0E]=%04x\n", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_POL_CTRL_LO, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[10]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_DLT_CTRL_LO, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[12]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_PROD_ID1, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[6A]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_PROD_ID2, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[6C]=%04x\n", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_PROD_ID3, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[6E]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_PROD_ID4, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[70]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_VERSION, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[72]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_SERIAL_NUM1, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[74]=%04x\n", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_SERIAL_NUM2, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "REG[76]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_SERIAL_NUM3, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[78]=%04x", regval);
	lentot = lentot+len;
	ret |= epson_read_reg_16(&st->epson, REG_SERIAL_NUM4, &regval);
	len = snprintf(buf+lentot, sizeof(buf)-lentot, "\tREG[7A]=%04x\n\n", regval);
	lentot = lentot+len;

	//end the register dump
	if ( ret ) {
		len = snprintf(buf+lentot, sizeof(buf)-lentot, "Register Dump Complete with ERROR in reading...\n");
	}else{
		len = snprintf(buf+lentot, sizeof(buf)-lentot, "Register Dump Complete...\n");
	}
	lentot = lentot+len;

	return simple_read_from_buffer(userbuf, count, ppos, buf, lentot);
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
//----------------------------------------------------------------------
static int epsonimu_debugfs_init(struct iio_dev *indio_dev)
{
	struct epsonimu_state *st = iio_priv(indio_dev);
	struct dentry *debugfs_dentry = iio_get_debugfs_dentry(indio_dev);

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

	//create diagnose always
	debugfs_create_file("diagnose", 0400,
		debugfs_dentry, st,
		&epsonimu_diagnose_fops);

	//create regdump always
	debugfs_create_file("regdump", 0400,
		debugfs_dentry, st,
		&epsonimu_regdump_fops);

	return 0;
}

#else

static int epsonimu_debugfs_init(struct iio_dev *indio_dev)
{
	return 0;
}

#endif


//----------------------------------------------------------------------
// epsonimu_probe()
// Probes the Epson IMU spi device
//
// Parameters:
// - spi: the SPI device
//
//----------------------------------------------------------------------
static int epsonimu_probe(struct spi_device *spi)
{
	struct epsonimu_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	printk(KERN_INFO "%s: IMU driver probe\n", __FUNCTION__);
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	//used for removal purposes
	spi_set_drvdata(spi, indio_dev);

	//setup the IIO
	// put some defaults in place that get adjusted after the
	// IMU chip has been ID'd
	st->variant = &epsonimu_chips[spi_get_device_id(spi)->driver_data];
	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &epsonimu_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
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

	ret = epson_setup_buffer_and_trigger(&st->epson, indio_dev, NULL);
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
// - spi: the SPI device
//
//----------------------------------------------------------------------
static int epsonimu_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct epsonimu_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	epson_cleanup_buffer_and_trigger(&st->epson, indio_dev);
	// TODO: ensure the device is placed in a power down or sleep state

	return 0;
}

static struct spi_driver epsonimu_driver = {
	.driver = {
		.name = "epson_imus",
	},
	.id_table = epsonimu_id,
	.probe = epsonimu_probe,
	.remove = epsonimu_remove,
};
module_spi_driver(epsonimu_driver);

MODULE_DEVICE_TABLE(spi, epsonimu_id);
MODULE_AUTHOR("Dennis Henderson <eeavdctech_info@eea.epson.com>");
MODULE_DESCRIPTION("EPSON IMU SPI driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.20");

