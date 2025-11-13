//-----------------------------------------------------------------------------
// linux/drivers/iio/imu/epson/epson.c
//
// This is the source for the Epson IMU common libraries.
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

#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

// For Overlay include the following
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>

#include <linux/unaligned.h>

#include "epson_imus.h"



//----------------------------------------------------------------------
// epson_write_reg()
// Generic write function for Epson IMUs. All register writes are done
// as separate 8-bit writes.
//
// Parameters:
// - epson: pointer to the Epson device structure
// - reg: the address of the low register
// - val: the value to write to the device
// - size: the size of the write, in bytes
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
int epson_write_reg(struct epson* epson, unsigned int reg,
    unsigned int value, unsigned int size)
{
    unsigned int window = (reg >> 8);   // Window is encoded in the register information
    int ret, i;
    struct spi_message msg;
    struct spi_transfer xfers[] = {
        {
            .tx_buf = epson->tx,
            .bits_per_word = EPSON_BITS_PER_WORD,
            .len = 2,
            .cs_change = 1,
            .delay.value = epson->data->write_delay,
            .delay.unit = SPI_DELAY_UNIT_USECS,
        }, {
            .tx_buf = epson->tx + 2,
            .bits_per_word = EPSON_BITS_PER_WORD,
            .len = 2,
            .cs_change = 1,
            .delay.value = epson->data->write_delay,
            .delay.unit = SPI_DELAY_UNIT_USECS,
        }, {
            .tx_buf = epson->tx + 4,
            .bits_per_word = EPSON_BITS_PER_WORD,
            .len = 2,
            .cs_change = 1,
            .delay.value = epson->data->write_delay,
            .delay.unit = SPI_DELAY_UNIT_USECS,
        }, {
            .tx_buf = epson->tx + 6,
            .bits_per_word = EPSON_BITS_PER_WORD,
            .len = 2,
            .delay.value = epson->data->write_delay,
            .delay.unit = SPI_DELAY_UNIT_USECS,
        }, {
            .tx_buf = epson->tx + 8,
            .bits_per_word = EPSON_BITS_PER_WORD,
            .len = 2,
            .delay.value = epson->data->write_delay,
            .delay.unit = SPI_DELAY_UNIT_USECS,
        },
    };

    mutex_lock(&epson->state_lock);

    spi_message_init(&msg);
    if (epson->current_window != window) {
        epson->tx[0] = EPSON_WRITE_REG(WIN_CTRL);
        epson->tx[1] = window;
        spi_message_add_tail(&xfers[0], &msg);
    }

    switch (size) {
    case 4:
        epson->tx[8] = EPSON_WRITE_REG(reg + 3);
        epson->tx[9] = (value >> 24) & 0xff;
        epson->tx[6] = EPSON_WRITE_REG(reg + 2);
        epson->tx[7] = (value >> 16) & 0xff;
        fallthrough;
    case 2:
        epson->tx[4] = EPSON_WRITE_REG(reg + 1);
        epson->tx[5] = (value >> 8) & 0xff;
        fallthrough;
    case 1:
        epson->tx[2] = EPSON_WRITE_REG(reg);
        epson->tx[3] = value & 0xff;
        break;
    default:
        ret = -EINVAL;
        goto out_unlock;
    }

    xfers[size].cs_change = 0;

    for (i = 1; i <= size; i++)
        spi_message_add_tail(&xfers[i], &msg);

    ret = spi_sync(epson->spi, &msg);
    if (ret) {
        dev_err(&epson->spi->dev, "Failed to write register 0x%02X: %d\n", reg, ret);
    }
    else {
        epson->current_window = window;
    }

out_unlock:
    mutex_unlock(&epson->state_lock);

    return ret;
}
EXPORT_SYMBOL_GPL(epson_write_reg);



//----------------------------------------------------------------------
// epson_read_reg()
// Generic read function for Epson IMUs.
//
// Parameters:
// - epson: pointer to the Epson device structure
// - reg: the address of the low register
// - val: the value read back from the device
// - size: the size of the read, in bytes
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
int epson_read_reg(struct epson* epson, unsigned int reg,
    unsigned int* val, unsigned int size)
{
    unsigned int window = (reg >> 8);
    struct spi_message msg;
    int ret;
    struct spi_transfer xfers[] = {
        {
            .tx_buf = epson->tx,
            .bits_per_word = 8,
            .len = 2,
            .cs_change = 1,
            .delay.value = epson->data->write_delay,
            .delay.unit = SPI_DELAY_UNIT_USECS,
        }, {
            .tx_buf = epson->tx + 2,
            .bits_per_word = 8,
            .len = 2,
            .cs_change = 1,
            .delay.value = epson->data->write_delay,
            .delay.unit = SPI_DELAY_UNIT_USECS,
        }, {
            .tx_buf = epson->tx + 4,
            .rx_buf = epson->rx,
            .bits_per_word = 8,
            .len = 2,
            .cs_change = 1,
            .delay.value = epson->data->write_delay,
            .delay.unit = SPI_DELAY_UNIT_USECS,
        }, {
            .rx_buf = epson->rx + 2,
            .bits_per_word = 8,
            .len = 2,
            .cs_change = 1,
            .delay.value = epson->data->write_delay,
            .delay.unit = SPI_DELAY_UNIT_USECS,
        },
    };

    mutex_lock(&epson->state_lock);
    spi_message_init(&msg);
    if (epson->current_window != window) {
        epson->tx[0] = EPSON_WRITE_REG(WIN_CTRL);
        epson->tx[1] = window;
        spi_message_add_tail(&xfers[0], &msg);
    }

    switch (size) {
    case 4:
        epson->tx[2] = EPSON_READ_REG(reg + 2);
        epson->tx[3] = 0;
        spi_message_add_tail(&xfers[1], &msg);
        fallthrough;
    case 2:
        epson->tx[4] = EPSON_READ_REG(reg);
        epson->tx[5] = 0;
        spi_message_add_tail(&xfers[2], &msg);
        spi_message_add_tail(&xfers[3], &msg);
        break;
    default:
        ret = -EINVAL;
        dev_err(&epson->spi->dev, "%s bad size: %u\n", __FUNCTION__, size);
        goto out_unlock;
    }

    ret = spi_sync(epson->spi, &msg);
    if (ret) {
        dev_err(&epson->spi->dev, "Failed to read register 0x%02X: %d\n",
            reg, ret);
        goto out_unlock;
    }
    else {
        epson->current_window = window;
    }

    switch (size) {
    case 4:
        *val = get_unaligned_be32(epson->rx);
        break;
    case 2:
        *val = get_unaligned_be16(epson->rx + 2);
        break;
    }

out_unlock:
    mutex_unlock(&epson->state_lock);

    return ret;
}
EXPORT_SYMBOL_GPL(epson_read_reg);



#ifdef CONFIG_DEBUG_FS
//----------------------------------------------------------------------
// epson_debugfs_reg_access()
// DebugFS register IO for Epson IMUs
//
// Parameters:
// - indio_dev: the IIO device
// - reg: the address of the low register
// - writeval: the value to write to the device
// - readval: the value to read back from the device
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
int epson_debugfs_reg_access(struct iio_dev* indio_dev,
    unsigned int reg, unsigned int writeval, unsigned int* readval)
{
    struct epson* epson = iio_device_get_drvdata(indio_dev);

    if (readval) {
        uint16_t val16;
        int ret;

        ret = epson_read_reg_16(epson, reg, &val16);
        *readval = val16;

        return ret;
    }
    else {
        return epson_write_reg_16(epson, reg, writeval);
    }
}
EXPORT_SYMBOL(epson_debugfs_reg_access);
#endif



//----------------------------------------------------------------------
// epson_enable_irq()
// Enables/disables the IRQ for data ready (DRDY).
//
// Parameters:
// - epson: pointer to the Epson device structure
// - enable: boolean indicating whether to enable the IRQ
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
int epson_enable_irq(struct epson* epson, bool enable)
{
    int ret = 0;
    uint16_t msc;

    if (epson->data->enable_irq)
        return epson->data->enable_irq(epson, enable);

    ret = epson_read_reg_16(epson, epson->data->msc_ctrl_reg, &msc);
    if (ret)
        goto error_ret;

    //set DRDY polarity
    msc |= EPSON_MSC_CTRL_DRDY_POL;

    //enable DRDY if used
    if (enable)
        msc |= EPSON_MSC_CTRL_DRDY_ON;
    else
        msc &= ~EPSON_MSC_CTRL_DRDY_ON;

    ret = epson_write_reg_16(epson, epson->data->msc_ctrl_reg, msc);

error_ret:
    return ret;
}
EXPORT_SYMBOL(epson_enable_irq);



//----------------------------------------------------------------------
// epson_check_status()
// Checks the IMU for error conditions.
//
// Parameters:
// - epson: pointer to the Epson device structure
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
int epson_check_status(struct epson* epson)
{
    uint16_t status;
    int ret;
    int i;

    ret = epson_read_reg_16(epson, epson->data->diag_stat_reg, &status);
    if (ret < 0)
        return ret;

    status &= epson->data->status_error_mask;

    if (status == 0)
        return 0;

    //if errors, generate messages
    for (i = 0; i < 16; ++i) {
        if (status & BIT(i)) {
            dev_err(&epson->spi->dev, "%s.\n", epson->data->status_error_msgs[i]);
        }
    }

    return -EIO;
}
EXPORT_SYMBOL_GPL(epson_check_status);



//----------------------------------------------------------------------
// epson_check_status()
// Resets the IMU.
//
// Parameters:
// - epson: pointer to the Epson device structure
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
int epson_reset(struct epson* epson)
{
    int ret;
    int retries;
    uint16_t value;

#define EPSON_IMU_HW_RESET
#ifdef EPSON_IMU_HW_RESET
    // HW RESET using reset pin method
    gpio_set_value(epson->reset_pin, 0);
    msleep(DELAY_EPSON_RESET);
    gpio_set_value(epson->reset_pin, 1);
#else
    // SW RESET method
    // trigger a software reset
    // note when software reset is triggered, the device goes into Config Mode.
    ret = epson_write_reg_8(epson, REG_GLOB_CMD_LO, CMD_SOFTRESET);
#endif

    // After triggering a reset, wait for it to clear before leaving.
    // Then force the window control to be invalid so it will get written again
    msleep(EPSON_POWER_ON_DELAY);

    //check the NOT_READY bit (must go 0 before proceeding)
    ret = epson_read_reg_16(epson, REG_GLOB_CMD_LO, &value);
    retries = 0;
    while (value & 0x0400) {
        if (retries++ > 20) {
            return -1;
        }
        msleep(20);
        ret = epson_read_reg_16(epson, REG_GLOB_CMD_LO, &value);
    }

    // Force the current window to invalid state
    epson->current_window = -1;

    return ret;
}
EXPORT_SYMBOL_GPL(epson_reset);



//----------------------------------------------------------------------
// epson_self_test()
// Performs self test of the IMU
//
// Parameters:
// - epson: pointer to the Epson device structure
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epson_self_test(struct epson* epson)
{
    int ret = 0;
    uint16_t regval;

    ret = epson_write_reg_8(epson, REG_MSC_CTRL_HI, CMD_SELFTEST);
    if (ret) {
        dev_err(&epson->spi->dev, "Failed to initiate self test: %d\n", ret);
        return ret;
    }
    msleep(EPSON_SELF_TEST_DELAY);

    // NOTE: This can get stuck if the device is not responding correctly
    // in a test environment. Check all connections.
    do {
        ret = epson_read_reg_16(epson, REG_MSC_CTRL_LO, &regval);
    } while ((regval & (uint16_t)(CMD_SELFTEST << 8)) != 0);

    ret = epson_read_reg_16(epson, REG_DIAG_STAT, &regval);
    if (regval & EPSON_DIAG_STAT_SELF_TEST_ERR) {
        ret = epson_check_status(epson);
        dev_err(&epson->spi->dev, "Self test failed: %d\n", ret);
    }
    else {
        dev_info(&epson->spi->dev, "Self test passed\n");
    }

    return ret;
}



//----------------------------------------------------------------------
// epson_inital_startup()
// Performs IMU self-test and checks the HW_ERR bits
// NOTE: This function should be called early on in the device initialization
// sequence to ensure that the device is in a sane and known state and that
// it is usable.
//
// Parameters:
// - epson: pointer to the Epson device structure
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
int epson_initial_startup(struct epson* epson)
{
    int ret;

    //make sure self test happened correctly
    ret = epson_self_test(epson);
    if (ret) {
        dev_err(&epson->spi->dev, "Self-test failed, trying reset.\n");
        epson_reset(epson);
        msleep(epson->data->startup_delay);
        ret = epson_self_test(epson);
        if (ret) {
            dev_err(&epson->spi->dev, "Second self-test failed, giving up.\n");
            return ret;
        }
    }

    //---------------------------
    //set some default settings
    //---------------------------

    //output rate = 15.625sps
    ret |= epson_write_reg_8(epson, REG_SMPL_CTRL_HI, 0x07);

    //Filter = TAP128
    ret |= epson_write_reg_8(epson, REG_FILTER_CTRL_LO, 0x07);
    msleep(80);

    //TEST defaults
    //ND Flags: SIG_CTRL = Temp, Gyro, Accel
    ret |= epson_write_reg_8(epson, REG_SIG_CTRL_LO, 0xFC);
    ret |= epson_write_reg_8(epson, REG_SIG_CTRL_HI, 0xFE);

    //make sure DRDY set
    ret |= epson_write_reg_8(epson, REG_MSC_CTRL_LO, 0x06);

    //BURST_CTRL1: GPIO=off, Count=On, Chksm=On, Temp=On, Gyro=On, Accel=On
    ret |= epson_write_reg_8(epson, REG_BURST_CTRL1_LO, 0x03);
    ret |= epson_write_reg_8(epson, REG_BURST_CTRL1_HI, 0xF0);

#ifdef CONFIG_EPSONIMU_SAMPLE_32BIT
    //BURST_CTRL2: Temp=32bit, Gyro=32bit, Accel=32bit
    ret |= epson_write_reg_8(epson, REG_BURST_CTRL2_HI, 0x70);
#else
    //BURST_CTRL2: Temp=16bit, Gyro=16bit, Accel=16bit
    ret |= epson_write_reg_8(epson, REG_BURST_CTRL2_HI, 0x00);
#endif

    //check for a write error
    if (ret) {
        dev_err(&epson->spi->dev, "Default setup failed\n");
        return ret;
    }

    return 0;
}
EXPORT_SYMBOL_GPL(epson_initial_startup);



//----------------------------------------------------------------------
// epson_single_conversion()
// Performs a single sample conversion
// The function performs a single conversion on a given channel and post
// processes the value accordingly to the channel spec. If a error_mask is given
// the function will check if the mask is set in the returned raw value. If it
// is set the function will perform a self-check. If the device does not report
// a error bit in the channels raw value set error_mask to 0.
//
// Parameters:
// - indio_dev: the IIO device
// - chan: the IIO channel
// - error_mask: mask for the error bit
// - val: result of the conversion
//
// Returns IIO_VAL_INT on success, a negative error code
//
//----------------------------------------------------------------------
int epson_single_conversion(struct iio_dev* indio_dev,
    const struct iio_chan_spec* chan, unsigned int error_mask, int* val)
{
    struct epson* epson = iio_device_get_drvdata(indio_dev);
    unsigned int uval;
    int ret;

    ret = epson_read_reg(epson, chan->address, &uval, chan->scan_type.storagebits / 8);
    if (ret)
        goto err;

    if (uval & error_mask) {
        ret = epson_check_status(epson);
        if (ret)
            goto err;
    }

    if (chan->scan_type.sign == 's')
        *val = sign_extend32(uval, chan->scan_type.realbits - 1);
    else
        *val = uval & ((1 << chan->scan_type.realbits) - 1);

    ret = IIO_VAL_INT;

err:
    return ret;
}
EXPORT_SYMBOL_GPL(epson_single_conversion);



//----------------------------------------------------------------------
// epson_init()
// Initializes the epson IMU device structure
// This function must be called before any other function is called.
//
// Parameters:
// - indio_dev: the IIO device
// - spi: the SPI device
// - data: the IMU data
//
// Return: 0 on success, or a negative errno on failure
//
// NOTE: This function includes some platform specific code to support
//       Device Tree Overlays, setting up the GPIO pins used
//       for the DRDY and RESET lines. The implementation for other
//       platforms will vary.
//
//----------------------------------------------------------------------
int epson_init(struct epson* epson, struct iio_dev* indio_dev,
    struct spi_device* spi, const struct epson_data* data)
{
    int ret;
    int irq = 0, drdy;
    struct epsonimu_state* st;
    struct device_node* np = spi->dev.of_node;

    st = iio_priv(indio_dev);

    mutex_init(&epson->state_lock);
    epson->spi = spi;
    epson->data = data;
    iio_device_set_drvdata(indio_dev, epson);

    // Platform specific code to use info from the Device Tree Overlay
    //------------------------------------------------------------------
    // Code to map the GPIOs passed in the overlay

    // DRDY
    drdy = of_get_named_gpio(np, "drdy-gpios", 0);
    dev_info(&spi->dev, "DRDY is GPIO %d\n", drdy);
    if (drdy <= 0) {
        drdy = 24;
        dev_err(&spi->dev, "Forcing DRDY to GPIO %d\n", drdy);
    }
    if (!gpio_is_valid(drdy)) {
        dev_err(&spi->dev, "Failed to map DRDY to GPIO\n");
        return -EINVAL;
    }

    irq = gpio_to_irq(drdy);

    // setup the IIO driver allocated elements
    dev_info(&indio_dev->dev, "epson_init(), DRDY uses gpio %d, irq %d\n", drdy, irq);

    // RESET
    st->epson.reset_pin = of_get_named_gpio(np, "reset-gpios", 0);
    dev_info(&spi->dev, "RST is GPIO %d\n", st->epson.reset_pin);
    if (!gpio_is_valid(st->epson.reset_pin)) {
        dev_err(&spi->dev, "Failed to map RESET to GPIO\n");
        return -EINVAL;
    }

    ret = devm_gpio_request_one(&spi->dev, st->epson.reset_pin, GPIOF_OUT_INIT_HIGH, "reset-gpios");
    if (ret) {
        dev_err(&spi->dev, "Failed to request GPIO %d: %d\n", st->epson.reset_pin, ret);
        return -EINVAL;
    }

    gpio_direction_output(st->epson.reset_pin, 1);  // Force the GPIO line
    // used for reset control to high so #RESET is not active

    // if epson-proto exists
    st->epson.proto = false;
    if (of_find_property(np, "epson-proto", NULL)) {
        dev_info(&spi->dev, "epson-proto enabled\n");
        st->epson.proto = true;
    }

    // Get the maximum SPI speed from the Devicetree overlay
    ret = of_property_read_u32(np, "spi-max-frequency", &st->epson.spi_max_freq);
    if (ret || st->epson.spi_max_freq > SPI_MAX)
        st->epson.spi_max_freq = SPI_MAX;
    dev_info(&spi->dev, "Spi max freq is %d, ret: %i\n", st->epson.spi_max_freq, ret);

    spi->irq = irq;
    // End of Devicei Tree Mapping code
    //------------------------------------------------------------------

    //G320 IMU has register paging so set current window
    epson->current_window = -1;

    return epson_enable_irq(epson, false);
}
EXPORT_SYMBOL_GPL(epson_init);



MODULE_AUTHOR("Dennis Henderson <sensingsystem_support@ea.epson.com>");
MODULE_DESCRIPTION("Library for Epson IMUs");
MODULE_LICENSE("GPL v2");
