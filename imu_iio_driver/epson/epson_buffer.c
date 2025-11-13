//-----------------------------------------------------------------------------
// linux/drivers/iio/imu/epson/epson_buffer.c
//
// Source code for the Epson IMU buffer implementation functions.
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
#include <linux/export.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "epson_imus.h"



//----------------------------------------------------------------------
// epsonimu_update_scan_mode_no_burst()
// Default function to update the scan channel information
//
// Parameters:
// - indio_dev: the IIO device
// - scan_mask: the channel scan mask
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epsonimu_update_scan_mode_no_burst(struct iio_dev* indio_dev,
    const unsigned long* scan_mask)
{
    struct epson* epson = iio_device_get_drvdata(indio_dev);
    const struct iio_chan_spec* chan;
    unsigned int scan_count;
    unsigned int i, j;
    __be16* tx, * rx;

    kfree(epson->xfer);
    kfree(epson->buffer);

    scan_count = indio_dev->scan_bytes / 2;

    epson->xfer = kcalloc(scan_count + 1, sizeof(*epson->xfer), GFP_KERNEL);
    if (!epson->xfer)
        return -ENOMEM;

    epson->buffer = kzalloc(indio_dev->scan_bytes * 2, GFP_KERNEL);
    if (!epson->buffer) {
        kfree(epson->xfer);
        epson->xfer = NULL;
        return -ENOMEM;
    }

    rx = epson->buffer;
    tx = rx + scan_count;

    spi_message_init(&epson->msg);

    for (j = 0; j <= scan_count; j++) {
        epson->xfer[j].bits_per_word = EPSON_BITS_PER_WORD;
        if (j != scan_count)
            epson->xfer[j].cs_change = 1;
        epson->xfer[j].len = 2;
        epson->xfer[j].delay.value = epson->data->read_delay;
        epson->xfer[j].delay.unit = SPI_DELAY_UNIT_USECS;
        if (j < scan_count)
            epson->xfer[j].tx_buf = &tx[j];
        if (j >= 1)
            epson->xfer[j].rx_buf = &rx[j - 1];
        spi_message_add_tail(&epson->xfer[j], &epson->msg);
    }

    chan = indio_dev->channels;
    for (i = 0; i < indio_dev->num_channels; i++, chan++) {
        if (!test_bit(chan->scan_index, scan_mask))
            continue;
        if (chan->scan_type.storagebits == 32)
            *tx++ = cpu_to_be16((chan->address + 2) << 8);
        *tx++ = cpu_to_be16(chan->address << 8);
    }

    return 0;
}



//----------------------------------------------------------------------
// epsonimu_update_scan_mode()
// Function to update scan channel information, used for IMU with burst mode
//
// Parameters:
// - indio_dev: the IIO device
// - scan_mask: the channel scan mask
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
int epsonimu_update_scan_mode(struct iio_dev *indio_dev,
                              const unsigned long *scan_mask)
{
    struct epsonimu_state *st = iio_priv(indio_dev);
    struct epson *epson = &st->epson;
    unsigned int burst_len = 0;

    if (st->variant->flags & NO_BURST)
        return epsonimu_update_scan_mode_no_burst(indio_dev, scan_mask);
        
    kfree(epson->xfer);
    kfree(epson->buffer);

    // Sum storage bytes for each enabled channel (excluding timestamp here)
    for (int i = 0; i < indio_dev->num_channels; i++) {
        const struct iio_chan_spec *ch = &indio_dev->channels[i];
        
        if (!test_bit(i, scan_mask))
            continue;

        if (ch->type == IIO_TIMESTAMP)
            continue;

        burst_len += ch->scan_type.storagebits / 8;
    }

    // If device adds a diagnostic/status word in the burst when enabled
    if (st->variant->flags & BURST_DIAG_STAT) {
        burst_len += sizeof(u16);
    }

    // include CRC
    const unsigned int crc = sizeof(uint16_t);
    const unsigned int spi_len = burst_len + crc;

    epson->xfer = kcalloc(2, sizeof(*epson->xfer), GFP_KERNEL);
    if (!epson->xfer)
        return -ENOMEM;

    epson->buffer = kzalloc(spi_len, GFP_KERNEL);
    if (!epson->buffer) {
        kfree(epson->xfer);
        return -ENOMEM;
    }

    dev_info(&indio_dev->dev, "spi_len=%u burst_len=%u (mask=%*pb)\n",
             spi_len, burst_len, indio_dev->masklength, scan_mask);

    epson->xfer[0].bits_per_word = EPSON_BITS_PER_WORD;
    epson->xfer[0].len = spi_len;
    epson->xfer[0].delay.value = BURST_STALL2;
    epson->xfer[0].delay.unit = SPI_DELAY_UNIT_USECS;
    epson->xfer[0].rx_buf = epson->buffer;

    spi_message_init(&epson->msg);
    spi_message_add_tail(&epson->xfer[0], &epson->msg);

    return 0;
}
EXPORT_SYMBOL_GPL(epsonimu_update_scan_mode);



//----------------------------------------------------------------------
// epson_trigger_handler()
// IRQ handler for IIO data-ready triggers
//
// Parameters:
// - irq: interrupt line number that fired
// - p:   pointer to struct iio_poll_func
//
// Return: IRQ_HANDLED if recognized and serviced the interrupt,
//           or IRQ_NONE otherwise.
//----------------------------------------------------------------------
#if IS_ENABLED(CONFIG_IIO_BUFFER)
struct __packed imu_frame {
#ifdef CONFIG_EPSONIMU_SAMPLE_32BIT
    uint32_t ch[7]; //< 32 bit
#else
    uint16_t ch[7]; //< 16 bit
#endif
    uint16_t count;
    uint16_t checksum;
};

#define WORD_CNT (sizeof(struct imu_frame) / 2)

static irqreturn_t epson_trigger_handler(int irq, void* p)
{
    struct iio_poll_func* pf = p;
    struct iio_dev* indio_dev = pf->indio_dev;
    struct epson* epson = iio_device_get_drvdata(indio_dev);
    int ret;
    uint16_t checksum_tot = 0;
    uint16_t* data;
    uint16_t status = 0;

    if (!epson->buffer) {
        dev_err(&epson->spi->dev, "No epson->buffer");
        return IRQ_NONE;
    }

    // start the burst
    mutex_lock(&epson->state_lock);
    if (epson->data->has_paging) {
        if (epson->current_window != 0) {
            epson->tx[0] = EPSON_WRITE_REG(WIN_CTRL);
            epson->tx[1] = 0;
            spi_write(epson->spi, epson->tx, 2);
        }
    }
    epson->tx[0] = CMD_BURST;
    epson->tx[1] = 0x00;
    spi_write(epson->spi, epson->tx, 2);

    data = epson->xfer[0].rx_buf;
    data[0] = 0; // clear the flags word for testing.. Just to ensure it is getting set.

    ret = spi_sync(epson->spi, &epson->msg);
    if (ret) {
        dev_err(&epson->spi->dev, "Failed to read data: %d", ret);
        goto irq_done;
    }

    if (epson->data->has_paging) {
        epson->current_window = 0;
    }
    mutex_unlock(&epson->state_lock);

    data = epson->buffer;
    struct imu_frame* frame = (struct imu_frame*)&data[1];

    // validate checksum, need to byteswap data values
    // swap high and low bytes of each sample, then mask to 16-bit checksum
    for (int i = 0; i < WORD_CNT; i++) {
        checksum_tot += (((data[i] >> 8) | (data[i] << 8)) & 0xFFFF);
    }
    // if calculated checksum total != IMU checksum value, report failure
    if (checksum_tot != (((data[WORD_CNT] >> 8) | (data[WORD_CNT] << 8)) & 0xFFFF)) {
        dev_err(&epson->spi->dev, "Checksum failure");
        goto irq_done;
    }

    // Check the Flag for the EA bit.. report the error
    if ((data[0] >> 8) & 0x0001) {
        epson_read_reg_16(epson, REG_DIAG_STAT, &status);
        dev_err(&epson->spi->dev, "Error Flag detected, DIAG_STAT is %02x\r\n", status);
        goto irq_done;
    }

    // skip past the flag data when passing up to IIO
    iio_push_to_buffers_with_timestamp(indio_dev, frame, pf->timestamp);

irq_done:
    iio_trigger_notify_done(indio_dev->trig);

    return IRQ_HANDLED;
}
#else
#define epson_trigger_handler   NULL
#endif



//----------------------------------------------------------------------
// epson_cleanup_buffer_and_trigger()
// Frees buffer and trigger resources
// Frees resources allocated by epson_setup_buffer_and_trigger()
//
// Parameters:
// - epson: pointer to the Epson device structure
// - indio_dev: the IIO device
//
//----------------------------------------------------------------------
void epson_cleanup_buffer_and_trigger(struct epson* epson, struct iio_dev* indio_dev)
{
    if (epson->spi->irq)
        epson_remove_trigger(epson);
    kfree(epson->buffer);
    kfree(epson->xfer);
    iio_triggered_buffer_cleanup(indio_dev);
}
EXPORT_SYMBOL_GPL(epson_cleanup_buffer_and_trigger);



//----------------------------------------------------------------------
// epson_setup_buffer_and_trigger()
// Configure a triggered IIO buffer for Epson IMU
//
// Parameters:
// - epson: pointer to the Epson device structure
// - indio_dev:       the IIO device
// - ops:             callbacks for allocating, freeing and validating the IIO buffer
// - trigger_handler: IRQ handler invoked when the device’s data-ready line fires
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
int epson_setup_buffer_and_trigger(struct epson *epson, struct iio_dev *indio_dev,
    const struct iio_buffer_setup_ops *ops,
    irqreturn_t (*trigger_handler)(int, void *))
{
    int ret;

    if (!trigger_handler) {
        trigger_handler = epson_trigger_handler;
    }

    ret = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
        trigger_handler, NULL);
    if (ret)
        return ret;

    if (epson->spi->irq) {
        ret = epson_probe_trigger(epson, indio_dev);
        if (ret)
            goto error_buffer_cleanup;
    }
    else {
        ret = -1;
        goto error_buffer_cleanup;
    }

    return 0;

error_buffer_cleanup:
    iio_triggered_buffer_cleanup(indio_dev);
    return ret;
}
EXPORT_SYMBOL_GPL(epson_setup_buffer_and_trigger);



//----------------------------------------------------------------------
// epson_data_rdy_trigger_set_state()
// Sets the state of the IIO trigger
//
// Parameters:
// - trig: the IIO trigger
// - state: boolean defines whether trigger to be enabled (1) or disabled (0)
//
// Return: 0 on success, or a negative errno on failure
//
//----------------------------------------------------------------------
static int epson_data_rdy_trigger_set_state(struct iio_trigger* trig, bool state)
{
    struct epson* epson = iio_trigger_get_drvdata(trig);
    int ret = epson_enable_irq(epson, state);

    if (ret)
        goto error_ret;

    if (state) {
        ret = epson_write_reg_8(epson, epson->data->mode_ctrl_reg, CMD_BEGIN_SAMPLING);
    }
    else {
        ret = epson_write_reg_8(epson, epson->data->mode_ctrl_reg, CMD_END_SAMPLING);
    }

    if (ret)
        dev_err(&epson->spi->dev, "Error writing mode_ctrl_reg\n");

error_ret:
    return ret;
}

static const struct iio_trigger_ops epson_trigger_ops = {
    .set_trigger_state = &epson_data_rdy_trigger_set_state,
};



//----------------------------------------------------------------------
// epson_probe_trigger()
// Sets up trigger for Epson IMU
//
// Parameters:
// - epson: pointer to the Epson device structure
// - indio_dev: the IIO device
//
// Return: 0 on success, or a negative errno on failure
//
// Note: epson_remove_trigger() should be used to free the trigger.
//
//----------------------------------------------------------------------
int epson_probe_trigger(struct epson* epson, struct iio_dev* indio_dev)
{
    int ret;

    epson->trig = iio_trigger_alloc(indio_dev->dev.parent, "%s-dev%d", indio_dev->name,
        iio_device_id(indio_dev));
    if (epson->trig == NULL)
        return -ENOMEM;

    ret = request_irq(epson->spi->irq,
        &iio_trigger_generic_data_rdy_poll,
        IRQF_TRIGGER_RISING,
        indio_dev->name,
        epson->trig);
    if (ret)
        goto error_free_trig;

    epson->trig->dev.parent = &epson->spi->dev;
    epson->trig->ops = &epson_trigger_ops;
    iio_trigger_set_drvdata(epson->trig, epson);
    ret = iio_trigger_register(epson->trig);

    indio_dev->trig = iio_trigger_get(epson->trig);
    if (ret)
        goto error_free_irq;

    return 0;

error_free_irq: //failed so free everthing up
    dev_err(&indio_dev->dev, "Failed epson probe irq\n");
    free_irq(epson->spi->irq, epson->trig);
error_free_trig:
    iio_trigger_free(epson->trig);
    return ret;
}
EXPORT_SYMBOL_GPL(epson_probe_trigger);



//----------------------------------------------------------------------
// epson_remove_trigger()
// Remove trigger for Epson IMU
// Removes the trigger previously registered using epson_probe_trigger().
//
// Parameters:
// - epson: pointer to the Epson device structure
//
//----------------------------------------------------------------------
void epson_remove_trigger(struct epson* epson)
{
    iio_trigger_unregister(epson->trig);
    free_irq(epson->spi->irq, epson->trig);
    iio_trigger_free(epson->trig);
}
EXPORT_SYMBOL_GPL(epson_remove_trigger);
