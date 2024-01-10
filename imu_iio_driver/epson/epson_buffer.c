//-----------------------------------------------------------------------------
// linux/drivers/iio/imu/epson/epson_buffer.c
//
// Source code for the Epson IMU buffer implementation functions.
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
// epson_update_scan_mode()
// Default function to update the scan channel information
//
// Parameters: 
// indio_dev: the IIO device
// scan_mask: the channel scan mask
//
//----------------------------------------------------------------------
int epson_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *scan_mask)
{
	struct epson *epson = iio_device_get_drvdata(indio_dev);
	const struct iio_chan_spec *chan;
	unsigned int scan_count;
	unsigned int i, j;
	__be16 *tx, *rx;

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
		epson->xfer[j].delay_usecs = epson->data->read_delay;
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
EXPORT_SYMBOL_GPL(epson_update_scan_mode);

//----------------------------------------------------------------------
// epsonimu_num_channels()
// Function to update burst length information
//
// Parameters: 
// indio_dev: the IIO device
//
//----------------------------------------------------------------------
int epsonimu_num_channels(struct iio_dev *indio_dev)
{
	uint16_t val16, byte_len = 0;
	struct epsonimu_state *st = iio_priv(indio_dev);
	struct epson *epson = &st->epson;
	int ret;
	
	ret = epson_read_reg_16(&st->epson, REG_BURST_CTRL1_LO, &val16);
	if (ret < 0) {
		return ret;
	}
#ifdef CONFIG_EPSONIMU_SAMPLE_32BIT	
	/* max: 2+4+12+12+2+2+2 = 36 */
	if (val16 & EPSON_BRST_CTRL1_FLAG_OUT)  byte_len += 2;
	if (val16 & EPSON_BRST_CTRL1_TEMP_OUT)  byte_len += 4;
	if (val16 & EPSON_BRST_CTRL1_GYRO_OUT)  byte_len += 12;
	if (val16 & EPSON_BRST_CTRL1_ACCL_OUT)  byte_len += 12;
	if (val16 & EPSON_BRST_CTRL1_COUNT_OUT) byte_len += 2;
	if (val16 & EPSON_BRST_CTRL1_GPIO_OUT)  byte_len += 2;
	if (val16 & EPSON_BRST_CTRL1_CHKSM_OUT) byte_len += 2;
#else
	/* max: 2+2+6+6+2+2+2 = 16 */
	if (val16 & EPSON_BRST_CTRL1_FLAG_OUT)  byte_len += 2;
	if (val16 & EPSON_BRST_CTRL1_TEMP_OUT)  byte_len += 2;
	if (val16 & EPSON_BRST_CTRL1_GYRO_OUT)  byte_len += 6;
	if (val16 & EPSON_BRST_CTRL1_ACCL_OUT)  byte_len += 6;
	if (val16 & EPSON_BRST_CTRL1_COUNT_OUT) byte_len += 2;
	if (val16 & EPSON_BRST_CTRL1_GPIO_OUT)  byte_len += 2;
	if (val16 & EPSON_BRST_CTRL1_CHKSM_OUT) byte_len += 2;
#endif
	epson->burst_length = byte_len;
	dev_info(&st->epson.spi->dev, "Burst count %u [R:%02x]\n", epson->burst_length/2, val16);

	return 0;
}
EXPORT_SYMBOL_GPL(epsonimu_num_channels);

//----------------------------------------------------------------------
// epsonimu_update_scan_mode()
// Function to update scan channel information, used for IMU with burst mode
//
// Parameters: 
// indio_dev: the IIO device
// scan_mask: the channel scan mask
//
//----------------------------------------------------------------------
int epsonimu_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *scan_mask)
{
	struct epsonimu_state *st = iio_priv(indio_dev);
	struct epson *epson = &st->epson;
	const struct iio_chan_spec *chan;
	unsigned int i;
	uint16_t burst_cfg = EPSON_BRST_CTRL1_FLAG_OUT | EPSON_BRST_CTRL1_CHKSM_OUT;
	
	if (st->variant->flags & NO_BURST) {
		dev_info(&st->epson.spi->dev, "NO_BURST\n");
		return epson_update_scan_mode(indio_dev, scan_mask);
	}
	//calculate burst_length
	dev_info(&st->epson.spi->dev, "scan_mask: %lxh\n", *scan_mask);
	chan = indio_dev->channels;
	for (i = 0; i < indio_dev->num_channels; i++, chan++) {
		if (!test_bit(chan->scan_index, scan_mask))
			continue;

		switch (chan->scan_index) {
			case SCAN_TEMP: 
				burst_cfg |= EPSON_BRST_CTRL1_TEMP_OUT; 
				break;
			case SCAN_GYRO_X: 
			case SCAN_GYRO_Y: 
			case SCAN_GYRO_Z: 
				burst_cfg |= EPSON_BRST_CTRL1_GYRO_OUT;
				break;
			case SCAN_ACC_X:
			case SCAN_ACC_Y:
			case SCAN_ACC_Z:
				burst_cfg |= EPSON_BRST_CTRL1_ACCL_OUT; 
				break;
			case SCAN_COUNT: 
				burst_cfg |= EPSON_BRST_CTRL1_COUNT_OUT; 
				break;
			default:
				dev_err(&st->epson.spi->dev, "unknown scan_index: %u\n", chan->scan_index );
				break;
		}
	}	

	if ( epson_write_reg_16(&st->epson, REG_BURST_CTRL1_LO, burst_cfg) )
		return -ENOMEM;			
	if ( epsonimu_num_channels(indio_dev) )
		return -ENOMEM;	

	kfree(epson->xfer);
	kfree(epson->buffer);

	epson->xfer = kcalloc(2, sizeof(*epson->xfer), GFP_KERNEL);
	if (!epson->xfer)
		return -ENOMEM;

	dev_info(&st->epson.spi->dev, "scan_bytes: %u\n", indio_dev->scan_bytes);
	epson->buffer = kzalloc(epson->burst_length + sizeof(int64_t), GFP_KERNEL);

	//if buffer allocation fails, free up xfer and return error
	if (!epson->buffer){
		kfree(epson->xfer);
		return -ENOMEM;
	}

	epson->xfer[0].bits_per_word = EPSON_BITS_PER_WORD;
	epson->xfer[0].len = epson->burst_length;
	epson->xfer[0].delay_usecs = BURST_STALL2;
	epson->xfer[0].rx_buf = epson->buffer;

	spi_message_init(&epson->msg);
	spi_message_add_tail(&epson->xfer[0], &epson->msg);
	return 0;
}
EXPORT_SYMBOL_GPL(epsonimu_update_scan_mode);

//----------------------------------------------------------------------
// epson_trigger_handler()
// Trigger handler for the IMU
//
//----------------------------------------------------------------------
static irqreturn_t epson_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct epson *epson = iio_device_get_drvdata(indio_dev);
	int ret;
	uint16_t checksum_tot = 0;
	int i;
	uint16_t *data;
	uint16_t status = 0;
	unsigned int reads_cnt = epson->burst_length/2 - 1 /*check sum*/;

	if (!epson->buffer)
		return IRQ_NONE;
	
	//start the burst
	mutex_lock(&epson->txrx_lock);
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
	
	data = epson->xfer[0].rx_buf;  ///use data pointer for a few lines of code. 
	data[0] = 0; // clear the flags word for testing.. Just to ensure it is getting set.
	
	ret = spi_sync(epson->spi, &epson->msg);
	if (ret)
		dev_err(&epson->spi->dev, "Failed to read data: %d", ret);

	if (epson->data->has_paging) {
		epson->current_window = 0;
	}
	mutex_unlock(&epson->txrx_lock);

	//validate checksum, need to byteswap data values 
	//swap high and low byte of each sample, then mask to 16-bit checksum 
	for (i=0; i<reads_cnt; i++) {
		checksum_tot += (((data[i] >>8) | (data[i] <<8)) & 0xFFFF);
	}
	//if calculated checksum total != IMU checksum value, report failure 
	if (checksum_tot != (((data[reads_cnt] >>8) | (data[reads_cnt] <<8)) & 0xFFFF)){
		dev_err(&epson->spi->dev, "Checksum failure");
	}

	// Check the Flag for the EA bit.. report the error 
	// bytes swaped in data[0]
	if ((data[0] >>8) & 0x0001){
		epson_read_reg_16(epson, REG_DIAG_STAT, &status);
		dev_err(&epson->spi->dev, "Error Flag detected, DIAG_STAT is 0x%04x\r\n", status);
	}
	// Check the Flag for the RTD bit.. report the error 
	if ((data[0] >>8) & 0x0002){
		epson_read_reg_16(epson, REG_RT_DIAG, &status);
		dev_err(&epson->spi->dev, "Error Flag detected, RT_DIAG is 0x%04x\r\n", status);
	}
        // Check the Flag for the RO bit.. report the error 
	if ((data[0] >>0) & 0x0001){
		epson_read_reg_16(epson, REG_RANGE_OVER, &status);
		dev_err(&epson->spi->dev, "Error Flag detected, RANGE_OVER is 0x%04x\r\n", status);
	}

	// skip past the flag data when passing up to IIO
	iio_push_to_buffers_with_timestamp(indio_dev, (void *)&data[1], pf->timestamp);

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}


//----------------------------------------------------------------------
// epson_setup_buffer_and_trigger()
// This function sets up the buffer and trigger for a epson devices.  If
// 'trigger_handler' is NULL the default trigger handler will be used. The
// default trigger handler will simply read the registers assigned to the
// currently active channels.
// 
// epson_cleanup_buffer_and_trigger() should be called to free the resources
// allocated by this function.
//
// Parameters:
// - epson: the Epson device.
// - indio_dev: the IIO device.
// - trigger_handler: trigger handler, may be NULL.
//
// Returns 0 on success, a negative error code otherwise.
// 
//
//----------------------------------------------------------------------
int epson_setup_buffer_and_trigger(struct epson *epson, struct iio_dev *indio_dev,
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
		ret = EPSON_ERROR;
		goto error_buffer_cleanup;
	}
	
	//success 
	return 0;

error_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);
	return ret;
}
EXPORT_SYMBOL_GPL(epson_setup_buffer_and_trigger);


//----------------------------------------------------------------------
// epson_cleanup_buffer_and_trigger()
// Frees buffer and trigger resources
// Frees resources allocated by epson_setup_buffer_and_trigger()
//
// Parameters:
// - epson: the Epson device
// - indio_dev: the IIO device
// 
//----------------------------------------------------------------------
void epson_cleanup_buffer_and_trigger(struct epson *epson, struct iio_dev *indio_dev)
{
	if (epson->spi->irq)
		epson_remove_trigger(epson);
	kfree(epson->buffer);
	kfree(epson->xfer);
	iio_triggered_buffer_cleanup(indio_dev);
}
EXPORT_SYMBOL_GPL(epson_cleanup_buffer_and_trigger);


//----------------------------------------------------------------------
// epson_data_rdy_trigger_set_state()
// Sets the state of the IIO trigger
//
// Parameters:
// - trig: the IIO trigger
// - state: boolean defines whether trigger to be enabled (1) or disabled (0)
//
//----------------------------------------------------------------------
static int epson_data_rdy_trigger_set_state(struct iio_trigger *trig, bool state)
{
	struct epson *epson = iio_trigger_get_drvdata(trig);
	int ret = epson_enable_irq(epson, state);

	if (ret)
		goto error_ret;
                        
	if ( state ) {       
		ret = epson_write_reg_8(epson, epson->data->mode_ctrl_reg, CMD_BEGIN_SAMPLING);
	} else {
		ret = epson_write_reg_8(epson, epson->data->mode_ctrl_reg, CMD_END_SAMPLING);
	}
	if (ret) 
		printk(KERN_INFO "Error writing mode_ctrl_reg\n");
        
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
// - epson: the Epson device
// - indio_dev: the IIO device
// 
// Returns 0 on success or a negative error code
// 
// Note: epson_remove_trigger() should be used to free the trigger.
//
//----------------------------------------------------------------------
int epson_probe_trigger(struct epson *epson, struct iio_dev *indio_dev)
{
	int ret;

	epson->trig = iio_trigger_alloc("%s-dev%d", indio_dev->name, indio_dev->id);
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
	printk(KERN_INFO "epson_probe_trigger, irq succeeded\n");

	return 0;

error_free_irq:	//failed so free everthing up
	printk(KERN_INFO "epson_probe_trigger, irq failed\n");
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
// - epson: the Epson device
// 
//----------------------------------------------------------------------
void epson_remove_trigger(struct epson *epson)
{
	iio_trigger_unregister(epson->trig);
	free_irq(epson->spi->irq, epson->trig);
	iio_trigger_free(epson->trig);
}
EXPORT_SYMBOL_GPL(epson_remove_trigger);

