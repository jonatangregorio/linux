/*
 * Texas Instruments ADS1246/7/8 temperature sensor and ADC driver
 *
 * Copyright (c) 2013 O.S. Systems Software LTDA.
 * Copyright (c) 2013 Otavio Salvador <otavio@ossystems.com.br>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/delay.h>

/* Register addresses for ADS1247 and ADS1248 */
#define ADS124X_REG_MUX0      0x00
#define ADS124X_REG_VBIAS     0x01
#define ADS124X_REG_MUX1      0x02
#define ADS124X_REG_SYS0      0x03
#define ADS124X_REG_OFC0      0x04
#define ADS124X_REG_OFC1      0x05
#define ADS124X_REG_OFC2      0x06
#define ADS124X_REG_FSC0      0x07
#define ADS124X_REG_FSC1      0x08
#define ADS124X_REG_FSC2      0x09
#define ADS124X_REG_IDAC0     0x0a
#define ADS124X_REG_IDAC1     0x0b
#define ADS124X_REG_GPIOCFG   0x0c
#define ADS124X_REG_GPIODIR   0x0d
#define ADS124X_REG_GPIODAT   0x0e

/* SPI Commands */
#define ADS124X_SPI_WAKEUP    0x00
#define ADS124X_SPI_SLEEP     0x02
#define ADS124X_SPI_SYNC1     0x04
#define ADS124X_SPI_SYNC2     0x04
#define ADS124X_SPI_RESET     0x06
#define ADS124X_SPI_NOP       0xff
#define ADS124X_SPI_RDATA     0x12
#define ADS124X_SPI_RDATAC    0x14
#define ADS124X_SPI_SDATAC    0x16
#define ADS124X_SPI_RREG      0x20
#define ADS124X_SPI_WREG      0x40
#define ADS124X_SPI_SYSOCAL   0x60
#define ADS124X_SPI_SYSGCAL   0x61
#define ADS124X_SPI_SELFOCAL  0x62

#define ADS124X_SINGLE_REG    0x00

#define ADS124X_SAMPLE_RATE   2000 /* Number of samples per second */

struct ads124x_state {
	struct spi_device *spi;
	int drdy_gpio;
	int start_gpio;
	int reset_gpio;
	int vref_uvad;

        struct spi_transfer single_xfer[1];
        struct spi_message single_msg;

        /* FIXME: this is the data buffer.  Understand it better and */
        /* maybe fix types/size */
        int data[8] ____cacheline_aligned;
};

static const struct of_device_id ads124x_ids[] = {
	{ .compatible = "ti,ads1247" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ads124x_ids);

static int ads124x_read_reg(struct ads124x_state *st,
                            u8 reg,
                            u8 *buf)
{
        u8 read_cmd[2];
        int ret;

        *buf = 0xff;

        read_cmd[0] = ADS124X_SPI_RREG | reg;
        read_cmd[1] = ADS124X_SINGLE_REG;
        printk(KERN_INFO "DRDY (before) = 0x%x\n", gpio_get_value(st->drdy_gpio));
        ret = spi_write_then_read(st->spi, read_cmd, 2, buf, 1);
        printk(KERN_INFO "DRDY (after) = 0x%x\n", gpio_get_value(st->drdy_gpio));
        return ret;
}

static int ads124x_write_reg(struct ads124x_state *st,
                             u8 reg,
                             u8 *buf,
                             size_t len)
{
        struct spi_transfer transfer_data[2];
        u8 write_cmd[2];
        struct spi_message message;
        int ret;

        write_cmd[0] = ADS124X_SPI_WREG | reg;
        write_cmd[1] = ADS124X_SINGLE_REG;

        transfer_data[0].tx_buf = write_cmd;
        transfer_data[0].rx_buf = NULL;
        transfer_data[0].len = 2;
        transfer_data[0].speed_hz = 2000000;
        transfer_data[0].bits_per_word = 8;

        printk(KERN_INFO "==== write cmd = 0x%x\n", write_cmd[0]);
        printk(KERN_INFO "==== write 2nd cmd = 0x%x\n", write_cmd[1]);
        printk(KERN_INFO "==== write arg = 0x%x\n", *buf);

        transfer_data[1].tx_buf = buf;
        transfer_data[1].rx_buf = NULL;
        transfer_data[1].len = len;
        transfer_data[1].speed_hz = 2000000;
        transfer_data[1].bits_per_word = 8;

        spi_message_init(&message);
        spi_message_add_tail(transfer_data, &message);
        ret = spi_sync(st->spi, &message);
        printk(KERN_INFO "==== write ret = %d\n", ret);
        return ret;
}

static int ads124x_select_input(struct ads124x_state *st, unsigned int nr)
{
        /* FIXME: assuming nr is a decimal number indicating the input
         * number. */

        u8 mux0_val;

        ads124x_read_reg(st, ADS124X_REG_MUX0, &mux0_val);

        mux0_val &= ~(1 << 0);
        mux0_val &= ~(1 << 1);
        mux0_val &= ~(1 << 2);
        mux0_val |= (u8)nr;

        return spi_write(st->spi, &mux0_val, 1);
}


static int ads124x_update_scan_mode(struct iio_dev *indio_dev,
                                    const unsigned long *scan_mask)
{
        struct ads124x_state *st = iio_priv(indio_dev);

        /* TODO: check what scan_mask really is and select inputs
         * according to it */
        unsigned int nr = find_first_bit(scan_mask, indio_dev->masklength);

        ads124x_select_input(st, nr);

        return 0;
}

static int ads124x_read_single(struct ads124x_state *st, 
                               int *val,
                               unsigned int address)
{
        int ret;

        ads124x_select_input(st, address);

        /* TODO */

        return ret;
}

static int ads124x_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long m)
{
	/* struct ads124x_state *st = iio_priv(indio_dev); */
	/* int ret; */

	/* TODO */
	return 0;
}

static const struct iio_info ads124x_iio_info = {
	.read_raw = &ads124x_read_raw,
        .update_scan_mode = &ads124x_update_scan_mode,
	.driver_module = THIS_MODULE,
};

/*               */
/* Start & reset */
/*               */
static void ads124x_reset(struct ads124x_state *adc)
{
        printk(KERN_INFO "ADS124x: resetting\n");

        gpio_set_value(adc->reset_gpio, 0);
        msleep(600);
        gpio_set_value(adc->reset_gpio, 1);
        msleep(600);
        return;
}

static void ads124x_start(struct ads124x_state *adc)
{
        printk(KERN_INFO "ADS124x: starting\n");
        gpio_set_value(adc->start_gpio, 1);
        /* FIXME: the sleep time is not accurate: see the datasheet, */
        /* table 15 at page 33. */
        msleep(1000000 / ADS124X_SAMPLE_RATE);
        return;
}

static int ads124x_wakeup(struct ads124x_state *adc)
{
        u8 wakeup_cmd[1];
        int ret;
        wakeup_cmd[0] = ADS124X_SPI_WAKEUP;
 
        ret = spi_write(adc->spi, wakeup_cmd, 1);

        printk(KERN_INFO "ADS124x: waking up. ret=%d\n", ret);

        return ret;
}

/*                            */
/* Information from registers */
/*                            */

static int ads124x_get_oscilator_status(struct ads124x_state *adc) {
        /* 0 => Internal oscilator */
        /* 1 => External oscilator */
        u8 result;
        int ret;

        ret = ads124x_read_reg(adc, ADS124X_REG_MUX1, &result);
        return (ret < 0) ? ret : (result & 0x01);
}

static int ads124x_get_pga_gain(struct ads124x_state *adc)
{
        u8 result;
        int ret;

        ret = ads124x_read_reg(adc, ADS124X_REG_SYS0, &result);
        printk(KERN_INFO "SYS0 (pga_gain) = 0x%x\n", result);
        return (ret < 0) ? ret : (result & 0x70);
}

static int ads124x_get_output_data_rate(struct ads124x_state *adc)
{
        u8 result;
        int ret;

        ret = ads124x_read_reg(adc, ADS124X_REG_SYS0, &result);
        printk(KERN_INFO "SYS0 (output_data_rate) = 0x%x\n", result);
        return (ret < 0) ? ret : (result & 0x0f);
}

static int ads124x_get_adc_id(struct ads124x_state *adc)
{
        u8 result;
        int ret;

        ret = ads124x_read_reg(adc, ADS124X_REG_IDAC0, &result);
        return (ret < 0) ? ret : (result >> 4);
}

static int ads124x_get_negative_input(struct ads124x_state *adc)
{
        u8 result;
        int ret;
        ret = ads124x_read_reg(adc, ADS124X_REG_MUX0, &result);
        return (ret < 0) ? ret : (result & 0x07);
}


/* Setting registers */

static int ads124x_set_pga_gain(struct ads124x_state *adc, u8 gain)
{
        u8 cur;
        int ret;

        cur = ads124x_get_pga_gain(adc);

        if (cur < 0)
                return cur;

        printk(KERN_INFO "======= Current SYS0=0x%x\n", cur);

        cur &= ~(1 << 4);
        cur &= ~(1 << 5);
        cur &= ~(1 << 6);
        gain = cur | gain << 4;
        
        printk(KERN_INFO "======= Setting gain=0x%x\n", gain);
        ret = ads124x_write_reg(adc, ADS124X_REG_SYS0, &gain, 1);

        return 0;
}

/*                                      */
/* Probing, removing and binding to spi */
/*                                      */

static int ads124x_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct iio_dev *indio_dev;
	struct ads124x_state *adc;
	int ret = -ENODEV;
        int status;
        int err;

        printk(KERN_INFO "ADS124x: probing\n");
        printk(KERN_INFO "ADS124x speed: %d\n", spi->max_speed_hz);

	indio_dev = iio_device_alloc(sizeof(*adc));
	if (indio_dev == NULL)
		return -ENOMEM;

        adc = iio_priv(indio_dev);

	/* Read node values */
	adc->drdy_gpio = of_get_named_gpio(np, "drdy-gpio", 0);
	adc->start_gpio = of_get_named_gpio(np, "start-gpio", 0);
	adc->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);


        err = devm_gpio_request_one(&indio_dev->dev, adc->drdy_gpio,
                                    GPIOF_IN, "adc-drdy");
        if (err) {
                dev_err(&indio_dev->dev, "failed to get adc-drdy-gpios: %d\n", err);
                return err;
        }

        err = devm_gpio_request_one(&indio_dev->dev, adc->start_gpio,
                                    GPIOF_OUT_INIT_LOW, "adc-start");
        if (err) {
                dev_err(&indio_dev->dev, "failed to get adc-start-gpios: %d\n", err);
                return err;
        }

        err = devm_gpio_request_one(&indio_dev->dev, adc->reset_gpio,
                                    GPIOF_OUT_INIT_LOW, "adc-reset");
        if (err) {
                dev_err(&indio_dev->dev, "failed to get adc-reset-gpios: %d\n", err);
                return err;
        }


        /* gpio_direction_output(adc->reset_gpio, 1); */
        /* gpio_direction_output(adc->start_gpio, 0); */

        printk(KERN_INFO "reset GPIO=%d\n", adc->reset_gpio);

	/* TODO: External ref */
	adc->vref_uvad = 2048000; /* 2.048V - page 29 */

	spi_set_drvdata(spi, indio_dev);
	adc->spi = spi;

        if (adc->spi->mode != SPI_MODE_3)
                adc->spi->mode = SPI_MODE_0;
        adc->spi->bits_per_word = 8;

        status = spi_setup(spi);
        printk(KERN_INFO "spi_setup returned %d\n", status);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = np->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ads124x_iio_info;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error;

        ads124x_start(adc);
        ads124x_reset(adc);
        ads124x_wakeup(adc);

        printk(KERN_INFO "ADS124x: Negative input=%x\n",
               ads124x_get_negative_input(adc));
        printk(KERN_INFO "ADS124x: ID=%x\n", ads124x_get_adc_id(adc));

        printk(KERN_INFO "ADS124x: Output data rate=%x\n",
               ads124x_get_output_data_rate(adc));

        printk(KERN_INFO "ADS124x: PGA gain=%x\n", ads124x_get_pga_gain(adc));
        ads124x_set_pga_gain(adc, 4);
        msleep(1000);
        printk(KERN_INFO "ADS124x: PGA gain=%x\n", ads124x_get_pga_gain(adc));

        printk(KERN_INFO "ADS124x: PGA gain=%x\n", ads124x_get_pga_gain(adc));
        printk(KERN_INFO "ADS124x: PGA gain=%x\n", ads124x_get_pga_gain(adc));
        printk(KERN_INFO "ADS124x: PGA gain=%x\n", ads124x_get_pga_gain(adc));

        printk(KERN_INFO "ADS124x: Output data rate=%x\n",
               ads124x_get_output_data_rate(adc));
        printk(KERN_INFO "ADS124x: Oscilator status=%x\n",
               ads124x_get_oscilator_status(adc));
        printk(KERN_INFO "ADS124x: Negative input=%x\n",
               ads124x_get_negative_input(adc));
	return 0;

error:
	iio_device_free(indio_dev);
	dev_err(&spi->dev, "Error while doing probe\n");

	return ret;
}


static int ads124x_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);

	return 0;
}

static struct spi_driver ads124x_driver = {
	.driver = {
		.name = "ti-ads124x",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ads124x_ids),
	},
	.probe = ads124x_probe,
	.remove = ads124x_remove,
};
module_spi_driver(ads124x_driver);

MODULE_AUTHOR("Otavio Salvador <otavio@ossystems.com.br>");
MODULE_DESCRIPTION("Texas Instruments ADS1246/7/8 driver");
MODULE_LICENSE("GPL v2");
