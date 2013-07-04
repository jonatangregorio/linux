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

        /* FIXME: this is the data buffer.  Understand it better and */
        /* maybe fix types/size */
        int data[8] ____cacheline_aligned;
};

static const struct of_device_id ads124x_ids[] = {
	{ .compatible = "ti,ads1247" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ads124x_ids);

static int ads124x_stop_reading_continuously(struct ads124x_state *st)
{
        u8 cmd[1];
        int ret;
        cmd[0] = ADS124X_SPI_SDATAC;

        ret = spi_write(st->spi, cmd, 1);

        printk(KERN_INFO "%s: stopping reading continuously. ret=%d\n",
               __FUNCTION__, ret);

        return ret;
}

static int ads124x_read_reg(struct ads124x_state *st,
                            u8 reg,
                            u8 *buf)
{
        u8 read_cmd[2];
        int ret;

        read_cmd[0] = ADS124X_SPI_RREG | reg;
        read_cmd[1] = ADS124X_SINGLE_REG;
        spi_write(st->spi, read_cmd, 2);

        ret = spi_read(st->spi, buf, 1);

        return ret;
}

static int ads124x_write_reg(struct ads124x_state *st,
                             u8 reg,
                             u8 *buf,
                             size_t len)
{
        u8 write_cmd[3];
        int ret;
        
        write_cmd[0] = ADS124X_SPI_WREG | reg;
        write_cmd[1] = ADS124X_SINGLE_REG;
        write_cmd[2] = *buf;

        printk(KERN_DEBUG "%s: cmd[0]=0x%x, cmd[1]=0x%x, cmd[2]=0x%x\n",
               __FUNCTION__, write_cmd[0], write_cmd[1], write_cmd[2]);
        
        ret = spi_write(st->spi, write_cmd, 3);

        printk(KERN_DEBUG "%s: write ret = %d\n", __FUNCTION__, ret);
        printk(KERN_DEBUG "%s: DRDY (after write) = 0x%x\n",
               __FUNCTION__, gpio_get_value(st->drdy_gpio));

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
static void ads124x_start(struct ads124x_state *st)
{
        gpio_set_value(st->start_gpio, 1);
        printk(KERN_INFO "%s: starting.\n", __FUNCTION__);
        /* FIXME: the sleep time is not accurate: see the datasheet, */
        /* table 15 at page 33. */
        msleep(200);
        return;
}

static void ads124x_reset(struct ads124x_state *st)
{
        u8 cmd[1];
        int ret;

        gpio_set_value(st->reset_gpio, 0);
        msleep(200);
        gpio_set_value(st->reset_gpio, 1);
        msleep(200);


        cmd[0] = ADS124X_SPI_RESET;
        ret = spi_write(st->spi, cmd, 1);

        printk(KERN_INFO "%s: resetting. ret = %d\n", __FUNCTION__, ret);

        msleep(200);

        return;
}

static int ads124x_wakeup(struct ads124x_state *st)
{
        u8 wakeup_cmd[4];
        int ret;
        wakeup_cmd[0] = ADS124X_SPI_WAKEUP;
        wakeup_cmd[1] = ADS124X_SPI_NOP;
        wakeup_cmd[2] = ADS124X_SPI_NOP;
        wakeup_cmd[3] = ADS124X_SPI_NOP;
 
        ret = spi_write(st->spi, wakeup_cmd, 4);

        printk(KERN_INFO "%s: waking up. ret=%d\n", __FUNCTION__, ret);

        return ret;
}

/*                            */
/* Information from registers */
/*                            */

static int ads124x_get_oscilator_status(struct ads124x_state *st) {
        /* 0 => Internal oscilator */
        /* 1 => External oscilator */
        u8 result;
        int ret;

        ret = ads124x_read_reg(st, ADS124X_REG_MUX1, &result);
        return (ret < 0) ? ret : (result & 0x01);
}

static int ads124x_get_pga_gain(struct ads124x_state *st)
{
        u8 result;
        int ret;

        ret = ads124x_read_reg(st, ADS124X_REG_SYS0, &result);
        return (ret < 0) ? ret : (result & 0x70);
}

static int ads124x_get_output_data_rate(struct ads124x_state *st)
{
        u8 result;
        int ret;

        ret = ads124x_read_reg(st, ADS124X_REG_SYS0, &result);
        return (ret < 0) ? ret : (result & 0x0f);
}

static int ads124x_get_adc_id(struct ads124x_state *st)
{
        u8 result;
        int ret;

        ret = ads124x_read_reg(st, ADS124X_REG_IDAC0, &result);
        return (ret < 0) ? ret : (result >> 4);
}

static int ads124x_get_negative_input(struct ads124x_state *st)
{
        u8 result;
        int ret;
        ret = ads124x_read_reg(st, ADS124X_REG_MUX0, &result);
        return (ret < 0) ? ret : (result & 0x07);
}


/* Setting registers */

static int ads124x_set_pga_gain(struct ads124x_state *st, u8 gain)
{
        u8 cur;
        int ret;

        cur = ads124x_get_pga_gain(st);

        if (cur < 0)
                return cur;

        printk(KERN_DEBUG "%s: Current SYS0=0x%x\n", __FUNCTION__, cur);

        cur &= ~(1 << 4);
        cur &= ~(1 << 5);
        cur &= ~(1 << 6);
        gain = cur | (gain << 4);
        
        printk(KERN_DEBUG "%s: Setting gain=0x%x\n", __FUNCTION__, gain);
        ret = ads124x_write_reg(st, ADS124X_REG_SYS0, &gain, 1);

        return 0;
}


/*       */
/* Tests */
/*       */

#define ADS124X_TEST

#ifdef ADS124X_TEST

#define ads124x_ok printk(KERN_INFO "    PASSED")
#define ads124x_fail printk(KERN_INFO "    FAILED!")

void ads124x_test(struct ads124x_state *st)
{
        int res;

        printk(KERN_INFO "=== Testing negative input\n");
        res = ads124x_get_negative_input(st);
        if (res == 0x01)
                ads124x_ok;
        else {
                ads124x_fail;
                printk(KERN_INFO "Expected %x, got %x\n", 0x01, res);
        }

        printk(KERN_INFO "=== Testing PGA gain (setting to 2)\n");
        ads124x_set_pga_gain(st, 2);
        res = ads124x_get_pga_gain(st);
        if (res == 0x20)
                ads124x_ok;
        else {
                ads124x_fail;
                printk(KERN_INFO "Expected %x, got %x\n", 0x20, res);
        }

        printk(KERN_INFO "=== Testing PGA gain (setting to 5)\n");
        ads124x_set_pga_gain(st, 5);
        res = ads124x_get_pga_gain(st);
        if (res == 0x50)
                ads124x_ok;
        else {
                ads124x_fail;
                printk(KERN_INFO "Expected %x, got %x\n", 0x50, res);
        }

        printk(KERN_INFO "ADS124x: ID=%x\n", ads124x_get_adc_id(st));

        printk(KERN_INFO "ADS124x: Output data rate=%x\n",
               ads124x_get_output_data_rate(st));

        printk(KERN_INFO "ADS124x: Oscilator status=%x\n",
               ads124x_get_oscilator_status(st));

}

#endif


/*                                      */
/* Probing, removing and binding to spi */
/*                                      */

static int ads124x_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct iio_dev *indio_dev;
	struct ads124x_state *st;
	int ret = -ENODEV;
        int status;
        int err;

        printk(KERN_INFO "%s: probing\n", __FUNCTION__);
        printk(KERN_INFO "%s: speed: %d\n", __FUNCTION__, spi->max_speed_hz);

	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

        st = iio_priv(indio_dev);

	/* Read node values */
	st->drdy_gpio = of_get_named_gpio(np, "drdy-gpio", 0);
	st->start_gpio = of_get_named_gpio(np, "start-gpio", 0);
	st->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);


        err = devm_gpio_request_one(&indio_dev->dev, st->drdy_gpio,
                                    GPIOF_IN, "adc-drdy");
        if (err) {
                dev_err(&indio_dev->dev, "failed to get adc-drdy-gpios: %d\n", err);
                return err;
        }

        err = devm_gpio_request_one(&indio_dev->dev, st->start_gpio,
                                    GPIOF_OUT_INIT_LOW, "adc-start");
        if (err) {
                dev_err(&indio_dev->dev, "failed to get adc-start-gpios: %d\n", err);
                return err;
        }

        err = devm_gpio_request_one(&indio_dev->dev, st->reset_gpio,
                                    GPIOF_OUT_INIT_LOW, "adc-reset");
        if (err) {
                dev_err(&indio_dev->dev, "failed to get adc-reset-gpios: %d\n", err);
                return err;
        }

        printk(KERN_INFO "%s: reset GPIO=%d\n", __FUNCTION__, st->reset_gpio);

	/* TODO: External ref */
	st->vref_uvad = 2048000; /* 2.048V - page 29 */

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;

        st->spi->mode = SPI_MODE_1;
        st->spi->bits_per_word = 8;

        status = spi_setup(spi);
        printk(KERN_INFO "%s: spi_setup returned %d\n", __FUNCTION__, status);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = np->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ads124x_iio_info;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error;

        ads124x_reset(st);
        ads124x_start(st);
        ads124x_stop_reading_continuously(st);

#ifdef ADS124X_TEST
        ads124x_test(st);
#endif
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
