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
#include <linux/iio/sysfs.h>
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

static const u16 ads124x_sample_freq_avail[] = {5, 10, 20, 40, 80, 160,
                                                320, 640, 1000, 2000};

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("5 10 20 40 80 160 320 640 1000 2000");

static struct attribute *ads124x_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ads124x_attribute_group = {
	.attrs = ads124x_attributes,
};

struct ads124x_state {
	struct spi_device *spi;
	int drdy_gpio;
	int start_gpio;
	int reset_gpio;
	int vref_mv;
        int sample_rate;

        struct mutex lock;
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

        ret = spi_write(st->spi, write_cmd, 3);

        printk(KERN_DEBUG "%s: write ret = %d\n", __FUNCTION__, ret);

        return ret;
}


/*            */
/* Converting */
/*            */
static u32 ads124x_sample_to_32bit(u8 *sample)
{
        int sample32 = 0;
        sample32 = sample[0] << 16;
        sample32 |= sample[1] << 8;
        sample32 |= sample[2];
        return sign_extend32(sample32, 23);
}


static void wait_for_drdy(int drdy_gpio)
{
        u8 drdy;

        for(;;) {
                drdy = gpio_get_value(drdy_gpio);
                printk(KERN_INFO "Waiting for DRDY = 0x%x\n", drdy);
                if (drdy == 0x00)
                        return;
                msleep(20);
        }
}

static int ads124x_convert(struct ads124x_state *st)
{
        u8 cmd[1], res[3];
        u32 res32;
        int ret;
        cmd[0] = ADS124X_SPI_RDATA;

        /* Wait for the SPI bus to receive the register settings made
         * before this function was called. */
        wait_for_drdy(st->drdy_gpio);

        ret = spi_write(st->spi, cmd, 1);

        /* Wait for conversion results */
        wait_for_drdy(st->drdy_gpio);

        ret = spi_read(st->spi, res, 3);

        printk(KERN_INFO "%s: ret: %d\n", __FUNCTION__, ret);
        printk(KERN_INFO "%s: Conversion (hex): %x %x %x\n",
               __FUNCTION__, res[0], res[1], res[2]);

        res32 = ads124x_sample_to_32bit(res);
        printk(KERN_INFO "%s: Conversion (32bit): %d\n",
               __FUNCTION__, res32);

        return res32;
}


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
        gpio_set_value(st->reset_gpio, 1);

        cmd[0] = ADS124X_SPI_RESET;
        ret = spi_write(st->spi, cmd, 1);

        printk(KERN_INFO "%s: resetting. ret = %d\n", __FUNCTION__, ret);

        msleep(200);

        return;
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

static int ads124x_get_sample_rate(struct ads124x_state *st)
{
        u8 result;
        int ret;
        ret = ads124x_read_reg(st, ADS124X_REG_SYS0, &result);
        return (ret < 0) ? ret : (result & 0x0f);
}


/* Setting registers */
static int ads124x_select_input(struct ads124x_state *st,
                                struct iio_dev *indio_dev,
                                struct iio_chan_spec const *chan)
{
        u8 mux0;
        int ret;

        ret = ads124x_read_reg(st, ADS124X_REG_MUX0, &mux0);

        if (ret < 0)
                return ret;

        /* Preserve the two most significant bits */
        mux0 &= 0xc0;

        /* Select positive and negative inputs */
        mux0 |= (chan->channel << 3) | chan->channel2;

        ret = ads124x_write_reg(st, ADS124X_REG_MUX0, &mux0, 1);

        return ret;
}

static int ads124x_set_pga_gain(struct ads124x_state *st, u8 gain)
{
        int ret;

        mutex_lock(&st->lock);

        ret = ads124x_get_pga_gain(st);

        if (ret < 0)
                goto ads124x_release_lock;

        ret &= ~(1 << 4);
        ret &= ~(1 << 5);
        ret &= ~(1 << 6);
        gain = ret | (gain << 4);
        
        printk(KERN_DEBUG "%s: Setting gain=0x%x\n", __FUNCTION__, gain);

        ret = ads124x_write_reg(st, ADS124X_REG_SYS0, &gain, 1);

        if (ret < 0)
                goto ads124x_release_lock;

ads124x_release_lock:
        mutex_unlock(&st->lock);
        return ret;
}


static int ads124x_set_sample_rate(struct ads124x_state *st)
{
        u8 result;
        int ret;
        ret = ads124x_read_reg(st, ADS124X_REG_SYS0, &result);
        if (ret < 0)
                return ret;

        result |= 0x0f & st->sample_rate;

        ret = ads124x_write_reg(st, ADS124X_REG_SYS0, &result, 1);

        return ret;
}

static int ads124x_read_raw(struct iio_dev *indio_dev,
                            struct iio_chan_spec const *chan,
                            int *val, int *val2, long mask)
{
	struct ads124x_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
                ads124x_select_input(st, indio_dev, chan);
                wait_for_drdy(st->drdy_gpio);
                ret = ads124x_convert(st);
		mutex_unlock(&st->lock);
                *val = ret;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv;
		*val2 =  (1 << 23) - 1;
                return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_SAMP_FREQ:
                *val = ads124x_sample_freq_avail[st->sample_rate];
                *val2 = 0;
		return IIO_VAL_INT;

	default:
		break;
	}

	return -EINVAL;
}


static int ads124x_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct ads124x_state *st = iio_priv(indio_dev);
	int ret;
        u8 i;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		for (i = 0; i < 8; i++) /* 8 possible values for PGA gain */
			if (val2 == i)
				return ads124x_set_pga_gain(st, 1 << i);
		break;

	case IIO_CHAN_INFO_SAMP_FREQ:
		for (i = 0; i < ARRAY_SIZE(ads124x_sample_freq_avail); i++)
			if (val == ads124x_sample_freq_avail[i]) {
                                mutex_lock(&st->lock);
                                st->sample_rate = i;
                                ret = ads124x_set_sample_rate(st);
                                mutex_unlock(&st->lock);
                                return ret;
                                break;
                        }

	default:
		break;
	}

	return -EINVAL;
}

static int ads124x_write_raw_get_fmt(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan,
				     long mask)
{
	return IIO_VAL_INT_PLUS_NANO;
}

static const struct iio_info ads124x_iio_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &ads124x_read_raw,
	.write_raw = &ads124x_write_raw,
	.write_raw_get_fmt = ads124x_write_raw_get_fmt,
	.attrs = &ads124x_attribute_group,
};


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
        u8 buf;

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

        ads124x_set_pga_gain(st, 0);

        ads124x_read_reg(st, ADS124X_REG_SYS0, &buf);
        printk(KERN_INFO "SYS0 = 0x%x\n", buf);

        ads124x_read_reg(st, ADS124X_REG_SYS0, &buf);
        printk(KERN_INFO "SYS0 = 0x%x\n", buf);

        ads124x_read_reg(st, ADS124X_REG_MUX0, &buf);
        printk(KERN_INFO "MUX0 = 0x%x\n", buf);

        buf = 0x13;
        ads124x_write_reg(st, ADS124X_REG_MUX0, &buf, 1);
        printk(KERN_INFO "MUX0 = 0x%x\n", buf);

        ads124x_read_reg(st, ADS124X_REG_MUX1, &buf);
        printk(KERN_INFO "MUX1 = 0x%x\n", buf);

        printk(KERN_INFO "PGA gain = 0x%x\n", ads124x_get_pga_gain(st));

        ads124x_convert(st);

        buf = 0x1b;
        ads124x_write_reg(st, ADS124X_REG_MUX0, &buf, 1);
        printk(KERN_INFO "MUX0 = 0x%x\n", buf);

        ads124x_convert(st);
}

#endif


/*                                      */
/* Probing, removing and binding to spi */
/*                                      */

static int ads124x_init_chan_array(struct iio_dev *indio_dev,
                                   struct device_node *np)
{
        struct iio_chan_spec *chan_array;
        int num_inputs = indio_dev->num_channels * 2;
        int *channels_config;
        int i, ret;

        channels_config = kcalloc(num_inputs,
                                  sizeof(u32),
                                  GFP_KERNEL);

        ret = of_property_read_u32_array(np, "channels",
                                         channels_config, num_inputs);
        if (ret < 0)
                return ret;

        chan_array = kcalloc(indio_dev->num_channels,
                             sizeof(struct iio_chan_spec),
                             GFP_KERNEL);

        if (chan_array == NULL)
                return -ENOMEM;

        for (i = 0; i < num_inputs; i++) {
                if (i % 2 == 0) { /* Group inputs into pairs */
                        struct iio_chan_spec *chan = chan_array + (i / 2);
                        chan->type = IIO_TEMP;
                        chan->indexed = 1;
                        chan->channel = channels_config[i];
                        chan->channel2 = channels_config[i + 1];
                        chan->differential = 1;
                        chan->scan_index = i;
                        chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
                        chan->info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
                                BIT(IIO_CHAN_INFO_SAMP_FREQ);
                        chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
                }
        }

        indio_dev->channels = chan_array;

        return indio_dev->num_channels;
}

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

	/* FIXME: External ref (move to dt) */
        st->vref_mv = 2670;

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

        st->sample_rate = 0;

	/* Setup the ADC channels available on the board */
        ret = of_property_read_u32(np, "#channels", &indio_dev->num_channels);
        if (ret < 0)
                goto error; /* FIXME: raise a decent error */

        ads124x_init_chan_array(indio_dev, np);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error;

        ads124x_reset(st);
        ads124x_start(st);
        ads124x_stop_reading_continuously(st);

        mutex_init(&st->lock);

#ifdef ADS124X_TEST
        ads124x_test(st);
#endif
	return 0;

error:
	iio_device_free(indio_dev);
	dev_err(&spi->dev, "ADS124x: Error while probing.\n");

	return ret;
}


static int ads124x_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
        struct ads124x_state *st;

	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);

        st = iio_priv(indio_dev);
        mutex_destroy(&st->lock);

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
