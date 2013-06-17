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

static void ads124x_select_input(struct ads124x_state *st, unsigned int nr)
{
        /* TODO */
        return;
}


static int ads124x_read_single(struct ads124x_state *st, int *val,
	unsigned int address)
{
        /* TODO */
        return 0;
}

static int ads124x_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long m)
{
	struct ads124x_state *st = iio_priv(indio_dev);
	int ret;

	/* TODO */
	return 0;
}

static const struct iio_info ads124x_iio_info = {
	.read_raw = &ads124x_read_raw,
	.driver_module = THIS_MODULE,
};

static int ads124x_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct iio_dev *indio_dev;
	struct ads124x_state *adc;
	int ret = -ENODEV;

	indio_dev = iio_device_alloc(sizeof(*adc));
	if (indio_dev == NULL)
		return -ENOMEM;

	adc = iio_priv(indio_dev);

	/* Read node values */
	adc->drdy_gpio = of_get_named_gpio(np, "drdy-gpio", 0);
	adc->start_gpio = of_get_named_gpio(np, "start-gpio", 0);
	adc->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);

	/* TODO: External ref */
	adc->vref_uvad = 2048000; /* 2.048V - page 29 */

	spi_set_drvdata(spi, indio_dev);
	adc->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = np->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ads124x_iio_info;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error;

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
