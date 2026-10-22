// SPDX-License-Identifier: GPL-2.0+
/*
 * Analog Devices AD5940/AD5941 high precision, impedance and
 * electrochemical front end driver
 *
 * Copyright (C) 2026 Analog Devices, Inc.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

/* SPI command words */
#define AD5940_SPI_CMD_SETADDR		0x20
#define AD5940_SPI_CMD_WRITEREG		0x2d
#define AD5940_SPI_CMD_READREG		0x6d

/* Registers in this range are 32 bits wide, all others are 16 bits wide. */
#define AD5940_REG32_MIN		0x1000
#define AD5940_REG32_MAX		0x3014

#define AD5940_REG_ADIID		0x0400
#define AD5940_REG_CHIPID		0x0404
#define AD5940_CHIPID_PARTID_MSK	GENMASK(15, 4)
#define AD5940_CHIPID_REV_MSK		GENMASK(3, 0)

#define AD5940_ADIID			0x4144
#define AD5940_PARTID			0x550

#define AD5940_MAX_REGISTER		0xffff

#define AD5940_RESET_PULSE_US		1000
#define AD5940_RESET_WAIT_MS		10

/*
 * The AD5940 SPI protocol needs a first transfer to set the register address
 * and a second one, with chip select toggled in between, to access it.
 */
struct ad5940_bus {
	struct spi_device *spi;
	/* DMA safe buffers, accesses are serialized by the regmap lock. */
	u8 setaddr[3] __aligned(IIO_DMA_MINALIGN);
	u8 wr[5];
	u8 rd_cmd[2];
	u8 rd[4];
};

struct ad5940_state {
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
};

static bool ad5940_reg_is_32bit(unsigned int reg)
{
	return reg >= AD5940_REG32_MIN && reg <= AD5940_REG32_MAX;
}

static void ad5940_bus_set_addr(struct ad5940_bus *bus,
				struct spi_transfer *xfer, unsigned int reg)
{
	bus->setaddr[0] = AD5940_SPI_CMD_SETADDR;
	put_unaligned_be16(reg, &bus->setaddr[1]);

	xfer->tx_buf = bus->setaddr;
	xfer->len = sizeof(bus->setaddr);
	xfer->cs_change = 1;
	xfer->cs_change_delay.value = 1;
	xfer->cs_change_delay.unit = SPI_DELAY_UNIT_USECS;
}

static int ad5940_bus_write(void *context, const void *data, size_t count)
{
	struct ad5940_bus *bus = context;
	struct spi_transfer xfers[2] = { };
	unsigned int reg, len;

	/* data is a big-endian 16-bit address followed by a 32-bit value. */
	if (count != 6)
		return -EINVAL;

	reg = get_unaligned_be16(data);
	ad5940_bus_set_addr(bus, &xfers[0], reg);

	/* 16-bit registers only use the two least significant value bytes. */
	len = ad5940_reg_is_32bit(reg) ? 4 : 2;
	bus->wr[0] = AD5940_SPI_CMD_WRITEREG;
	memcpy(&bus->wr[1], data + 2 + (4 - len), len);

	xfers[1].tx_buf = bus->wr;
	xfers[1].len = 1 + len;

	return spi_sync_transfer(bus->spi, xfers, ARRAY_SIZE(xfers));
}

static int ad5940_bus_read(void *context, const void *reg_buf, size_t reg_size,
			   void *val_buf, size_t val_size)
{
	struct ad5940_bus *bus = context;
	struct spi_transfer xfers[3] = { };
	unsigned int reg, len;
	int ret;

	if (reg_size != 2 || val_size != 4)
		return -EINVAL;

	reg = get_unaligned_be16(reg_buf);
	ad5940_bus_set_addr(bus, &xfers[0], reg);

	len = ad5940_reg_is_32bit(reg) ? 4 : 2;
	/* The command is followed by one dummy byte before data is returned. */
	bus->rd_cmd[0] = AD5940_SPI_CMD_READREG;
	bus->rd_cmd[1] = 0;

	xfers[1].tx_buf = bus->rd_cmd;
	xfers[1].len = sizeof(bus->rd_cmd);
	xfers[2].rx_buf = bus->rd;
	xfers[2].len = len;

	ret = spi_sync_transfer(bus->spi, xfers, ARRAY_SIZE(xfers));
	if (ret)
		return ret;

	memset(val_buf, 0, val_size);
	memcpy(val_buf + (val_size - len), bus->rd, len);

	return 0;
}

static const struct regmap_bus ad5940_regmap_bus = {
	.write = ad5940_bus_write,
	.read = ad5940_bus_read,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_config ad5940_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.max_register = AD5940_MAX_REGISTER,
	.use_single_read = true,
	.use_single_write = true,
	.cache_type = REGCACHE_NONE,
};

static int ad5940_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad5940_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_info ad5940_info = {
	.debugfs_reg_access = ad5940_reg_access,
};

static void ad5940_hw_reset(struct ad5940_state *st)
{
	if (!st->reset_gpio)
		return;

	gpiod_set_value_cansleep(st->reset_gpio, 1);
	fsleep(AD5940_RESET_PULSE_US);
	gpiod_set_value_cansleep(st->reset_gpio, 0);
	msleep(AD5940_RESET_WAIT_MS);
}

static int ad5940_check_id(struct device *dev, struct ad5940_state *st)
{
	unsigned int adiid, chipid;
	int ret;

	/*
	 * The AFE may be hibernating and wakes up on bus activity, so the
	 * result of this first access is discarded.
	 */
	regmap_read(st->regmap, AD5940_REG_ADIID, &adiid);

	ret = regmap_read(st->regmap, AD5940_REG_ADIID, &adiid);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read ADIID\n");

	ret = regmap_read(st->regmap, AD5940_REG_CHIPID, &chipid);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read CHIPID\n");

	if (adiid != AD5940_ADIID)
		dev_warn(dev, "Unexpected ADIID 0x%04x, expected 0x%04x\n",
			 adiid, AD5940_ADIID);
	else if (FIELD_GET(AD5940_CHIPID_PARTID_MSK, chipid) != AD5940_PARTID)
		dev_warn(dev, "Unexpected part ID 0x%03lx\n",
			 FIELD_GET(AD5940_CHIPID_PARTID_MSK, chipid));
	else
		dev_dbg(dev, "Silicon revision %lu\n",
			FIELD_GET(AD5940_CHIPID_REV_MSK, chipid));

	return 0;
}

static int ad5940_probe(struct spi_device *spi)
{
	static const char * const regulators[] = { "avdd", "dvdd" };
	struct device *dev = &spi->dev;
	struct ad5940_state *st;
	struct ad5940_bus *bus;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	bus = devm_kzalloc(dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;

	bus->spi = spi;

	st->regmap = devm_regmap_init(dev, &ad5940_regmap_bus, bus,
				      &ad5940_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to initialize regmap\n");

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(regulators),
					     regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable supplies\n");

	st->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(st->reset_gpio),
				     "Failed to get reset GPIO\n");

	ad5940_hw_reset(st);

	ret = ad5940_check_id(dev, st);
	if (ret)
		return ret;

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad5940_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct spi_device_id ad5940_id[] = {
	{ "ad5940" },
	{ "ad5941" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad5940_id);

static const struct of_device_id ad5940_of_match[] = {
	{ .compatible = "adi,ad5940" },
	{ .compatible = "adi,ad5941" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad5940_of_match);

static struct spi_driver ad5940_driver = {
	.driver = {
		.name = "ad5940",
		.of_match_table = ad5940_of_match,
	},
	.probe = ad5940_probe,
	.id_table = ad5940_id,
};
module_spi_driver(ad5940_driver);

MODULE_DESCRIPTION("Analog Devices AD5940/AD5941 AFE driver");
MODULE_LICENSE("GPL");
