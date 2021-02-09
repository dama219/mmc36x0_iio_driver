/*
 * MMC36X0 - MEMSIC 3-axis Magnetic Sensor
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for MMC36X0KL (7-bit I2C slave address 0x30).
 *
 * TODO: offset, ACPI, continuous measurement mode, PM
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/acpi.h>
#include <linux/pm.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/workqueue.h>

//#define MMC36X0_DEBUG

#ifdef MMC36X0_DEBUG
#define mmc36x0_debug(fmt, ...) \
	do { \
		pr_info("[mmc36x0] %s: " fmt, __func__, ##__VA_ARGS__); \
	} while(0)
#else
	#define mmc36x0_debug(fmt, ...)
#endif

#define MMC36X0_DRV_NAME "mmc36x0_magn"
#define MMC36X0_REGMAP_NAME "mmc36x0_regmap"

#define MMC36X0_REG_XOUT_L  	0x00
#define MMC36X0_REG_XOUT_H		0x01
#define MMC36X0_REG_YOUT_L		0x02
#define MMC36X0_REG_YOUT_H		0x03
#define MMC36X0_REG_ZOUT_L		0x04
#define MMC36X0_REG_ZOUT_H		0x05
#define MMC36X0_REG_TEMP		0x06

#define MMC36X0_REG_STATUS		0x07
#define MMC36X0_REG_CTRL0		0x08
#define MMC36X0_REG_CTRL1		0x09
#define MMC36X0_REG_CTRL2		0x0A

#define MMC36X0_REG_XTHRESH		0x0B
#define MMC36X0_REG_YTHRESH		0x0C
#define MMC36X0_REG_ZTHRESH		0x0D

#define MMC36X0_REG_SELFTEST	0x0E
#define MMC36X0_REG_PASSWORD	0x0F
#define MMC36X0_REG_OTPMODE		0x12
#define MMC36X0_REG_TESTMODE	0x13
#define MMC36X0_REG_SR_PWIDTH	0x20
#define MMC36X0_REG_OTP			0x2A
#define MMC36X0_REG_ID			0x2F



#define MMC36X0_STATUS_MEAS_M_DONE_BIT		BIT(0)
#define MMC36X0_STATUS_MEAS_T_DONE_BIT		BIT(1)
#define MMC36X0_STATUS_MOTION_DETECT_BIT	BIT(2)
#define MMC36X0_STATUS_INTERRUPT_MASK		(MMC36X0_STATUS_MEAS_M_DONE_BIT | \
			MMC36X0_STATUS_MEAS_T_DONE_BIT | MMC36X0_STATUS_MOTION_DETECT_BIT)
#define MMC36X0_STATUS_PUMP_ON_BIT			BIT(3)

#define MMC36X0_CTRL0_REFILL_BIT    BIT(5)
#define MMC36X0_CTRL0_RESET_BIT     BIT(4)
#define MMC36X0_CTRL0_SET_BIT		BIT(3)
#define MMC36X0_CTRL0_TMT_BIT		BIT(1)
#define MMC36X0_CTRL0_TMM_BIT		BIT(0)

/* output resolution bits BW0 and BW1*/
#define MMC36X0_CTRL1_BW0_BIT		BIT(0)
#define MMC36X0_CTRL1_BW1_BIT		BIT(1)
#define MMC36X0_CTRL1_SW_RESET		BIT(7)

#define MMC36X0_CTRL1_BW_MASK	 (MMC36X0_CTRL1_BW0_BIT | \
		 MMC36X0_CTRL1_BW1_BIT)
#define MMC36X0_CTRL1_BW_SHIFT		0

#define MMC36X0_CTRL2_INT_MDT_EN		BIT(5)
#define MMC36X0_CTRL2_INT_MEAS_DONE_EN	BIT(6)
#define MMC36X0_CTRL2_INT_ULP_SEL		BIT(7)

/*CMD*/
#define MMC36X0_PASSWORD_OPEN		0xE1
#define MMC3X0_OPT_MODE_OPR			0x11
#define MMC36X0_TESTMODE_OTP_MR		0x80

#define MMC36X0_WAIT_CHARGE_PUMP	50000	/* us */
#define MMC53240_WAIT_SET_RESET		2000	/* us */

/*
 * Memsic OTP process code piece is put here for reference:
 *
 * #define OTP_CONVERT(REG)  ((float)((REG) >=32 ? (32 - (REG)) : (REG)) * 0.006
 * 1) For X axis, the COEFFICIENT is always 1.
 * 2) For Y axis, the COEFFICIENT is as below:
 *    f_OTP_matrix[4] = OTP_CONVERT((reg_data[0] & 0x3f)) + 1.0;
 * 3) For Z axis, the COEFFICIENT is as below:
 *    f_OTP_matrix[8] = (OTP_CONVERT(((reg_data[1] & 0x0f) << 2 |
 *						(reg_data[0] & 0xc0) >> 6 + 1)) * 1.35;
 * We implemented the OTP logic into driver.
 */

/* scale = 1000 here for Y otp */
#define MMC36X0_OTP_CONVERT_Y(REG) (((REG) >= 32 ? (32 - (REG)) : (REG)) * 6)

/* 0.006 * 1.35 = 0.0081, scale 10000 for Z otp */
#define MMC36X0_OTP_CONVERT_Z(REG) (((REG) >= 32 ? (32 - (REG)) : (REG)) * 81)

#define MMC36X0_X_COEFF(x)	(x)
#define MMC36X0_Y_COEFF(y)	(y + 1000)
#define MMC36X0_Z_COEFF(z)	(z + 13500)

#define MMC36X0_OTP_START_ADDR		0x1B

enum mmc36x0_resolution {
	MMC36X0_16_BITS_SLOW = 0, /* 7.92 ms */
	MMC36X0_16_BITS_FAST,     /* 4.08 ms */
	MMC36X0_14_BITS,          /* 2.16 ms */
	MMC36X0_12_BITS,          /* 1.20 ms */
};

enum mmc36x0_axis {
	AXIS_X = 0,
	AXIS_Y,
	AXIS_Z,
};

static const struct {
	int sens[3]; /* sensitivity per X, Y, Z axis */
	int nfo; /* null field output */
} mmc36x0_props = {
	/* 16 bits*/
	{1024, 1024, 1024},
	32768,
};

struct mmc36x0_data {
	struct i2c_client *client;
	struct mutex mutex;
	struct regmap *regmap;
	enum mmc36x0_resolution res;

	/* OTP compensation */
	int axis_coef[3];
	int axis_scale[3];
	int irq;
	struct iio_trigger *dready_trig;
	bool dready_trigger_on;
	bool single_supply;//Dual Supply or Single Supply
	struct delayed_work set_work;
	u64 timestamp;
};

static const struct {
	int val;
	int val2;
} mmc36x0_samp_freq[] = {
	{100, 0},
	{200, 0},
	{400, 0},
	{600, 0}
};

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("100 200 400 600");

#define MMC36X0_CHANNEL(_axis, index) { \
	.type = IIO_MAGN, \
	.modified = 1, \
	.channel2 = IIO_MOD_ ## _axis, \
	.address = AXIS_ ## _axis, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = index, \
	.scan_type = { \
		.sign = 's', \
		.realbits = 16, \
		.storagebits = 16, \
		.shift = 0, \
		.endianness = IIO_LE, \
	}, \
}

static const struct iio_chan_spec mmc36x0_channels[] = {
	MMC36X0_CHANNEL(X, 0),
	MMC36X0_CHANNEL(Y, 1),
	MMC36X0_CHANNEL(Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static struct attribute *mmc36x0_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const unsigned long mmc36x0_scan_masks[] = {
	BIT(AXIS_X) | BIT(AXIS_Y) | BIT(AXIS_Z),
	0
};

static const struct attribute_group mmc36x0_attribute_group = {
	.attrs = mmc36x0_attributes,
};

static int mmc36x0_get_samp_freq_index(struct mmc36x0_data *data,
					int val, int val2)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mmc36x0_samp_freq); i++)
		if (mmc36x0_samp_freq[i].val == val &&
		    mmc36x0_samp_freq[i].val2 == val2)
			return i;
	return -EINVAL;
}

static __maybe_unused int mmc36x0_hw_set(struct mmc36x0_data *data, bool set)
{
	int ret;
	u8 coil_bit;

	/*
	 * When using single supply, recharge the capacitor at VCAP pin, requested to be issued
	 * before a SET/RESET command.
	 */
	if (data->single_supply) {
		pr_info("%s: single_supply\n", __func__);
		ret = regmap_write_bits(data->regmap, MMC36X0_REG_CTRL0,
					 MMC36X0_CTRL0_REFILL_BIT,
					 MMC36X0_CTRL0_REFILL_BIT);
		if (ret) {
			dev_err(&data->client->dev, "%s: write MMC36X0_REG_CTRL0 failed\n", __func__);
			return ret;
		}
		usleep_range(MMC36X0_WAIT_CHARGE_PUMP, MMC36X0_WAIT_CHARGE_PUMP + 1000);
	}

	if (set)
		coil_bit = MMC36X0_CTRL0_SET_BIT;
	else
		coil_bit = MMC36X0_CTRL0_RESET_BIT;

	ret = regmap_write_bits(data->regmap, MMC36X0_REG_CTRL0,
				  coil_bit, coil_bit);
	if (!ret)
		usleep_range(MMC53240_WAIT_SET_RESET, MMC53240_WAIT_SET_RESET + 200);
	else
		dev_err(&data->client->dev, "write MMC36X0_REG_CTRL0 set/reset failed, ret=%d\n", ret);
	return ret;
}

/* Get sensitivity compensation value */
static int mmc36x0_get_comp_matrix(struct mmc36x0_data *data)
{
	int ret = 0;
	int y_convert, z_convert;
	u8 otp_data[2] = {0};
	struct regmap *regmap = data->regmap;

	/* Write 0xE1 to register 0x0F, write password */
	ret = regmap_write(regmap, MMC36X0_REG_PASSWORD, MMC36X0_PASSWORD_OPEN);
	if (ret) {
		dev_err(&data->client->dev, "write MMC36X0_REG_PASSWORD failed\n");
		goto out;
	}

	ret = regmap_write(regmap, MMC36X0_REG_TESTMODE, MMC36X0_TESTMODE_OTP_MR);
	if (ret) {
		dev_err(&data->client->dev, "write MMC36X0_REG_TESTMODE failed\n");
		goto out;
	}

	ret = regmap_write(regmap, MMC36X0_REG_CTRL2, MMC36X0_CTRL2_INT_ULP_SEL);
	if (ret) {
		dev_err(&data->client->dev, "write MMC36X0_REG_CTRL2 failed\n");
		goto out;
	}

	ret = regmap_bulk_read(regmap, MMC36X0_REG_OTP, otp_data, sizeof(otp_data));
	if (ret) {
		dev_err(&data->client->dev, "read MMC36X0_REG_OTP failed\n");
		goto out;
	}
	mmc36x0_debug("otp_data=[%d %d]", otp_data[1], otp_data[0]);

	ret = regmap_write(regmap, MMC36X0_REG_CTRL2, 0);
	if (ret) {
		dev_err(&data->client->dev, "write MMC36X0_REG_CTRL2 failed\n");
		goto out;
	}

	y_convert = MMC36X0_OTP_CONVERT_Y(otp_data[0] & 0x3f);
	z_convert = MMC36X0_OTP_CONVERT_Z((otp_data[1] & 0x0f) << 2 | (otp_data[0] & 0xc0) >> 6);

	data->axis_coef[0] = MMC36X0_X_COEFF(1);
	data->axis_coef[1] = MMC36X0_Y_COEFF(y_convert);
	data->axis_coef[2] = MMC36X0_Z_COEFF(z_convert);

	data->axis_scale[0] = 1;
	data->axis_scale[1] = 1000;
	data->axis_scale[2] = 10000;
out:
	return ret;
}

static int mmc36x0_init(struct mmc36x0_data *data)
{
	int ret;
	unsigned int reg_id;

	/*reset chip to default state*/
	ret = regmap_write(data->regmap, MMC36X0_REG_CTRL1, MMC36X0_CTRL1_SW_RESET);
	if (!ret) {
		usleep_range(5000, 5000 + 200);
	} else {
		dev_err(&data->client->dev, "%s: reset failed, ret=%d\n", __func__, ret);
		return ret;
	}

	ret = regmap_read(data->regmap, MMC36X0_REG_ID, &reg_id);
	if (!ret) {
		dev_info(&data->client->dev, "MMC36X0 chip id is :0x%x\n", reg_id);
		regmap_write(data->regmap, MMC36X0_REG_CTRL1, 0);
	} else {
		dev_err(&data->client->dev, "Error reading product id\n");
		return ret;
	}

	/*
	 * make sure we restore sensor characteristics, by doing
	 * a SET/RESET sequence, the axis polarity being naturally
	 * aligned after RESET
	 */
	ret = mmc36x0_hw_set(data, true);
	if (ret)
		return ret;

	ret = mmc36x0_hw_set(data, false);
	if (ret)
		return ret;

	ret = mmc36x0_get_comp_matrix(data);
	return ret;
}

static int mmc36x0_take_one_measurement(struct mmc36x0_data *data)
{
	int ret, tries = 100;
	unsigned int reg_status;

	ret = regmap_write_bits(data->regmap, MMC36X0_REG_CTRL0, MMC36X0_CTRL0_TMM_BIT, MMC36X0_CTRL0_TMM_BIT);
	if (ret < 0)
		return ret;

	while (tries-- > 0) {
		ret = regmap_read(data->regmap, MMC36X0_REG_STATUS,
				  &reg_status);
		if (ret < 0)
			return ret;
		if (reg_status & MMC36X0_STATUS_MEAS_M_DONE_BIT)
			break;
		/* minimum wait time to complete measurement is 10 ms */
		usleep_range(10000, 11000);
	}

	if (tries < 0) {
		dev_err(&data->client->dev, "data not ready\n");
		return -EIO;
	}

	return 0;
}

static int mmc36x0_read_measure_result(struct mmc36x0_data *data, __le16 buf[3])
{
	return regmap_bulk_read(data->regmap, MMC36X0_REG_XOUT_L, (u8 *)buf,
				3 * sizeof(__le16));
}

/**
 * mmc36x0_raw_to_mgauss - convert raw readings to milli gauss. Also apply
			    compensation for output value.
 *
 * @data: device private data
 * @index: axis index for which we want the conversion
 * @buf: raw data to be converted, 2 bytes in little endian format
 * @val: compensated output reading (unit is milli gauss)
 *
 * Returns: 0 in case of success, -EINVAL when @index is not valid
 */
static int mmc36x0_raw_to_mgauss(struct mmc36x0_data *data, int index,
				  __le16 buf[], s16*val)
{
	int raw[3];

	raw[AXIS_X] = le16_to_cpu(buf[AXIS_X]);
	raw[AXIS_Y] = le16_to_cpu(buf[AXIS_Y]);
	raw[AXIS_Z] = le16_to_cpu(buf[AXIS_Z]);

	mmc36x0_debug("index=%d, rawX=%d, rawY=%d, rawZ=%d\n", index, raw[AXIS_X], raw[AXIS_Y], raw[AXIS_Z]);

	switch (index) {
	case AXIS_X:
		*val = (raw[AXIS_X] - mmc36x0_props.nfo) * 1000 / mmc36x0_props.sens[AXIS_X];
		break;
	case AXIS_Y:
		*val = (raw[AXIS_Y] - mmc36x0_props.nfo) * 1000 / mmc36x0_props.sens[AXIS_Y] -
			(raw[AXIS_Z] - mmc36x0_props.nfo)  * 1000 / mmc36x0_props.sens[AXIS_Z];
		break;
	case AXIS_Z:
		*val = (raw[AXIS_Y] - mmc36x0_props.nfo) * 1000 / mmc36x0_props.sens[AXIS_Y] +
			(raw[AXIS_Z] - mmc36x0_props.nfo) * 1000 / mmc36x0_props.sens[AXIS_Z];
		break;
	default:
		return -EINVAL;
	}

	/* apply OTP compensation */
	*val = (*val) * data->axis_coef[index] / data->axis_scale[index];
	return 0;
}

static int mmc36x0_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int *val,
			     int *val2, long mask)
{
	struct mmc36x0_data *data = iio_priv(indio_dev);
	int ret, i;
	unsigned int reg;
	__le16 buf[3];

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (iio_buffer_enabled(indio_dev))
			return -EBUSY;
		mutex_lock(&data->mutex);
		ret = mmc36x0_take_one_measurement(data);
		if (ret) {
			dev_err(&data->client->dev, "take on measurement failed\n");
		}
		ret = mmc36x0_read_measure_result(data, buf);
		if (ret) {
			dev_err(&data->client->dev, "read measure result failed\n");
		}
		mutex_unlock(&data->mutex);
		if (ret < 0)
			return ret;
		ret = mmc36x0_raw_to_mgauss(data, chan->address, buf, (s16*)val);
		if (ret < 0)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = 1000;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = regmap_read(data->regmap, MMC36X0_REG_CTRL1, &reg);
		if (ret < 0)
			return ret;

		i = (reg & MMC36X0_CTRL1_BW_MASK) >> MMC36X0_CTRL1_BW_SHIFT;
		if (i < 0 || i >= ARRAY_SIZE(mmc36x0_samp_freq))
			return -EINVAL;

		*val = mmc36x0_samp_freq[i].val;
		*val2 = mmc36x0_samp_freq[i].val2;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int mmc36x0_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int val,
			      int val2, long mask)
{
	struct mmc36x0_data *data = iio_priv(indio_dev);
	int i, ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		i = mmc36x0_get_samp_freq_index(data, val, val2);
		if (i < 0)
			return -EINVAL;

		mutex_lock(&data->mutex);
		ret = regmap_write_bits(data->regmap, MMC36X0_REG_CTRL1,
					 MMC36X0_CTRL1_BW_MASK,
					 i << MMC36X0_CTRL1_BW_SHIFT);
		if (!ret)
			data->res = i;
		mutex_unlock(&data->mutex);
		return ret;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct iio_info mmc36x0_info = {
	.driver_module	= THIS_MODULE,
	.read_raw	= mmc36x0_read_raw,
	.write_raw	= mmc36x0_write_raw,
	.attrs		= &mmc36x0_attribute_group,
};

static bool mmc36x0_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MMC36X0_REG_CTRL0:
	case MMC36X0_REG_CTRL1:
	case MMC36X0_REG_CTRL2:
	case MMC36X0_REG_PASSWORD:
	case MMC36X0_REG_OTPMODE:
	case MMC36X0_REG_TESTMODE:
	case MMC36X0_REG_STATUS:
	case MMC36X0_REG_XTHRESH:
	case MMC36X0_REG_YTHRESH:
	case MMC36X0_REG_ZTHRESH:
		return true;
	default:
		return false;
	}
}

static bool mmc36x0_is_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MMC36X0_REG_XOUT_L:
	case MMC36X0_REG_XOUT_H:
	case MMC36X0_REG_YOUT_L:
	case MMC36X0_REG_YOUT_H:
	case MMC36X0_REG_ZOUT_L:
	case MMC36X0_REG_ZOUT_H:
	case MMC36X0_REG_STATUS:
	case MMC36X0_REG_OTP:
	case MMC36X0_REG_ID:
		return true;
	default:
		return false;
	}
}

static bool mmc36x0_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MMC36X0_REG_CTRL0:
	case MMC36X0_REG_CTRL1:
	case MMC36X0_REG_CTRL2:
	case MMC36X0_REG_XTHRESH:
	case MMC36X0_REG_YTHRESH:
	case MMC36X0_REG_ZTHRESH:
		return false;
	default:
		return true;
	}
}

static struct reg_default mmc36x0_reg_defaults[] = {
	{ MMC36X0_REG_CTRL0, 0x00 },
	{ MMC36X0_REG_CTRL1, 0x00 },
	{ MMC36X0_REG_CTRL2, 0x00 },
	{ MMC36X0_REG_XTHRESH, 0x00 },
	{ MMC36X0_REG_YTHRESH, 0x00 },
	{ MMC36X0_REG_ZTHRESH, 0x00 },
};

static const struct regmap_config mmc36x0_regmap_config = {
	.name = MMC36X0_REGMAP_NAME,

	.reg_bits = 8,
	.val_bits = 8,

	.max_register = MMC36X0_REG_ID,
	.cache_type = REGCACHE_FLAT,

	.writeable_reg = mmc36x0_is_writeable_reg,
	.readable_reg = mmc36x0_is_readable_reg,
	.volatile_reg = mmc36x0_is_volatile_reg,

	.reg_defaults = mmc36x0_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(mmc36x0_reg_defaults),
};

static int mmc36x0_buffer_preenable(struct iio_dev *indio_dev)
{
	int ret;
	struct mmc36x0_data *data = iio_priv(indio_dev);

	ret = mmc36x0_hw_set(data, true);
	if (ret) {
		dev_err(&data->client->dev, "%s: hw set failed\n", __func__);
	}
	return ret;
}

static const struct iio_buffer_setup_ops mmc36x0_buffer_setup_ops = {
	.preenable = mmc36x0_buffer_preenable,
	.postenable = iio_triggered_buffer_postenable,
	.predisable = iio_triggered_buffer_predisable,
};

static int mmc36x0_set_trigge_state(struct iio_trigger *trig,
						  bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct mmc36x0_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->mutex);
	if (state == data->dready_trigger_on)
		goto err_unlock;

	/*clear interrupt*/
	ret = regmap_write_bits(data->regmap, MMC36X0_REG_STATUS,
					MMC36X0_STATUS_INTERRUPT_MASK, 0x07);
	if (ret) {
		dev_err(&data->client->dev, "write MMC36X0_REG_STATUS failed\n");
		goto err_unlock;
	}

	/*enable/disabled the interrupt when a magnetic or temperature measurement even is completed.*/
	ret = regmap_write_bits(data->regmap, MMC36X0_REG_CTRL2,
			MMC36X0_CTRL2_INT_MEAS_DONE_EN, state ? MMC36X0_CTRL2_INT_MEAS_DONE_EN : 0);
	if (ret) {
		dev_err(&data->client->dev, "write MMC36X0_REG_CTRL2 failed\n");
		goto err_unlock;
	}

	if (state) {
		/*start magnetic field measurement*/
		ret = regmap_write_bits(data->regmap, MMC36X0_REG_CTRL0,
				MMC36X0_CTRL0_TMM_BIT, MMC36X0_CTRL0_TMM_BIT);
		if (ret)
			goto err_unlock;
		schedule_delayed_work(&data->set_work, 5000);
	} else {
		cancel_delayed_work_sync(&data->set_work);
	}

	data->dready_trigger_on = state;

err_unlock:
	mutex_unlock(&data->mutex);
	return ret;
}

static const struct iio_trigger_ops mmc36x0_trigger_ops = {
	.set_trigger_state = mmc36x0_set_trigge_state,
	.owner = THIS_MODULE,
};

irqreturn_t mmc36x0_irq_handler(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct mmc36x0_data *data = iio_priv(indio_dev);
	data->timestamp = iio_get_time_ns();
	return IRQ_WAKE_THREAD;
}

irqreturn_t mmc36x0_irq_thread_handler(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct mmc36x0_data *data = iio_priv(indio_dev);
	__le16 raw[3];
	s16 mgauss[8];
	int status;
	int i;
	int ret;

	ret = regmap_read(data->regmap, MMC36X0_REG_STATUS, &status);
	if (ret) {
		dev_err(&indio_dev->dev, "%s: read status failed, ret=%d\n",__func__, ret);
		goto err;
	}

	/*whether a measurement event of magnetic field is completed*/
	if (status & MMC36X0_STATUS_INTERRUPT_MASK) {
		if (status & MMC36X0_STATUS_MEAS_M_DONE_BIT) {
			ret = mmc36x0_read_measure_result(data, raw);
			if (ret) {
				dev_err(&indio_dev->dev, "read xyz data failed, ret=%d\n", ret);
				goto err;
			}
		}

		/*clear interrupt*/
		ret = regmap_write_bits(data->regmap, MMC36X0_REG_STATUS,
					MMC36X0_STATUS_INTERRUPT_MASK, 0x07);
		if (ret) {
			dev_err(&indio_dev->dev, "%s: clear interrupt failed, ret=%d\n",__func__, ret);
			goto err;
		}

		/*start magnetic field measurement again*/
		ret = regmap_write_bits(data->regmap, MMC36X0_REG_CTRL0, MMC36X0_CTRL0_TMM_BIT, MMC36X0_CTRL0_TMM_BIT);
		if (ret) {
				dev_err(&indio_dev->dev, "%s: write MMC36X0_REG_CTRL0 failed, ret=%d\n", __func__, ret);
				goto err;
		}

		if (status & MMC36X0_STATUS_MEAS_M_DONE_BIT) {
			for (i = 0; i < 3; i++) {
				mmc36x0_raw_to_mgauss(data, i, raw, &mgauss[i]);
				mmc36x0_debug("mgauss[%d]=%d\n", i, mgauss[i]);
			}
			iio_push_to_buffers_with_timestamp(indio_dev, mgauss, data->timestamp);
		}
	}
	return IRQ_HANDLED;
err:
	cancel_delayed_work(&data->set_work);
	schedule_delayed_work(&data->set_work, 0);
	return IRQ_HANDLED;
}

irqreturn_t mmc36x0_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}


static int mmc36x0_probe_trigger(struct mmc36x0_data *data)
{
	int ret = 0;
	struct i2c_client *client = data->client;
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	unsigned long irq_type = irqd_get_trigger_type(irq_get_irq_data(data->irq));

	data->dready_trig = devm_iio_trigger_alloc(&client->dev,
						   "%s-dev%d",
						   indio_dev->name,
						   indio_dev->id);
	if (!data->dready_trig) {
		dev_err(&client->dev, "iio trigger alloc failed\n");
		return -ENOMEM;
	}

	data->dready_trig->dev.parent = &client->dev;
	iio_trigger_set_drvdata(data->dready_trig, indio_dev);
	data->dready_trig->ops = &mmc36x0_trigger_ops;
	ret = iio_trigger_register(data->dready_trig);
	if (ret) {
		dev_err(&client->dev, "iio trigger register failed\n");
		return ret;
	}

	ret = devm_request_threaded_irq(&client->dev, client->irq,
				   mmc36x0_irq_handler,
				   mmc36x0_irq_thread_handler,
				   irq_type | IRQF_ONESHOT,
				   MMC36X0_DRV_NAME,
				   indio_dev);
	if (ret) {
		dev_err(&client->dev, "request irq %d failed, ret=%d\n", client->irq, ret);
		goto out;
	}

	ret = iio_triggered_buffer_setup(indio_dev,
					 NULL,
					 mmc36x0_trigger_handler,
					 &mmc36x0_buffer_setup_ops);
	if (ret) {
		dev_err(&client->dev, "iio triggered buffer setup failed\n");
		goto out;
	}
	return 0;
out:
	iio_trigger_unregister(data->dready_trig);
	return ret;

}

static void set_work_function(struct work_struct *work)
{
	int ret;
	struct delayed_work *delayed_work = container_of(work, struct delayed_work, work);
	struct mmc36x0_data *data = container_of(delayed_work, struct mmc36x0_data, set_work);

	disable_irq(data->irq);
	ret = mmc36x0_hw_set(data, true);
	if (ret) {
		dev_err(&data->client->dev, "%s: hw set failed\n", __func__);
	}

	/*clear interrupt*/
	ret = regmap_write_bits(data->regmap, MMC36X0_REG_STATUS,
					MMC36X0_STATUS_INTERRUPT_MASK, 0x07);
	if (ret) {
		dev_err(&data->client->dev, "write MMC36X0_REG_STATUS failed\n");
	}

	enable_irq(data->irq);

	/*start magnetic field measurement again*/
	ret = regmap_write_bits(data->regmap, MMC36X0_REG_CTRL0, MMC36X0_CTRL0_TMM_BIT, MMC36X0_CTRL0_TMM_BIT);
	if (ret) {
		dev_err(&data->client->dev, "%s: write MMC36X0_REG_CTRL0 failed, ret=%d\n", __func__, ret);
	}

	schedule_delayed_work(&data->set_work, 5000);
}

static int mmc36x0_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct mmc36x0_data *data;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_i2c(client, &mmc36x0_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "regmap initialization failed\n");
		return PTR_ERR(regmap);
	}

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	data->regmap = regmap;
	data->res = MMC36X0_16_BITS_FAST;
	data->irq = client->irq;
	mutex_init(&data->mutex);
	INIT_DELAYED_WORK(&data->set_work, set_work_function);
	if (of_property_read_bool(client->dev.of_node, "single-supply"))
		data->single_supply = true;

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &mmc36x0_info;
	indio_dev->name = MMC36X0_DRV_NAME;
	indio_dev->channels = mmc36x0_channels;
	indio_dev->num_channels = ARRAY_SIZE(mmc36x0_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = mmc36x0_scan_masks;
	if (client->irq > 0) {
		ret = mmc36x0_probe_trigger(data);
		if (ret)
			goto out;
		indio_dev->trig = iio_trigger_get(data->dready_trig);
	}

	ret = mmc36x0_init(data);
	if (ret < 0) {
		dev_err(&client->dev, "mmc36x0 chip init failed\n");
		goto out;
	}
	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret)
		dev_err(&client->dev, "register iio device failed, ret=%d\n", ret);
	return ret;
out:
	mutex_destroy(&data->mutex);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int mmc36x0_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct mmc36x0_data *data = iio_priv(indio_dev);

	regcache_cache_only(data->regmap, true);

	return 0;
}

static int mmc36x0_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct mmc36x0_data *data = iio_priv(indio_dev);
	int ret;

	regcache_mark_dirty(data->regmap);
	ret = regcache_sync_region(data->regmap, MMC36X0_REG_CTRL0,
				   MMC36X0_REG_CTRL2);
	if (ret < 0)
		dev_err(dev, "Failed to restore control registers\n");

	regcache_cache_only(data->regmap, false);

	return 0;
}
#endif

static const struct dev_pm_ops mmc36x0_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mmc36x0_suspend, mmc36x0_resume)
};

static const struct of_device_id mmc36x0_of_match[] = {
	{ .compatible = "memsic,mmc36x0", },
	{ }
};
MODULE_DEVICE_TABLE(of, mmc36x0_of_match);

static const struct acpi_device_id mmc36x0_acpi_match[] = {
	{"MMC36X0", 0},
	{ },
};
MODULE_DEVICE_TABLE(acpi, mmc36x0_acpi_match);

static const struct i2c_device_id mmc36x0_id[] = {
	{"mmc36x0", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, mmc36x0_id);

static struct i2c_driver mmc36x0_driver = {
	.driver = {
		.name = MMC36X0_DRV_NAME,
		.of_match_table = mmc36x0_of_match,
		.pm = &mmc36x0_pm_ops,
		.acpi_match_table = ACPI_PTR(mmc36x0_acpi_match),
	},
	.probe		= mmc36x0_probe,
	.id_table	= mmc36x0_id,
};

module_i2c_driver(mmc36x0_driver);

MODULE_AUTHOR("yinghua.ma <yinghua.ma@ninebot.com>");
MODULE_DESCRIPTION("MEMSIC MMC36X0 magnetic sensor driver");
MODULE_LICENSE("GPL v2");

