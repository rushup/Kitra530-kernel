#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/iio/events.h>
#include <linux/kfifo.h>
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_api.h"
#include "vl53l0x_def.h"

typedef enum {
	LONG_RANGE 		= 0, /*!< Long range mode */
	HIGH_SPEED 		= 1, /*!< High speed mode */
	HIGH_ACCURACY	= 2, /*!< High accuracy mode */
} RangingConfig_e;

struct vl53l0x_sensor {
	char name[10];
	RangingConfig_e range_type;
	s64 ts;
	VL53L0X_DEV vl53l0x_data;
};



#define I2C_M_WR			0x00
#define STATUS_OK			0x00
#define STATUS_FAIL			(-1)

int VL53L0X_I2CWrite(VL53L0X_DEV dev, uint8_t *buff, uint8_t len)
{

	struct i2c_msg msg[1];
	struct i2c_client *client = (struct i2c_client*) dev->i2c_data;
	int err = 0;


	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_WR;
	msg[0].buf = buff;
	msg[0].len = len;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err != 1) {
		return STATUS_FAIL;
	}

	return 0;
}

int VL53L0X_I2CRead(VL53L0X_DEV dev, uint8_t *buff, uint8_t len)
{

	struct i2c_msg msg[1];
	struct i2c_client *client = (struct i2c_client*) dev->i2c_data;
	int err = 0;

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_RD|client->flags;
	msg[0].buf = buff;
	msg[0].len = len;

	err = i2c_transfer(client->adapter, &msg[0], 1);

	if (err != 1) {
		return STATUS_FAIL;
	}

	return 0;
}

static int setup_single_shot(VL53L0X_DEV dev, RangingConfig_e rangingConfig)
{
	int status;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;


	VL53L0X_DataInit(dev);

	status=VL53L0X_StaticInit(dev);

	if( status ){
		return -1;
	}

	status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);

	if( status ){
		return -1;
	}

	status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);

	if( status ){
		return -1;
	}

	status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
	if( status ){
		return -1;
	}

	status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
	if( status ){
		return -1;
	}

	status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
	if( status ){
		return -1;
	}
	/* Ranging configuration */
	switch(rangingConfig) {
	case LONG_RANGE:
		signalLimit = (FixPoint1616_t)(0.1*65536);
		sigmaLimit = (FixPoint1616_t)(60*65536);
		timingBudget = 33000;
		preRangeVcselPeriod = 18;
		finalRangeVcselPeriod = 14;
		break;
	case HIGH_ACCURACY:
		signalLimit = (FixPoint1616_t)(0.25*65536);
		sigmaLimit = (FixPoint1616_t)(18*65536);
		timingBudget = 200000;
		preRangeVcselPeriod = 14;
		finalRangeVcselPeriod = 10;
		break;
	case HIGH_SPEED:
		signalLimit = (FixPoint1616_t)(0.25*65536);
		sigmaLimit = (FixPoint1616_t)(32*65536);
		timingBudget = 20000;
		preRangeVcselPeriod = 14;
		finalRangeVcselPeriod = 10;
		break;
	default:
		return -1;
	}

	status = VL53L0X_SetLimitCheckValue(dev,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
	if( status ){
		return -1;
	}

	status = VL53L0X_SetLimitCheckValue(dev,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
	if( status ){
		return -1;
	}

	status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,  timingBudget);
	if( status ){
		return -1;
	}

	status = VL53L0X_SetVcselPulsePeriod(dev,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
	if( status ){
		return -1;
	}

	status = VL53L0X_SetVcselPulsePeriod(dev,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
	if( status ){
		return -1;
	}

	status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
	if( status ){
		return -1;
	}

	return 0;
}

static ssize_t vl53l0x_range_avaiable(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int len;

	len = scnprintf(buf, PAGE_SIZE, "long speed accuracy\n");

	return len;
}


static ssize_t vl53l0x_get_range(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int len;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct vl53l0x_sensor *sensor = iio_priv(indio_dev);

	if(sensor->range_type == HIGH_ACCURACY)
		len = scnprintf(buf, PAGE_SIZE, "accuracy\n");
	else if(sensor->range_type == HIGH_SPEED)
		len = scnprintf(buf, PAGE_SIZE, "speed\n");
	else if(sensor->range_type == LONG_RANGE)
		len = scnprintf(buf, PAGE_SIZE, "long\n");
	else
		len = -EINVAL;

	return len;
}


static ssize_t vl53l0x_set_range(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct vl53l0x_sensor *sensor = iio_priv(indio_dev);
	int err = 0, ret;

	ret = len;

	mutex_lock(&indio_dev->mlock);


	if(strncmp("long", buf, len>4?4:len) == 0) {
		err = setup_single_shot(sensor->vl53l0x_data, LONG_RANGE);
	}
	else if(strncmp("speed", buf, len>5?5:len) == 0) {
		err = setup_single_shot(sensor->vl53l0x_data, HIGH_SPEED);
	}
	else if(strncmp("accuracy", buf, len>8?8:len) == 0) {
		err = setup_single_shot(sensor->vl53l0x_data, HIGH_ACCURACY);
	}
	else
		ret = -EINVAL;

	if(err)
		ret = -EIO;

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int vl53l0x_read_raw(struct iio_dev *iio_dev,
			       struct iio_chan_spec const *ch,
			       int *val, int *val2, long mask)
{

	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
	struct vl53l0x_sensor *sensor = iio_priv(iio_dev);
	int ret = -1;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:

		mutex_lock(&iio_dev->mlock);

		if(sensor->vl53l0x_data->inited == 1)
			ret = VL53L0X_PerformSingleRangingMeasurement(sensor->vl53l0x_data,
					&RangingMeasurementData);
		if( ret ) {
		    ret = -EIO;
		}
        	else {
        		*val = RangingMeasurementData.RangeMilliMeter;
			ret = IIO_VAL_INT;		
		}

		mutex_unlock(&iio_dev->mlock);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int vl53l0x_write_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	int err = 0;

	mutex_lock(&iio_dev->mlock);


	mutex_unlock(&iio_dev->mlock);

	return err;
}

#define VL53L0X_CHANNEL(idx)					\
	{							\
		.type = IIO_PROXIMITY,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),	\
		.channel = idx,					\
		.scan_index = idx,				\
		.scan_type = {					\
			.sign = 'u',				\
			.realbits = 16,				\
			.storagebits = 16,			\
			.shift = 0,				\
		},						\
	}

static IIO_DEVICE_ATTR(in_proximity_range_avaiable, S_IRUGO,
		       vl53l0x_range_avaiable, NULL, 0);

static IIO_DEVICE_ATTR(in_proximity_set_range, S_IWUSR | S_IRUGO,
			vl53l0x_get_range, vl53l0x_set_range, 0);

static struct attribute *vl53l0x_attributes[] = {
	&iio_dev_attr_in_proximity_range_avaiable.dev_attr.attr,
	&iio_dev_attr_in_proximity_set_range.dev_attr.attr,
	NULL,
};

static const struct attribute_group vl53l0x_attribute_group = {
	.attrs = vl53l0x_attributes,
};

static const struct iio_chan_spec vl53l0x_channels[] = {
	VL53L0X_CHANNEL(0),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_info vl53l0x_info = {
	.driver_module = THIS_MODULE,
	.attrs = &vl53l0x_attribute_group,
	.read_raw = vl53l0x_read_raw,
	.write_raw = vl53l0x_write_raw,
};

static void vl53l0x_work_handler(struct work_struct *work)
{
	VL53L0X_DEV data = container_of(work, VL53L0X_Dev_t,
				dwork.work);

	mutex_lock(data->iio_mutex);

	if( setup_single_shot(data, HIGH_ACCURACY) == 0)
		data->inited = 1;

	mutex_unlock(data->iio_mutex);

}

static int vl53l0x_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{

	VL53L0X_DEV vl53l0x_data;
	struct vl53l0x_sensor *sensor;
	struct iio_dev *iio_dev;
	int err;

	vl53l0x_data = kzalloc(sizeof(VL53L0X_Dev_t), GFP_KERNEL);
	if (!vl53l0x_data) {
		return -ENOMEM;
	}

	vl53l0x_data->i2c_data = client;

	iio_dev = devm_iio_device_alloc(&client->dev, sizeof(struct vl53l0x_sensor));
	if (!iio_dev)
		return -ENOMEM;

	sensor = iio_priv(iio_dev);
	scnprintf(sensor->name,8,"vl53l0x");
	sensor->range_type = HIGH_ACCURACY;
	sensor->vl53l0x_data = vl53l0x_data;
	iio_dev->channels = vl53l0x_channels;
	iio_dev->num_channels = ARRAY_SIZE(vl53l0x_channels);
	iio_dev->name = sensor->name;
	iio_dev->info = &vl53l0x_info;
	vl53l0x_data->iio_mutex = &iio_dev->mlock;
	vl53l0x_data->inited = 0;

	err =  devm_iio_device_register(&client->dev, iio_dev);

	INIT_DELAYED_WORK(&vl53l0x_data->dwork, vl53l0x_work_handler);

	schedule_delayed_work(&vl53l0x_data->dwork, 5 * HZ);

	if (err)
		return err;

	return 0;
}

static const struct of_device_id vl53l0x_of_match[] = {
		{
			.compatible = "st,vl53l0x",
			.data = 0,
		},
		{
			.compatible = "st,stmvl53l0",
			.data = 0,
		}
};
MODULE_DEVICE_TABLE(of, vl53l0x_of_match);

static const struct i2c_device_id vl53l0x_id_table[] = {
	{"vl53l0x", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, vl53l0x_id_table);

static struct i2c_driver vl53l0x_driver = {
	.driver = {
		.name = "vl53l0x",
		.of_match_table = of_match_ptr(vl53l0x_of_match),
	},
	.probe = vl53l0x_probe,
	.id_table = vl53l0x_id_table,
};
module_i2c_driver(vl53l0x_driver);


MODULE_AUTHOR("Matteo Fumagalli <m.fumagalli@rushup.tech>");
MODULE_DESCRIPTION("ST vl53l0x iio driver");
MODULE_LICENSE("GPL v2");
