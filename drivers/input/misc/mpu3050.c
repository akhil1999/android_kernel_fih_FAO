/*
 * MPU3050 Tri-axis gyroscope driver
 *
 * Copyright (C) 2011 Wistron Co.Ltd
 * Joseph Lai <joseph_lai@wistron.com>
 *
 * Trimmed down by Alan Cox <alan@linux.intel.com> to produce this version
 *
 * This is a 'lite' version of the driver, while we consider the right way
 * to present the other features to user space. In particular it requires the
 * device has an IRQ, and it only provides an input interface, so is not much
 * use for device orientation. A fuller version is available from the Meego
 * tree.
 *
 * This program is based on bma023.c.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/input/mpu3050.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#define	MPU3050_DEV_NAME_GYRO           "gyroscope"

#define MPU3050_AUTO_DELAY              1000

#define MPU3050_MIN_VALUE               -32768
#define MPU3050_MAX_VALUE               32767

#define MPU3050_MIN_POLL_INTERVAL       1
#define MPU3050_MAX_POLL_INTERVAL       250
#define MPU3050_DEFAULT_POLL_INTERVAL   200
#define MPU3050_DEFAULT_FS_RANGE        3

/* Register map */
#define MPU3050_CHIP_ID_REG             0x00
#define MPU3050_SMPLRT_DIV              0x15
#define MPU3050_DLPF_FS_SYNC            0x16
#define MPU3050_INT_CFG                 0x17
#define MPU3050_XOUT_H                  0x1D
#define MPU3050_PWR_MGM                 0x3E
#define MPU3050_PWR_MGM_POS             6
/* Register bits */

/* DLPF_FS_SYNC */
#define MPU3050_EXT_SYNC_NONE           0x00
#define MPU3050_EXT_SYNC_TEMP           0x20
#define MPU3050_EXT_SYNC_GYROX          0x40
#define MPU3050_EXT_SYNC_GYROY          0x60
#define MPU3050_EXT_SYNC_GYROZ          0x80
#define MPU3050_EXT_SYNC_ACCELX         0xA0
#define MPU3050_EXT_SYNC_ACCELY         0xC0
#define MPU3050_EXT_SYNC_ACCELZ         0xE0
#define MPU3050_EXT_SYNC_MASK           0xE0
#define MPU3050_FS_250DPS               0x00
#define MPU3050_FS_500DPS               0x08
#define MPU3050_FS_1000DPS              0x10
#define MPU3050_FS_2000DPS              0x18
#define MPU3050_FS_MASK                 0x18
#define MPU3050_DLPF_CFG_256HZ_NOLPF2   0x00
#define MPU3050_DLPF_CFG_188HZ          0x01
#define MPU3050_DLPF_CFG_98HZ           0x02
#define MPU3050_DLPF_CFG_42HZ           0x03
#define MPU3050_DLPF_CFG_20HZ           0x04
#define MPU3050_DLPF_CFG_10HZ           0x05
#define MPU3050_DLPF_CFG_5HZ            0x06
#define MPU3050_DLPF_CFG_2100HZ_NOLPF   0x07
#define MPU3050_DLPF_CFG_MASK           0x07
/* INT_CFG */
#define MPU3050_RAW_RDY_EN              0x01
#define MPU3050_MPU_RDY_EN              0x04
#define MPU3050_LATCH_INT_EN            0x20
#define MPU3050_OPEN_DRAIN              0x40
#define MPU3050_ACTIVE_LOW              0x80
/* PWR_MGM */
#define MPU3050_PWR_MGM_PLL_X           0x01
#define MPU3050_PWR_MGM_PLL_Y           0x02
#define MPU3050_PWR_MGM_PLL_Z           0x03
#define MPU3050_PWR_MGM_CLKSEL          0x07
#define MPU3050_PWR_MGM_STBY_ZG         0x08
#define MPU3050_PWR_MGM_STBY_YG         0x10
#define MPU3050_PWR_MGM_STBY_XG         0x20
#define MPU3050_PWR_MGM_SLEEP           0x40
#define MPU3050_PWR_MGM_RESET           0x80
#define MPU3050_PWR_MGM_MASK            0x40

#define MPU3050_PINCTRL_DEFAULT         "mpu_default"
#define MPU3050_PINCTRL_SUSPEND         "mpu_sleep"
#define CONFIG_OF                       1

struct axis_data {
    s16 x;
    s16 y;
    s16 z;
};

struct mpu3050_sensor {
    struct i2c_client *client;
    struct device *dev;
    struct input_dev *idev;
    struct mpu3050_gyro_platform_data *platform_data;
    struct workqueue_struct  *data_wq_h;
    struct workqueue_struct  *data_wq;
    struct work_struct input_work;
    struct hrtimer timer_gyro;
    struct sensors_classdev cdev;
    u32    use_poll;
    u32    poll_interval;
    u32    dlpf_index;
//  u32    enable_gpio;
    u32    enable;
    /* pinctrl */
    struct pinctrl       *pinctrl;
    struct pinctrl_state *pin_default;
    struct pinctrl_state *pin_sleep;
    struct mutex op_lock;
    bool  high_q;
};

static struct sensors_classdev sensors_cdev = {
    .name = "itg3050-gyro",
    .vendor = "Invensense",
    .version = 1,
    .handle = SENSORS_GYROSCOPE_HANDLE,
    .type = SENSOR_TYPE_GYROSCOPE,
    .max_range = "35.0",
    .resolution = "0.06",
    .sensor_power = "0.2",
    .min_delay = 10000,
    .max_delay = 200000,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .enabled = 0,
    .delay_msec = MPU3050_DEFAULT_POLL_INTERVAL,
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
    .sensors_set_latency = NULL,
    .sensors_flush = NULL,
};

struct sensor_regulator {
    struct regulator *vreg;
    const char *name;
    u32  min_uV;
    u32  max_uV;
};

struct sensor_regulator mpu_vreg[] = {
    {NULL, "vdd", 2100000, 3600000},
    {NULL, "vlogic", 1800000, 1800000},
};

static const int mpu3050_chip_ids[] = {
    0x68,
    0x69,
};

struct dlpf_cfg_tb {
    u8  cfg;          /* cfg index */
    u32 lpf_bw;       /* low pass filter bandwidth in Hz */
    u32 sample_rate;  /* analog sample rate in Khz, 1 or 8 */
};

static struct dlpf_cfg_tb dlpf_table[] = {
    {6,   5, 1},
    {5,  10, 1},
    {4,  20, 1},
    {3,  42, 1},
    {2,  98, 1},
    {1, 188, 1},
    {0, 256, 8},
};

static void mpu3050_pinctrl_state(struct mpu3050_sensor *sensor, bool active);
static void mpu3050_read_xyz(struct i2c_client *client, struct mpu3050_gyro_platform_data *pdata, struct axis_data *coords);

static u8 interval_to_dlpf_cfg(u32 interval)
{
    u32 sample_rate = 1000 / interval;
    u32 i;

    /* the filter bandwidth needs to be greater or
     * equal to half of the sample rate
     */
    for (i = 0; i < sizeof(dlpf_table)/sizeof(dlpf_table[0]); i++) {
        if (dlpf_table[i].lpf_bw * 2 >= sample_rate)
            return i;
    }
    /* return the maximum possible */
    return --i;
}

static int mpu3050_config_regulator(struct i2c_client *client, bool on)
{
    int rc = 0, i;
    int num_reg = sizeof(mpu_vreg) / sizeof(struct sensor_regulator);

    if (on) {
        for (i = 0; i < num_reg; i++) {
            mpu_vreg[i].vreg = regulator_get(&client->dev,mpu_vreg[i].name);
            if (IS_ERR(mpu_vreg[i].vreg)) {
                rc = PTR_ERR(mpu_vreg[i].vreg);
                pr_err("%s:regulator get failed rc=%d\n",__func__, rc);
                mpu_vreg[i].vreg = NULL;
                goto error_vdd;
            }
            if (regulator_count_voltages(mpu_vreg[i].vreg) > 0) {
                rc = regulator_set_voltage(mpu_vreg[i].vreg,mpu_vreg[i].min_uV, mpu_vreg[i].max_uV);
                if (rc) {
                    pr_err("%s:set_voltage failed rc=%d\n",__func__, rc);
                    regulator_put(mpu_vreg[i].vreg);
                    mpu_vreg[i].vreg = NULL;
                    goto error_vdd;
                }
            }
            rc = regulator_enable(mpu_vreg[i].vreg);
            if (rc) {
                pr_err("%s: regulator_enable failed rc =%d\n",__func__,rc);
                if (regulator_count_voltages(
                    mpu_vreg[i].vreg) > 0) {
                    regulator_set_voltage(mpu_vreg[i].vreg,0, mpu_vreg[i].max_uV);
                }
                regulator_put(mpu_vreg[i].vreg);
                mpu_vreg[i].vreg = NULL;
                goto error_vdd;
            }
        }
        return rc;
    } else {
        i = num_reg;
    }
error_vdd:
    while (--i >= 0) {
        if (!IS_ERR_OR_NULL(mpu_vreg[i].vreg)) {
            if (regulator_count_voltages(mpu_vreg[i].vreg) > 0) {
                regulator_set_voltage(mpu_vreg[i].vreg, 0,mpu_vreg[i].max_uV);
            }
            regulator_disable(mpu_vreg[i].vreg);
            regulator_put(mpu_vreg[i].vreg);
            mpu_vreg[i].vreg = NULL;
        }
    }
    return rc;
}

static int mpu3050_poll_delay_set(struct sensors_classdev *sensors_cdev, unsigned int delay_msec)
{
    struct mpu3050_sensor *sensor = container_of(sensors_cdev,struct mpu3050_sensor, cdev);
    unsigned int  dlpf_index;
    u8  divider, reg;
    int ret;

    dlpf_index = interval_to_dlpf_cfg(delay_msec);
    divider = delay_msec * dlpf_table[dlpf_index].sample_rate - 1;

    if (sensor->dlpf_index != dlpf_index) {
         /* Set low pass filter and full scale */
         reg = dlpf_table[dlpf_index].cfg;
         reg |= MPU3050_DEFAULT_FS_RANGE << 3;
         reg |= MPU3050_EXT_SYNC_NONE << 5;
         ret = i2c_smbus_write_byte_data(sensor->client,MPU3050_DLPF_FS_SYNC, reg);
         if (!ret)
             sensor->dlpf_index = dlpf_index;
    }

    if (sensor->poll_interval != delay_msec) {
        /* Output frequency divider. The poll interval */
        ret = i2c_smbus_write_byte_data(sensor->client,MPU3050_SMPLRT_DIV, divider);
        if (!ret)
            sensor->poll_interval = delay_msec;
    }

    return 0;
}

/**
 *  mpu3050_attr_get_polling_rate - get the sampling rate
 */
static ssize_t mpu3050_attr_get_polling_rate(struct device *dev,struct device_attribute *attr,char *buf)
{
    int val;
    struct mpu3050_sensor *sensor = dev_get_drvdata(dev);
    val = sensor ? sensor->poll_interval : 0;
    return snprintf(buf, 8, "%d\n", val);
}
/**
 *  mpu3050_attr_set_polling_rate - set the sampling rate
 */
static ssize_t mpu3050_attr_set_polling_rate(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
    struct mpu3050_sensor *sensor = dev_get_drvdata(dev);
    unsigned long interval_ms;
    int ret;

    if (kstrtoul(buf, 10, &interval_ms))
        return -EINVAL;
    if ((interval_ms < MPU3050_MIN_POLL_INTERVAL) || (interval_ms > MPU3050_MAX_POLL_INTERVAL))
        return -EINVAL;

    ret = mpu3050_poll_delay_set(&sensor->cdev, interval_ms);

    return ret < 0 ? ret : size;
}
static int mpu3050_enable_set(struct sensors_classdev *sensors_cdev,unsigned int enabled)
{
    struct mpu3050_sensor *sensor = container_of(sensors_cdev,struct mpu3050_sensor, cdev);
    ktime_t ktime;
    if (enabled && (!sensor->enable)) {
        sensor->enable = enabled;
        pm_runtime_get_sync(sensor->dev);
        mpu3050_poll_delay_set(&sensor->cdev,sensor->poll_interval);
        if (sensor->use_poll){
            ktime = ktime_set(0, (sensor->poll_interval) * NSEC_PER_MSEC);
            hrtimer_start(&sensor->timer_gyro, ktime, HRTIMER_MODE_REL);
//            schedule_delayed_work(&sensor->input_work,msecs_to_jiffies(sensor->poll_interval));
        }else
            enable_irq(sensor->client->irq);
    } else if (!enabled && sensor->enable) {
        if (sensor->use_poll){
            hrtimer_cancel(&sensor->timer_gyro);
            cancel_work_sync(&sensor->input_work);
//            flush_workqueue(sensor->data_wq);
            if(sensor->high_q)
                flush_workqueue(sensor->data_wq_h);
            else
                flush_workqueue(sensor->data_wq);
        }
        else
            disable_irq(sensor->client->irq);
        pm_runtime_put_sync(sensor->dev);
        sensor->enable = enabled;
    } else {
        dev_warn(&sensor->client->dev,"ignore enable state change from %d to %d\n",sensor->enable, enabled);
    }

    return 0;
}

/**
 *  Set/get enable function is just needed by sensor HAL.
 */

static ssize_t mpu3050_attr_set_enable(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    struct mpu3050_sensor *sensor = dev_get_drvdata(dev);
    unsigned long val;
    int err;

    if (kstrtoul(buf, 10, &val))
        return -EINVAL;
    err = mpu3050_enable_set(&sensor->cdev, val);
    if (err < 0)
        return err;

    return count;
}

static ssize_t mpu3050_attr_get_enable(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct mpu3050_sensor *sensor = dev_get_drvdata(dev);
    return snprintf(buf, 4, "%d\n", sensor->enable);
}

static ssize_t attr_get_data(struct device *dev, struct device_attribute *attr,char *buf)
{
    ssize_t ret;
    struct mpu3050_sensor *sensor = dev_get_drvdata(dev);
    //u8 data;
    struct axis_data data_out;

    mpu3050_read_xyz(sensor->client,sensor->platform_data, &data_out);
    /*TODO: error need to be managed */
    printk(KERN_INFO "x = %d y = %d z = %d\n", data_out.x, data_out.y, data_out.z);
    ret = sprintf(buf, "%d %d %d\n", data_out.x, data_out.y, data_out.z);
    return ret;
}

static ssize_t attr_set_data(struct device *dev, struct device_attribute *attr, const char *buf,size_t count)
{
    struct mpu3050_sensor *sensor = dev_get_drvdata(dev);
    unsigned long val;
  
    if (kstrtoul(buf, 10, &val))
        return -EINVAL;

    pr_err("[****]attr_set_data = %lu \n",val);
    
    if(val)
        sensor->high_q = true;
    else
        sensor->high_q = false;
    return count;
}

static struct device_attribute attributes[] = {
    __ATTR(pollrate_ms, 0664,mpu3050_attr_get_polling_rate,mpu3050_attr_set_polling_rate),
    __ATTR(enable, 0644,mpu3050_attr_get_enable,mpu3050_attr_set_enable),
    __ATTR(value, 0644, attr_get_data, attr_set_data),
};

static int create_sysfs_interfaces(struct device *dev)
{
    int i;
    int err;
    for (i = 0; i < ARRAY_SIZE(attributes); i++) {
        err = device_create_file(dev, attributes + i);
        if (err)
            goto error;
    }
    return 0;
error:
    for ( ; i >= 0; i--)
        device_remove_file(dev, attributes + i);
    dev_err(dev, "%s:Unable to create interface\n", __func__);
    return err;
}

static int remove_sysfs_interfaces(struct device *dev)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(attributes); i++)
        device_remove_file(dev, attributes + i);
    return 0;
}

/**
 *  mpu3050_xyz_read_reg - read the axes values
 *  @buffer: provide register addr and get register
 *  @length: length of register
 *
 *  Reads the register values in one transaction or returns a negative
 *  error code on failure.
 */
static int mpu3050_xyz_read_reg(struct i2c_client *client,u8 *buffer, int length)
{
    /*
     * Annoying we can't make this const because the i2c layer doesn't
     * declare input buffers const.
     */
    char cmd = MPU3050_XOUT_H;
    struct i2c_msg msg[] = {
         {
              .addr = client->addr,
              .flags = 0,
              .len = 1,
              .buf = &cmd,
         },
         {
              .addr = client->addr,
              .flags = I2C_M_RD,
              .len = length,
              .buf = buffer,
         },
    };

    return i2c_transfer(client->adapter, msg, 2);
}

/**
 *  mpu3050_read_xyz - get co-ordinates from device
 *  @client: i2c address of sensor
 *  @coords: co-ordinates to update
 *
 *  Return the converted X Y and Z co-ordinates from the sensor device
 */
static void mpu3050_read_xyz(struct i2c_client *client,struct mpu3050_gyro_platform_data *pdata ,struct axis_data *coords)
{
    u16 buffer[3];
    u16 temp[3];

    mpu3050_xyz_read_reg(client, (u8 *)buffer, 6);
    temp[0] = be16_to_cpu(buffer[0]);
    temp[1] = be16_to_cpu(buffer[1]);
    temp[2] = be16_to_cpu(buffer[2]);

    coords->x = ((pdata->negate_x) ? (-temp[pdata->axis_map_x]):(temp[pdata->axis_map_x]));
    coords->y = ((pdata->negate_y) ? (-temp[pdata->axis_map_y]):(temp[pdata->axis_map_y]));
    coords->z = ((pdata->negate_z) ? (-temp[pdata->axis_map_z]):(temp[pdata->axis_map_z]));
//  dev_dbg(&client->dev, "%s: x %d, y %d, z %d\n", __func__,coords->x, coords->y, coords->z);
}

/**
 *  mpu3050_set_power_mode - set the power mode
 *  @client: i2c client for the sensor
 *  @val: value to switch on/off of power, 1: normal power, 0: low power
 *
 *  Put device to normal-power mode or low-power mode.
 */
static void mpu3050_set_power_mode(struct i2c_client *client, u8 val)
{
    u8 value;
    struct mpu3050_sensor *sensor = i2c_get_clientdata(client);

    if (val) {
        mpu3050_config_regulator(client, 1);
        udelay(10);
//      gpio_set_value(sensor->enable_gpio, 1);
//      msleep(60);
        mpu3050_pinctrl_state(sensor, true);
    }

    value = i2c_smbus_read_byte_data(client, MPU3050_PWR_MGM);
    value = (value & ~MPU3050_PWR_MGM_MASK)|(((val << MPU3050_PWR_MGM_POS) & MPU3050_PWR_MGM_MASK)^MPU3050_PWR_MGM_MASK);
    i2c_smbus_write_byte_data(client, MPU3050_PWR_MGM, value);
    if (!val) {
        mpu3050_pinctrl_state(sensor, false);
        udelay(10);
//      gpio_set_value(sensor->enable_gpio, 0);
//      udelay(10);
        mpu3050_config_regulator(client, 0);
    }
}

static enum hrtimer_restart mpu3050_timer_handle(struct hrtimer *hrtimer)
{
    ktime_t ktime;
    struct mpu3050_sensor *sensor;
    sensor = container_of(hrtimer, struct mpu3050_sensor, timer_gyro);

    if(sensor->high_q)
        queue_work(sensor->data_wq_h, &sensor->input_work);
    else
        queue_work(sensor->data_wq, &sensor->input_work);
    ktime = ktime_set(0,sensor->poll_interval * NSEC_PER_MSEC);
    hrtimer_forward_now(&sensor->timer_gyro,ktime);
    //hrtimer_start(&sensor->timer_gyro, ktime, HRTIMER_MODE_REL);

    return HRTIMER_RESTART;
}

/**
 *  mpu3050_interrupt_thread - handle an IRQ
 *  @irq: interrupt numner
 *  @data: the sensor
 *
 *  Called by the kernel single threaded after an interrupt occurs. Read
 *  the sensor data and generate an input event for it.
 */
static irqreturn_t mpu3050_interrupt_thread(int irq, void *data)
{
    struct mpu3050_sensor *sensor = data;
    struct axis_data axis;

    ktime_t timestamp;
    timestamp = ktime_get_boottime();

    mpu3050_read_xyz(sensor->client,sensor->platform_data, &axis);
//  printk(KERN_EMERG "%s: x(%d) y(%d) z(%d)\n", __func__,axis.x,axis.y,axis.z);
    input_report_abs(sensor->idev, ABS_RX, axis.x);
    input_report_abs(sensor->idev, ABS_RY, axis.y);
    input_report_abs(sensor->idev, ABS_RZ, axis.z);

    input_event(sensor->idev,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
    input_event(sensor->idev,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);

    input_sync(sensor->idev);

    return IRQ_HANDLED;
}

/**
 *  mpu3050_input_work_fn -	polling work
 *  @work: the work struct
 *
 *  Called by the work queue; read sensor data and generate an input
 *  event
 */
static void mpu3050_input_work_fn(struct work_struct *work)
{
    struct mpu3050_sensor *sensor;
    struct axis_data axis;
    ktime_t timestamp;
    sensor = container_of(work,struct mpu3050_sensor, input_work);
    mpu3050_read_xyz(sensor->client,sensor->platform_data,&axis);
    timestamp = ktime_get_boottime();
    input_report_abs(sensor->idev, ABS_RX, axis.x);
    input_report_abs(sensor->idev, ABS_RY, axis.y);
    input_report_abs(sensor->idev, ABS_RZ, axis.z);
    input_event(sensor->idev,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
    input_event(sensor->idev,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
    input_sync(sensor->idev);

//    if (sensor->use_poll)
//        schedule_delayed_work(&sensor->input_work,msecs_to_jiffies(sensor->poll_interval));
}

/**
 *  mpu3050_hw_init - initialize hardware
 *  @sensor: the sensor
 *
 *  Called during device probe; configures the sampling method.
 */
static int mpu3050_hw_init(struct mpu3050_sensor *sensor)
{
    struct i2c_client *client = sensor->client;
    int ret;
    u8 reg;

    ret = i2c_smbus_read_byte_data(client, MPU3050_PWR_MGM);
    if (ret < 0)
        return ret;

    ret &= ~MPU3050_PWR_MGM_CLKSEL;
    ret |= MPU3050_PWR_MGM_PLL_Z;
    ret = i2c_smbus_write_byte_data(client, MPU3050_PWR_MGM, ret);
    if (ret < 0)
        return ret;

    /* Output frequency divider. The poll interval */
    ret = i2c_smbus_write_byte_data(client, MPU3050_SMPLRT_DIV,sensor->poll_interval - 1);
    if (ret < 0)
        return ret;

    /* Set low pass filter and full scale */
    reg = MPU3050_DLPF_CFG_42HZ;
    reg |= MPU3050_DEFAULT_FS_RANGE << 3;
    reg |= MPU3050_EXT_SYNC_NONE << 5;
    ret = i2c_smbus_write_byte_data(client, MPU3050_DLPF_FS_SYNC, reg);
    if (ret < 0)
        return ret;

    /* Enable interrupts */
    if (!sensor->use_poll) {
        reg = MPU3050_ACTIVE_LOW;
        reg |= MPU3050_OPEN_DRAIN;
        reg |= MPU3050_RAW_RDY_EN;
        ret = i2c_smbus_write_byte_data(client, MPU3050_INT_CFG, reg);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int mpu3050_pinctrl_init(struct mpu3050_sensor *sensor)
{
    struct i2c_client *client = sensor->client;

    sensor->pinctrl = devm_pinctrl_get(&client->dev);
    if (IS_ERR_OR_NULL(sensor->pinctrl)) {
        dev_err(&client->dev,"Failed to get pinctrl\n");
        return PTR_ERR(sensor->pinctrl);
    }
    sensor->pin_default = pinctrl_lookup_state(sensor->pinctrl, MPU3050_PINCTRL_DEFAULT);
    if (IS_ERR_OR_NULL(sensor->pin_default))
        dev_err(&client->dev,"Failed to look up default state\n");
    sensor->pin_sleep = pinctrl_lookup_state(sensor->pinctrl, MPU3050_PINCTRL_SUSPEND);
    if (IS_ERR_OR_NULL(sensor->pin_sleep))
        dev_err(&client->dev,"Failed to look up sleep state\n");
    return 0;
}

static void mpu3050_pinctrl_state(struct mpu3050_sensor *sensor, bool active)
{
    struct i2c_client *client = sensor->client;
    int ret;

    dev_dbg(&client->dev,"mpu3050_pinctrl_state en=%d\n", active);
    if (active) {
        if (!IS_ERR_OR_NULL(sensor->pin_default)) {
            ret = pinctrl_select_state(sensor->pinctrl, sensor->pin_default);
            if (ret)
                dev_err(&client->dev,"Error pinctrl_select_state(%s) err:%d\n",MPU3050_PINCTRL_DEFAULT, ret);
        }
    } else {
        if (!IS_ERR_OR_NULL(sensor->pin_sleep)) {
            ret = pinctrl_select_state(sensor->pinctrl, sensor->pin_sleep);
            if (ret)
                dev_err(&client->dev,"Error pinctrl_select_state(%s) err:%d\n",MPU3050_PINCTRL_SUSPEND, ret);
        }
    }
    return;
}

#ifdef CONFIG_OF
static int mpu3050_parse_dt(struct device *dev, struct mpu3050_gyro_platform_data *pdata)
{
    struct device_node *np = dev->of_node;
    u32 temp_val;
    u32 irq_flags;
    int rc = 0;

    rc = of_property_read_u32(dev->of_node, "invn,poll-interval",&pdata->poll_interval);
    if (rc) {
        dev_err(dev, "Failed to read poll-interval\n");
        return rc;
    }

    /* check gpio_int later, if it is invalid, just use poll */
    pdata->gpio_int = of_get_named_gpio_flags(dev->of_node,"invn,gpio-int", 0, &irq_flags);
    if(rc && (rc != -EINVAL)){
        dev_err(dev, "Unable to read gpio-int\n");
        return rc;
    }
    rc = of_property_read_u32(np, "invn,axis-map-x", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read axis-map_x\n");
        return rc;
    } else {
        pdata->axis_map_x = (u8)temp_val;
    }
    rc = of_property_read_u32(np, "invn,axis-map-y", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read axis_map_y\n");
        return rc;
    } else {
        pdata->axis_map_y = (u8)temp_val;
    }

    rc = of_property_read_u32(np, "invn,axis-map-z", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read axis-map-z\n");
        return rc;
    } else {
        pdata->axis_map_z = (u8)temp_val;
    }

    pdata->negate_x = of_property_read_bool(np, "invn,negate-x");
    pdata->negate_y = of_property_read_bool(np, "invn,negate-y");
    pdata->negate_z = of_property_read_bool(np, "invn,negate-z");
    pr_err("[itg3050]:axismap %d,%d,%d\n",pdata->axis_map_x,pdata->axis_map_y,pdata->axis_map_z);

    return 0;
}
#else
static int mpu3050_parse_dt(struct device *dev,struct mpu3050_gyro_platform_data *pdata)
{
    return -EINVAL;
}
#endif

/**
 *  mpu3050_probe - device detection callback
 *  @client: i2c client of found device
 *  @id: id match information
 *
 *  The I2C layer calls us when it believes a sensor is present at this
 *  address. Probe to see if this is correct and to validate the device.
 *
 *  If present install the relevant sysfs interfaces and input device.
 */
static int mpu3050_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    struct mpu3050_sensor *sensor;
    struct input_dev *idev;
    struct mpu3050_gyro_platform_data *pdata;
    int ret;
    int error;
    u32 i;

    sensor = kzalloc(sizeof(struct mpu3050_sensor), GFP_KERNEL);
    idev = devm_input_allocate_device(&client->dev);
    if (!sensor || !idev) {
        dev_err(&client->dev, "failed to allocate driver data\n");
        error = -ENOMEM;
        goto err_free_mem;
    }

    sensor->client = client;
    sensor->dev = &client->dev;
    sensor->idev = idev;
    i2c_set_clientdata(client, sensor);

    if (client->dev.of_node) {
        pdata = devm_kzalloc(&client->dev,sizeof(struct mpu3050_gyro_platform_data), GFP_KERNEL);
        if (!pdata) {
            dev_err(&client->dev, "Failed to allcated memory\n");
            error = -ENOMEM;
            goto err_free_mem;
        }
        ret = mpu3050_parse_dt(&client->dev, pdata);
        if (ret) {
            dev_err(&client->dev, "Failed to parse device tree\n");
            error = ret;
            goto err_free_mem;
        }
    } else
        pdata = client->dev.platform_data;
    sensor->platform_data = pdata;
    if (sensor->platform_data) {
        u32 interval = sensor->platform_data->poll_interval;
//      sensor->enable_gpio = sensor->platform_data->gpio_en;
        if ((interval < MPU3050_MIN_POLL_INTERVAL) || (interval > MPU3050_MAX_POLL_INTERVAL))
            sensor->poll_interval = MPU3050_DEFAULT_POLL_INTERVAL;
        else
            sensor->poll_interval = interval;
    } else {
        sensor->poll_interval = MPU3050_DEFAULT_POLL_INTERVAL;
//      sensor->enable_gpio = -EINVAL;
    }
//    sensor->cdev = sensors_cdev;
//    sensor->cdev.min_delay = MPU3050_MIN_POLL_INTERVAL * 1000;
//    sensor->cdev.delay_msec = sensor->poll_interval;
//    sensor->cdev.sensors_enable = mpu3050_enable_set;
//    sensor->cdev.sensors_poll_delay = mpu3050_poll_delay_set;
//    ret = sensors_classdev_register(&sensor->idev->dev, &sensor->cdev);
//
//    if (ret) {
//        dev_err(&client->dev, "class device create failed: %d\n", ret);
//        error = -EINVAL;
//        goto err_free_mem;
//    }
    ret = mpu3050_pinctrl_init(sensor);
    if (ret) {
        dev_err(&client->dev, "Can't initialize pinctrl\n");
        goto err_free_mem;
    }

//  if (gpio_is_valid(sensor->enable_gpio)) {
//      ret = gpio_request(sensor->enable_gpio, "GYRO_EN_PM");
//      gpio_direction_output(sensor->enable_gpio, 1);
//  }
    mpu3050_set_power_mode(client, 1);
    ret = i2c_smbus_read_byte_data(client, MPU3050_CHIP_ID_REG);
    if (ret < 0) {
        dev_err(&client->dev, "failed to detect device\n");
        error = -ENXIO;
        goto err_free_mem;
    }
    for (i = 0; i < ARRAY_SIZE(mpu3050_chip_ids); i++)
        if (ret == mpu3050_chip_ids[i])
            break;

    if (i == ARRAY_SIZE(mpu3050_chip_ids)) {
        dev_err(&client->dev, "unsupported chip id\n");
        error = -ENXIO;
        goto err_free_mem;
    }
    idev->name = MPU3050_DEV_NAME_GYRO;
    idev->id.bustype = BUS_I2C;

    input_set_capability(idev, EV_ABS, ABS_MISC);
    input_set_abs_params(idev, ABS_RX,MPU3050_MIN_VALUE, MPU3050_MAX_VALUE, 0, 0);
    input_set_abs_params(idev, ABS_RY,MPU3050_MIN_VALUE, MPU3050_MAX_VALUE, 0, 0);
    input_set_abs_params(idev, ABS_RZ,MPU3050_MIN_VALUE, MPU3050_MAX_VALUE, 0, 0);
    input_set_events_per_packet(sensor->idev, 1000);
    input_set_drvdata(idev, sensor);
    pm_runtime_set_active(&client->dev);
    error = mpu3050_hw_init(sensor);
    if (error)
        goto err_pm_set_suspended;

    client->irq = 0;
    sensor->data_wq = create_freezable_workqueue("mpu3050_data_work");
    sensor->data_wq_h = alloc_workqueue("mpu3050_data_work1", WQ_HIGHPRI, 0);
    if (!sensor->data_wq) {
        dev_err(&client->dev, "Cannot create workqueue!\n");
        goto err_free_gpio;
    }
    if (!sensor->data_wq_h) {
        dev_err(&client->dev, "Cannot create workqueue!\n");
        goto err_free_gpio;
    }
    sensor->high_q = false;
    hrtimer_init(&sensor->timer_gyro, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
    sensor->timer_gyro.function = mpu3050_timer_handle;
    mutex_init(&sensor->op_lock);
    if (client->irq == 0) {
        sensor->use_poll = 1;
//        INIT_DELAYED_WORK(&sensor->input_work, mpu3050_input_work_fn);
        INIT_WORK(&sensor->input_work, mpu3050_input_work_fn);
    } else {
        sensor->use_poll = 0;

        if (gpio_is_valid(sensor->platform_data->gpio_int)) {
            /* configure interrupt gpio */
            ret = gpio_request(sensor->platform_data->gpio_int,"gyro_gpio_int");
            if (ret) {
                pr_err("%s: unable to request interrupt gpio %d\n",__func__,sensor->platform_data->gpio_int);
                goto err_pm_set_suspended;
            }
            ret = gpio_direction_input(sensor->platform_data->gpio_int);
            if (ret) {
                pr_err("%s: unable to set direction for gpio %d\n",__func__, sensor->platform_data->gpio_int);
                goto err_free_gpio;
            }
            client->irq = gpio_to_irq(sensor->platform_data->gpio_int);
        } else {
            ret = -EINVAL;
            goto err_pm_set_suspended;
        }
        error = request_threaded_irq(client->irq,NULL, mpu3050_interrupt_thread,IRQF_TRIGGER_FALLING | IRQF_ONESHOT,"mpu3050", sensor);
        if (error) {
            dev_err(&client->dev,"can't get IRQ %d, error %d\n",client->irq, error);
            goto err_pm_set_suspended;
        }
        disable_irq(client->irq);
    }
    sensor->enable = 0;
    mpu3050_set_power_mode(client, 0);

    error = input_register_device(idev);
    if (error) {
        dev_err(&client->dev, "failed to register input device\n");
        goto err_free_irq;
    }

    error = create_sysfs_interfaces(&idev->dev);
    if (error < 0) {
        dev_err(&client->dev, "failed to create sysfs\n");
        goto err_free_irq;
    }

    sensor->cdev = sensors_cdev;
    sensor->cdev.delay_msec = sensor->poll_interval;
    sensor->cdev.sensors_enable = mpu3050_enable_set;
    sensor->cdev.sensors_poll_delay = mpu3050_poll_delay_set;
    sensor->cdev.sensors_set_latency = mpu3050_poll_delay_set;
    ret = sensors_classdev_register(&sensor->idev->dev, &sensor->cdev);
    if (ret) {
        dev_err(&client->dev, "class device create failed: %d\n", ret);
        error = -EINVAL;
        goto err_free_irq;
    }
    pm_runtime_enable(&client->dev);
    pm_runtime_set_autosuspend_delay(&client->dev, MPU3050_AUTO_DELAY);

    return 0;

//err_class_sysfs:
//    sensors_classdev_unregister(&sensor->cdev);
err_free_irq:
    if (client->irq > 0)
        free_irq(client->irq, sensor);
err_free_gpio:
    if ((client->irq > 0) && (gpio_is_valid(sensor->platform_data->gpio_int)))
        gpio_free(sensor->platform_data->gpio_int);
err_pm_set_suspended:
    pm_runtime_set_suspended(&client->dev);
err_free_mem:
    kfree(sensor);
    return error;
}

/**
 *  mpu3050_remove - remove a sensor
 *  @client: i2c client of sensor being removed
 *
 *  Our sensor is going away, clean up the resources.
 */
static int mpu3050_remove(struct i2c_client *client)
{
    struct mpu3050_sensor *sensor = i2c_get_clientdata(client);

    pm_runtime_disable(&client->dev);
    pm_runtime_set_suspended(&client->dev);

    if (client->irq)
        free_irq(client->irq, sensor);
    destroy_workqueue(sensor->data_wq);
    destroy_workqueue(sensor->data_wq_h);
    remove_sysfs_interfaces(&client->dev);
//  if (gpio_is_valid(sensor->enable_gpio))
//      gpio_free(sensor->enable_gpio);

    kfree(sensor);
    return 0;
}

#ifdef CONFIG_PM
/**
 *  mpu3050_suspend - called on device suspend
 *  @dev: device being suspended
 *
 *  Put the device into sleep mode before we suspend the machine.
 */
static int mpu3050_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mpu3050_sensor *sensor = i2c_get_clientdata(client);

    if (sensor->enable) {
        if (!sensor->use_poll)
            disable_irq(client->irq);
        mpu3050_set_power_mode(client, 0);
    }
    return 0;
}

/**
 *  mpu3050_resume - called on device resume
 *  @dev: device being resumed
 *
 *  Put the device into powered mode on resume.
 */
static int mpu3050_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mpu3050_sensor *sensor = i2c_get_clientdata(client);

    if (sensor->enable) {
    mpu3050_set_power_mode(client, 1);
    mpu3050_hw_init(sensor);
    if (!sensor->use_poll)
        enable_irq(client->irq);
    }

    return 0;
}

/**
 *  mpu3050_runtime_suspend - called on device enters runtime suspend
 *  @dev: device being suspended
 *
 *  Put the device into sleep mode.
 */
static int mpu3050_runtime_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mpu3050_sensor *sensor = i2c_get_clientdata(client);

    if (sensor->enable)
        mpu3050_set_power_mode(client, 0);

    return 0;
}

/**
 *  mpu3050_runtime_resume - called on device enters runtime resume
 *  @dev: device being resumed
 *
 *  Put the device into powered mode.
 */
static int mpu3050_runtime_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mpu3050_sensor *sensor = i2c_get_clientdata(client);

    if (sensor->enable) {
        mpu3050_set_power_mode(client, 1);
        mpu3050_hw_init(sensor);
    }

    return 0;
}
#endif

static const struct dev_pm_ops mpu3050_pm = {
    .runtime_suspend = mpu3050_runtime_suspend,
    .runtime_resume = mpu3050_runtime_resume,
    .runtime_idle = NULL,
    .suspend = mpu3050_suspend,
    .resume = mpu3050_resume,
    .freeze = mpu3050_suspend,
    .thaw = mpu3050_resume,
    .poweroff = mpu3050_suspend,
    .restore = mpu3050_resume,
};

static const struct i2c_device_id mpu3050_ids[] = {
    { "mpu3050", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mpu3050_ids);

static const struct of_device_id mpu3050_of_match[] = {
    { .compatible = "invn,mpu3050", },
    { },
};
MODULE_DEVICE_TABLE(of, mpu3050_of_match);

static struct i2c_driver mpu3050_i2c_driver = {
    .driver   = {
        .name       = "mpu3050",
        .owner      = THIS_MODULE,
        .pm         = &mpu3050_pm,
        .of_match_table = mpu3050_of_match,
    },
    .probe    = mpu3050_probe,
    .remove   = mpu3050_remove,
    .id_table = mpu3050_ids,
};

module_i2c_driver(mpu3050_i2c_driver);

MODULE_AUTHOR("Wistron Corp.");
MODULE_DESCRIPTION("MPU3050 Tri-axis gyroscope driver");
MODULE_LICENSE("GPL");
