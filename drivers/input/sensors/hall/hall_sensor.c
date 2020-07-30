#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>

#define HALLSENSOR_EVENT_WAKELOCK (2*HZ)
#define DEV_NAME "HALL_SENSOR"
#define HALL_INPUT "/dev/input/lid_indev"
#define DEBOUNCE_TIME 50
static struct workqueue_struct *hall_sensor_wq;
static struct kobject *hall_sensor_kobj;
struct wake_lock Wake_Lock, Event_wakelock;
static struct hall_sensor_str {
 	int irq;
	int status;
	int gpio;
	int enable; 
	spinlock_t mHallSensorLock;
	struct input_dev *lid_indev;
 	struct delayed_work hall_sensor_work;
}* hall_sensor_dev;

/* This file node can show the hall sensor action status */
static ssize_t show_action_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	if (!hall_sensor_dev)
	    return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->status);
}
static ssize_t store_action_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	unsigned long flags;
	char *MSG = NULL;
	if (!hall_sensor_dev)
            return sprintf(MSG, "Hall sensor does not exist!\n");
        sscanf(buf, "%du", &request);
        
        spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
        if (!request)
            hall_sensor_dev->status = 0;
	else
            hall_sensor_dev->status = 1;
	spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
        
        /*Report SW_LID event to input system */
        input_report_switch(hall_sensor_dev->lid_indev, SW_LID, !hall_sensor_dev->status);
        input_sync(hall_sensor_dev->lid_indev);
        pr_info("[%s] SW_LID rewite value = %d\n",DEV_NAME ,!hall_sensor_dev->status);
	return count;
}

/* This file node can enable or disable hall sensor function */
static ssize_t show_hall_sensor_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (!hall_sensor_dev)
            return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->enable);
}
static ssize_t store_hall_sensor_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	char *MSG = NULL;
	if (!hall_sensor_dev)
            return sprintf(MSG, "Hall sensor does not exist!\n");
	sscanf(buf, "%du", &request);
	if (!!request==hall_sensor_dev->enable){
	    return count;
	}
	else {
	    unsigned long flags;
	    spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
	    hall_sensor_dev->enable=!!request;
	    spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
	}
	return count;
}

/* This file node can direct control the gpio high/low which used by hall sensor */
static ssize_t show_gpio_status(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (!hall_sensor_dev)
            return sprintf(buf, "Hall sensor does not exist!\n");
        return sprintf(buf, "GPIO:[%d],Status:[%d]\n",hall_sensor_dev->gpio,gpio_get_value(hall_sensor_dev->gpio));
}
static ssize_t store_gpio_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int request;
        char *MSG = NULL;
        if (!hall_sensor_dev)
            return sprintf(MSG, "Hall sensor does not exist!\n");
        sscanf(buf, "%du", &request);
        if (request > 1){
            return count;
        }
        else {
            unsigned long flags;
            spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
            gpio_direction_output(hall_sensor_dev->gpio,request);
            spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
        }
        return count;
}


static SENSOR_DEVICE_ATTR_2(action_status, S_IRUGO|S_IWUSR, show_action_status, store_action_status, 0, 0);
static SENSOR_DEVICE_ATTR_2(activity, S_IRUGO|S_IWUSR,show_hall_sensor_enable, store_hall_sensor_enable, 0, 0);
static SENSOR_DEVICE_ATTR_2(gpio_status, S_IRUGO|S_IWUSR, show_gpio_status, store_gpio_status, 0, 0);

static struct attribute *hall_sensor_attrs[] = {
	&sensor_dev_attr_action_status.dev_attr.attr,
	&sensor_dev_attr_activity.dev_attr.attr,
        &sensor_dev_attr_gpio_status.dev_attr.attr,
	NULL
};

static struct attribute_group hall_sensor_group = {
	.name = "hall_control",
	.attrs = hall_sensor_attrs
};

static int lid_input_device_create(void)
{
	int err = -1;
        
        /* allocate managed input device */
	hall_sensor_dev->lid_indev = input_allocate_device();     
	if (!hall_sensor_dev->lid_indev){
            pr_info("input device allocation failed\n");
	    return -ENOMEM;
	}	

	hall_sensor_dev->lid_indev->name = DEV_NAME;
	hall_sensor_dev->lid_indev->phys= HALL_INPUT;
	hall_sensor_dev->lid_indev->dev.parent= NULL;
        /* Set event type to EV_SW -> SW_LID */
	input_set_capability(hall_sensor_dev->lid_indev, EV_SW, SW_LID);
        /* register hall_sensor_dev with input core */
	err = input_register_device(hall_sensor_dev->lid_indev);
	if (err < 0) {
            pr_info("unable to register input device %s\n",DEV_NAME);
            input_free_device(hall_sensor_dev->lid_indev);
            hall_sensor_dev->lid_indev = NULL;
	    return err;
	}

	return 0;
}

static void lid_report_function(struct work_struct *dat)
{
        unsigned long flags;
	msleep(DEBOUNCE_TIME);
        if (!hall_sensor_dev->enable){
            pr_info("Disable hall sensor that requested by user!\n");
	    wake_unlock(&Wake_Lock);
	    return;
        }

        /*Change SW hall sensor status which depends on GPIO status*/
        spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
        if (gpio_get_value(hall_sensor_dev->gpio) > 0)
            hall_sensor_dev->status = 1;
        else
            hall_sensor_dev->status = 0;
        spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);

        /* Create the timeout wakelock to prevent input event transport fail during suspend procedure */
        wake_lock_timeout(&Event_wakelock,HALLSENSOR_EVENT_WAKELOCK);
        
        /* Report SW_LID event to input system */
        input_report_switch(hall_sensor_dev->lid_indev, SW_LID, !hall_sensor_dev->status);
        input_sync(hall_sensor_dev->lid_indev);
	wake_unlock(&Wake_Lock);
        pr_info("[%s] SW_LID report value = %d\n",DEV_NAME ,!hall_sensor_dev->status);
}

static irqreturn_t hall_sensor_interrupt_handler(int irq, void *dev_id)
{
	pr_info("[%s] hall_sensor_interrupt = %d\n",DEV_NAME ,hall_sensor_dev->irq);
        /* Begin to deal with interrupt bottom half */
	queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
	/* Create the wakelock to ensure work can be finished before system suspend */
        wake_lock(&Wake_Lock);
	return IRQ_HANDLED;
}

static int set_irq_hall_sensor(struct platform_device *pdev)
{
	int rc = 0 ;
        int irq_flags;

        /* Accroding to irq domain mappping GPIO number to IRQ number */
	hall_sensor_dev->irq = gpio_to_irq(hall_sensor_dev->gpio);
	pr_info("[%s] hall_sensor irq = %d\n",DEV_NAME ,hall_sensor_dev->irq);
        
        irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
        /* IRQs requested with this function will be automatically freed on driver detach */
	rc = devm_request_irq(&pdev->dev ,hall_sensor_dev->irq ,hall_sensor_interrupt_handler,
			      irq_flags ,"hall_sensor_irq" ,hall_sensor_dev);
	if (rc < 0) {
	    pr_info("[%s]Couldn't register for hall sensor interrupt,irq = %d, rc = %d\n",DEV_NAME ,hall_sensor_dev->irq ,rc);
	    rc = -EIO;
	    return rc ;
	}

        /* Enable this irq line */
	enable_irq_wake(hall_sensor_dev->irq);
	return 0;
}

static int hall_parse_dt(struct device *dev){

        /* Parisng GPIO number with "ASUS,hall-intr-gpio" from devie tree  */
        hall_sensor_dev->gpio = of_get_named_gpio_flags(dev->of_node,"ASUS,hall-intr-gpio",0,0);
        if (!gpio_is_valid(hall_sensor_dev->gpio) || hall_sensor_dev->gpio == 0){
             dev_err(dev, "hall gpio[%d] is not valid\n", hall_sensor_dev->gpio);
             return -EINVAL;
        }
        return 0;
}

static int lid_probe(struct platform_device *pdev){
	
        int ret = 0;
        
        dev_info(&pdev->dev, "=====%s probe start=====\n",DEV_NAME);
        /* Create kobjet for hall sensor */
        hall_sensor_kobj = kobject_create_and_add("hall_sensor", kernel_kobj);
        if (!hall_sensor_kobj){
            ret = -ENOMEM;
            dev_err(&pdev->dev,"failed to creat kobject %d\n", ret);
            platform_device_unregister(pdev);
            goto exit;
        }

        /* given a directory kobject, create an attribute group */
        ret = sysfs_create_group(hall_sensor_kobj, &hall_sensor_group);
        if (ret){
            kobject_put(hall_sensor_kobj);
            return ret;
        }

        /* Allocating memory space for hall_sensor_dev */
        hall_sensor_dev = devm_kzalloc(&pdev->dev, sizeof(struct hall_sensor_str), GFP_KERNEL);
        if (!hall_sensor_dev){
            ret = -ENOMEM;
            dev_err(&pdev->dev,"failed to allocate memory %d\n", ret);
            goto exit;
        }

        /* initialize lock structure */
        spin_lock_init(&hall_sensor_dev->mHallSensorLock);
        wake_lock_init(&Wake_Lock, WAKE_LOCK_SUSPEND, "lid_suspend_blocker");
        wake_lock_init(&Event_wakelock, WAKE_LOCK_SUSPEND, "hall_event_timeout");
        hall_sensor_dev->enable = 1;

        /* Parsing hall sensor resource from device tree */
        if (pdev->dev.of_node) {
           ret = hall_parse_dt(&pdev->dev);
           if (ret < 0){
               dev_err(&pdev->dev,"fail to parse device tree\n");
               kfree(hall_sensor_dev);
               hall_sensor_dev=NULL;
               goto exit;
           }
        } else {
              dev_err(&pdev->dev, "No valid platform data.\n");
              ret = -ENODEV;
              goto exit;
        }

        /* Checking GPIO pin which requested by driver can be used or not */
        ret = gpio_request(hall_sensor_dev->gpio,"hall_sensor_gpio");
        if (ret) {
            dev_err(&pdev->dev, "unable to request gpio %d\n", hall_sensor_dev->gpio);
            goto exit;
        }
        gpio_direction_input(hall_sensor_dev->gpio);

        /* Initialize workqueue and assign lid_report_function to work */
        hall_sensor_wq = create_singlethread_workqueue("hall_sensor_wq");
        INIT_DEFERRABLE_WORK(&hall_sensor_dev->hall_sensor_work, lid_report_function);

        /* Start create input_dev */
        hall_sensor_dev->lid_indev = NULL;
        ret = lid_input_device_create();
        if (ret < 0){
            dev_err(&pdev->dev, "fail to create input device\n");
            goto exit;
        }
        queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);

        /* Config irq setting */
        ret = set_irq_hall_sensor(pdev);
        if (ret < 0){
            dev_err(&pdev->dev,"fail to set irq\n");
            gpio_free(hall_sensor_dev->gpio);
            goto exit;
        }

        dev_info(&pdev->dev, "=====%s probe success=====\n",DEV_NAME);
	return 0;

exit:
        return ret;
}

static const struct of_device_id hall_sensor_of_match[] = {
        { .compatible = "ASUS,hall"},
	{}
};

static struct platform_driver lid_platform_driver = {
	.driver = {
	     .name  = DEV_NAME,
	     .owner = THIS_MODULE,
	     .of_match_table = hall_sensor_of_match,
	},
	.probe          = lid_probe,
};

static int __init hall_sensor_init(void)
{	
        pr_info("\n++++++[%s]%s++++++\n",DEV_NAME,__func__);
        return platform_driver_register(&lid_platform_driver);
}

static void __exit hall_sensor_exit(void)
{
	platform_driver_unregister(&lid_platform_driver);
        kobject_put(hall_sensor_kobj);
	wake_lock_destroy(&Wake_Lock);
        wake_lock_destroy(&Event_wakelock);
}

module_init(hall_sensor_init);
module_exit(hall_sensor_exit);
MODULE_DESCRIPTION("hall sensor Driver");
MODULE_LICENSE("GPL v2");

