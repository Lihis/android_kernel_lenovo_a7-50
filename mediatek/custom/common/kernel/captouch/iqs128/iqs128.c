/* drivers/hwmon/mt6516/amit/IQS128.c - IQS128/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_captouch.h>
#include "iqs128.h"
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/
#define CAPTOUCH_TAG                  "[CAPTOUCH] "
#define CAPTOUCH_FUN(f)               printk(KERN_INFO CAPTOUCH_TAG"%s\n", __FUNCTION__)
#define CAPTOUCH_ERR(fmt, args...)    printk(KERN_ERR  CAPTOUCH_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define CAPTOUCH_LOG(fmt, args...)    printk(KERN_ERR CAPTOUCH_TAG fmt, ##args)
#define CAPTOUCH_DBG(fmt, args...)    printk(KERN_INFO CAPTOUCH_TAG fmt, ##args) 

/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);

/*----------------------------------------------------------------------------*/
#define CAPTOUCH_EINT_TOUCH	(1)
#define CAPTOUCH_EINT_NO_TOUCH	(0)
/*----------------------------------------------------------------------------*/
static struct work_struct captouch_eint_work;
int captouch_eint_status=0;

/*-----------------------------------------------------------------------------*/
void IQS128_eint_func(void)
{
	//CAPTOUCH_LOG(" debug eint function performed!\n");
	schedule_work(&captouch_eint_work);
}

/*----------------------------------------------------------------------------*/
int IQS128_setup_eint(void)
{	
	mt_set_gpio_dir(GPIO_CAPTOUCH_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_CAPTOUCH_EINT_PIN, GPIO_CAPTOUCH_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_CAPTOUCH_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_CAPTOUCH_EINT_PIN, GPIO_PULL_UP);

	mt_eint_set_hw_debounce(CUST_EINT_CAPTOUCH_NUM, CUST_EINT_CAPTOUCH_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_CAPTOUCH_NUM, CUST_EINT_CAPTOUCH_TYPE, IQS128_eint_func, 0);

	mt_eint_unmask(CUST_EINT_CAPTOUCH_NUM);
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static void IQS128_eint_work(struct work_struct *work)
{
	hwm_sensor_data sensor_data;
	int err;

	if (captouch_eint_status == CAPTOUCH_EINT_NO_TOUCH)
	{
		captouch_eint_status = CAPTOUCH_EINT_TOUCH;
		if (CUST_EINT_CAPTOUCH_TYPE == CUST_EINTF_TRIGGER_LOW)
		{
			mt_eint_set_polarity(CUST_EINT_CAPTOUCH_NUM, 1);
		}
		else
		{
			mt_eint_set_polarity(CUST_EINT_CAPTOUCH_NUM, 0);
		}
	}
	else
	{
		captouch_eint_status = CAPTOUCH_EINT_NO_TOUCH;
		if (CUST_EINT_CAPTOUCH_TYPE == CUST_EINTF_TRIGGER_LOW)
		{
			mt_eint_set_polarity(CUST_EINT_CAPTOUCH_NUM, 0);
		}
		else
		{
			mt_eint_set_polarity(CUST_EINT_CAPTOUCH_NUM, 1);
		}
	}

	sensor_data.values[0] = captouch_eint_status;
	sensor_data.value_divide = 1;
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

	//let up layer to know
	if((err = hwmsen_get_interrupt_data(ID_CAPTOUCH, &sensor_data)))
	{
		CAPTOUCH_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
	}	

	mt_eint_unmask(CUST_EINT_CAPTOUCH_NUM);
	
	return;
}
/*----------------------------------------------------------------------------*/
int IQS128_captouch_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;

	switch (command)
	{
		case SENSOR_DELAY:	
			break;

		case SENSOR_ENABLE:			
			break;

		case SENSOR_GET_DATA:
			break;
			
		default:
			CAPTOUCH_ERR("captouch sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/
static int IQS128_probe(struct platform_device *pdev) 
{
	struct hwmsen_object obj_captouch;
	int err = 0;

	CAPTOUCH_FUN();

	INIT_WORK(&captouch_eint_work, IQS128_eint_work);

	IQS128_setup_eint();

	obj_captouch.self = NULL;
	obj_captouch.polling = 0;
	obj_captouch.sensor_operate = IQS128_captouch_operate;
	if((err = hwmsen_attach(ID_CAPTOUCH, &obj_captouch)))
	{
		CAPTOUCH_ERR("attach captouch fail = %d\n", err);
		return err;
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int IQS128_remove(struct platform_device *pdev)
{
	CAPTOUCH_FUN();
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver IQS128_captouch_driver = {
	.probe      = IQS128_probe,
	.remove     = IQS128_remove,    
	.driver     = {
		.name  = "captouch",
//		.owner = THIS_MODULE,
	}
};
/*----------------------------------------------------------------------------*/
static int __init IQS128_init(void)
{
	CAPTOUCH_FUN();
	
	if(platform_driver_register(&IQS128_captouch_driver))
	{
		CAPTOUCH_ERR("failed to register captouch driver");
		return -ENODEV;
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit IQS128_exit(void)
{
	CAPTOUCH_FUN();
	
	platform_driver_unregister(&IQS128_captouch_driver);
}
/*----------------------------------------------------------------------------*/
module_init(IQS128_init);
module_exit(IQS128_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("IQS128 driver");
MODULE_LICENSE("GPL");
