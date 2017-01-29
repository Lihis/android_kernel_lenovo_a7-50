#ifdef BUILD_LK
#include <string.h>
#else
#include <linux/string.h>
#endif
 
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>
#endif
#include "lcm_drv.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)


#define LCM_DSI_CMD_MODE    0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {
    .set_gpio_out = NULL,
};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

static void lcd_power_en(unsigned char enabled)
{
	if (enabled)
	{
		mt_set_gpio_mode(GPIO_LCM_PWR, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCM_PWR, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ONE);
	}
	else
	{    
		mt_set_gpio_mode(GPIO_LCM_PWR, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCM_PWR, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ZERO);
	}
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	
	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;


	params->dsi.mode   = BURST_VDO_MODE;
	
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count=720*3;

		
	params->dsi.vertical_sync_active				= 1;
	params->dsi.vertical_backporch					= 3;
	params->dsi.vertical_frontporch					= 5;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 1;
	params->dsi.horizontal_backporch				= 48;
	params->dsi.horizontal_frontporch				= 16;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.cont_clock = 1;
	
	params->dsi.PLL_CLOCK = 230;
}

extern void DSI_clk_HS_mode(unsigned char enter);
#if !defined(BUILD_LK)
#if defined(CUSTOM_KERNEL_CAPTOUCH)
extern volatile int call_status;
#endif
#endif
static void init_lcm_registers(void)
{
    unsigned int data_array[16];
    
#ifdef BUILD_LK
    printf("%s, LK \n", __func__);
#else
    printk("%s, kernel", __func__);
#endif
    
    data_array[0] = 0x00010500;  //software reset					 
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);
    DSI_clk_HS_mode(1);
    MDELAY(80);
}

static void lcm_init(void)
{
	lcd_power_en(1);
	MDELAY(40);//Must > 5ms

	init_lcm_registers();
}

static void lcm_suspend(void)
{
#if !defined(BUILD_LK)
#if defined(CUSTOM_KERNEL_CAPTOUCH)
	if (call_status == 0)
	{
		lcd_power_en(0);
	}
#else
	lcd_power_en(0);
#endif
#else
	lcd_power_en(0);
#endif
}

static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int lcm_compare_id()
{
	unsigned int id = 1;
	lcd_power_en(1);
	MDELAY(40);//Must > 5ms
	mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_IN);
	id = mt_get_gpio_in(GPIO_LCM_RST);
#ifdef BUILD_LK
	printf("%s, LK id = %d!!!\n",__func__,id);
#else
	printk("%s, kernel id = %d!!!\n",__func__,id);
#endif
	return (1 == id)?1:0;
	
}

LCM_DRIVER auo_b070ean014_himax_lcm_drv = 
{
	.name = "AUO_B070EAN014_HIMAX",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id    = lcm_compare_id,
};
