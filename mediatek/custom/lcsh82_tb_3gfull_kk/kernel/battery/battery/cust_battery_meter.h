#ifndef _CUST_BATTERY_METER_H
#define _CUST_BATTERY_METER_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
//#define SOC_BY_AUXADC
//#define SOC_BY_HW_FG
#define SOC_BY_SW_FG

//#define CONFIG_DIS_CHECK_BATTERY
//#define FIXED_TBAT_25

#if defined(SLT_DRV_P3550_CONFIG)
#define MTK_MULTI_BAT_PROFILE_SUPPORT
#define MTK_GET_BATTERY_ID_BY_AUXADC
#endif

#if defined(MTK_NCP1851_SUPPORT) \
    || defined(MTK_BQ24196_SUPPORT)\
    || defined(MTK_NCP1854_SUPPORT) \
    || defined(MTK_BQ24296_SUPPORT)
#define BAT_VOL_USE_ISENSE
#define SWCHR_POWER_PATH //for hw ocv use
#endif
/* ADC Channel Number */
#define CUST_TABT_NUMBER 17
#if defined(BAT_VOL_USE_ISENSE)
#define VBAT_CHANNEL_NUMBER      6
#define ISENSE_CHANNEL_NUMBER	 7
#else
#define VBAT_CHANNEL_NUMBER      7
#define ISENSE_CHANNEL_NUMBER	 6
#endif
#define VCHARGER_CHANNEL_NUMBER  4
#define VBATTEMP_CHANNEL_NUMBER  5

/* ADC resistor  */
#define R_BAT_SENSE 4					
#define R_I_SENSE 4						
#define R_CHARGER_1 330
#define R_CHARGER_2 39

#define TEMPERATURE_T0             110
#define TEMPERATURE_T1             0
#define TEMPERATURE_T2             25
#define TEMPERATURE_T3             50
#define TEMPERATURE_T              255 // This should be fixed, never change the value

#define FG_METER_RESISTANCE 	0

/* For multi-battery support */
//#define MTK_MULTI_BAT_PROFILE_SUPPORT
//#define MTK_GET_BATTERY_ID_BY_AUXADC
//#define BATTERY_ID_CHANNEL_NUM 3
//#define TOTAL_BATTERY_NUMBER 3
//#define MTK_GET_BATTERY_ID_BY_GPIO

#if defined(MTK_MULTI_BAT_PROFILE_SUPPORT)
/* Qmax for battery  */
#define Q_MAX_POS_50_COS	3517
#define Q_MAX_POS_25_COS	3528
#define Q_MAX_POS_0_COS		3537
#define Q_MAX_NEG_10_COS	3469

#define Q_MAX_POS_50_H_CURRENT_COS	3484
#define Q_MAX_POS_25_H_CURRENT_COS	3480
#define Q_MAX_POS_0_H_CURRENT_COS	3194
#define Q_MAX_NEG_10_H_CURRENT_COS	1974

/* Qmax for battery  */
#define Q_MAX_POS_50_ATL	3559
#define Q_MAX_POS_25_ATL	3584
#define Q_MAX_POS_0_ATL		3484
#define Q_MAX_NEG_10_ATL	3441

#define Q_MAX_POS_50_H_CURRENT_ATL	3520
#define Q_MAX_POS_25_H_CURRENT_ATL	3560
#define Q_MAX_POS_0_H_CURRENT_ATL	3301
#define Q_MAX_NEG_10_H_CURRENT_ATL	2736
#else
/* Qmax for battery  */
#define Q_MAX_POS_50	3895
#define Q_MAX_POS_25	3895
#define Q_MAX_POS_0		3895
#define Q_MAX_NEG_10	3895

#define Q_MAX_POS_50_H_CURRENT	3865
#define Q_MAX_POS_25_H_CURRENT	3865
#define Q_MAX_POS_0_H_CURRENT	3865
#define Q_MAX_NEG_10_H_CURRENT	3865
#endif

#if defined(SLT_DRV_P3550_CONFIG)
/* Discharge Percentage */
#define OAM_D5		 0		//  1 : D5,   0: D2
#else
/* Discharge Percentage */
#define OAM_D5		 1		//  1 : D5,   0: D2
#endif


/* battery meter parameter */
#define CUST_TRACKING_POINT  14
#ifdef MTK_FAN5405_SUPPORT
#define CUST_R_SENSE         56
#elif defined(MTK_NCP1851_SUPPORT)
#define CUST_R_SENSE         68
#elif defined(MTK_NCP1854_SUPPORT)
#define CUST_R_SENSE         56
#else
#define CUST_R_SENSE         200
#endif
#define CUST_HW_CC 		    0
#define AGING_TUNING_VALUE   103
#define CUST_R_FG_OFFSET    0

#define OCV_BOARD_COMPESATE	0 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
#define CAR_TUNE_VALUE		94 //1.00


/* HW Fuel gague  */
#define CURRENT_DETECT_R_FG	10  //1mA
#define MinErrorOffset       1000
#define FG_VBAT_AVERAGE_SIZE 18
#define R_FG_VALUE 			0 // mOhm, base is 20

/* Power on capacity */
#define CUST_POWERON_DELTA_CAPACITY_TOLRANCE	40
#define CUST_POWERON_LOW_CAPACITY_TOLRANCE		5
#define CUST_POWERON_MAX_VBAT_TOLRANCE			90
#define CUST_POWERON_DELTA_VBAT_TOLRANCE		30

/* Disable Battery check for HQA */
#ifdef MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
#define FIXED_TBAT_25
#endif

/* Dynamic change wake up period of battery thread when suspend*/
#define VBAT_NORMAL_WAKEUP		3600		//3.6V
#define VBAT_LOW_POWER_WAKEUP		3500		//3.5v
#define NORMAL_WAKEUP_PERIOD		5400 		//90 * 60 = 90 min
#define LOW_POWER_WAKEUP_PERIOD		300		//5 * 60 = 5 min
#define CLOSE_POWEROFF_WAKEUP_PERIOD	30	//30 s

#endif	//#ifndef _CUST_BATTERY_METER_H
