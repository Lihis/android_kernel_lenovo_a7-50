/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or  its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/********************************************************************************************
 *     LEGAL DISCLAIMER
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED,
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION,
TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.
 ************************************************************************************************/
#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_gc2235raw.h"
#include "camera_info_gc2235raw.h"
#include "camera_custom_AEPlinetable.h"

const NVRAM_CAMERA_ISP_PARAM_STRUCT CAMERA_ISP_DEFAULT_VALUE =
{{
    //Version
    Version: NVRAM_CAMERA_PARA_FILE_VERSION,

    //SensorId
    SensorId: SENSOR_ID,
    ISPComm:{
        {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    }
    },
    ISPPca: {
        #include INCLUDE_FILENAME_ISP_PCA_PARAM
    },
    ISPRegs:{
        #include INCLUDE_FILENAME_ISP_REGS_PARAM
    },
    ISPMfbMixer:{{
        {//00: MFB mixer for ISO 100
            0x00000000, 0x00000000
        },
        {//01: MFB mixer for ISO 200
            0x00000000, 0x00000000
        },
        {//02: MFB mixer for ISO 400
            0x00000000, 0x00000000
        },
        {//03: MFB mixer for ISO 800
            0x00000000, 0x00000000
        },
        {//04: MFB mixer for ISO 1600
            0x00000000, 0x00000000
        },
        {//05: MFB mixer for ISO 2400
            0x00000000, 0x00000000
        },
        {//06: MFB mixer for ISO 3200
            0x00000000, 0x00000000
        }
    }},
    ISPCcmPoly22:{
        62925,    // i4R_AVG
        14695,    // i4R_STD
        94225,    // i4B_AVG
        28877,    // i4B_STD
        {  // i4P00[9]
            4830000, -1512500, -757500, -737500, 3592500, -295000, 0, -1657500, 4220000
        },
        {  // i4P10[9]
            1063497, -986711, -77657, -144619, 132355, 12264, -26950, 375122, -349962
        },
        {  // i4P01[9]
            220171, -305056, 88374, -119426, -126213, 245639, 4205, -80457, 74603
        },
        { // i4P20[9]
            0,  0,   0,  0,   0,  0, 0,  0,  0
        },
        { // i4P11[9]
            0,  0,   0,  0,   0,  0, 0,  0,  0
        },
        { // i4P02[9]
            0,  0,   0,  0,   0,  0, 0,  0,  0
        }        
    }
}};

const NVRAM_CAMERA_3A_STRUCT CAMERA_3A_NVRAM_DEFAULT_VALUE =
{
    NVRAM_CAMERA_3A_FILE_VERSION, // u4Version
    SENSOR_ID, // SensorId

    // AE NVRAM
    {
        // rDevicesInfo
        {
            1024,   // u4MinGain, 1024 base =  1x
            6144,  // u4MaxGain, 16x
            100,     // u4MiniISOGain, ISOxx
            128,    // u4GainStepUnit, 1x/8
            43000,     // u4PreExpUnit
            30,     // u4PreMaxFrameRate
            43000,     // u4VideoExpUnit
            30,     // u4VideoMaxFrameRate
            1024,   // u4Video2PreRatio, 1024 base = 1x
            43000,     // u4CapExpUnit
            30,     // u4CapMaxFrameRate
            1024,   // u4Cap2PreRatio, 1024 base = 1x
            28,      // u4LensFno, Fno = 2.8
            350     // u4FocusLength_100x
         },
         // rHistConfig
        {
            2,   // u4HistHighThres
            40,  // u4HistLowThres
            2,   // u4MostBrightRatio
            1,   // u4MostDarkRatio
            160, // u4CentralHighBound
            20,  // u4CentralLowBound
            {240, 230, 220, 210, 200}, // u4OverExpThres[AE_CCT_STRENGTH_NUM]
            {86, 108, 128, 148, 170},  // u4HistStretchThres[AE_CCT_STRENGTH_NUM]
            {18, 22, 26, 30, 34}       // u4BlackLightThres[AE_CCT_STRENGTH_NUM]
        },
        // rCCTConfig
        {
            TRUE,            // bEnableBlackLight
            TRUE,            // bEnableHistStretch
            FALSE,           // bEnableAntiOverExposure
            TRUE,            // bEnableTimeLPF
            TRUE,            // bEnableCaptureThres
            TRUE,            // bEnableVideoThres
            TRUE,            // bEnableStrobeThres
            38,    // u4AETarget
            47,                // u4StrobeAETarget

            20,                // u4InitIndex
            4,                 // u4BackLightWeight
            32,                // u4HistStretchWeight
            4,                 // u4AntiOverExpWeight
            2,                 // u4BlackLightStrengthIndex
            2,                 // u4HistStretchStrengthIndex
            2,                 // u4AntiOverExpStrengthIndex
            2,                 // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8}, // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM]
            90,                // u4InDoorEV = 9.0, 10 base
            -10,               // i4BVOffset delta BV = -2.3
            64,                 // u4PreviewFlareOffset
            64,                 // u4CaptureFlareOffset
            5,                 // u4CaptureFlareThres
            64,                 // u4VideoFlareOffset
            5,                 // u4VideoFlareThres
            32,                 // u4StrobeFlareOffset
            2,                 // u4StrobeFlareThres
            50,                 // u4PrvMaxFlareThres
            0,                 // u4PrvMinFlareThres
            50,                 // u4VideoMaxFlareThres
            0,                 // u4VideoMinFlareThres            
            18,                // u4FlatnessThres              // 10 base for flatness condition.
            75                 // u4FlatnessStrength
         }
    },

    // AWB NVRAM
    {
    // AWB calibration data
    {
        // rUnitGain (unit gain: 1.0 = 512)
         {
             0,  // i4R
             0,  // i4G
             0   // i4B
         },
         // rGoldenGain (golden sample gain: 1.0 = 512)
         {
                0,    // i4R
                0,    // i4G
                0 // i4B
            },
         // rTuningUnitGain (Tuning sample unit gain: 1.0 = 512)
         {
                0,    // i4R
                0,    // i4G
                0 // i4B
            },
            // rD65Gain (D65 WB gain: 1.0 = 512)
            {
                772,    // i4R
                512,    // i4G
                531    // i4B
            }
        },
        // Original XY coordinate of AWB light source
        {
           // Strobe
            {
                0,   // i4X
                0    // i4Y
            },
            // Horizon
            {
                -521,    // i4X
                -274    // i4Y
            },
            // A
            {
                -363,    // i4X
                -258    // i4Y
            },
            // TL84
            {
                -163,    // i4X
                -278    // i4Y
            },
            // CWF
            {
                -97,    // i4X
                -369    // i4Y
            },
            // DNP
            {
                3,    // i4X
                -191    // i4Y
            },
            // D65
            {
                138,    // i4X
                -165    // i4Y
            },
            // DF
            {
                4,    // i4X
                -191    // i4Y
            }
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                0,    // i4X
                0    // i4Y
            },
            // Horizon
            {
                -563,    // i4X
                -169    // i4Y
            },
            // A
            {
                -405,    // i4X
                -183    // i4Y
            },
            // TL84
            {
                -213,    // i4X
                -241    // i4Y
            },
            // CWF
            {
                -166,    // i4X
                -343    // i4Y
            },
            // DNP
            {
                -34,    // i4X
                -188    // i4Y
            },
            // D65
            {
                104,    // i4X
                -188    // i4Y
            },
            // DF
            {
                -33,    // i4X
                -188    // i4Y
            }
        },
        // AWB gain of AWB light source
        {
            // Strobe 
            {
                512,    // i4R
                512,    // i4G
                512    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                715,    // i4G
                2100    // i4B
            },
            // A 
            {
                512,    // i4R
                590,    // i4G
                1369    // i4B
            },
            // TL84 
            {
                598,    // i4R
                512,    // i4G
                931    // i4B
            },
            // CWF 
            {
                740,    // i4R
                512,    // i4G
                961    // i4B
            },
            // DNP 
            {
                666,    // i4R
                512,    // i4G
                660    // i4B
            },
            // D65 
            {
                772,    // i4R
                512,    // i4G
                531    // i4B
            },
            // DF 
            {
                667,    // i4R
                512,    // i4G
                659    // i4B
            }
        },
        // Rotation matrix parameter
        {
            11,    // i4RotationAngle
            251,    // i4Cos
            49    // i4Sin
        },
        // Daylight locus parameter
        {
            -189,    // i4SlopeNumerator
            128    // i4SlopeDenominator
        },
        // AWB light area
        {
            // Strobe:FIXME
            {
            0,    // i4RightBound
            0,    // i4LeftBound
            0,    // i4UpperBound
            0    // i4LowerBound
            },
            // Tungsten
            {
            -263,    // i4RightBound
            -913,    // i4LeftBound
            -126,    // i4UpperBound
            -226    // i4LowerBound
            },
            // Warm fluorescent
            {
            -263,    // i4RightBound
            -913,    // i4LeftBound
            -226,    // i4UpperBound
            -346    // i4LowerBound
            },
            // Fluorescent
            {
            -84,    // i4RightBound
            -263,    // i4LeftBound
            -117,    // i4UpperBound
            -292    // i4LowerBound
            },
            // CWF
            {
            -84,    // i4RightBound
            -263,    // i4LeftBound
            -292,    // i4UpperBound
            -480    // i4LowerBound
            },
            // Daylight
            {
            129,    // i4RightBound
            -84,    // i4LeftBound
            -108,    // i4UpperBound
            -268    // i4LowerBound
            },
            // Shade
            {
            489,    // i4RightBound
            129,    // i4LeftBound
            -108,    // i4UpperBound
            -268    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            129,    // i4RightBound
            -84,    // i4LeftBound
            -268,    // i4UpperBound
            -400    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            489,    // i4RightBound
            -913,    // i4LeftBound
            0,    // i4UpperBound
            -480    // i4LowerBound
            },
            // Daylight
            {
            154,    // i4RightBound
            -84,    // i4LeftBound
            -108,    // i4UpperBound
            -268    // i4LowerBound
            },
            // Cloudy daylight
            {
            254,    // i4RightBound
            79,    // i4LeftBound
            -108,    // i4UpperBound
            -268    // i4LowerBound
            },
            // Shade
            {
            354,    // i4RightBound
            79,    // i4LeftBound
            -108,    // i4UpperBound
            -268    // i4LowerBound
            },
            // Twilight
            {
            -84,    // i4RightBound
            -244,    // i4LeftBound
            -108,    // i4UpperBound
            -268    // i4LowerBound
            },
            // Fluorescent
            {
            154,    // i4RightBound
            -313,    // i4LeftBound
            -138,    // i4UpperBound
            -393    // i4LowerBound
            },
            // Warm fluorescent
            {
            -305,    // i4RightBound
            -505,    // i4LeftBound
            -138,    // i4UpperBound
            -393    // i4LowerBound
            },
            // Incandescent
            {
            -305,    // i4RightBound
            -505,    // i4LeftBound
            -108,    // i4UpperBound
            -268    // i4LowerBound
            },
            // Gray World
            {
            5000,    // i4RightBound
            -5000,    // i4LeftBound
            5000,    // i4UpperBound
            -5000    // i4LowerBound
            }
        },
        // PWB default gain	
        {
            // Daylight
            {
            717,    // i4R
            512,    // i4G
            592    // i4B
            },
            // Cloudy daylight
            {
            825,    // i4R
            512,    // i4G
            481    // i4B
            },
            // Shade
            {
            870,    // i4R
            512,    // i4G
            444    // i4B
            },
            // Twilight
            {
            579,    // i4R
            512,    // i4G
            813    // i4B
            },
            // Fluorescent
            {
            717,    // i4R
            512,    // i4G
            772    // i4B
            },
            // Warm fluorescent
            {
            506,    // i4R
            512,    // i4G
            1295    // i4B
            },
            // Incandescent
            {
            447,    // i4R
            512,    // i4G
            1192    // i4B
            },
            // Gray World
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        // AWB preference color	
        {
            // Tungsten
            {
            0,    // i4SliderValue
            8381    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            0,    // i4SliderValue
            6095    // i4OffsetThr
            },
            // Shade
            {
            0,    // i4SliderValue
            1347    // i4OffsetThr
            },
            // Daylight WB gain
            {
            666,    // i4R
            512,    // i4G
            661    // i4B
            },
            // Preference gain: strobe
            {
            512,    // i4R
            512,    // i4G
            512 // i4B
        },
        // Preference gain: tungsten
        {
            512,    // i4R
            512,    // i4G
            512     // i4B
        },
        // Preference gain: warm fluorescent
        {
            512,    // i4R
            512,    // i4G
            512     // i4B
        },
        // Preference gain: fluorescent
        {
            512,    // i4R
            512,    // i4G
            512     // i4B
        },
        // Preference gain: CWF
        {
            512,    // i4R
            512,    // i4G
            512     // i4B
        },
        // Preference gain: daylight
        {
            512,    // i4R
            512,    // i4G
            512     // i4B
        },
        // Preference gain: shade
        {
            512,    // i4R
            512,    // i4G
            512     // i4B
        },
        // Preference gain: daylight fluorescent
        {
            512,    // i4R
            512,    // i4G
            512     // i4B
         }
    },
        {// CCT estimation
            {// CCT
                2300,    // i4CCT[0]
             2850,   // i4CCT[1]
             4100,   // i4CCT[2]
             5100,   // i4CCT[3]
             6500    // i4CCT[4]
         },
            {// Rotated X coordinate
                -667,    // i4RotatedXCoordinate[0]
                -509,    // i4RotatedXCoordinate[1]
                -317,    // i4RotatedXCoordinate[2]
                -138,    // i4RotatedXCoordinate[3]
                0    // i4RotatedXCoordinate[4]
            }
        }
    },
    {0}
};
 
#include INCLUDE_FILENAME_ISP_LSC_PARAM
//};  //  namespace
const CAMERA_TSF_TBL_STRUCT CAMERA_TSF_DEFAULT_VALUE =
{
    #include "camera_tsf_para_gc2235raw.h"
    #include "camera_tsf_data_gc2235raw.h"
};


typedef NSFeature::RAWSensorInfo<SENSOR_ID> SensorInfoSingleton_T;


namespace NSFeature {
template <>
UINT32
SensorInfoSingleton_T::
impGetDefaultData(CAMERA_DATA_TYPE_ENUM const CameraDataType, VOID*const pDataBuf, UINT32 const size) const
{
    UINT32 dataSize[CAMERA_DATA_TYPE_NUM] = {sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT),
                                             sizeof(NVRAM_CAMERA_3A_STRUCT),
                                             sizeof(NVRAM_CAMERA_SHADING_STRUCT),
                                             sizeof(NVRAM_LENS_PARA_STRUCT),
                                             sizeof(AE_PLINETABLE_T),
                                             0,
                                             sizeof(CAMERA_TSF_TBL_STRUCT)};

    if (CameraDataType > CAMERA_DATA_TSF_TABLE || NULL == pDataBuf || (size < dataSize[CameraDataType]))
    {
        return 1;
    }

    switch(CameraDataType)
    {
        case CAMERA_NVRAM_DATA_ISP:
            memcpy(pDataBuf,&CAMERA_ISP_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_3A:
            memcpy(pDataBuf,&CAMERA_3A_NVRAM_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_3A_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_SHADING:
            memcpy(pDataBuf,&CAMERA_SHADING_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_SHADING_STRUCT));
            break;
        case CAMERA_DATA_AE_PLINETABLE:
            memcpy(pDataBuf,&g_PlineTableMapping,sizeof(AE_PLINETABLE_T));
            break;
        case CAMERA_DATA_TSF_TABLE:
            memcpy(pDataBuf,&CAMERA_TSF_DEFAULT_VALUE,sizeof(CAMERA_TSF_TBL_STRUCT));
            break;
        default:
            break;
    }
    return 0;
}};  //  NSFeature


