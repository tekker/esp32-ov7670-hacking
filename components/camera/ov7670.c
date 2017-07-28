/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV7670 driver.
 *
 */
 #include <stdint.h>
 #include <stdlib.h>
 #include <string.h>
#include "sccb.h"
#include "ov7670.h"
#include "ov7670_regs.h"
#include <stdio.h>


static const uint8_t default_regs[][2] = {
    {COM7, COM7_RESET},
	{TSLB,  0x04},	/* OV */
	{COM7, 0},	/* VGA */
	{CLKRC, 0x01},
	/*
	 * Set the hardware window.  These values from OV don't entirely
	 * make sense - hstop is less than hstart.  But they work...
	 */
	{HSTART, 0x13},	{HSTOP, 0x01},
	{HREF, 0xb6},	{VSTART, 0x02},
	{VSTOP, 0x7a},	{VREF, 0x0a},

	{COM3, 0},	{COM14, 0},
	/* Mystery scaling numbers */
	{SCALING_XSC, 0x3a},		{SCALING_YSC, 0x35},
	{SCALING_DCWCTR, 0x11},		{SCALING_PCLK_DIV, 0xf0},
	{SCALING_PCLK_DELAY,/* 0x02 changed to 1*/1},
	{COM10, COM10_VSYNC_NEG},
	/* Gamma curve values */
	{0x7a, 0x20},		{0x7b, 0x10},
	{0x7c, 0x1e},		{0x7d, 0x35},
	{0x7e, 0x5a},		{0x7f, 0x69},
	{0x80, 0x76},		{0x81, 0x80},
	{0x82, 0x88},		{0x83, 0x8f},
	{0x84, 0x96},		{0x85, 0xa3},
	{0x86, 0xaf},		{0x87, 0xc4},
	{0x88, 0xd7},		{0x89, 0xe8},
	/* AGC and AEC parameters.  Note we start by disabling those features,
	   then turn them only after tweaking the values. */
	{COM8, COM8_FAST_AUTO | COM8_STEP_UNLIMIT},
	{GAIN, 0},	{AECH, 0},
	{COM4, 0x40}, /* magic reserved bit */
// TEKKER MOD
//	{COM9, 0x18}, /* 4x gain + magic rsvd bit */
  { COM9, 0x6A }, /* 128x gain ceiling; 0x8 is reserved bit */
	{BD50MAX, 0x05},	{BD60MAX, 0x07},
	{AEW, 0x95},	{AEB, 0x33},
	{VPT, 0xe3},	{HAECC1, 0x78},
	{HAECC2, 0x68},	{0xa1, 0x03}, /* magic */
	{HAECC3, 0xd8},	{HAECC4, 0xd8},
	{HAECC5, 0xf0},	{HAECC6, 0x90},
	{HAECC7, 0x94},
	{COM8, COM8_FAST_AUTO|COM8_STEP_UNLIMIT|COM8_AGC_EN|COM8_AEC_EN},
	{0x30,0},{0x31,0},//disable some delays
	/* Almost all of these are magic "reserved" values.  */
	{COM5, 0x61},	{COM6, 0x4b},
	{0x16, 0x02},		{MVFP, 0x07},
	{0x21, 0x02},		{0x22, 0x91},
	{0x29, 0x07},		{0x33, 0x0b},
	{0x35, 0x0b},		{0x37, 0x1d},
	{0x38, 0x71},		{0x39, 0x2a},
	{COM12, 0x78},	{0x4d, 0x40},
	{0x4e, 0x20},		{GFIX, 0},
	/*{0x6b, 0x4a},*/		{0x74,0x10},
	{0x8d, 0x4f},		{0x8e, 0},
	{0x8f, 0},		{0x90, 0},
	{0x91, 0},		{0x96, 0},
	{0x9a, 0},		{0xb0, 0x84},
	{0xb1, 0x0c},		{0xb2, 0x0e},
	{0xb3, 0x82},		{0xb8, 0x0a},

	/* More reserved magic, some of which tweaks white balance */
	{0x43, 0x0a},		{0x44, 0xf0},
	{0x45, 0x34},		{0x46, 0x58},
	{0x47, 0x28},		{0x48, 0x3a},
	{0x59, 0x88},		{0x5a, 0x88},
	{0x5b, 0x44},		{0x5c, 0x67},
	{0x5d, 0x49},		{0x5e, 0x0e},
	{0x6c, 0x0a},		{0x6d, 0x55},
	{0x6e, 0x11},		{0x6f, 0x9e}, /* it was 0x9F "9e for advance AWB" */
	{0x6a, 0x40},		{BLUE, 0x40},
	{RED, 0x60},
	{COM8, COM8_FAST_AUTO|COM8_STEP_UNLIMIT|COM8_AGC_EN|COM8_AEC_EN|COM8_AWB_EN},

	/* Matrix coefficients */
	{0x4f, 0x80},		{0x50, 0x80},
	{0x51, 0},		{0x52, 0x22},
	{0x53, 0x5e},		{0x54, 0x80},
	{0x58, 0x9e},

	{COM16, COM16_AWBGAIN},	{EDGE, 0},
	{0x75, 0x05},		{REG76, 0xe1},
	{0x4c, 0},		{0x77, 0x01},
	{COM13, /*0xc3*/0x48},	{0x4b, 0x09},
	{0xc9, 0x60},		/*{COM16, 0x38},*/
	{0x56, 0x40},

	{0x34, 0x11},		{COM11, COM11_EXP|COM11_HZAUTO},
	{0xa4, 0x82/*Was 0x88*/},		{0x96, 0},
	{0x97, 0x30},		{0x98, 0x20},
	{0x99, 0x30},		{0x9a, 0x84},
	{0x9b, 0x29},		{0x9c, 0x03},
	{0x9d, 0x4c},		{0x9e, 0x3f},
	{0x78, 0x04},

	/* Extra-weird stuff.  Some sort of multiplexor register */
	{0x79, 0x01},		{0xc8, 0xf0},
	{0x79, 0x0f},		{0xc8, 0x00},
	{0x79, 0x10},		{0xc8, 0x7e},
	{0x79, 0x0a},		{0xc8, 0x80},
	{0x79, 0x0b},		{0xc8, 0x01},
	{0x79, 0x0c},		{0xc8, 0x0f},
	{0x79, 0x0d},		{0xc8, 0x20},
	{0x79, 0x09},		{0xc8, 0x80},
	{0x79, 0x02},		{0xc8, 0xc0},
	{0x79, 0x03},		{0xc8, 0x40},
	{0x79, 0x05},		{0xc8, 0x30},
	{0x79, 0x26},

//  {UNDOC_COLOR_CORRECTION, 0x8C}, // Undocumented color correction, set earlier to 0x84 but test this...
	{0xff, 0xff},	/* END MARKER */
};

static const uint8_t VGA_regs[][2] = {
	{COM3, 0x00},
	{COM14, 0x00},
	{SCALING_XSC, 0x3A},
	{SCALING_YSC, 0x35},
	{SCALING_DCWCTR, 0x11},
	{SCALING_PCLK_DIV, 0xF0},
	{SCALING_PCLK_DELAY, 0x02}
};

static const uint8_t QVGA_regs[][2] = {
	{COM3, 0x04},
	{COM14, 0x19},
	{SCALING_XSC, 0x3A},
	{SCALING_YSC, 0x35},
	{SCALING_DCWCTR, 0x11},
	{SCALING_PCLK_DIV, 0xF1},
	{SCALING_PCLK_DELAY, 0x02}
};

static const uint8_t QQVGA_regs[][2] = {
	{COM3, 0x04},
	{COM14, 0x1A},
	{SCALING_XSC, 0x3A},
	{SCALING_YSC, 0x35},
	{SCALING_DCWCTR, 0x22},
	{SCALING_PCLK_DIV, 0xF2},
	{SCALING_PCLK_DELAY, 0x02}
};

#define NUM_BRIGHTNESS_LEVELS (9)
static const uint8_t brightness_regs[NUM_BRIGHTNESS_LEVELS][2] = {
    {0x38, 0x0e}, /* -4 */
    {0x28, 0x0e}, /* -3 */
    {0x18, 0x0e}, /* -2 */
    {0x08, 0x0e}, /* -1 */
    {0x08, 0x06}, /*  0 */
    {0x18, 0x06}, /* +1 */
    {0x28, 0x06}, /* +2 */
    {0x38, 0x06}, /* +3 */
    {0x48, 0x06}, /* +4 */
};

#define NUM_CONTRAST_LEVELS (9)
static const uint8_t contrast_regs[NUM_CONTRAST_LEVELS][1] = {
    {0x10}, /* -4 */
    {0x14}, /* -3 */
    {0x18}, /* -2 */
    {0x1C}, /* -1 */
    {0x20}, /*  0 */
    {0x24}, /* +1 */
    {0x28}, /* +2 */
    {0x2C}, /* +3 */
    {0x30}, /* +4 */
};

#define NUM_SATURATION_LEVELS (9)
static const uint8_t saturation_regs[NUM_SATURATION_LEVELS][2] = {
    {0x00, 0x00}, /* -4 */
    {0x10, 0x10}, /* -3 */
    {0x20, 0x20}, /* -2 */
    {0x30, 0x30}, /* -1 */
    {0x40, 0x40}, /*  0 */
    {0x50, 0x50}, /* +1 */
    {0x60, 0x60}, /* +2 */
    {0x70, 0x70}, /* +3 */
    {0x80, 0x80}, /* +4 */
};

/* FREESCALE OV7670 Driver functions */
#include "ov7670_def.h"

typedef struct ov7670_frame_rate_cfg
{
  uint8_t       clkrc;
  uint8_t       dblv;
  uint8_t       exhch;
  uint8_t       exhcl;
  uint8_t       dm_lnl;
  uint8_t       dm_lnh;
} ov7670_frame_rate_cfg_t;


/*
const ov7670_frame_rate_cfg_t OV7670_30FPS_26MHZ_XCLK = {0x80, 0x0a, 0x00, 0x00, 0x2b, 0x00};
const ov7670_frame_rate_cfg_t OV7670_25FPS_26MHZ_XCLK = {0x80, 0x0a, 0x00, 0x00, 0x99, 0x00};
const ov7670_frame_rate_cfg_t OV7670_15FPS_26MHZ_XCLK = {0x00, 0x0a, 0x00, 0x00, 0x2b, 0x00};
const ov7670_frame_rate_cfg_t OV7670_14FPS_26MHZ_XCLK = {0x00, 0x0a, 0x00, 0x00, 0x46, 0x00};
*/
const ov7670_frame_rate_cfg_t OV7670_30FPS_24MHZ_XCLK = {0x80, 0x0a, 0x00, 0x00, 0x00, 0x00};
const ov7670_frame_rate_cfg_t OV7670_25FPS_24MHZ_XCLK = {0x80, 0x0a, 0x00, 0x00, 0x66, 0x00};
const ov7670_frame_rate_cfg_t OV7670_15FPS_24MHZ_XCLK = {0x00, 0x0a, 0x00, 0x00, 0x00, 0x00};
const ov7670_frame_rate_cfg_t OV7670_14FPS_24MHZ_XCLK = {0x00, 0x0a, 0x00, 0x00, 0x1a, 0x00};
/*
const ov7670_frame_rate_cfg_t OV7670_30FPS_13MHZ_XCLK = {0x00, 0x4a, 0x00, 0x00, 0x2b, 0x00};
const ov7670_frame_rate_cfg_t OV7670_25FPS_13MHZ_XCLK = {0x00, 0x4a, 0x00, 0x00, 0x99, 0x00};
const ov7670_frame_rate_cfg_t OV7670_15FPS_13MHZ_XCLK = {0x01, 0x4a, 0x00, 0x00, 0x2b, 0x00};
const ov7670_frame_rate_cfg_t OV7670_14FPS_13MHZ_XCLK = {0x01, 0x4a, 0x00, 0x00, 0x46, 0x00};
*/
const ov7670_frame_rate_cfg_t OV7670_30FPS_12MHZ_XCLK = {0x00, 0x4a, 0x00, 0x00, 0x2b, 0x00};
const ov7670_frame_rate_cfg_t OV7670_25FPS_12MHZ_XCLK = {0x00, 0x4a, 0x00, 0x00, 0x66, 0x00};
const ov7670_frame_rate_cfg_t OV7670_15FPS_12MHZ_XCLK = {0x01, 0x4a, 0x00, 0x00, 0x2b, 0x00};
const ov7670_frame_rate_cfg_t OV7670_14FPS_12MHZ_XCLK = {0x01, 0x4a, 0x00, 0x00, 0x46, 0x00};


// @brief Night mode configuration data structure
typedef struct ov7670_night_mode_cfg
{
  uint8_t       com11;
} ov7670_night_mode_cfg_t;
// end of night mode configuration data structure

// @brief Banding filter selection data structure
typedef struct ov7670_filter_cfg
{
  uint8_t       com8;
  uint8_t       bd50st;
  uint8_t       bd60st;
  uint8_t       bd50max;
  uint8_t       bd60max;
  uint8_t       com11;
} ov7670_filter_cfg_t;
// end of Banding filter selection data structure

// @brief White balance configuration structure
typedef struct ov7670_white_balance_cfg
{
  uint8_t       com8;
  uint8_t       awbctr0;
  uint8_t       awbctr1;
  uint8_t       awbctr2;
  uint8_t       awbctr3;
  uint8_t       awbc1;
  uint8_t       awbc2;
  uint8_t       awbc3;
  uint8_t       awbc4;
  uint8_t       awbc5;
  uint8_t       awbc6;
  uint8_t       com16;
} ov7670_white_balance_cfg_t;
// end of White balance configuration structure

// @brief Light mode configuration structure
typedef struct ov7670_light_mode_cfg
{
  uint8_t       com8;
  uint8_t       com9;
  uint8_t       red;
  uint8_t       green;
  uint8_t       blue;
} ov7670_light_mode_cfg_t;
// end of Light mode configuration structure

// @brief Color saturation configuration structure
typedef struct ov7670_color_saturation_cfg
{
  uint8_t       mtx1;
  uint8_t       mtx2;
  uint8_t       mtx3;
  uint8_t       mtx4;
  uint8_t       mtx5;
  uint8_t       mtx6;
  uint8_t       mtxs;
  uint8_t       com16;
} ov7670_color_saturation_cfg_t;
// end of Color saturation configuration structure

// @brief Special effects configuration structure
typedef struct ov7670_special_effect_cfg
{
  uint8_t       tslb;
  uint8_t       manu;
  uint8_t       manv;
} ov7670_special_effect_cfg_t;
// end of Special effects configuration structure

// @brief Windowing configuration structure
typedef struct ov7670_windowing_cfg
{
  uint8_t       href;
  uint8_t       hstart;
  uint8_t       hstop;
  uint8_t       vref;
  uint8_t       vstart;
  uint8_t       vstop;
} ov7670_windowing_cfg_t;

// @brief Gamma curve slope configuration structure
typedef struct ov7670_gamma_curve_slope_cfg
{
  uint8_t       slope;
  uint8_t       gam1;
  uint8_t       gam2;
  uint8_t       gam3;
  uint8_t       gam4;
  uint8_t       gam5;
  uint8_t       gam6;
  uint8_t       gam7;
  uint8_t       gam8;
  uint8_t       gam9;
  uint8_t       gam10;
  uint8_t       gam11;
  uint8_t       gam12;
  uint8_t       gam13;
  uint8_t       gam14;
  uint8_t       gam15;
} ov7670_gamma_curve_slope_cfg_t;
// end of Gamma curve slope  configuration structure

//
const ov7670_night_mode_cfg_t OV7670_NIGHT_MODE_DISABLED = {0x00};
const ov7670_night_mode_cfg_t OV7670_NIGHT_MODE_AUTO_FR_DIVBY2 = {0xa0};
const ov7670_night_mode_cfg_t OV7670_NIGHT_MODE_AUTO_FR_DIVBY4 = {0xc0};
const ov7670_night_mode_cfg_t OV7670_NIGHT_MODE_AUTO_FR_DIVBY8 = {0xe0};


// @brief White balance initialization structure data
const ov7670_white_balance_cfg_t OV7670_WHITE_BALANCE_DEFAULT =  {0x02, 0x9a, 0xc0, 0x55, 0x02, 0x14,  \
                                                                  0xf0, 0x45, 0x61, 0x51, 0x79, 0x08};
const ov7670_white_balance_cfg_t OV7670_WHITE_BALANCE_DISABLED = {0x00, 0x9a, 0xc0, 0x55, 0x02, 0x14, \
                                                                  0xf0, 0x45, 0x61, 0x51, 0x79, 0x00};
const ov7670_white_balance_cfg_t OV7670_WHITE_BALANCE_SIMPLE =   {0x02, 0x9f, 0x10, 0x55, 0x02, 0x14,   \
                                                                  0xf0, 0x45, 0x61, 0x51, 0x79, 0x08};

// @brief Light mode configuration initialization structure data
const ov7670_light_mode_cfg_t OV7670_LIGHT_MODE_DISABLED = {0x05, 0x0a, 0x08, 0x00, 0x08};
const ov7670_light_mode_cfg_t OV7670_LIGHT_MODE_AUTO =     {0xc5, 0x3a, 0x08, 0x00, 0x08};
const ov7670_light_mode_cfg_t OV7670_LIGHT_MODE_SUNNY =    {0xc5, 0x6a, 0x5a, 0x00, 0x5c};
const ov7670_light_mode_cfg_t OV7670_LIGHT_MODE_CLOUDY =   {0xc5, 0x0a, 0x58, 0x00, 0x60};
const ov7670_light_mode_cfg_t OV7670_LIGHT_MODE_OFFICE =   {0xc5, 0x2a, 0x84, 0x00, 0x4c};
const ov7670_light_mode_cfg_t OV7670_LIGHT_MODE_HOME =     {0xc5, 0x1a, 0x96, 0x00, 0x40};

/*
// @brief Color saturation configuration initialization structure data
const ov7670_color_saturation_cfg_t OV7670_COLOR_SATURATION_4PLUS =  {0xc0, 0xc0, 0x00, 0x33, 0x8d, 0xc0, 0x9e, 0x02};
const ov7670_color_saturation_cfg_t OV7670_COLOR_SATURATION_3PLUS =  {0x99, 0x99, 0x00, 0x28, 0x71, 0x99, 0x9e, 0x02};
const ov7670_color_saturation_cfg_t OV7670_COLOR_SATURATION_2PLUS =  {0xc0, 0xc0, 0x00, 0x33, 0x8d, 0xc0, 0x9e, 0x00};
const ov7670_color_saturation_cfg_t OV7670_COLOR_SATURATION_1PLUS =  {0x99, 0x99, 0x00, 0x28, 0x71, 0x99, 0x9e, 0x00};
const ov7670_color_saturation_cfg_t OV7670_COLOR_SATURATION_0 =      {0x80, 0x80, 0x00, 0x22, 0x5e, 0x80, 0x9e, 0x00};
const ov7670_color_saturation_cfg_t OV7670_COLOR_SATURATION_DEFAULT ={0x40, 0x34, 0x0c, 0x17, 0x29, 0x40, 0x1e, 0x00};
const ov7670_color_saturation_cfg_t OV7670_COLOR_SATURATION_1MINUS = {0x66, 0x66, 0x00, 0x1b, 0x4b, 0x66, 0x9e, 0x00};
const ov7670_color_saturation_cfg_t OV7670_COLOR_SATURATION_2MINUS = {0x40, 0x40, 0x00, 0x11, 0x2f, 0x40, 0x9e, 0x00};
*/

// @brief Special effects configuration initialization structure data
const ov7670_special_effect_cfg_t OV7670_SPECIAL_EFFECT_ANTIQUE =     {0x18, 0, 255};
const ov7670_special_effect_cfg_t OV7670_SPECIAL_EFFECT_SEPHIA =      {0x18, 16, 146};
const ov7670_special_effect_cfg_t OV7670_SPECIAL_EFFECT_BLUISH =      {0x18, 240, 146};
const ov7670_special_effect_cfg_t OV7670_SPECIAL_EFFECT_GREENISH =    {0x18, 0, 30};
const ov7670_special_effect_cfg_t OV7670_SPECIAL_EFFECT_REDISH =      {0x18, 90, 240};
const ov7670_special_effect_cfg_t OV7670_SPECIAL_EFFECT_BW =          {0x18, 110, 110};
const ov7670_special_effect_cfg_t OV7670_SPECIAL_EFFECT_NEGATIVE =    {0x28, 0x80, 0x80};
const ov7670_special_effect_cfg_t OV7670_SPECIAL_EFFECT_BW_NEGATIVE = {0x38, 110, 110};
const ov7670_special_effect_cfg_t OV7670_SPECIAL_EFFECT_NORMAL =      {0x0c, 0x80, 0x80};
const ov7670_special_effect_cfg_t OV7670_SPECIAL_EFFECT_DISABLED =    {0x08, 0x80, 0x80};

// @brief Special effects configuration initialization structure data
const ov7670_gamma_curve_slope_cfg_t OV7670_GAMMA_CURVE_SLOPE_DEFAULT   = {0x24, 0x04, 0x07, 0x10, 0x28, 0x36, 0x44, 0x52, 0x60, 0x6c, 0x78, 0x8c, 0x9e, 0xbb, 0xd2, 0xe5};
const ov7670_gamma_curve_slope_cfg_t OV7670_GAMMA_CURVE_SLOPE1          = {0x20, 0x10, 0x1e, 0x35, 0x5a, 0x69, 0x76, 0x80, 0x88, 0x8f, 0x96, 0xa3, 0xaf, 0xc4, 0xd7, 0xe8};

/*
// Banding filter initialization structure data
const ov7670_filter_cfg_t OV7670_FILTER_DISABLED = {0x00, 0x98, 0x7f, 0x02, 0x03, 0x02};
const ov7670_filter_cfg_t OV7670_FILTER_30FPS_60HZ = {0x20, 0x98, 0x7f, 0x02, 0x03, 0x02};
const ov7670_filter_cfg_t OV7670_FILTER_15FPS_60HZ = {0x20, 0x4c, 0x3f, 0x05, 0x07, 0x02};
const ov7670_filter_cfg_t OV7670_FILTER_25FPS_50HZ = {0x20, 0x98, 0x7f, 0x03, 0x03, 0x0a};
const ov7670_filter_cfg_t OV7670_FILTER_14FPS_50HZ = {0x20, 0x4c, 0x3f, 0x06, 0x07, 0x0a};
const ov7670_filter_cfg_t OV7670_FILTER_30FPS_60HZ_AUTO_LIGHT_FREQ_DETECT = {0x20, 0x98, 0x7f, 0x02, 0x03, 0x12};
const ov7670_filter_cfg_t OV7670_FILTER_15FPS_60HZ_AUTO_LIGHT_FREQ_DETECT = {0x20, 0x4c, 0x3f, 0x05, 0x07, 0x12};
const ov7670_filter_cfg_t OV7670_FILTER_25FPS_50HZ_AUTO_LIGHT_FREQ_DETECT = {0x20, 0x98, 0x7f, 0x03, 0x03, 0x1a};
const ov7670_filter_cfg_t OV7670_FILTER_14FPS_50HZ_AUTO_LIGHT_FREQ_DETECT = {0x20, 0x4c, 0x3f, 0x06, 0x07, 0x1a};

*/

/*
typedef struct ov7670_advanced_config
{
  ov7670_filter_cfg_t *filter;
  ov7670_night_mode_cfg_t *night_mode;
  ov7670_white_balance_cfg_t *white_balance;
  ov7670_light_mode_cfg_t *light_mode;
  ov7670_color_saturation_cfg_t *color_saturation;
  ov7670_special_effect_cfg_t *special_effect;
  ov7670_gamma_curve_slope_cfg_t *gamma_curve_slope;
} ov7670_advanced_config_t;

#define OV7670_ADVANCED_CONFIGURATION                                           \
{                                                                               \
  .filter = (ov7670_filter_cfg_t*)&OV7670_FILTER_DISABLED,                      \
  .night_mode = (ov7670_night_mode_cfg_t*)&OV7670_NIGHT_MODE_DISABLED,          \
  .white_balance = (ov7670_white_balance_cfg_t*)&OV7670_WHITE_BALANCE_SIMPLE,   \
  .light_mode = (ov7670_light_mode_cfg_t*)&OV7670_LIGHT_MODE_HOME,          \
  .color_saturation = (ov7670_color_saturation_cfg_t*)&OV7670_COLOR_SATURATION_2PLUS, \
  .special_effect = (ov7670_special_effect_cfg_t*)&OV7670_SPECIAL_EFFECT_NORMAL, \
  .gamma_curve_slope = (ov7670_gamma_curve_slope_cfg_t*)&OV7670_GAMMA_CURVE_SLOPE1, \
}

static const ov7670_advanced_config_t ov7670_advanced_config = OV7670_ADVANCED_CONFIGURATION;
*/

static inline void I2CSet(uint8_t device, uint8_t reg, uint8_t mask, uint8_t value) {
    // Perform a Read-Modify-Write
    uint8_t content = SCCB_Read(device, reg);
    content &= ~mask;
    content |= value & mask;
    SCCB_Write(device, reg, content);
    //I2CWrite(device, reg, content);
    // Validate content
    uint8_t verify = SCCB_Read(device, reg);
    if (verify != content) {
      // register mismatch
    }
}

/* frame-rate related registers */
/*
#define OV7670_CLKRC_REG                0x11    ///< Clocl control
#define OV7670_EXHCH_REG                0x2a    ///< dummy pixel insert MSB
#define OV7670_EXHCL_REG                0x2b    ///< dummy pixel insert LSB
#define OV7670_DBLV_REG                 0x6b
#define OV7670_DM_LNL_REG               0x92    ///< dummy line low 8 bits
#define OV7670_DM_LNH_REG               0x93    ///< dummy line high 8 bits
*/

/* V4L2 Video4Linux ov7670 driver from linux kernel */

/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */
#define REG_CMATRIX_BASE 0x4f
#define CMATRIX_LEN 6
#define REG_CMATRIX_SIGN 0x58

typedef struct ov7670_info {
	int cmatrix[CMATRIX_LEN];
	unsigned char sat;		/* Saturation value */
	int hue;			/* Hue value */
	uint8_t clkrc;			/* Clock divider value */
} ov7670_info_t;

static ov7670_info_t current_ov7670_state;

static const int cmatrixYUV[CMATRIX_LEN]	= { 128, -128, 0, -34, -94, 128 };
static const int cmatrix565[CMATRIX_LEN]	= { 179, -179, 0, -61, -176, 228 };
static const int cmatrix444[CMATRIX_LEN]	= { 179, -179, 0, -61, -176, 228 };



/*!
 * @brief OV7670 frame rate adjustment.
 * @param @ref ov7670_handler_t structure.
 * @param @ref ov7670_frame_rate_cfg_t structure.
 */
void OV7670_FrameRateAdjustment(sensor_t *sensor, ov7670_frame_rate_cfg_t *frame_rate_cfg)
{
  SCCB_Write(sensor->slv_addr, OV7670_CLKRC_REG, frame_rate_cfg->clkrc); systick_sleep(1);
  SCCB_Write(sensor->slv_addr, OV7670_DBLV_REG, frame_rate_cfg->dblv); systick_sleep(1);
  SCCB_Write(sensor->slv_addr, OV7670_EXHCH_REG, frame_rate_cfg->exhch); systick_sleep(1);
  SCCB_Write(sensor->slv_addr, OV7670_EXHCL_REG, frame_rate_cfg->exhcl); systick_sleep(1);
  SCCB_Write(sensor->slv_addr, OV7670_DM_LNL_REG, frame_rate_cfg->dm_lnl); systick_sleep(1);
  SCCB_Write(sensor->slv_addr, OV7670_DM_LNL_REG, frame_rate_cfg->dm_lnh); systick_sleep(1);
  // store new clkrc
  current_ov7670_state.clkrc = frame_rate_cfg->clkrc;
}

void OV7670_UndocumentedRegisterFix(sensor_t *sensor) {
  SCCB_Write(sensor->slv_addr, 0xb0, 0x84);
}

static void OV7670_NightMode(sensor_t *sensor, ov7670_night_mode_cfg_t *night_mode_cfg)
{
  I2CSet(sensor->slv_addr, OV7670_COM11_REG, 0xe0, night_mode_cfg->com11);
  systick_sleep(1);
}

static void OV7670_SpecialEffects(sensor_t *sensor, ov7670_special_effect_cfg_t *special_effect_cfg)
{
  I2CSet(sensor->slv_addr, OV7670_TSLB_REG, 0xfe, special_effect_cfg->tslb);
  SCCB_Write(sensor->slv_addr, OV7670_MANU_REG, special_effect_cfg->manu);
  SCCB_Write(sensor->slv_addr, OV7670_MANV_REG, special_effect_cfg->manv);
}

static void OV7670_LightMode(sensor_t *sensor, ov7670_light_mode_cfg_t *light_mode_cfg)
{
  I2CSet(sensor->slv_addr, OV7670_COM8_REG, 0xc5, light_mode_cfg->com8);
  I2CSet(sensor->slv_addr, OV7670_COM9_REG, 0x7a, light_mode_cfg->com9);
  SCCB_Write(sensor->slv_addr, OV7670_RED_REG, light_mode_cfg->red);
  SCCB_Write(sensor->slv_addr, OV7670_GGAIN_REG, light_mode_cfg->green);
  SCCB_Write(sensor->slv_addr, OV7670_BLUE_REG, light_mode_cfg->blue);
  SCCB_Write(sensor->slv_addr, OV7670_GAIN_REG, 0x00); systick_sleep(1);

  // Exposure value
  SCCB_Write(sensor->slv_addr, OV7670_AECH_REG, 0x00); systick_sleep(1);
  SCCB_Write(sensor->slv_addr, OV7670_AECHH_REG, 0x00); systick_sleep(1);
  I2CSet(sensor->slv_addr, OV7670_COM1_REG, 0x3, 0x00); systick_sleep(1);
  // AGC/AEC stable operation region configuration
  SCCB_Write(sensor->slv_addr, OV7670_AEW_REG, 0x75); systick_sleep(1);
  SCCB_Write(sensor->slv_addr, OV7670_AEB_REG, 0x63); systick_sleep(1);
  SCCB_Write(sensor->slv_addr, OV7670_VPT_REG, 0xd4); systick_sleep(1);
}

static void OV7670_WhiteBalance(sensor_t *sensor, ov7670_white_balance_cfg_t *white_balance_cfg)
{
  I2CSet(sensor->slv_addr, OV7670_COM8_REG, 0x02, white_balance_cfg->com8); //AWB on/off
  SCCB_Write(sensor->slv_addr, OV7670_AWBCTR0_REG, white_balance_cfg->awbctr0);
  SCCB_Write(sensor->slv_addr, OV7670_AWBCTR1_REG, white_balance_cfg->awbctr1);
  SCCB_Write(sensor->slv_addr, OV7670_AWBCTR2_REG, white_balance_cfg->awbctr2);
  SCCB_Write(sensor->slv_addr, OV7670_AWBCTR3_REG, white_balance_cfg->awbctr3);
  SCCB_Write(sensor->slv_addr, OV7670_AWBC1_REG, white_balance_cfg->awbc1);
  SCCB_Write(sensor->slv_addr, OV7670_AWBC2_REG, white_balance_cfg->awbc2);
  SCCB_Write(sensor->slv_addr, OV7670_AWBC3_REG, white_balance_cfg->awbc3);
  SCCB_Write(sensor->slv_addr, OV7670_AWBC4_REG, white_balance_cfg->awbc4);
  SCCB_Write(sensor->slv_addr, OV7670_AWBC5_REG, white_balance_cfg->awbc5);
  SCCB_Write(sensor->slv_addr, OV7670_AWBC6_REG, white_balance_cfg->awbc6);
  SCCB_Write(sensor->slv_addr, 0x59, 0x91);
  SCCB_Write(sensor->slv_addr, 0x5a, 0x94);
  SCCB_Write(sensor->slv_addr, 0x5b, 0xaa);
  SCCB_Write(sensor->slv_addr, 0x5c, 0x71);
  SCCB_Write(sensor->slv_addr, 0x5d, 0x8d);
  SCCB_Write(sensor->slv_addr, 0x5e, 0x0f);
  SCCB_Write(sensor->slv_addr, 0x5f, 0xf0);
  SCCB_Write(sensor->slv_addr, 0x60, 0xf0);
  SCCB_Write(sensor->slv_addr, 0x61, 0xf0);
  I2CSet(sensor->slv_addr, OV7670_COM16_REG, 0x08, white_balance_cfg->com16); //AWB gain on

}


static void OV7670_GammaCurveSlope(sensor_t *sensor, ov7670_gamma_curve_slope_cfg_t *gamma_curve_slope_cfg)
{
  SCCB_Write(sensor->slv_addr, OV7670_SLOP_REG, gamma_curve_slope_cfg->slope);
  SCCB_Write(sensor->slv_addr, OV7670_GAM1_REG, gamma_curve_slope_cfg->gam1);
  SCCB_Write(sensor->slv_addr, OV7670_GAM2_REG, gamma_curve_slope_cfg->gam2);
  SCCB_Write(sensor->slv_addr, OV7670_GAM3_REG, gamma_curve_slope_cfg->gam3);
  SCCB_Write(sensor->slv_addr, OV7670_GAM4_REG, gamma_curve_slope_cfg->gam4);
  SCCB_Write(sensor->slv_addr, OV7670_GAM5_REG, gamma_curve_slope_cfg->gam5);
  SCCB_Write(sensor->slv_addr, OV7670_GAM6_REG, gamma_curve_slope_cfg->gam6);
  SCCB_Write(sensor->slv_addr, OV7670_GAM7_REG, gamma_curve_slope_cfg->gam7);
  SCCB_Write(sensor->slv_addr, OV7670_GAM8_REG, gamma_curve_slope_cfg->gam8);
  SCCB_Write(sensor->slv_addr, OV7670_GAM9_REG, gamma_curve_slope_cfg->gam9);
  SCCB_Write(sensor->slv_addr, OV7670_GAM10_REG, gamma_curve_slope_cfg->gam10);
  SCCB_Write(sensor->slv_addr, OV7670_GAM11_REG, gamma_curve_slope_cfg->gam11);
  SCCB_Write(sensor->slv_addr, OV7670_GAM12_REG, gamma_curve_slope_cfg->gam12);
  SCCB_Write(sensor->slv_addr, OV7670_GAM13_REG, gamma_curve_slope_cfg->gam13);
  SCCB_Write(sensor->slv_addr, OV7670_GAM14_REG, gamma_curve_slope_cfg->gam14);
  SCCB_Write(sensor->slv_addr, OV7670_GAM15_REG, gamma_curve_slope_cfg->gam15);
}


static int ov7670_store_cmatrix(sensor_t *sensor,
		int matrix[CMATRIX_LEN])
{
	int i, ret=0;
	unsigned char signbits = 0;

	/*
	 * Weird crap seems to exist in the upper part of
	 * the sign bits register, so let's preserve it.
	 */
	//ret = SCCB_Read(sensor->slv_addr, REG_CMATRIX_SIGN, &signbits);
  signbits = SCCB_Read(sensor->slv_addr, REG_CMATRIX_SIGN);
  signbits &= 0xc0;

	for (i = 0; i < CMATRIX_LEN; i++) {
		unsigned char raw;

		if (matrix[i] < 0) {
			signbits |= (1 << i);
			if (matrix[i] < -255)
				raw = 0xff;
			else
				raw = (-1 * matrix[i]) & 0xff;
		}
		else {
			if (matrix[i] > 255)
				raw = 0xff;
			else
				raw = matrix[i] & 0xff;
		}
		ret += SCCB_Write(sensor->slv_addr, REG_CMATRIX_BASE + i, raw);
	}
	ret += SCCB_Write(sensor->slv_addr, REG_CMATRIX_SIGN, signbits);
	return ret;
}


/*
 * Hue also requires messing with the color matrix.  It also requires
 * trig functions, which tend not to be well supported in the kernel.
 * So here is a simple table of sine values, 0-90 degrees, in steps
 * of five degrees.  Values are multiplied by 1000.
 *
 * The following naive approximate trig functions require an argument
 * carefully limited to -180 <= theta <= 180.
 */
#define SIN_STEP 5
static const int ov7670_sin_table[] = {
	   0,	 87,   173,   258,   342,   422,
	 499,	573,   642,   707,   766,   819,
	 866,	906,   939,   965,   984,   996,
	1000
};

static int ov7670_sine(int theta)
{
	int chs = 1;
	int sine;

	if (theta < 0) {
		theta = -theta;
		chs = -1;
	}
	if (theta <= 90)
		sine = ov7670_sin_table[theta/SIN_STEP];
	else {
		theta -= 90;
		sine = 1000 - ov7670_sin_table[theta/SIN_STEP];
	}
	return sine*chs;
}

static int ov7670_cosine(int theta)
{
	theta = 90 - theta;
	if (theta > 180)
		theta -= 360;
	else if (theta < -180)
		theta += 360;
	return ov7670_sine(theta);
}

static void ov7670_calc_cmatrix(ov7670_info_t *info,
		int matrix[CMATRIX_LEN])
{
	int i;
	/*
	 * Apply the current saturation setting first.
	 */
	for (i = 0; i < CMATRIX_LEN; i++)
		matrix[i] = (info->cmatrix[i]*info->sat) >> 7;
	/*
	 * Then, if need be, rotate the hue value.
	 */
	if (info->hue != 0) {
		int sinth, costh, tmpmatrix[CMATRIX_LEN];

		memcpy(tmpmatrix, matrix, CMATRIX_LEN*sizeof(int));
		sinth = ov7670_sine(info->hue);
		costh = ov7670_cosine(info->hue);

		matrix[0] = (matrix[3]*sinth + matrix[0]*costh)/1000;
		matrix[1] = (matrix[4]*sinth + matrix[1]*costh)/1000;
		matrix[2] = (matrix[5]*sinth + matrix[2]*costh)/1000;
		matrix[3] = (matrix[3]*costh - matrix[0]*sinth)/1000;
		matrix[4] = (matrix[4]*costh - matrix[1]*sinth)/1000;
		matrix[5] = (matrix[5]*costh - matrix[2]*sinth)/1000;
	}
}

static int ov7670_set_sat(sensor_t *sensor, ov7670_info_t* info, int value)
{
	int matrix[CMATRIX_LEN];
	int ret;
	info->sat = value;
	ov7670_calc_cmatrix(info, matrix);
	ret = ov7670_store_cmatrix(sensor, matrix);
	return ret;
}

static int ov7670_set_hue(sensor_t *sensor, ov7670_info_t* info, int value)
{
	int matrix[CMATRIX_LEN];
	int ret;
	if (value < -180 || value > 180)
		return -1;
	info->hue = value;
	ov7670_calc_cmatrix(info, matrix);
	ret = ov7670_store_cmatrix(sensor, matrix);
	return ret;
}

static int reset(sensor_t *sensor)
{
    int i=0;
    const uint8_t (*regs)[2];

    // Reset all registers
    SCCB_Write(sensor->slv_addr, COM7, COM7_RESET);

    // Delay 10 ms
    systick_sleep(10);

    // Write default regsiters
    for (i=0, regs = default_regs; regs[i][0]; i++) {
        SCCB_Write(sensor->slv_addr, regs[i][0], regs[i][1]);
    }

    // Delay
    systick_sleep(30);

    current_ov7670_state.sat = 0;
    current_ov7670_state.hue = 0;
    //current_ov7670_state.clkrc = 0; // not used, tie to framerate / rgb565 set as needed...

    return 0;
}


static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret=0;
    // Read register COM7
    uint8_t reg = SCCB_Read(sensor->slv_addr, COM7);
    uint8_t reg2 = SCCB_Read(sensor->slv_addr, COM15);
    switch (pixformat) {
        case PIXFORMAT_RGB565:
            reg =  COM7_SET_FMT(reg, COM7_FMT_RGB);
	          reg2 = COM15_SET_RGB565(reg2, 1);
            // can swap byte order for ILI9341 direct display
            // then its wrong order for BMP encoded pics!
            // I2CSet(sensor->slv_addr, TSLB, (1 << 3), 0x00); // swap byte order LH -> HL
            for (int i = 0; i < CMATRIX_LEN; i++)
              current_ov7670_state.cmatrix[i] = cmatrix565[i];

            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_GRAYSCALE:
	      default:
            reg =  COM7_SET_FMT(reg, COM7_FMT_YUV);
	          reg2 = COM15_SET_RGB565(reg2, 0);
            for (int i = 0; i < CMATRIX_LEN; i++)
              current_ov7670_state.cmatrix[i] = cmatrixYUV[i];
            break;
    }

    // Write back register COM7
    ret = SCCB_Write(sensor->slv_addr, COM7, reg);
    ret = reg | SCCB_Write(sensor->slv_addr, COM15, reg2);


    // Delay
    systick_sleep(30);

/*
    if (pixformat == PIXFORMAT_YUV422) {
      // from ArduCAM - setup YUV color matrix...
      // https://github.com/ArduCAM/Arduino/blob/master/OV7670FIFO/OV7670FIFO.ino
      int rv;
      rv = SCCB_Write(sensor->slv_addr, YMTX1, MTX1_VALUE);
      rv = SCCB_Write(sensor->slv_addr, YMTX2, MTX2_VALUE);
      rv = SCCB_Write(sensor->slv_addr, YMTX3, MTX3_VALUE);
      rv = SCCB_Write(sensor->slv_addr, YMTX4, MTX4_VALUE);
      rv = SCCB_Write(sensor->slv_addr, YMTX5, MTX5_VALUE);
      rv = SCCB_Write(sensor->slv_addr, YMTX6, MTX6_VALUE);
      rv = SCCB_Write(sensor->slv_addr, YCONTRAS, CONTRAS_VALUE);
      rv = SCCB_Write(sensor->slv_addr, YMTXS, MTXS_VALUE);

    }
*/

    //ov7670_store_cmatrix(sensor, current_ov7670_state.cmatrix);

    /*
	 * If we're running RGB565, we must rewrite clkrc after setting
	 * the other parameters or the image looks poor.  If we're *not*
	 * doing RGB565, we must not rewrite clkrc or the image looks
	 * *really* poor.
	 *
	 * (Update) Now that we retain clkrc state, we should be able
	 * to write it unconditionally, and that will make the frame
	 * rate persistent too.
	 */
	  if (pixformat == PIXFORMAT_RGB565) {
		    ret = SCCB_Write(sensor->slv_addr, OV7670_CLKRC_REG, current_ov7670_state.clkrc ); // ret = ov7670_write(sd, REG_CLKRC, info->clkrc);
    }

    return ret;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
	const uint8_t (*regs)[2];
	int ret=0;
	int i=0;

  // store clkrc before changing window settings...
  int reg =  SCCB_Read(sensor->slv_addr, OV7670_CLKRC_REG);
  current_ov7670_state.clkrc = reg;

	switch(framesize)
	{
		case FRAMESIZE_VGA:
			for (i=0, regs = VGA_regs; regs[i][0]; i++)
			{
				ret |= SCCB_Write(sensor->slv_addr, regs[i][0], regs[i][1]);
			}
			break;
		case FRAMESIZE_QVGA:
			for (i=0, regs = QVGA_regs; regs[i][0]; i++)
			{
				ret |= SCCB_Write(sensor->slv_addr, regs[i][0], regs[i][1]);
			}
			break;
		case FRAMESIZE_QQVGA:
			for (i=0, regs = QQVGA_regs; regs[i][0]; i++)
			{
				ret |= SCCB_Write(sensor->slv_addr, regs[i][0], regs[i][1]);
			}
			break;
		default:
			return -1;
	}
	return ret;
}


static int set_framerate(sensor_t *sensor, framerate_t framerate)
{

  // test with 12mhz clock first...
  if (framerate == 0) OV7670_FrameRateAdjustment(sensor,&OV7670_14FPS_12MHZ_XCLK);
  if (framerate == 1) OV7670_FrameRateAdjustment(sensor,&OV7670_15FPS_12MHZ_XCLK);
  if (framerate == 2) OV7670_FrameRateAdjustment(sensor,&OV7670_25FPS_12MHZ_XCLK);
  if (framerate == 3) OV7670_FrameRateAdjustment(sensor,&OV7670_30FPS_12MHZ_XCLK);

  return 0;

}

static int set_contrast(sensor_t *sensor, int level)
{
    int ret=0;

    level += (NUM_CONTRAST_LEVELS / 2);
    if (level < 0 || level >= NUM_CONTRAST_LEVELS) {
        return -1;
    }

    ret |= SCCB_Write(sensor->slv_addr, CONTRAST, contrast_regs[level][0]);
    return ret;
}

static int set_brightness(sensor_t *sensor, int level)
{
    int ret=0;

    level += (NUM_BRIGHTNESS_LEVELS / 2);
    if (level < 0 || level >= NUM_BRIGHTNESS_LEVELS) {
        return -1;
    }

    ret |= SCCB_Write(sensor->slv_addr, BRIGHTNESS, brightness_regs[level][0]);
    return ret;
}

static int set_saturation(sensor_t *sensor, int level)
{
/*
    int ret=0;

    level += (NUM_SATURATION_LEVELS / 2 );
    if (level < 0 || level >= NUM_SATURATION_LEVELS) {
        return -1;
    }

    ret |= SCCB_Write(sensor->slv_addr, USAT, saturation_regs[level][0]);
    ret |= SCCB_Write(sensor->slv_addr, VSAT, saturation_regs[level][1]);
    return ret;
*/
    ov7670_set_sat(sensor,&current_ov7670_state,level);

	return 0;
}

static int set_hue(sensor_t *sensor, int level)
{
    ov7670_set_hue(sensor,&current_ov7670_state,level);

	return 0;
}

static int set_gainceiling(sensor_t *sensor, gainceiling_t gainceiling)
{
    // Read register COM9
    uint8_t reg = SCCB_Read(sensor->slv_addr, COM9);

    // Set gain ceiling
    reg = COM9_SET_AGC(reg, gainceiling);

    // Write back register COM9
    return SCCB_Write(sensor->slv_addr, COM9, reg);
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    uint8_t ret = 0;
    // Read register scaling_xsc
    uint8_t reg = SCCB_Read(sensor->slv_addr, SCALING_XSC);

    // Pattern to set color bar bit[0]=0 in every case
    reg = SCALING_XSC_CBAR(reg);

    // Write pattern to SCALING_XSC
    ret = SCCB_Write(sensor->slv_addr, SCALING_XSC, reg);

    // Read register scaling_ysc
    reg = SCCB_Read(sensor->slv_addr, SCALING_YSC);

    // Pattern to set color bar bit[0]=0 in every case
    reg = SCALING_YSC_CBAR(reg,enable);

    // Write pattern to SCALING_YSC
    ret = ret | SCCB_Write(sensor->slv_addr, SCALING_YSC, reg);

    // Set mirror on/off to pass self-tests
    // Read register MVFP
    reg = SCCB_Read(sensor->slv_addr, MVFP);

    // Set mirror on/off
    reg = MVFP_SET_MIRROR(reg, enable);

    // Write back register MVFP
    ret = ret | SCCB_Write(sensor->slv_addr, MVFP, reg);

    // return 0 or 0xFF
    return ret;
}


static int set_whitebal(sensor_t *sensor, int enable)
{
    // Read register COM8
    uint8_t reg = SCCB_Read(sensor->slv_addr, COM8);

    // Set white bal on/off
    reg = COM8_SET_AWB(reg, enable);

    // Write back register COM8
    return SCCB_Write(sensor->slv_addr, COM8, reg);
}

static int set_gain_ctrl(sensor_t *sensor, int enable)
{
    // Read register COM8
    uint8_t reg = SCCB_Read(sensor->slv_addr, COM8);

    // Set white bal on/off
    reg = COM8_SET_AGC(reg, enable);

    // Write back register COM8
    return SCCB_Write(sensor->slv_addr, COM8, reg);
}

static int set_exposure_ctrl(sensor_t *sensor, int enable)
{
    // Read register COM8
    uint8_t reg = SCCB_Read(sensor->slv_addr, COM8);

    // Set white bal on/off
    reg = COM8_SET_AEC(reg, enable);

    // Write back register COM8
    return SCCB_Write(sensor->slv_addr, COM8, reg);
}


static int set_hmirror(sensor_t *sensor, int enable)
{
    // Read register MVFP
    uint8_t reg = SCCB_Read(sensor->slv_addr, MVFP);

    // Set mirror on/off
    reg = MVFP_SET_MIRROR(reg, enable);

    // Write back register COM3
    return SCCB_Write(sensor->slv_addr, MVFP, reg);
}



static int set_vflip(sensor_t *sensor, int enable)
{
    // Read register MVFP
    uint8_t reg = SCCB_Read(sensor->slv_addr, MVFP);

    // Set mirror on/off
    reg = MVFP_SET_FLIP(reg, enable);

    // Write back register COM3
    return SCCB_Write(sensor->slv_addr, MVFP, reg);
}



static int set_ov7670_night_mode(sensor_t *sensor, int sde) //sde_t sde)
{
  int ret=0;
  switch (sde)
  {
    case 0:
      OV7670_NightMode(sensor,&OV7670_NIGHT_MODE_DISABLED);
      break;
    case 1:
      OV7670_NightMode(sensor,&OV7670_NIGHT_MODE_AUTO_FR_DIVBY2);
      break;
    case 2:
      OV7670_NightMode(sensor,&OV7670_NIGHT_MODE_AUTO_FR_DIVBY4);
      break;
    case 3:
      OV7670_NightMode(sensor,&OV7670_NIGHT_MODE_AUTO_FR_DIVBY8);
      break;
    default:
      break;
    }
    OV7670_UndocumentedRegisterFix(sensor);
    return 0;
}

static int set_ov7670_light_mode(sensor_t *sensor, int sde) //sde_t sde)
{
  int ret=0;
  switch (sde)
  {
    case 0:
      OV7670_LightMode(sensor,&OV7670_LIGHT_MODE_DISABLED);
      break;
    case 1:
      OV7670_LightMode(sensor,&OV7670_LIGHT_MODE_AUTO);
      break;
    case 2:
      OV7670_LightMode(sensor,&OV7670_LIGHT_MODE_SUNNY);
      break;
    case 3:
      OV7670_LightMode(sensor,&OV7670_LIGHT_MODE_CLOUDY);
      break;
    case 4:
      OV7670_LightMode(sensor,&OV7670_LIGHT_MODE_OFFICE);
        break;
    case 5:
      OV7670_LightMode(sensor,&OV7670_LIGHT_MODE_HOME);
      break;
    default:
      break;
    }
    OV7670_UndocumentedRegisterFix(sensor);
    return 0;
}


static int set_special_effect(sensor_t *sensor, int sde) //sde_t sde)
{
  int ret=0;

    switch (sde)
    {
    case 0:
      OV7670_SpecialEffects(sensor,&OV7670_SPECIAL_EFFECT_ANTIQUE);
      break;
    case 1:
      OV7670_SpecialEffects(sensor,&OV7670_SPECIAL_EFFECT_SEPHIA);
      break;
    case 2:
      OV7670_SpecialEffects(sensor,&OV7670_SPECIAL_EFFECT_BLUISH);
      break;
    case 3:
      OV7670_SpecialEffects(sensor,&OV7670_SPECIAL_EFFECT_GREENISH);
      break;
    case 4:
      OV7670_SpecialEffects(sensor,&OV7670_SPECIAL_EFFECT_REDISH);
      break;
    case 5:
      OV7670_SpecialEffects(sensor,&OV7670_SPECIAL_EFFECT_BW);
      break;
    case 6:
      OV7670_SpecialEffects(sensor,&OV7670_SPECIAL_EFFECT_NEGATIVE);
      break;
    case 7:
      OV7670_SpecialEffects(sensor,&OV7670_SPECIAL_EFFECT_BW_NEGATIVE);
      break;
    case 8:
      OV7670_SpecialEffects(sensor,&OV7670_SPECIAL_EFFECT_NORMAL);
      break;
    case 9:
      OV7670_SpecialEffects(sensor,&OV7670_SPECIAL_EFFECT_DISABLED);
      break;
    default:
      break;
    }

    return 0;
}

static int set_ov7670_gamma(sensor_t *sensor, int sde) //sde_t sde)
  {
    int ret=0;
    switch (sde)
    {
      case 0:
        OV7670_GammaCurveSlope(sensor,&OV7670_GAMMA_CURVE_SLOPE_DEFAULT);
        break;
      case 1:
        OV7670_GammaCurveSlope(sensor,&OV7670_GAMMA_CURVE_SLOPE1);
        break;
      default:
        break;
      }
      return 0;
  }

static int set_ov7670_whitebalance(sensor_t *sensor, int sde) //sde_t sde)
  {
    int ret=0;
    switch (sde)
    {
      case 0:
        OV7670_WhiteBalance(sensor,&OV7670_WHITE_BALANCE_DEFAULT);
        break;
      case 1:
        OV7670_WhiteBalance(sensor,&OV7670_WHITE_BALANCE_DISABLED);
        break;
      case 2:
        OV7670_WhiteBalance(sensor,&OV7670_WHITE_BALANCE_SIMPLE);
        break;
      default:
        break;
      }
      return 0;
  }


/*
static int set_register(sensor_t *sensor, uint8_t reg, uint8_t regVal)
{
    return SCCB_Write(sensor->slv_addr, reg, regVal);
}
*/

int ov7670_init(sensor_t *sensor)
{
    // Set function pointers
    sensor->reset = reset;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_framerate = set_framerate;
    sensor->set_contrast  = set_contrast;
    sensor->set_brightness= set_brightness;
    sensor->set_saturation= set_saturation;
    sensor->set_hue =       set_hue;
    sensor->set_gainceiling = set_gainceiling;
    sensor->set_colorbar = set_colorbar;
    sensor->set_whitebal = set_whitebal;
    sensor->set_gain_ctrl = set_gain_ctrl;
    sensor->set_exposure_ctrl = set_exposure_ctrl;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;

    sensor->set_special_effect = set_special_effect;
    sensor->set_ov7670_night_mode = set_ov7670_night_mode;
    sensor->set_ov7670_light_mode = set_ov7670_light_mode;
    sensor->set_ov7670_gamma = set_ov7670_gamma;
    sensor->set_ov7670_whitebalance = set_ov7670_whitebalance;

    // Retrieve sensor's signature
    sensor->id.MIDH = SCCB_Read(sensor->slv_addr, REG_MIDH);
    sensor->id.MIDL = SCCB_Read(sensor->slv_addr, REG_MIDL);
    sensor->id.PID = SCCB_Read(sensor->slv_addr, REG_PID);
    sensor->id.VER = SCCB_Read(sensor->slv_addr, REG_VER);

    // Set sensor flags
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_VSYNC, 1);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_HSYNC, 0);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_PIXCK, 1);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_FSYNC, 1);

//    sensor->set_register = set_register;

    return 0;
}
