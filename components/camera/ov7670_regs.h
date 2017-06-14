/*
 * This file is for the OpenMV project so the OV7670 can be used
 * author: Wolfgang Baumgartner
 * TODO: revise common control registers, AEC exposure value registers different
 *
 *
 * OV7670 register definitions.
 */

#ifndef __REG_REGS_H__
#define __REG_REGS_H__
#define GAIN                    0x00 /* AGC – Gain control gain setting  */
#define BLUE                    0x01 /* AWB – Blue channel gain setting  */
#define RED                     0x02 /* AWB – Red channel gain setting   */
#define VREF                    0x03 /* AWB – Green channel gain setting */
#define COM1			0x04 /* Common Control 1 */
#define BAVG                    0x05 /* U/B Average Level   */
#define GAVG                    0x06 /* Y/Gb Average Level  */
#define AECH                    0x07 /* Exposure VAlue - AEC MSB 5 bits  */
#define RAVG                    0x08 /* V/R Average Level */

#define COM2                    0x09 /* Common Control 2 */
#define COM2_SOFT_SLEEP         0x10 /* Soft sleep mode  */
#define COM2_OUT_DRIVE_1x       0x00 /* Output drive capability 1x */
#define COM2_OUT_DRIVE_2x       0x01 /* Output drive capability 2x */
#define COM2_OUT_DRIVE_3x       0x02 /* Output drive capability 3x */
#define COM2_OUT_DRIVE_4x       0x03 /* Output drive capability 4x */

#define REG_PID                     0x0A /* Product ID Number MSB */
#define REG_VER                     0x0B /* Product ID Number LSB */

#define COM3                    0x0C /* Common Control 3 				        */
#define COM3_VFLIP              0x80 /* Vertical flip image ON/OFF selection                    */
#define COM3_MIRROR             0x40 /* Horizontal mirror image ON/OFF selection                */
#define COM3_SWAP_BR            0x20 /* Swap B/R output sequence in RGB output mode             */
#define COM3_SWAP_YUV           0x10 /* Swap Y/UV output sequence in YUV output mode            */
#define COM3_SWAP_MSB           0x08 /* Swap output MSB/LSB                                     */
#define COM3_TRI_CLOCK          0x04 /* Tri-state option for output clock at power-down period  */
#define COM3_TRI_DATA           0x02 /* Tri-state option for output data at power-down period   */
#define COM3_COLOR_BAR          0x01 /* Sensor color bar test pattern output enable             */
#define COM3_SET_CBAR(r, x)     ((r&0xFE)|((x&1)<<0))
#define COM3_SET_MIRROR(r, x)   ((r&0xBF)|((x&1)<<6))
#define COM3_SET_FLIP(r, x)     ((r&0x7F)|((x&1)<<7))

#define COM4                    0x0D /* Common Control 4         */
#define COM4_PLL_BYPASS         0x00 /* Bypass PLL               */
#define COM4_PLL_4x             0x40 /* PLL frequency 4x         */
#define COM4_PLL_6x             0x80 /* PLL frequency 6x         */
#define COM4_PLL_8x             0xc0 /* PLL frequency 8x         */
#define COM4_AEC_FULL           0x00 /* AEC evaluate full window */
#define COM4_AEC_1_2            0x10 /* AEC evaluate 1/2 window  */
#define COM4_AEC_1_4            0x20 /* AEC evaluate 1/4 window  */
#define COM4_AEC_2_3            0x30 /* AEC evaluate 2/3 window  */

#define COM5                    0x0E /* Common Control 5 */
#define COM5_AFR                0x80 /* Auto frame rate control ON/OFF selection (night mode) */
#define COM5_AFR_SPEED          0x40 /* Auto frame rate control speed selection */
#define COM5_AFR_0              0x00 /* No reduction of frame rate          */
#define COM5_AFR_1_2            0x10 /* Max reduction to 1/2 frame rate     */
#define COM5_AFR_1_4            0x20 /* Max reduction to 1/4 frame rate     */
#define COM5_AFR_1_8            0x30 /* Max reduction to 1/8 frame rate     */
#define COM5_AFR_4x             0x04 /* Add frame when AGC reaches 4x gain  */
#define COM5_AFR_8x             0x08 /* Add frame when AGC reaches 8x gain  */
#define COM5_AFR_16x            0x0c /* Add frame when AGC reaches 16x gain */
#define COM5_AEC_NO_LIMIT       0x01 /* No limit to AEC increase step       */

#define COM6                    0x0F /* Common Control 6 */
#define COM6_AUTO_WINDOW        0x01 /* Auto window setting ON/OFF selection when format changes */

#define AEC                     0x10 /* AEC[7:0] (see register AECH for AEC[15:8]) */
#define CLKRC                   0x11 /* Internal Clock */

#define COM7                    0x12 /* Common Control 7         */
#define COM7_RESET              0x80 /* SCCB Register Reset      */
#define COM7_RES_VGA            0x00 /* Resolution VGA           */
#define COM7_RES_QVGA           0x40 /* Resolution QVGA          */
#define COM7_BT656              0x20 /* BT.656 protocol ON/OFF   */
#define COM7_SENSOR_RAW         0x10 /* Sensor RAW               */
#define COM7_FMT_GBR422         0x00 /* RGB output format GBR422 */
#define COM7_FMT_RGB565         0x04 /* RGB output format RGB565 */
#define COM7_FMT_RGB555         0x08 /* RGB output format RGB555 */
#define COM7_FMT_RGB444         0x0C /* RGB output format RGB444 */
#define COM7_FMT_YUV            0x00 /* Output format YUV        */
#define COM7_FMT_P_BAYER        0x01 /* Output format Processed Bayer RAW */
#define COM7_FMT_RGB            0x04 /* Output format RGB        */
#define COM7_FMT_R_BAYER        0x03 /* Output format Bayer RAW  */
#define COM7_SET_FMT(r, x)      ((r&0xFC)|((x&0x5)<<0))

#define COM8                    0x13 /* Common Control 8                */
#define COM8_FAST_AUTO          0x80 /* Enable fast AGC/AEC algorithm   */
#define COM8_STEP_VSYNC         0x00 /* AEC - Step size limited to vertical blank */
#define COM8_STEP_UNLIMIT       0x40 /* AEC - Step size unlimited step size       */
#define COM8_BANDF_EN           0x20 /* Banding filter ON/OFF */
#define COM8_AEC_BANDF          0x10 /* Enable AEC below banding value */
#define COM8_AEC_FINE_EN        0x08 /* Fine AEC ON/OFF control */
#define COM8_AGC_EN             0x04 /* AGC Enable */
#define COM8_AWB_EN             0x02 /* AWB Enable */
#define COM8_AEC_EN             0x01 /* AEC Enable */
#define COM8_SET_AGC(r, x)      ((r&0xFB)|((x&0x1)<<2))
#define COM8_SET_AWB(r, x)      ((r&0xFD)|((x&0x1)<<1))
#define COM8_SET_AEC(r, x)      ((r&0xFE)|((x&0x1)<<0))

#define COM9                    0x14 /* Common Control 9 */
#define COM9_HISTO_AVG          0x80 /* Histogram or average based AEC/AGC selection */
#define COM9_AGC_GAIN_2x        0x00 /* Automatic Gain Ceiling 2x  */
#define COM9_AGC_GAIN_4x        0x10 /* Automatic Gain Ceiling 4x  */
#define COM9_AGC_GAIN_8x        0x20 /* Automatic Gain Ceiling 8x  */
#define COM9_AGC_GAIN_16x       0x30 /* Automatic Gain Ceiling 16x */
#define COM9_AGC_GAIN_32x       0x40 /* Automatic Gain Ceiling 32x */
#define COM9_DROP_VSYNC         0x04 /* Drop VSYNC output of corrupt frame */
#define COM9_DROP_HREF          0x02 /* Drop HREF output of corrupt frame  */
#define COM9_SET_AGC(r, x)      ((r&0x8F)|((x&0x07)<<4))

#define COM10                   0x15 /* Common Control 10 */
#define COM10_NEGATIVE          0x80 /* Output negative data */
#define COM10_HSYNC_EN          0x40 /* HREF changes to HSYNC */
#define COM10_PCLK_FREE         0x00 /* PCLK output option: free running PCLK */
#define COM10_PCLK_MASK         0x20 /* PCLK output option: masked during horizontal blank  */
#define COM10_PCLK_REV          0x10 /* PCLK reverse */
#define COM10_HREF_REV          0x08 /* HREF reverse */
#define COM10_VSYNC_FALLING     0x00 /* VSYNC changes on falling edge of PCLK */
#define COM10_VSYNC_RISING      0x04 /* VSYNC changes on rising edge of PCLK */
#define COM10_VSYNC_NEG         0x02 /* VSYNC negative */
#define COM10_OUT_RANGE_8       0x01 /* Output data range: Full range */
#define COM10_OUT_RANGE_10      0x00 /* Output data range: Data from [10] to [F0] (8 MSBs) */

#define RSVD                    0x16 /* Reserved register */
/* #define REG16_BIT_SHIFT         0x80  Bit shift test pattern options */
#define HSTART                  0x17  /* Horizontal Frame (HREF column) Start high 8-bit(low 3 bits are at HREF[2:0]) */
#define HSTOP                   0x18  /* Horizontal Frame (HREF column) end high 8-bit (low 3 bits are at HREF[5:3])  */
#define VSTART                  0x19  /* Vertical Frame (row) Start high 8-bit (low 2 bits are at VREF[1:0]) */
#define VSTOP                   0x1A  /* Vertical Frame (row) End high 8-bit (low 2 bits are at VREF[3:2]) */
#define PSHFT                   0x1B  /* Data Format - Pixel Delay Select */
#define REG_MIDH                    0x1C  /* Manufacturer ID Byte – High */
#define REG_MIDL                    0x1D  /* Manufacturer ID Byte – Low */
#define MVFP			0x3E  /* Mirror/Vflip Enable */
#define MVFP_SET_MIRROR(r,x)	((r&0xDF)|((x&1)<<5)) /* change only bit5 according to x */
#define MVFP_SET_FLIP(r,x)	((r&0xEF)|((x&1)<<4)) /* change only bit4 according to x */
#define LAEC                    0x1F  /* Fine AEC Value - defines exposure value less than one row period (Reserved?) */


#define ADCCTR0                 0x20 /* ADC control */
/*#define COM11_SNGL_FRAME_EN     0x02  Single frame ON/OFF selection
#define COM11_SNGL_XFR_TRIG     0x01  Single frame transfer trigger */
#define ADCCTR1			0x21 /* reserved */
#define ADCCTR2                 0x22 /* reserved */
#define ADCCTR3                 0x23 /* reserved */
#define AEW                     0x24 /* AGC/AEC - Stable Operating Region (Upper Limit) */
#define AEB                     0x25 /* AGC/AEC - Stable Operating Region (Lower Limit) */
#define VPT                     0x26 /* AGC/AEC Fast Mode Operating Region */
#define BBIAS 			0x27 /* B channel signal output bias (effective only when COM6[3]=1) */
#define GbBIAS                  0x28 /* Gb channel signal output bias (effective only when COM6[3]=1) */
//#define HOUTSIZE                0x29 /* reserved */
#define EXHCH                   0x2A /* Dummy Pixel Insert MSB */
#define EXHCL                   0x2B /* Dummy Pixel Insert LSB */
#define RBIAS                   0x2C /* R channel signal output bias (effective only when COM6[3]=1) */
#define ADVFL                   0x2D /* LSB of Insert Dummy Rows in Vertical Sync (1 bit equals 1 row)  */
#define ADVFH                   0x2E /* MSB of Insert Dummy Rows in Vertical Sync */
#define YAVE                    0x2F /* Y/G Channel Average Value */
#define HSYST                   0x30 /* HSync rising edge delay */
#define HSYEN                   0x31 /* HSync falling edge delay  */
#define HREF                    0x32 /* Image Start and Size Control DIFFERENT CONTROL SEQUENCE	 */
#define CHLF                    0x33 /* Array Current control  */
#define ARBLM                   0x34 /* Array reference control */
//#define ADOFF_B                 0x35 /* AD Offset Compensation Value for B Channel  */
//#define ADOFF_R                 0x36 /* AD Offset Compensation Value for R Channel  */
//#define ADC                     0x37 /* ADC control */
#define ACOM                    0x38 /* ADC and analog common mode control */
#define OFON                    0x39 /* ADC offset control */
#define TSLB                    0x3A /* Line buffer test option  */
#define COM11                   0x3B /* Common control 11 */
#define COM11_EXP		0x02
#define COM11_HZAUTO		0x10	/* Auto detect 50/60 Hz */
#define COM12                   0x3C /* Common control 12 */
#define COM13                   0x3D /* Common control 13 */

#define COM14                   0x3E /* Common Control 14 */
//#define COM13_BLC_EN            0x80 /* BLC enable */
//#define COM13_ADC_EN            0x40 /* ADC channel BLC ON/OFF control */
//#define COM13_ANALOG_BLC        0x20 /* Analog processing channel BLC ON/OFF control */
//#define COM13_ABLC_GAIN_EN      0x04 /* ABLC gain trigger enable */

#define EDGE                    0x3F /* edge enhancement adjustment */
#define COM15                   0x40 /* Common Control 15 DIFFERENT CONTROLS */
#define COM15_SET_RGB565(r,x)	((r&0xEF)|((x&1)<<4)) /* set rgb565 mode */
#define COM16                   0x41 /* Common Control 16 DIFFERENT CONTROLS */
#define COM16_AWBGAIN		0x08	/* AWB gain enable */
#define COM17                   0x42 /* Common Control 17   */
//#define TGT_R                   0x43 /* BLC Red Channel Target Value    */
//#define TGT_GB                  0x44 /* BLC Gb Channel Target Value     */
//#define TGT_GR                  0x45 /* BLC Gr Channel Target Value     */

//#define LC_CTR                  0x46 /* Lens Correction Control */
//#define LC_CTR_RGB_COMP_1       0x00 /* R, G, and B channel compensation coefficient is set by LC_COEF (0x49) */
//#define LC_CTR_RGB_COMP_3       0x04 /* R, G, and B channel compensation coefficient is set by registers
//                                        LC_COEFB (0x4B), LC_COEF (0x49), and LC_COEFR (0x4C), respectively */
//#define LC_CTR_EN               0x01 /* Lens correction enable */
//#define LC_XC                   0x47 /* X Coordinate of Lens Correction Center Relative to Array Center */
//#define LC_YC                   0x48 /* Y Coordinate of Lens Correction Center Relative to Array Center */
//#define LC_COEF                 0x49 /* Lens Correction Coefficient */
//#define LC_RADI                 0x4A /* Lens Correction Radius */
#define REG4B                   0x4B /* Register 4B */
#define DNSTH                  0x4C /* Denoise strength */

//#define FIXGAIN                 0x4D /* Analog Fix Gain Amplifier */
//#define AREF0                   0x4E /* Sensor Reference Control */
#define MTX1                    0x4F /* Matrix coefficient 1 */
#define MTX2                    0x50 /* Matrix coefficient 2 */
#define MTX3                    0x51 /* Matrix coefficient 3 */
#define MTX4                    0x52 /* Matrix coefficient 4 */
#define MTX5                    0x53 /* Matrix coefficient 5 */
#define MTX6                    0x54 /* Matrix coefficient 6 */
#define BRIGHTNESS              0x55 /* Brightness control */
#define CONTRAST		0x56 /* Contrast control */
#define CONTRASCENTER		0x57 /* Contrast center */
#define MTXS			0x58 /* Matrix coefficient sign for coefficient 5 to 0*/
//#define UFIX                    0x60 /* U Channel Fixed Value Output */
//#define VFIX                    0x61 /* V Channel Fixed Value Output */
#define LCC1                    0x62 /* Lens correction option 1  */

#define LCC2                    0x63 /* Lens correction option 2 */
#define LCC3 			0x64 /* Lens correction option 3 */
#define LCC4			0x65 /* Lens correction option 4 */
#define LCC5			0x66 /* Lens correction option 5 */
//#define AWB_CTRL0_GAIN_EN       0x80 /* AWB gain enable      */
//#define AWB_CTRL0_CALC_EN       0x40 /* AWB calculate enable */
//#define AWB_CTRL0_WBC_MASK      0x0F /* WBC threshold 2      */

//#define DSP_CTRL1               0x64 /* DSP Control Byte 1                  */
//#define DSP_CTRL1_FIFO_EN       0x80 /* FIFO enable/disable selection       */
//#define DSP_CTRL1_UV_EN         0x40 /* UV adjust function ON/OFF selection */
//#define DSP_CTRL1_SDE_EN        0x20 /* SDE enable                          */
//#define DSP_CTRL1_MTRX_EN       0x10 /* Color matrix ON/OFF selection       */
//#define DSP_CTRL1_INTRP_EN      0x08 /* Interpolation ON/OFF selection      */
//#define DSP_CTRL1_GAMMA_EN      0x04 /* Gamma function ON/OFF selection     */
//#define DSP_CTRL1_BLACK_EN      0x02 /* Black defect auto correction ON/OFF */
//#define DSP_CTRL1_WHITE_EN      0x01 /* White defect auto correction ON/OFF */

//#define DSP_CTRL2               0x65 /* DSP Control Byte 2          */
//#define DSP_CTRL2_VDCW_EN       0x08 /* Vertical DCW enable         */
//#define DSP_CTRL2_HDCW_EN       0x04 /* Horizontal DCW enable       */
//#define DSP_CTRL2_VZOOM_EN      0x02 /* Vertical zoom out enable    */
//#define DSP_CTRL2_HZOOM_EN      0x01 /* Horizontal zoom out enable  */

//#define DSP_CTRL3               0x66 /* DSP Control Byte 3                      */
//#define DSP_CTRL3_UV_EN         0x80 /* UV output sequence option               */
//#define DSP_CTRL3_CBAR_EN       0x20 /* DSP color bar ON/OFF selection          */
//#define DSP_CTRL3_FIFO_EN       0x08 /* FIFO power down ON/OFF selection        */
//#define DSP_CTRL3_SCAL1_PWDN    0x04 /* Scaling module power down control 1     */
//#define DSP_CTRL3_SCAL2_PWDN    0x02 /* Scaling module power down control 2     */
//#define DSP_CTRL3_INTRP_PWDN    0x01 /* Interpolation module power down control */

#define MANU	                0x67 /* Manual U value (effective only with TSLB[4] is high) */
//#define DSP_CTRL4_YUV_RGB       0x00 /* Output selection YUV or RGB */
//#define DSP_CTRL4_RAW8          0x02 /* Output selection RAW8       */
//#define DSP_CTRL4_RAW10         0x03 /* Output selection RAW10      */


#define MANV 	                0x68 /* Manual V value (effective only with TSLB[4] is high) */
#define GFIX                    0x69 /* Fix gain control */
#define GGAIN                   0x6A /* G channel AWB gain */

#define DBLV               	0x6B /* PLL and clock ? */
//#define AWB_CTRL3_ADVANCED      0x80 /* AWB mode select - Advanced AWB */
//#define AWB_CTRL3_SIMPLE        0x00 /* AWB mode select - Simple AWB */
// bis hier
#define AWBCTR3               	0x6C /* AWB Control 3  */
#define AWBCTR2	                0x6D /* AWB Control 2  */
#define AWBCTR1                 0x6E /* AWB Control 1  */
#define AWBCTR0                 0x6F /* AWB Control 0  */
#define SCALING_XSC             0x70 /* test pattern and horizontal scaling factor */
#define SCALING_XSC_CBAR(r)	(r&0x7F) /* make sure bit7 is 0 for color bar */
#define SCALING_YSC             0x71 /* test pattern and vertical scaling factor */
#define SCALING_YSC_CBAR(r,x)	((r&0x7F)|((x&1)<<7)) /* change bit7 for color bar on/off */
#define SCALING_DCWCTR          0x72 /* DCW control */
#define SCALING_PCLK_DIV        0x73 /*  */
#define REG74                   0x74 /*  */
#define REG75                   0x75 /*  */
#define REG76                   0x76 /*  */
#define REG77	             	0x77 /*  */
//#define AWB_CTRL16              0x78 /* AWB Control 16 */
//#define AWB_CTRL17              0x79 /* AWB Control 17 */
#define SLOP	                0x7A /* Gamma curve highest segment slope */
#define GAM1	                0x7B /* Gamma Curve 1st Segment Input End Point 0x04 Output Value */
#define GAM2	                0x7C /* Gamma Curve 2nd Segment Input End Point 0x08 Output Value */
#define GAM3                    0x7D /* Gamma Curve 3rd Segment Input End Point 0x10 Output Value */
#define GAM4                    0x7E /* Gamma Curve 4th Segment Input End Point 0x20 Output Value */
#define GAM5                    0x7F /* Gamma Curve 5th Segment Input End Point 0x28 Output Value */
#define GAM6                    0x80 /* Gamma Curve 6rd Segment Input End Point 0x30 Output Value */
#define GAM7                    0x81 /* Gamma Curve 7th Segment Input End Point 0x38 Output Value */
#define GAM8                    0x82 /* Gamma Curve 8th Segment Input End Point 0x40 Output Value */
#define GAM9                    0x83 /* Gamma Curve 9th Segment Input End Point 0x48 Output Value */
#define GAM10                   0x84 /* Gamma Curve 10th Segment Input End Point 0x50 Output Value */
#define GAM11                   0x85 /* Gamma Curve 11th Segment Input End Point 0x60 Output Value */
#define GAM12                   0x86 /* Gamma Curve 12th Segment Input End Point 0x70 Output Value */
#define GAM13                   0x87 /* Gamma Curve 13th Segment Input End Point 0x90 Output Value */
#define GAM14                   0x88 /* Gamma Curve 14th Segment Input End Point 0xB0 Output Value */
#define GAM15                   0x89 /* Gamma Curve 15th Segment Input End Point 0xD0 Output Value */
//#define GAM13                   0x8A /* Gamma Curve 13th Segment Input End Point 0xD0 Output Value */
//#define GAM14                   0x8B /* Gamma Curve 14th Segment Input End Point 0xB0 Output Value */
#define RGB444                  0x8C /*  */
//#define SLOP                    0x8D /* Gamma Curve Highest Segment Slope */
//#define DNSTH                   0x8E /* De-noise Threshold */
//#define EDGE0                   0x8F /* Edge Enhancement Strength Control */
//#define EDGE1                   0x90 /* Edge Enhancement Threshold Control */
//#define DNSOFF                  0x91 /* Auto De-noise Threshold Control */
#define DM_LNL                  0x92 /* Dummy line low 8 bit */
#define DM_LNH                  0x93 /* Dummy line high 8 bit */
#define LCC6                    0x94 /* Lens correction option 6 */
#define LCC7                    0x95 /* Lens correction option 7 */
//#define MTX3                    0x96 /* Matrix Coefficient 3 */
//#define MTX4                    0x97 /* Matrix Coefficient 4 */
//#define MTX5                    0x98 /* Matrix Coefficient 5 */
//#define MTX6                    0x99 /* Matrix Coefficient 6 */

//#define MTX_CTRL                0x9A /* Matrix Control */
//#define MTX_CTRL_DBL_EN         0x80 /* Matrix double ON/OFF selection */

//#define BRIGHTNESS              0x9B /* Brightness Control */
//#define CONTRAST                0x9C /* Contrast Gain */
#define BD50ST			0x9D /* 50 Hz banding filter value */
#define BD60ST                  0x9E /* 60 Hz banding filter value */
#define HAECC1                  0x9F /* Histogram-based AEC/AGC control 1 */
#define HAECC2                  0xA0 /* Histogram-based AEC/AGC control 2 */
//#define SCAL1                   0xA1 /* Horizontal Zoom Out Control */
#define SCALING_PCLK_DELAY      0xA2 /* Pixel clock delay */
//#define FIFODLYM                0xA3 /* FIFO Manual Mode Delay Control */
#define NT_CNTRL                0xA4 /*  */
#define BD50MAX			0xA5 /* 50 Hz banding step limit */
#define HAECC3                  0xA6 /* Histogram-based AEC/AGC control 3  */
#define HAECC4 	   	        0xA7 /* Histogram-based AEC/AGC control 4           */
#define HAECC5		        0xA8 /* Histogram-based AEC/AGC control 5         */
#define HAECC6		        0xA9 /* Histogram-based AEC/AGC control 6           */
#define HAECC7		        0xAA /* Histogram-based AEC/AGC control 7           */
//#define SDE_CONT_BRIGHT_EN      0x04 /* Contrast/Brightness enable      */
//#define SDE_SATURATION_EN       0x02 /* Saturation enable               */
//#define SDE_HUE_EN              0x01 /* Hue enable                      */

#define BD60MAX                 0xAB /* 60 Hz banding step limit */

#define STR_OPT                 0xAC /* Register AC */
#define STR_R			0xAD /* R gain for led output frame */
#define STR_G			0xAE /* G gain for led output frame */
#define STR_B			0xAF /* B gain for led output frame */
#define ABLC1			0xB1 /* */
#define THL_ST			0xB3 /* ABLC target */
#define THL_DLT			0xB5 /* ABLC stable range */
#define AD_CHB			0xBE /* blue channel black level compensation */
#define AD_CHR			0xBF /* Red channel black level compensation */
#define AD_CHGb			0xC0 /* Gb channel black level compensation */
#define AD_CHGr			0xC1 /* Gr channel black level compensation */
#define SATCTR			0xC9 /* Saturation control */
//#define DSPAUTO_AWB_EN          0x80 /* AWB auto threshold control */
//#define DSPAUTO_DENOISE_EN      0x40 /* De-noise auto threshold control */
//#define DSPAUTO_EDGE_EN         0x20 /* Sharpness (edge enhancement) auto strength control */
//#define DSPAUTO_UV_EN           0x10 /* UV adjust auto slope control */
//#define DSPAUTO_SCAL0_EN        0x08 /* Auto scaling factor control (register SCAL0 (0xA0)) */
//#define DSPAUTO_SCAL1_EN        0x04 /* Auto scaling factor control (registers SCAL1 (0xA1 and SCAL2 (0xA2))*/
#define SET_REG(reg, x)         (##reg_DEFAULT|x)

// extra registers from ArduCAM
// from https://github.com/ArduCAM/Arduino/blob/master/OV7670FIFO/OV7670FIFO.ino

// Needed Color Correction, green to red
//   result = OV7670WriteReg(0xB0, 0x8c);
#define UNDOC_COLOR_CORRECTION 0xB0 // set to 0x8C or 0x84?

// Automatic White Balance
#define AWBC1	   	0x43
#define AWBC1_VALUE	0x14

#define AWBC2	   	0x44
#define AWBC2_VALUE	0xf0

#define AWBC3	   	0x45
#define AWBC3_VALUE  	0x34

#define AWBC4	   	0x46
#define AWBC4_VALUE	0x58

#define AWBC5	        0x47
#define AWBC5_VALUE	0x28

#define AWBC6	   	0x48
#define AWBC6_VALUE	0x3a

#define AWBC7           0x59
#define AWBC7_VALUE     0x88

#define AWBC8          0x5A
#define AWBC8_VALUE    0x88

#define AWBC9          0x5B
#define AWBC9_VALUE    0x44

#define AWBC10         0x5C
#define AWBC10_VALUE   0x67

#define AWBC11         0x5D
#define AWBC11_VALUE   0x49

#define AWBC12         0x5E
#define AWBC12_VALUE   0x0E

// Color Matrix Control YUV
#define YMTX1	   	0x4f
#define MTX1_VALUE	0x80

#define YMTX2	   	0x50
#define MTX2_VALUE	0x80

#define YMTX3	   	0x51
#define MTX3_VALUE	0x00

#define YMTX4	   	0x52
#define MTX4_VALUE	0x22

#define YMTX5	   	0x53
#define MTX5_VALUE	0x5e

#define YMTX6	   	0x54
#define MTX6_VALUE	0x80

#define YCONTRAS	   	0x56
#define CONTRAS_VALUE	0x40

#define YMTXS	   	0x58
#define MTXS_VALUE	0x9e

#endif //__REG_REGS_H__
