/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 *  @addtogroup ov7670_module OV7670 module
 *  @{
 */

#if !defined(__FSL_OV7670_DEF_H__)
#define __FSL_OV7670_DEF_H__

#include <stdint.h>

/*!
 * @addtogroup ov7670
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Register definitions for the OV7670.*/

#define OV7670_GAIN_REG                 0x00    ///< Gain lower 8 bits (rest in vref) 

#define OV7670_BLUE_REG                 0x01    ///< blue gain 

#define OV7670_RED_REG                  0x02    ///< red gain 

#define OV7670_VREF_REG                 0x03    ///< Pieces of GAIN, VSTART, VSTOP 

#define OV7670_COM1_REG                 0x04    ///< Control 1 
#define OV7670_COM1_CCIR656_MASK        0x40    ///< CCIR656 enable

#define OV7670_BAVE_REG                 0x05    ///< U/B Average level 

#define OV7670_GbAVE_REG                0x06    ///< Y/Gb Average level 

#define OV7670_AECHH_REG                0x07    ///< AEC MS 5 bits 

#define OV7670_RAVE_REG                 0x08    ///< V/R Average level

#define OV7670_COM2_REG                 0x09    ///< Control 2 
#define OV7670_COM2_SSLEEP_MASK         0x10    ///< Soft sleep mode 

#define OV7670_PID_REG                  0x0a    ///< Product ID MSB register address
#define OV7670_PID_NUM                  0x76    ///< Product ID 

#define OV7670_VER_REG                  0x0b    ///< Product ID LSB register address
#define OV7670_VER_NUM                  0x73    ///< Product VERION 

#define OV7670_COM3_REG                 0x0c    ///< Control 3
#define OV7670_COM3_SWAP_MASK           0x40    ///< Byte swap 
#define OV7670_COM3_SCALEEN_MASK        0x08    ///< Enable scaling 
#define OV7670_COM3_DCWEN_MASK          0x04    ///< Enable downsamp/crop/window

#define OV7670_COM4_REG                 0x0d    ///< Control 4 
    
#define OV7670_COM5_REG                 0x0e    ///< All "reserved" 
    
#define OV7670_COM6_REG                 0x0f    ///< Control 6 
    
#define OV7670_AECH_REG                 0x10    ///< More bits of AEC value 
    
#define OV7670_CLKRC_REG                0x11    ///< Clocl control 
#define OV7670_CLK_EXT_MASK             0x40    ///< Use external clock directly 
#define OV7670_CLK_SCALE_MASK           0x3f    ///< Mask for internal clock scale 

#define OV7670_COM7_REG                 0x12    ///< Control 7
#define OV7670_COM7_RESET_MASK          0x80    ///< Register reset 
#define OV7670_COM7_FMT_MASK_MASK       0x38    ///<
#define OV7670_COM7_FMT_VGA_MASK        0x00    ///<
#define OV7670_COM7_FMT_CIF_MASK        0x20    ///< CIF format 
#define OV7670_COM7_FMT_QVGA_MASK       0x10    ///< QVGA format 
#define OV7670_COM7_FMT_QCIF_MASK       0x08    ///< QCIF format 
#define OV7670_COM7_RGB_MASK            0x04    ///< bits 0 and 2 - RGB format 
#define OV7670_COM7_YUV_MASK            0x00    ///< YUV 
#define OV7670_COM7_BAYER_MASK          0x01    ///< Bayer format 
#define OV7670_COM7_PBAYER_MASK         0x05    ///< "Processed bayer" 

#define OV7670_COM8_REG                 0x13    ///< Control 8 
#define OV7670_COM8_FASTAEC_MASK        0x80    ///< Enable fast AGC/AEC 
#define OV7670_COM8_AECSTEP_MASK        0x40    ///< Unlimited AEC step size 
#define OV7670_COM8_BFILT_MASK          0x20    ///< Band filter enable 
#define OV7670_COM8_AGC_MASK            0x04    ///< Auto gain enable 
#define OV7670_COM8_AWB_MASK            0x02    ///< White balance enable 
#define OV7670_COM8_AEC_MASK            0x01    ///< Auto exposure enable 

#define OV7670_COM9_REG                 0x14    ///< Control 9  - gain ceiling 

#define OV7670_COM10_REG                0x15    ///< Control 10 
#define OV7670_COM10_HSYNC_MASK         0x40    ///< HSYNC instead of HREF 
#define OV7670_COM10_PCLK_HB_MASK       0x20    ///< Suppress PCLK on horiz blank 
#define OV7670_COM10_HREF_REV_MASK      0x08    ///< Reverse HREF 
#define OV7670_COM10_VS_LEAD_MASK       0x04    ///< VSYNC on clock leading edge 
#define OV7670_COM10_VS_NEG_MASK        0x02    ///< VSYNC negative 
#define OV7670_COM10_HS_NEG_MASK        0x01    ///< HSYNC negative 

#define OV7670_RSVD_REG                 0x16    ///< reserved 
    
#define OV7670_HSTART_REG               0x17    ///< Horiz start high bits 
    
#define OV7670_HSTOP_REG                0x18    ///< Horiz stop high bits 
    
#define OV7670_VSTART_REG               0x19    ///< Vert start high bits 
    
#define OV7670_VSTOP_REG                0x1a    ///< Vert stop high bits 
    
#define OV7670_PSHFT_REG                0x1b    ///< Pixel delay after HREF 
    
#define OV7670_MIDH_REG                 0x1c    ///< Manuf. ID high 
    
#define OV7670_MIDL_REG                 0x1d    ///< Manuf. ID low 
    
#define OV7670_MVFP_REG                 0x1e    ///< Mirror / vflip 
#define OV7670_MVFP_MIRROR_MASK         0x20    ///< Mirror image
#define OV7670_MVFP_FLIP_MASK           0x10    ///< Vertical flip
    
#define OV7670_LAEC_REG                 0x1f    ///< reserved

#define OV7670_ADCCTR0_REG              0x20    ///< ADC control
#define OV7670_ADCCTR0_RANGE_ADJ_MASK   0x08	///< ADC range adjustment

#define OV7670_ADCCTR1_REG              0x21    ///< reserved

#define OV7670_ADCCTR2_REG              0x22    ///< reserved
  
#define OV7670_ADCCTR3_REG              0x23    ///< reserved  
  
#define OV7670_AEW_REG                  0x24    ///< AGC upper limit 
  
#define OV7670_AEB_REG                  0x25    ///< AGC lower limit
  
#define OV7670_VPT_REG                  0x26    ///< AGC/AEC fast mode op region
  
#define OV7670_BBIAS_REG                0x27    ///< B channel signal output bias
  
#define OV7670_GbBIAS_REG               0x28    ///< Gb channel signal output bias
  
#define OV7670_RSVD1_REG                0x29    ///< reserved 1
   
#define OV7670_EXHCH_REG                0x2a    ///< dummy pixel insert MSB
   
#define OV7670_EXHCL_REG                0x2b    ///< dummy pixel insert LSB

#define OV7670_HSYST_REG                0x30    ///< HSYNC rising edge delay 
  
#define OV7670_HSYEN_REG                0x31    ///< HSYNC falling edge delay 
    
#define OV7670_HREF_REG                 0x32    ///< HREF pieces 
    
#define OV7670_CHLF_REG                 0x33    ///< array current control 
    
#define OV7670_ARBLM_REG                0x34    ///< array reference control
    
#define OV7670_RSVD2_REG                0x35    ///< reserved 2
    
#define OV7670_RSVD3_REG                0x36    ///< reserved 3
    
#define OV7670_ADC_REG                  0x37    ///< ADC control 
    
#define OV7670_ACOM_REG                 0x38    ///< ADC and Analog common mode control 
    
#define OV7670_OFON_REG                 0x39    ///< ADC offset control 
    
#define OV7670_TSLB_REG                 0x3a    ///< lots of stuff 
#define OV7670_TSLB_YLAST_MASK          0x04    ///< UYVY or VYUY - see com13 

#define OV7670_COM11_REG                0x3b    ///< Control 11
#define OV7670_COM11_NIGHT_MASK         0x80    ///< NIght mode enable 
#define OV7670_COM11_NMFR_MASK          0x60    ///< Two bit NM frame rate 
#define OV7670_COM11_HZAUTO_MASK        0x10    ///< Auto detect 50/60 Hz 
#define OV7670_COM11_50HZ_MASK          0x08    ///< Manual 50Hz select 
#define OV7670_COM11_EXP_MASK           0x02    ///<

#define OV7670_COM12_REG                0x3c    ///< Control 12
#define OV7670_COM12_HREF_MASK          0x80    ///< HREF always 

#define OV7670_COM13_REG                0x3d    ///< Control 13
#define OV7670_COM13_GAMMA_MASK         0x80    ///< Gamma enable 
#define OV7670_COM13_UVSAT_MASK         0x40    ///< UV saturation auto adjustment 
#define OV7670_COM13_UVSWAP_MASK        0x01    ///< V before U - w/TSLB 

#define OV7670_COM14_REG                0x3e    ///< Control 14 
#define OV7670_COM14_DCWEN_MASK         0x10    ///< DCW/PCLK-scale enable 

#define OV7670_EDGE_REG                 0x3f    ///< Edge enhancement factor 

#define OV7670_COM15_REG                0x40    ///< Control 15 
#define OV7670_COM15_R10F0_MASK         0x00    ///< Data range 10 to F0 
#define OV7670_COM15_R01FE_MASK         0x80    ///<            01 to FE 
#define OV7670_COM15_R00FF_MASK         0xc0    ///<            00 to FF 
#define OV7670_COM15_RGB565_MASK        0x10    ///< RGB565 output 
#define OV7670_COM15_RGB555_MASK        0x30    ///< RGB555 output 

#define OV7670_COM16_REG                0x41    ///< Control 16 
#define OV7670_COM16_AWBGAIN_MASK       0x08    ///< AWB gain enable 

#define OV7670_COM17_REG                0x42    ///< Control 17
#define OV7670_COM17_AECWIN_MASK        0xc0    ///< AEC window - must match COM4 
#define OV7670_COM17_CBAR_MASK          0x08    ///< DSP Color bar
   
#define OV7670_AWBC1_REG                0x43    ///< AWB control 1
   
#define OV7670_AWBC2_REG                0x44    ///< AWB control 2
   
#define OV7670_AWBC3_REG                0x45    ///< AWB control 3
   
#define OV7670_AWBC4_REG                0x46    ///< AWB control 4
   
#define OV7670_AWBC5_REG                0x47    ///< AWB control 5
   
#define OV7670_AWBC6_REG                0x48    ///< AWB control 6

#define OV7670_MTX1_REG                 0x4f    ///< Matrix Coefficient 1
  
#define OV7670_MTX2_REG                 0x50    ///< Matrix Coefficient 2 
  
#define OV7670_MTX3_REG                 0x51    ///< Matrix Coefficient 3 
  
#define OV7670_MTX4_REG                 0x52    ///< Matrix Coefficient 4 
  
#define OV7670_MTX5_REG                 0x53    ///< Matrix Coefficient 5 
  
#define OV7670_MTX6_REG                 0x54    ///< Matrix Coefficient 6 
  
#define OV7670_BRIGHT_REG               0x55    ///< Brightness
  
#define OV7670_CONTRAS_REG              0x56    ///< Contrast control

#define OV7670_CONTRAS_CENTER_REG       0x57    ///< Contrast cetnter control 
  
#define OV7670_MTXS_REG                 0x58    ///< Matrix Coefficient Sign
#define OV7670_AWBC7_MASK               0x59    ///< AWB Control 7 
#define OV7670_AWBC8_MASK               0x5a    ///< AWB Control 8 
#define OV7670_AWBC9_MASK               0x5b    ///< AWB Control 9 
#define OV7670_AWBC10_MASK              0x5c    ///< AWB Control 10 
#define OV7670_AWBC11_MASK              0x5d    ///< AWB Control 11 
#define OV7670_AWBC12_MASK              0x5e    ///< AWB Control 12 

#define OV7670_MANU_REG                 0x67    ///< Manual U value
   
#define OV7670_MANV_REG                 0x68    ///< Manual V value
   
#define OV7670_GFIX_REG                 0x69    ///< Fix gain control
  
#define OV7670_GGAIN_REG                0x6a    ///< G Channel AWB Gain 
  
#define OV7670_DBLV_REG                 0x6b    
  
#define OV7670_AWBCTR3_REG              0x6c    ///< AWB Control 3 
  
#define OV7670_AWBCTR2_REG              0x6d    ///< AWB Control 2 
  
#define OV7670_AWBCTR1_REG              0x6e    ///< AWB Control 1 
  
#define OV7670_AWBCTR0_REG              0x6f    ///< AWB Control 0

#define OV7670_SCALING_XSC_REG          0x70    ///< horizontal scale factor
   
#define OV7670_SCALING_YSC_REG          0x71    ///< vertical scale factor
   
#define OV7670_SCALING_DCWCTR_REG       0x72    ///< DCW control
   
#define OV7670_SCALING_PCLK_DIV_REG     0x73    ///< clock divider control
   
#define OV7670_REG74_REG                0x74    ///< register 74
  
#define OV7670_REG76_REG                0x76    ///< OV's name
#define OV7670_REG76_BLKPCOR_MASK       0x80    ///< Black pixel correction enable 
#define OV7670_REG76_WHTPCOR_MASK       0x40    ///< White pixel correction enable

#define OV7670_SLOP_REG                 0x7a    ///< gamma curve highest segment slop

#define OV7670_GAM1_REG                 0x7b    ///< gamma curve 1 segment slop
#define OV7670_GAM2_REG                 0x7c    ///< gamma curve 2 segment slop
#define OV7670_GAM3_REG                 0x7d    ///< gamma curve 3 segment slop
#define OV7670_GAM4_REG                 0x7e    ///< gamma curve 4 segment slop
#define OV7670_GAM5_REG                 0x7f    ///< gamma curve 5 segment slop
#define OV7670_GAM6_REG                 0x80    ///< gamma curve 6 segment slop
#define OV7670_GAM7_REG                 0x81    ///< gamma curve 7 segment slop
#define OV7670_GAM8_REG                 0x82    ///< gamma curve 8 segment slop
#define OV7670_GAM9_REG                 0x83    ///< gamma curve 9 segment slop
#define OV7670_GAM10_REG                0x84    ///< gamma curve 10 segment slop
#define OV7670_GAM11_REG                0x85    ///< gamma curve 11 segment slop
#define OV7670_GAM12_REG                0x86    ///< gamma curve 12 segment slop
#define OV7670_GAM13_REG                0x87    ///< gamma curve 13 segment slop
#define OV7670_GAM14_REG                0x88    ///< gamma curve 14 segment slop
#define OV7670_GAM15_REG                0x89    ///< gamma curve 15 segment slop

#define OV7670_RGB444_REG               0x8c    ///< RGB 444 control
#define OV7670_R444_ENABLE_MASK         0x02    ///< Turn on RGB444, overrides 5x5 
#define OV7670_R444_RGBX_MASK           0x01    ///< Empty nibble at end 

#define OV7670_DM_LNL_REG               0x92    ///< dummy line low 8 bits
   
#define OV7670_DM_LNH_REG               0x93    ///< dummy line high 8 bits

#define OV7670_BD50ST_REG               0x9d    ///< 50Hz banding filter value
  
#define OV7670_BD60ST_REG               0x9e    ///< 60Hz banding filter value

#define OV7670_HAECC1_REG               0x9f    ///< Hist AEC/AGC control 1 
      
#define OV7670_HAECC2_REG               0xa0    ///< Hist AEC/AGC control 2 

#define OV7670_SCALING_PCLK_DELAY_REG   0xa2    ///< pixel clock delay

#define OV7670_BD50MAX_REG              0xa5    ///< 50hz banding step limit 
      
#define OV7670_HAECC3_REG               0xa6    ///< Hist AEC/AGC control 3 
      
#define OV7670_HAECC4_REG               0xa7    ///< Hist AEC/AGC control 4 
      
#define OV7670_HAECC5_REG               0xa8    ///< Hist AEC/AGC control 5 
      
#define OV7670_HAECC6_REG               0xa9    ///< Hist AEC/AGC control 6 
      
#define OV7670_HAECC7_REG               0xaa    ///< Hist AEC/AGC control 7 
      
#define OV7670_BD60MAX_REG              0xab    ///< 60hz banding step limit

#define OV7670_STR_OPT_REG              0xac    ///< strobe control
   
#define OV7670_STR_R_REG                0xad    ///< R gain for LED output frame
   
#define OV7670_STR_G_REG                0xae    ///< G gain for LED output frame

#define OV7670_STR_B_REG                0xaf    ///< B gain for LED output frame
   
#define OV7670_ABLC1_REG                0xb1    ///< ABLC function control
   
#define OV7670_THL_ST_REG               0xb3    ///< ABLC target
 
#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /*__FSL_OV7670_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/