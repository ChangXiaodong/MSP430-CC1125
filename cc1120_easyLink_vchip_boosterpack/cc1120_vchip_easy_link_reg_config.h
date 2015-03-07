/*****************************************************************************/
//  Filename: cc1120_vchip_easy_link_reg_config.h  
//    
//  Description: Template for CC1120 register export from SmartRF Studio 
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
/****************************************************************************/
#ifndef CC1120_VCHIP_EASY_LINK_REG_CONFIG_H
#define CC1120_VCHIP_EASY_LINK_REG_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
 * INCLUDES
 */
#include "cc112x_spi.h"
#include "hal_msp_exp430g2_spi.h"

  
/******************************************************************************
 * FUNCTIONS
 */ 
   
// RX filter BW = 15.625000 
// Address config = No address check 
// Packet length = 255 
// Symbol rate = 1.2 
// PA ramping = true 
// Performance mode = High Performance 
// Carrier frequency = 869.224854 
// Bit rate = 1.2 
// Packet bit length = 0 
// Whitening = false 
// Manchester enable = false 
// Modulation format = 2-GFSK 
// Packet length mode = Variable 
// Device address = 0 
// TX power = 15 
// Deviation = 3.995895 

static const registerSetting_t cc1125PreferredSettings[]= 
{
  {CC112X_IOCFG3,              0xB0},
  {CC112X_IOCFG2,              0x06},
  {CC112X_IOCFG1,              0xB0},
  {CC112X_IOCFG0,              0x40},
  {CC112X_SYNC3,               0x93},
  {CC112X_SYNC2,               0x0B},
  {CC112X_SYNC1,               0x51},
  {CC112X_SYNC0,               0xDE},
  {CC112X_SYNC_CFG1,           0x0B},
  {CC112X_SYNC_CFG0,           0x17},
  {CC112X_DEVIATION_M,         0x06},
  {CC112X_MODCFG_DEV_E,        0x03},
  {CC112X_DCFILT_CFG,          0x1C},
  {CC112X_PREAMBLE_CFG1,       0x14},
  {CC112X_PREAMBLE_CFG0,       0x2A},
  {CC112X_FREQ_IF_CFG,         0x40},
  {CC112X_IQIC,                0xC6},
  {CC112X_CHAN_BW,             0x19},
  {CC112X_MDMCFG1,             0x46},
  {CC112X_MDMCFG0,             0x05},
  {CC112X_SYMBOL_RATE2,        0x43},
  {CC112X_SYMBOL_RATE1,        0xA9},
  {CC112X_SYMBOL_RATE0,        0x2A},
  {CC112X_AGC_REF,             0x20},
  {CC112X_AGC_CS_THR,          0x19},
  {CC112X_AGC_GAIN_ADJUST,     0x00},
  {CC112X_AGC_CFG3,            0x91},
  {CC112X_AGC_CFG2,            0x20},
  {CC112X_AGC_CFG1,            0xA9},
  {CC112X_AGC_CFG0,            0xCF},
  {CC112X_FIFO_CFG,            0x00},
  {CC112X_DEV_ADDR,            0x00},
  {CC112X_SETTLING_CFG,        0x0B},
  {CC112X_FS_CFG,              0x14},
  {CC112X_WOR_CFG1,            0x08},
  {CC112X_WOR_CFG0,            0x21},
  {CC112X_WOR_EVENT0_MSB,      0x00},
  {CC112X_WOR_EVENT0_LSB,      0x00},
  {CC112X_PKT_CFG2,            0x04},
  {CC112X_PKT_CFG1,            0x05},
  {CC112X_PKT_CFG0,            0x20},
  {CC112X_RFEND_CFG1,          0x0F},
  {CC112X_RFEND_CFG0,          0x00},
  {CC112X_PA_CFG2,             0x7F},
  {CC112X_PA_CFG1,             0x56},
  {CC112X_PA_CFG0,             0x7E},
  {CC112X_PKT_LEN,             0xFF},
  {CC112X_IF_MIX_CFG,          0x00},
  {CC112X_FREQOFF_CFG,         0x22},
  {CC112X_TOC_CFG,             0x0B},
  {CC112X_MARC_SPARE,          0x00},
  {CC112X_ECG_CFG,             0x00},
  {CC112X_CFM_DATA_CFG,        0x00},
  {CC112X_EXT_CTRL,            0x01},
  {CC112X_RCCAL_FINE,          0x00},
  {CC112X_RCCAL_COARSE,        0x00},
  {CC112X_RCCAL_OFFSET,        0x00},
  {CC112X_FREQOFF1,            0x00},
  {CC112X_FREQOFF0,            0x00},
  {CC112X_FREQ2,               0x6C},
  {CC112X_FREQ1,               0x80},
  {CC112X_FREQ0,               0x00},
  {CC112X_IF_ADC2,             0x02},
  {CC112X_IF_ADC1,             0xA6},
  {CC112X_IF_ADC0,             0x04},
  {CC112X_FS_DIG1,             0x00},
  {CC112X_FS_DIG0,             0x5F},
  {CC112X_FS_CAL3,             0x00},
  {CC112X_FS_CAL2,             0x20},
  {CC112X_FS_CAL1,             0x00},
  {CC112X_FS_CAL0,             0x0E},
  {CC112X_FS_CHP,              0x28},
  {CC112X_FS_DIVTWO,           0x03},
  {CC112X_FS_DSM1,             0x00},
  {CC112X_FS_DSM0,             0x33},
  {CC112X_FS_DVC1,             0xFF},
  {CC112X_FS_DVC0,             0x17},
  {CC112X_FS_LBI,              0x00},
  {CC112X_FS_PFD,              0x50},
  {CC112X_FS_PRE,              0x6E},
  {CC112X_FS_REG_DIV_CML,      0x14},
  {CC112X_FS_SPARE,            0xAC},
  {CC112X_FS_VCO4,             0x14},
  {CC112X_FS_VCO3,             0x00},
  {CC112X_FS_VCO2,             0x00},
  {CC112X_FS_VCO1,             0x00},
  {CC112X_FS_VCO0,             0x81},
  {CC112X_GBIAS6,              0x00},
  {CC112X_GBIAS5,              0x02},
  {CC112X_GBIAS4,              0x00},
  {CC112X_GBIAS3,              0x00},
  {CC112X_GBIAS2,              0x10},
  {CC112X_GBIAS1,              0x00},
  {CC112X_GBIAS0,              0x00},
  {CC112X_IFAMP,               0x01},
  {CC112X_LNA,                 0x01},
  {CC112X_RXMIX,               0x01},
  {CC112X_XOSC5,               0x0E},
  {CC112X_XOSC4,               0xA0},
  {CC112X_XOSC3,               0xC7},
  {CC112X_XOSC2,               0x04},
  {CC112X_XOSC1,               0x07},
  {CC112X_XOSC0,               0x00},
  {CC112X_ANALOG_SPARE,        0x00},
  {CC112X_PA_CFG3,             0x00},
  {CC112X_WOR_TIME1,           0x00},
  {CC112X_WOR_TIME0,           0x00},
  {CC112X_WOR_CAPTURE1,        0x00},
  {CC112X_WOR_CAPTURE0,        0x00},
  {CC112X_BIST,                0x00},
  {CC112X_DCFILTOFFSET_I1,     0x00},
  {CC112X_DCFILTOFFSET_I0,     0x00},
  {CC112X_DCFILTOFFSET_Q1,     0x00},
  {CC112X_DCFILTOFFSET_Q0,     0x00},
  {CC112X_IQIE_I1,             0x00},
  {CC112X_IQIE_I0,             0x00},
  {CC112X_IQIE_Q1,             0x00},
  {CC112X_IQIE_Q0,             0x00},
  {CC112X_RSSI1,               0x80},
  {CC112X_RSSI0,               0x00},
  {CC112X_MARCSTATE,           0x41},
  {CC112X_LQI_VAL,             0x00},
  {CC112X_PQT_SYNC_ERR,        0xFF},
  {CC112X_DEM_STATUS,          0x00},
  {CC112X_FREQOFF_EST1,        0x00},
  {CC112X_FREQOFF_EST0,        0x00},
  {CC112X_AGC_GAIN3,           0x00},
  {CC112X_AGC_GAIN2,           0xD1},
  {CC112X_AGC_GAIN1,           0x00},
  {CC112X_AGC_GAIN0,           0x3F},
  {CC112X_CFM_RX_DATA_OUT,     0x00},
  {CC112X_CFM_TX_DATA_IN,      0x00},
  {CC112X_ASK_SOFT_RX_DATA,    0x30},
  {CC112X_RNDGEN,              0x7F},
  {CC112X_MAGN2,               0x00},
  {CC112X_MAGN1,               0x00},
  {CC112X_MAGN0,               0x00},
  {CC112X_ANG1,                0x00},
  {CC112X_ANG0,                0x00},
  {CC112X_CHFILT_I2,           0x08},
  {CC112X_CHFILT_I1,           0x00},
  {CC112X_CHFILT_I0,           0x00},
  {CC112X_CHFILT_Q2,           0x00},
  {CC112X_CHFILT_Q1,           0x00},
  {CC112X_CHFILT_Q0,           0x00},
  {CC112X_GPIO_STATUS,         0x00},
  {CC112X_FSCAL_CTRL,          0x01},
  {CC112X_PHASE_ADJUST,        0x00},
  {CC112X_PARTNUMBER,          0x00},
  {CC112X_PARTVERSION,         0x00},
  {CC112X_SERIAL_STATUS,       0x00},
  {CC112X_MODEM_STATUS1,       0x01},
  {CC112X_MODEM_STATUS0,       0x00},
  {CC112X_MARC_STATUS1,        0x00},
  {CC112X_MARC_STATUS0,        0x00},
  {CC112X_PA_IFAMP_TEST,       0x00},
  {CC112X_FSRF_TEST,           0x00},
  {CC112X_PRE_TEST,            0x00},
  {CC112X_PRE_OVR,             0x00},
  {CC112X_ADC_TEST,            0x00},
  {CC112X_DVC_TEST,            0x0B},
  {CC112X_ATEST,               0x40},
  {CC112X_ATEST_LVDS,          0x00},
  {CC112X_ATEST_MODE,          0x00},
  {CC112X_XOSC_TEST1,          0x3C},
  {CC112X_XOSC_TEST0,          0x00},
  {CC112X_RXFIRST,             0x00},
  {CC112X_TXFIRST,             0x00},
  {CC112X_RXLAST,              0x00},
  {CC112X_TXLAST,              0x00},
  {CC112X_NUM_TXBYTES,         0x00},
  {CC112X_NUM_RXBYTES,         0x00},
  {CC112X_FIFO_NUM_TXBYTES,    0x0F},
  {CC112X_FIFO_NUM_RXBYTES,    0x00},
};

#ifdef  __cplusplus
}
#endif

#endif