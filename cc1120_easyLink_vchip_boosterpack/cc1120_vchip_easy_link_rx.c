/*****************************************************************************/
//  Filename:        cc1120_vchip_easy_link_tx.c
//
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


/*****************************************************************************
* INCLUDES
*/
#include "msp430.h"
#include "hal_board.h"
#include "cc112x_spi.h"
#include "hal_int_rf_msp_exp430g2.h"
#include "cc1120_vchip_easy_link_reg_config.h"
#include "stdlib.h"
/******************************************************************************
 * CONSTANTS
 */ 

/******************************************************************************
* DEFINES
*/
#define ISR_ACTION_REQUIRED 1
#define ISR_IDLE            0
#define RX_FIFO_ERROR       0x11

#define PKTLEN              5
/******************************************************************************
* LOCAL VARIABLES
*/
static uint8  packetSemaphore;
static uint32 packetCounter;

/******************************************************************************
* STATIC FUNCTIONS
*/
static void registerConfig(void);
static void runRX(void);
static void createPacket(uint8 txBuffer[]);
static void radioRxTxISR(void);
static void manualCalibration(void);
/******************************************************************************
 * @fn          main
 *
 * @brief       Runs the main routine
 *                
 * @param       none
 *
 * @return      none
 */



uint8 test = 0;
uint8 buffer[255];
uint8 write = 0;
void main(void)
{
  //init MCU
  halInitMCU();
  Uart_init();
  //init LEDs
  //halLedInit();
  //init button
  //halButtonInit();
  //halButtonInterruptEnable();
  // init spi
  exp430RfSpiInit();
//  while(1)
//  {
////    SPI_BEGIN();
////    for(int i=0;i<255;i++)
////      SPI_TX(i);
////    SPI_WAIT_DONE();
////    SPI_END();
////    for(int j=0;j<500;j++)
////      NOP();
//    
//    SPI_BEGIN();
//    for(int i=0;i<255;i++)
//    {
//        SPI_WAIT_DONE();
//        buffer[i]=SPI_RX();
//    }
//      
//    
//    SPI_END();
//  
//  }
  // write radio registers
  registerConfig();
  
  cc112xSpiReadReg(CC112X_IOCFG3, &buffer[0], 1);
  cc112xSpiReadReg(CC112X_IOCFG2, &buffer[1], 1);
  cc112xSpiReadReg(CC112X_IOCFG1, &buffer[2], 1);
  cc112xSpiReadReg(CC112X_IOCFG0, &buffer[3], 1);
  cc112xSpiReadReg(CC112X_SYNC3, &buffer[4], 1);
  cc112xSpiReadReg(CC112X_SYNC2, &buffer[5], 1);
  
  
  
  // run either TX or RX dependent of build define  
  runRX();
 
}
/******************************************************************************
 * @fn          runTX
 *
 * @brief       sends one packet on button push. Updates packet counter and
 *              display for each packet sent.
 *                
 * @param       none
 *
 * @return      none
 */
uint8 rxBuffer[128] = {0};
uint8 rxBytes;
uint8 marcStatus;
uint8 status = 0;
uint8 test_buffer[128] = {0};
uint8 rssi = 0;
uint8 status1[5];
static void runRX(void)
{
  //status = trxSpiCmdStrobe(CC112X_SNOP);
  manualCalibration(); 
  trxSpiCmdStrobe(CC112X_SRX);
  cc112xSpiReadReg(CC112X_RSSI1,&rssi,1);
  cc112xSpiReadReg(CC112X_MARCSTATE,&status1[0],1);
  //status = trxSpiCmdStrobe(CC112X_SNOP);
  // infinite loop
  while(TRUE)
  {
    cc112xSpiReadReg(CC112X_NUM_RXBYTES, &rxBytes, 1);
    //cc112xSpiReadReg(CC112X_RSSI1,&rssi,1);
    //cc112xSpiReadReg(CC112X_MARCSTATE,&status1[1],1);
    //cc112xSpiReadRxFifo(rxBuffer, 10);
    trxSpiCmdStrobe(CC112X_SRX);
    //status = trxSpiCmdStrobe(CC112X_SNOP);
    cc112xSpiReadReg(CC112X_MARCSTATE,&status1[2],1);
    if(rxBytes!=0)
    {
        cc112xSpiReadRxFifo(rxBuffer, rxBytes);  
        Uart_Write(rxBuffer,rxBytes);
    }
    
    if((status1[2] & 0x1F) == RX_FIFO_ERROR){
          
          // Flush RX Fifo
          trxSpiCmdStrobe(CC112X_SFRX);
     }
    for(int j=0;j<20000;j++)
      NOP();
  }
}
/*******************************************************************************
* @fn          radioRxTxISR
*
* @brief       ISR for packet handling in RX. Sets packet semaphore, puts radio
*              in idle state and clears isr flag.
*
* @param       none
*
* @return      none
*/
static void radioRxTxISR(void) {

  // set packet semaphore
  packetSemaphore = ISR_ACTION_REQUIRED;
  // clear isr flag
  trxClearIntFlag(GPIO_0);
}
/*******************************************************************************
* @fn          registerConfig
*
* @brief       Write register settings as given by SmartRF Studio
*
* @param       none
*
* @return      none
*/
static void registerConfig(void) {
  uint8 writeByte;
  
  // reset radio
  trxSpiCmdStrobe(CC112X_SRES);
  // write registers to radio
  for(uint16 i = 0; i < (sizeof  cc1125PreferredSettings/sizeof(registerSetting_t)); i++) {
      writeByte =  cc1125PreferredSettings[i].data;
      cc112xSpiWriteReg( cc1125PreferredSettings[i].addr, &writeByte, 1);
    }
}
/******************************************************************************
 * @fn          createPacket
 *
 * @brief       This function is called before a packet is transmitted. It fills
 *              the txBuffer with a packet consisting of a length byte, two
 *              bytes packet counter and n random bytes.
 *
 *              The packet format is as follows:
 *              |--------------------------------------------------------------|
 *              |           |           |           |         |       |        |
 *              | pktLength | pktCount1 | pktCount0 | rndData |.......| rndData|
 *              |           |           |           |         |       |        |
 *              |--------------------------------------------------------------|
 *               txBuffer[0] txBuffer[1] txBuffer[2]  ......... txBuffer[PKTLEN]
 *                
 * @param       pointer to start of txBuffer
 *
 * @return      none
 */
static void createPacket(uint8 txBuffer[])
{
  
  txBuffer[0] = PKTLEN;                     // Length byte
  txBuffer[1] = (uint8) packetCounter >> 8; // MSB of packetCounter
  txBuffer[2] = (uint8) packetCounter;      // LSB of packetCounter
  
  // fill rest of buffer with random bytes
  for(uint8 i =3; i< (PKTLEN+1); i++)
  {
    txBuffer[i] = (uint8)rand();
  }
}
/******************************************************************************
 * @fn          manualCalibration
 *
 * @brief       calibrates radio according to CC112x errata
 *                
 * @param       none
 *
 * @return      none
 */
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2

uint8 marcstate;
static void manualCalibration(void){
  
    uint8 original_fs_cal2;
    uint8 calResults_for_vcdac_start_high[3];
    uint8 calResults_for_vcdac_start_mid[3];

    uint8 writeByte;
    
    // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
    
    // 2) Start with high VCDAC (original VCDAC_START + 2):
    cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
    writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);
    
    // 3) Calibrate and wait for calibration to be done (radio back in IDLE state)
    trxSpiCmdStrobe(CC112X_SCAL);
    
    do 
    {
        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
    } while (marcstate != 0x41);
    
    // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with high VCDAC_START value
    cc112xSpiReadReg(CC112X_FS_VCO2, &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_VCO4, &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_CHP, &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);
    
    // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
    
    // 6) Continue with mid VCDAC (original VCDAC_START):
    writeByte = original_fs_cal2;
    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);
    
    // 7) Calibrate and wait for calibration to be done (radio back in IDLE state)
    trxSpiCmdStrobe(CC112X_SCAL);
    
    do 
    {
        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
    } while (marcstate != 0x41);
    
    // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with mid VCDAC_START value
    cc112xSpiReadReg(CC112X_FS_VCO2, &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_VCO4, &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_CHP, &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);
    
    // 9) Write back highest FS_VCO2 and corresponding FS_VCO and FS_CHP result
    if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] > calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) 
    {
        writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
    }
    else 
    {
        writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
    }
}