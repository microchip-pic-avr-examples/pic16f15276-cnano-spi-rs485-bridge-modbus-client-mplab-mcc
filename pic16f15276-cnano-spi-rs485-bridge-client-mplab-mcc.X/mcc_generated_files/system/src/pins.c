/**
 * Generated Driver File
 * 
 * @file pins.c
 * 
 * @ingroup  pinsdriver
 * 
 * @brief This is generated driver implementation for pins. 
 *        This file provides implementations for pin APIs for all pins selected in the GUI.
 *
 * @version Driver Version 3.0.0
*/

/*
� [2022] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#include "../pins.h"


void PIN_MANAGER_Initialize(void)
{
   /**
    LATx registers
    */
    LATA = 0x0;
    LATB = 0x0;
    LATC = 0x0;
    LATD = 0x0;
    LATE = 0x4;

    /**
    TRISx registers
    */
    TRISA = 0x3F;
    TRISB = 0xFE;
    TRISC = 0x3B;
    TRISD = 0xDF;
    TRISE = 0xB;

    /**
    ANSELx registers
    */
    ANSELA = 0x3F;
    ANSELB = 0xFC;
    ANSELC = 0x1F;
    ANSELD = 0x9F;
    ANSELE = 0x3;

    /**
    WPUx registers
    */
    WPUA = 0x0;
    WPUB = 0x0;
    WPUC = 0x0;
    WPUD = 0x0;
    WPUE = 0x0;
  

    /**
    APFCONx registers
    */

    /**
    ODx registers
    */
   
    ODCONA = 0x0;
    ODCONB = 0x0;
    ODCONC = 0x0;
    ODCOND = 0x0;
    ODCONE = 0x0;
    /**
    SLRCONx registers
    */
    SLRCONA = 0xFF;
    SLRCONB = 0xFF;
    SLRCONC = 0xFF;
    SLRCOND = 0xFF;
    SLRCONE = 0x7;
    /**
    INLVLx registers
    */
    INLVLA = 0xFF;
    INLVLB = 0xFF;
    INLVLC = 0xFF;
    INLVLD = 0xFF;
    INLVLE = 0xF;

    /**
    PPS registers
    */
    RX1PPS = 0x9; //RB1->EUSART1:RX1;
    SSP1DATPPS = 0x15; //RC5->MSSP1:SDI1;
    RB0PPS = 0x05;  //RB0->EUSART1:TX1;
    RC6PPS = 0x07;  //RC6->MSSP1:SCK1;
    RC2PPS = 0x08;  //RC2->MSSP1:SDO1;
	
    
    SSP1CLKPPS = 0x16;   //RC6->MSSP1:SCK1;  //edited
    
    
   /**
    IOCx registers 
    */
    IOCAP = 0x0;
    IOCAN = 0x0;
    IOCAF = 0x0;
    IOCBP = 0x0;
    IOCBN = 0x0;
    IOCBF = 0x0;
    IOCCP = 0x0;
    IOCCN = 0x0;
    IOCCF = 0x0;
    IOCEP = 0x0;
    IOCEN = 0x0;
    IOCEF = 0x0;
      

}
  
void PIN_MANAGER_IOC(void)
{
}
/**
 End of File
*/