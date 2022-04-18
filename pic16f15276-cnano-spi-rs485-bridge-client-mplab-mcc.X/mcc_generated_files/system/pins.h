/**
 * Generated Pins header File
 * 
 * @file pins.h
 * 
 * @defgroup  pinsdriver Pins Driver
 * 
 * @brief This is generated driver header for pins. 
 *        This header file provides APIs for all pins selected in the GUI.
 *
 * @version Driver Version  3.0.0
*/

/*
© [2022] Microchip Technology Inc. and its subsidiaries.

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

#ifndef PINS_H
#define PINS_H

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set IO_RA6 aliases
#define SS_Thermo_TRIS                 TRISAbits.TRISA6
#define SS_Thermo_LAT                  LATAbits.LATA6
#define SS_Thermo_PORT                 PORTAbits.RA6
#define SS_Thermo_WPU                  WPUAbits.WPUA6
#define SS_Thermo_OD                   ODCONAbits.ODCA6
#define SS_Thermo_ANS                  ANSELAbits.ANSA6
#define SS_Thermo_SetHigh()            do { LATAbits.LATA6 = 1; } while(0)
#define SS_Thermo_SetLow()             do { LATAbits.LATA6 = 0; } while(0)
#define SS_Thermo_Toggle()             do { LATAbits.LATA6 = ~LATAbits.LATA6; } while(0)
#define SS_Thermo_GetValue()           PORTAbits.RA6
#define SS_Thermo_SetDigitalInput()    do { TRISAbits.TRISA6 = 1; } while(0)
#define SS_Thermo_SetDigitalOutput()   do { TRISAbits.TRISA6 = 0; } while(0)
#define SS_Thermo_SetPullup()          do { WPUAbits.WPUA6 = 1; } while(0)
#define SS_Thermo_ResetPullup()        do { WPUAbits.WPUA6 = 0; } while(0)
#define SS_Thermo_SetPushPull()        do { ODCONAbits.ODCA6 = 0; } while(0)
#define SS_Thermo_SetOpenDrain()       do { ODCONAbits.ODCA6 = 1; } while(0)
#define SS_Thermo_SetAnalogMode()      do { ANSELAbits.ANSA6 = 1; } while(0)
#define SS_Thermo_SetDigitalMode()     do { ANSELAbits.ANSA6 = 0; } while(0)

// get/set IO_RA7 aliases
#define DE_TRIS                 TRISAbits.TRISA7
#define DE_LAT                  LATAbits.LATA7
#define DE_PORT                 PORTAbits.RA7
#define DE_WPU                  WPUAbits.WPUA7
#define DE_OD                   ODCONAbits.ODCA7
#define DE_ANS                  ANSELAbits.ANSA7
#define DE_SetHigh()            do { LATAbits.LATA7 = 1; } while(0)
#define DE_SetLow()             do { LATAbits.LATA7 = 0; } while(0)
#define DE_Toggle()             do { LATAbits.LATA7 = ~LATAbits.LATA7; } while(0)
#define DE_GetValue()           PORTAbits.RA7
#define DE_SetDigitalInput()    do { TRISAbits.TRISA7 = 1; } while(0)
#define DE_SetDigitalOutput()   do { TRISAbits.TRISA7 = 0; } while(0)
#define DE_SetPullup()          do { WPUAbits.WPUA7 = 1; } while(0)
#define DE_ResetPullup()        do { WPUAbits.WPUA7 = 0; } while(0)
#define DE_SetPushPull()        do { ODCONAbits.ODCA7 = 0; } while(0)
#define DE_SetOpenDrain()       do { ODCONAbits.ODCA7 = 1; } while(0)
#define DE_SetAnalogMode()      do { ANSELAbits.ANSA7 = 1; } while(0)
#define DE_SetDigitalMode()     do { ANSELAbits.ANSA7 = 0; } while(0)

// get/set IO_RB0 aliases
#define IO_RB0_TRIS                 TRISBbits.TRISB0
#define IO_RB0_LAT                  LATBbits.LATB0
#define IO_RB0_PORT                 PORTBbits.RB0
#define IO_RB0_WPU                  WPUBbits.WPUB0
#define IO_RB0_OD                   ODCONBbits.ODCB0
#define IO_RB0_ANS                  ANSELBbits.ANSB0
#define IO_RB0_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define IO_RB0_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define IO_RB0_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define IO_RB0_GetValue()           PORTBbits.RB0
#define IO_RB0_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define IO_RB0_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define IO_RB0_SetPullup()          do { WPUBbits.WPUB0 = 1; } while(0)
#define IO_RB0_ResetPullup()        do { WPUBbits.WPUB0 = 0; } while(0)
#define IO_RB0_SetPushPull()        do { ODCONBbits.ODCB0 = 0; } while(0)
#define IO_RB0_SetOpenDrain()       do { ODCONBbits.ODCB0 = 1; } while(0)
#define IO_RB0_SetAnalogMode()      do { ANSELBbits.ANSB0 = 1; } while(0)
#define IO_RB0_SetDigitalMode()     do { ANSELBbits.ANSB0 = 0; } while(0)

// get/set IO_RB1 aliases
#define IO_RB1_TRIS                 TRISBbits.TRISB1
#define IO_RB1_LAT                  LATBbits.LATB1
#define IO_RB1_PORT                 PORTBbits.RB1
#define IO_RB1_WPU                  WPUBbits.WPUB1
#define IO_RB1_OD                   ODCONBbits.ODCB1
#define IO_RB1_ANS                  ANSELBbits.ANSB1
#define IO_RB1_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define IO_RB1_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define IO_RB1_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define IO_RB1_GetValue()           PORTBbits.RB1
#define IO_RB1_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define IO_RB1_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define IO_RB1_SetPullup()          do { WPUBbits.WPUB1 = 1; } while(0)
#define IO_RB1_ResetPullup()        do { WPUBbits.WPUB1 = 0; } while(0)
#define IO_RB1_SetPushPull()        do { ODCONBbits.ODCB1 = 0; } while(0)
#define IO_RB1_SetOpenDrain()       do { ODCONBbits.ODCB1 = 1; } while(0)
#define IO_RB1_SetAnalogMode()      do { ANSELBbits.ANSB1 = 1; } while(0)
#define IO_RB1_SetDigitalMode()     do { ANSELBbits.ANSB1 = 0; } while(0)

// get/set IO_RC2 aliases
#define IO_RC2_TRIS                 TRISCbits.TRISC2
#define IO_RC2_LAT                  LATCbits.LATC2
#define IO_RC2_PORT                 PORTCbits.RC2
#define IO_RC2_WPU                  WPUCbits.WPUC2
#define IO_RC2_OD                   ODCONCbits.ODCC2
#define IO_RC2_ANS                  ANSELCbits.ANSC2
#define IO_RC2_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define IO_RC2_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define IO_RC2_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define IO_RC2_GetValue()           PORTCbits.RC2
#define IO_RC2_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define IO_RC2_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define IO_RC2_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define IO_RC2_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define IO_RC2_SetPushPull()        do { ODCONCbits.ODCC2 = 0; } while(0)
#define IO_RC2_SetOpenDrain()       do { ODCONCbits.ODCC2 = 1; } while(0)
#define IO_RC2_SetAnalogMode()      do { ANSELCbits.ANSC2 = 1; } while(0)
#define IO_RC2_SetDigitalMode()     do { ANSELCbits.ANSC2 = 0; } while(0)

// get/set IO_RC5 aliases
#define IO_RC5_TRIS                 TRISCbits.TRISC5
#define IO_RC5_LAT                  LATCbits.LATC5
#define IO_RC5_PORT                 PORTCbits.RC5
#define IO_RC5_WPU                  WPUCbits.WPUC5
#define IO_RC5_OD                   ODCONCbits.ODCC5
#define IO_RC5_ANS                  ANSELCbits.ANSC5
#define IO_RC5_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define IO_RC5_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define IO_RC5_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define IO_RC5_GetValue()           PORTCbits.RC5
#define IO_RC5_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define IO_RC5_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define IO_RC5_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define IO_RC5_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)
#define IO_RC5_SetPushPull()        do { ODCONCbits.ODCC5 = 0; } while(0)
#define IO_RC5_SetOpenDrain()       do { ODCONCbits.ODCC5 = 1; } while(0)
#define IO_RC5_SetAnalogMode()      do { ANSELCbits.ANSC5 = 1; } while(0)
#define IO_RC5_SetDigitalMode()     do { ANSELCbits.ANSC5 = 0; } while(0)

// get/set IO_RC6 aliases
#define IO_RC6_TRIS                 TRISCbits.TRISC6
#define IO_RC6_LAT                  LATCbits.LATC6
#define IO_RC6_PORT                 PORTCbits.RC6
#define IO_RC6_WPU                  WPUCbits.WPUC6
#define IO_RC6_OD                   ODCONCbits.ODCC6
#define IO_RC6_ANS                  ANSELCbits.ANSC6
#define IO_RC6_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define IO_RC6_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define IO_RC6_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define IO_RC6_GetValue()           PORTCbits.RC6
#define IO_RC6_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define IO_RC6_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define IO_RC6_SetPullup()          do { WPUCbits.WPUC6 = 1; } while(0)
#define IO_RC6_ResetPullup()        do { WPUCbits.WPUC6 = 0; } while(0)
#define IO_RC6_SetPushPull()        do { ODCONCbits.ODCC6 = 0; } while(0)
#define IO_RC6_SetOpenDrain()       do { ODCONCbits.ODCC6 = 1; } while(0)
#define IO_RC6_SetAnalogMode()      do { ANSELCbits.ANSC6 = 1; } while(0)
#define IO_RC6_SetDigitalMode()     do { ANSELCbits.ANSC6 = 0; } while(0)

// get/set IO_RC7 aliases
#define SS_Pressure_TRIS                 TRISCbits.TRISC7
#define SS_Pressure_LAT                  LATCbits.LATC7
#define SS_Pressure_PORT                 PORTCbits.RC7
#define SS_Pressure_WPU                  WPUCbits.WPUC7
#define SS_Pressure_OD                   ODCONCbits.ODCC7
#define SS_Pressure_ANS                  ANSELCbits.ANSC7
#define SS_Pressure_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define SS_Pressure_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define SS_Pressure_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define SS_Pressure_GetValue()           PORTCbits.RC7
#define SS_Pressure_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define SS_Pressure_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define SS_Pressure_SetPullup()          do { WPUCbits.WPUC7 = 1; } while(0)
#define SS_Pressure_ResetPullup()        do { WPUCbits.WPUC7 = 0; } while(0)
#define SS_Pressure_SetPushPull()        do { ODCONCbits.ODCC7 = 0; } while(0)
#define SS_Pressure_SetOpenDrain()       do { ODCONCbits.ODCC7 = 1; } while(0)
#define SS_Pressure_SetAnalogMode()      do { ANSELCbits.ANSC7 = 1; } while(0)
#define SS_Pressure_SetDigitalMode()     do { ANSELCbits.ANSC7 = 0; } while(0)

// get/set IO_RD5 aliases
#define RE_TRIS                 TRISDbits.TRISD5
#define RE_LAT                  LATDbits.LATD5
#define RE_PORT                 PORTDbits.RD5
#define RE_WPU                  WPUDbits.WPUD5
#define RE_OD                   ODCONDbits.ODCD5
#define RE_ANS                  ANSELDbits.ANSD5
#define RE_SetHigh()            do { LATDbits.LATD5 = 1; } while(0)
#define RE_SetLow()             do { LATDbits.LATD5 = 0; } while(0)
#define RE_Toggle()             do { LATDbits.LATD5 = ~LATDbits.LATD5; } while(0)
#define RE_GetValue()           PORTDbits.RD5
#define RE_SetDigitalInput()    do { TRISDbits.TRISD5 = 1; } while(0)
#define RE_SetDigitalOutput()   do { TRISDbits.TRISD5 = 0; } while(0)
#define RE_SetPullup()          do { WPUDbits.WPUD5 = 1; } while(0)
#define RE_ResetPullup()        do { WPUDbits.WPUD5 = 0; } while(0)
#define RE_SetPushPull()        do { ODCONDbits.ODCD5 = 0; } while(0)
#define RE_SetOpenDrain()       do { ODCONDbits.ODCD5 = 1; } while(0)
#define RE_SetAnalogMode()      do { ANSELDbits.ANSD5 = 1; } while(0)
#define RE_SetDigitalMode()     do { ANSELDbits.ANSD5 = 0; } while(0)

// get/set IO_RD6 aliases
#define INT_Pressure_TRIS                 TRISDbits.TRISD6
#define INT_Pressure_LAT                  LATDbits.LATD6
#define INT_Pressure_PORT                 PORTDbits.RD6
#define INT_Pressure_WPU                  WPUDbits.WPUD6
#define INT_Pressure_OD                   ODCONDbits.ODCD6
#define INT_Pressure_ANS                  ANSELDbits.ANSD6
#define INT_Pressure_SetHigh()            do { LATDbits.LATD6 = 1; } while(0)
#define INT_Pressure_SetLow()             do { LATDbits.LATD6 = 0; } while(0)
#define INT_Pressure_Toggle()             do { LATDbits.LATD6 = ~LATDbits.LATD6; } while(0)
#define INT_Pressure_GetValue()           PORTDbits.RD6
#define INT_Pressure_SetDigitalInput()    do { TRISDbits.TRISD6 = 1; } while(0)
#define INT_Pressure_SetDigitalOutput()   do { TRISDbits.TRISD6 = 0; } while(0)
#define INT_Pressure_SetPullup()          do { WPUDbits.WPUD6 = 1; } while(0)
#define INT_Pressure_ResetPullup()        do { WPUDbits.WPUD6 = 0; } while(0)
#define INT_Pressure_SetPushPull()        do { ODCONDbits.ODCD6 = 0; } while(0)
#define INT_Pressure_SetOpenDrain()       do { ODCONDbits.ODCD6 = 1; } while(0)
#define INT_Pressure_SetAnalogMode()      do { ANSELDbits.ANSD6 = 1; } while(0)
#define INT_Pressure_SetDigitalMode()     do { ANSELDbits.ANSD6 = 0; } while(0)

// get/set IO_RE2 aliases
#define LED_TRIS                 TRISEbits.TRISE2
#define LED_LAT                  LATEbits.LATE2
#define LED_PORT                 PORTEbits.RE2
#define LED_WPU                  WPUEbits.WPUE2
#define LED_OD                   ODCONEbits.ODCE2
#define LED_ANS                  ANSELEbits.ANSE2
#define LED_SetHigh()            do { LATEbits.LATE2 = 1; } while(0)
#define LED_SetLow()             do { LATEbits.LATE2 = 0; } while(0)
#define LED_Toggle()             do { LATEbits.LATE2 = ~LATEbits.LATE2; } while(0)
#define LED_GetValue()           PORTEbits.RE2
#define LED_SetDigitalInput()    do { TRISEbits.TRISE2 = 1; } while(0)
#define LED_SetDigitalOutput()   do { TRISEbits.TRISE2 = 0; } while(0)
#define LED_SetPullup()          do { WPUEbits.WPUE2 = 1; } while(0)
#define LED_ResetPullup()        do { WPUEbits.WPUE2 = 0; } while(0)
#define LED_SetPushPull()        do { ODCONEbits.ODCE2 = 0; } while(0)
#define LED_SetOpenDrain()       do { ODCONEbits.ODCE2 = 1; } while(0)
#define LED_SetAnalogMode()      do { ANSELEbits.ANSE2 = 1; } while(0)
#define LED_SetDigitalMode()     do { ANSELEbits.ANSE2 = 0; } while(0)

/**
 * @ingroup  pinsdriver
 * @brief GPIO and peripheral I/O initialization
 * @param none
 * @return none
 */
void PIN_MANAGER_Initialize (void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handling routine
 * @param none
 * @return none
 */
void PIN_MANAGER_IOC(void);


#endif // PINS_H
/**
 End of File
*/