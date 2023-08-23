#include "mcc_generated_files/spi/mssp1.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/system/clock.h"
#include "mcc_generated_files/uart/eusart1.h"
#include "application.h"
#include "modbus.h"
#include "max31855k.h"
#include "bmp388.h"
#include <stdint.h>
#include <string.h>

#define SENSOR_DATA_REGISTER_ADDRESS (*(volatile uint8_t*)0x75u)   //Ram location address to store temperature sensor data

float sensor_Data_Register[2] __at(0x75);
volatile uint8_t sensorReadFlag;

extern volatile uint8_t eusart1TxHead;
extern volatile uint8_t eusart1TxTail;
extern volatile uint8_t eusart1TxBuffer[BUFFER_SIZE];
extern volatile uint8_t eusart1TxBufferRemaining;
extern volatile uint8_t eusart1RxHead;
extern volatile uint8_t eusart1RxTail;
extern volatile uint8_t eusart1RxCount;

sensorData_t sensordata_t;
ModbusRxFlags_t modBUSRxFlags_t;

static void ftoa(float value_f, uint8_t *res_uc); 
static void CnanoLED_TurnOFF(void);
static void CnanoLED_TurnOn(void);
static void ReadSensors(void);
void EUSART_Flush(void);

/*******************************************************************************
 * void Application(void)
 *
 * Read temperature and pressure from SPI sensors, send requested information 
 * (holding register data) to host over modbus
 *
 * @param None
 * @return None
 ******************************************************************************/
void Application(void) 
{ 
    bool dataReceived = NOT_RECEIVED;   
    
    if(sensorReadFlag == true)      //timer overflow every 3s
    {
       sensorReadFlag = false;
       CnanoLED_TurnOn();
       ReadSensors();                    //Read temperature and pressure sensor
       EUSART1PinSwap(DATAVISUALIZER); 
       SensorDataToClientTerminal();
       RS485RxEnable();                     //Enabling RS485 receiver for receive Rx commands
       EUSART1PinSwap(RS485);               //Swap EUSART1 pins to RC0, RC1  for connecting to RS485 module
       CnanoLED_TurnOFF();
    }
    dataReceived = ReceiveModbusRTUFrame();      //Receive RTU command from the host
    if (dataReceived == RECEIVED) 
    {
        dataReceived = NOT_RECEIVED;
        EUSART1PinSwap(DATAVISUALIZER);          //Swap pin to send data to terminal
        EUSART1_SendString((uint8_t*) "\r\nReceived Host MODBUS Frame: ", (uint8_t*) modbusRTUbuffer,HOST_MODBUS_FRAME_LENGTH + 2);   //Send Modbus frame to data visualizer
        EUSART1_SendString((uint8_t*)"\r\n------------------------------",(uint8_t*)"\r\n",NEW_LINE_LENGTH);  
        ValidateModbusQueryFrame(HOST_MODBUS_FRAME_LENGTH + 1);        //Validate received modbus query frame 
        if (modBUSRxFlags_t.SuccessMdBusRTUFrame == true) 
        {
            ReadAnalogHoldingRegisters(); //Read analog holding registers (Modbus Function)
        }
        else 
        {
            CreateModbusErrorResponseFrame();
            EUSART1_SendString((uint8_t*) "\r\nClient ERROR Response Frame: ", (uint8_t*) modbusRTUbuffer,ERROR_MODBUS_FRAME_LENGTH);
            EUSART1_SendString((uint8_t*)"\r\n------------------------------",(uint8_t*)"\r\n",NEW_LINE_LENGTH);  
            RS485TxEnable();              //redundant function to enable the RS485 transmission
            EUSART1PinSwap(RS485);        //Swap EUSART pin to RS485 module pins
            SendModbusRTUFrame(modbusRTUbuffer,ERROR_MODBUS_FRAME_LENGTH); //send response frame to host
        }
        if (modBUSRxFlags_t.ReadHoldingRegisters == true) 
        {
            modBUSRxFlags_t.ReadHoldingRegisters = false;
            CreateModbusResponseFrame();
            EUSART1_SendString((uint8_t*) "\r\nClient Success Response: ", (uint8_t*) modbusRTUbuffer,RESPONSE_MODBUS_FRAME_LENGTH);
            EUSART1_SendString((uint8_t*)"\r\n------------------------------",(uint8_t*)"\r\n",2);  
            RS485TxEnable();                                                 //redundant function to enable the RS485 transmission
            EUSART1PinSwap(RS485);                                            //Swap EUSART pin to RS485 module pins
            SendModbusRTUFrame(modbusRTUbuffer,RESPONSE_MODBUS_FRAME_LENGTH);     //send response frame to host
        }
        EUSART_Flush();
     }
   
}

/*******************************************************************************
 * void SensorDataToClientTerminal(void)
 *
 * Send sensor data to client terminal
 *
 * @param None
 * @return None
 ******************************************************************************/
void SensorDataToClientTerminal(void)
{                                                
    uint8_t sensorDataString[SENSOR_DATA_LENGTH + 1] = {0};
    
    ftoa(sensor_Data_Register[TEMPERATURE_DATA_LOCATION],sensorDataString);                  //Convert float to ascii                                                                //Swap pin to send data to terminal
    EUSART1_SendString((uint8_t*)"\n\rTemperature: ",sensorDataString,SENSOR_DATA_LENGTH);                                     //Send temperature to data visualizer
    memset(sensorDataString, '\0',sizeof(sensorDataString));                                                          //Clear buffer to load pressure sensor data                         
    ftoa(sensor_Data_Register[PRESSURE_DATA_LOCATION],sensorDataString);
    EUSART1_SendString((uint8_t*)"\n\rPressure   : ",sensorDataString,SENSOR_DATA_LENGTH);        
    EUSART1_SendString((uint8_t*)"\n\r------------------------------",(uint8_t*)"\r\n",NEW_LINE_LENGTH);                      
    memset(sensorDataString, '\0',sizeof(sensorDataString));                                                          //Clear buffer to load pressure sensor data
}
/*******************************************************************************
 * static void ReadSensors(void) 
 *
 * API to read temperature and pressure sensor 
 *
 * @param None
 * @return None
 ******************************************************************************/
static void ReadSensors(void) 
{
    memset(sensor_Data_Register, 0, sizeof (sensor_Data_Register));      //Clear the memory for loading new sensor data
    sensordata_t.thermoError = ReadTemperatureFromMAX31855K(&sensordata_t.temperatureInCelcius);   //Read temperature data from thermo click
    sensor_Data_Register[TEMPERATURE_DATA_LOCATION] = sensordata_t.temperatureInCelcius; 

    sensordata_t.pressure = Pressure5GetPressureData();   //Read pressure data from the pressure 5 click sensor
    sensor_Data_Register[PRESSURE_DATA_LOCATION] = sensordata_t.pressure; 
}

/*******************************************************************************
 * float GetTemperatureData(void)
 *
 * Wrapper function to get the temperature data  
 *
 * @param None
 * @return temperature value
 ******************************************************************************/
float GetTemperatureData(void)
{
    return sensor_Data_Register[TEMPERATURE_DATA_LOCATION];     //Get the temperature data from the assigned memory
}

/*******************************************************************************
 * float GetPressureData(void)
 *
 * Wrapper function to get the pressure data  
 *
 * @param None
 * @return pressure value
 ******************************************************************************/
float GetPressureData(void)
{
    return sensor_Data_Register[PRESSURE_DATA_LOCATION];     //Get the pressure data from the assigned memory
}

/*******************************************************************************
 * bool ReceiveModbusRTUFrame(void)
 *
 * API to receive the client RTU frame and store it in buffer for parsing  
 *
 * @param None
 * @return bool
 ******************************************************************************/
bool ReceiveModbusRTUFrame(void)
{
    uint8_t ch,index = 0;
    
    if(EUSART1_IsRxReady())
    {
        for(index = 0;index <= (HOST_MODBUS_FRAME_LENGTH+1);index++)
        {
          if(EUSART1_IsRxReady())
          {  
             ch = EUSART1_Read();
             if((ch != '\r') || (ch != '\n'))
             {
               modbusRTUbuffer[index] = ch;        //Loading the received characters to buffer
             }
          }
        }
        modbusRTUbuffer[index] = '\0';             //Insert null in the end of the buffer 
        return RECEIVED;
    }
    return NOT_RECEIVED;
}

/*******************************************************************************
 * static void CnanoLED_TurnOn(void)
 *
 * Function to Turn on the Led   
 *
 * @param None
 * @return None
 ******************************************************************************/
static void CnanoLED_TurnOn(void)
{
    LED_SetLow();         //Turn on the LED before start reading sensors  //
}
/*******************************************************************************
 * static void CnanoLED_TurnOFF(void)
 *
 * Function to Turn off the Led   
 *
 * @param None
 * @return None
 ******************************************************************************/
static void CnanoLED_TurnOFF(void)
{
    LED_SetHigh();       //Turn off the LED after reading sensors  //
}

/*******************************************************************************
 * void BMP388Init(void)
 *
 * Function to initialize pressure sensor 
 *
 * @param None
 * @return None
 ******************************************************************************/
void BMP388Init(void)
{
    Pressure5Init();
    Pressure5Default_cfg();
}

/*******************************************************************************
 * uint8_t ReadTemperatureFromMAX31855K(float *temperatureInCelcius)
 *
 * Function to read temperature from the Thermo click 
 *
 * @param empty float pointer to get temperature value
 * @return error value
 ******************************************************************************/
uint8_t ReadTemperatureFromMAX31855K(float *temperatureInCelcius) 
{
    uint8_t thermoError = 0;

    SPI1_Open(MSSP1_DEFAULT);
#if(TEMPERATURE_ERROR)
    if (Thermo_check_fault()) 
    {
        thermoError |= (1 << 0);
    }
    if (Thermo_short_circuited_vcc()) 
    {
        thermoError |= (1 << 1);
    }
    if (Thermo_short_circuited_gnd()) 
    {
        thermoError |= (1 << 2);
    }
    if (Thermo_check_connections()) 
    {
        thermoError |= (1 << 3);
    }
#endif
    *temperatureInCelcius = Thermo_get_temperature();

    SPI1_Close();
    return thermoError;
}

/*******************************************************************************
 * void RS485TxEnable(void)
 *
 * Function to enable transmission of RS485 
 *
 * @param None
 * @return None
 ******************************************************************************/
void RS485TxEnable()
{
    DE_SetHigh();      //Enable transmission 
    RE_SetHigh();     //Disable reception 
}

/*******************************************************************************
 * void RS485RxEnable(void)
 *
 * Function to enable reception of RS485 
 *
 * @param None
 * @return None
 ******************************************************************************/
void RS485RxEnable(void)
{
    RE_SetLow();    //Enable reception
    DE_SetLow();    //Disable transmission
}

/*******************************************************************************
 * void SendModbusRTUFrame(uint8_t *dat, uint8_t len)
 *
 * Function to send Modbus frame to Host
 *
 * @param None
 * @return  value
 ******************************************************************************/
void SendModbusRTUFrame(uint8_t *dat, uint8_t len)
{
    uint8_t index = 0;
    
    for (index = 0; index <= (len-1); index++) 
    {
        EUSART1_Write(dat[index]);
    }
    __delay_ms(DELAY_FOR_EUSART_COMM);
}

/*******************************************************************************
 * void EUSART1PinSwap(uint8_t option)
 *
 * Function to swap the tx/rx lines using PPS
 *
 * @param parameter to choose what need to be swapped
 * @return None
 *****************************************************************************/
void EUSART1PinSwap(uint8_t option)
{
    PIE1bits.RC1IE = 0;                           //Disable EUSART1 Rx interrupt before switching the EUSART pin
    switch(option)
    {
        case DATAVISUALIZER:
            TRISBbits.TRISB0 = CLEAR;
            TRISCbits.TRISC0 = SET;
            ANSELCbits.ANSC1 = SET;
            ANSELBbits.ANSB1 = CLEAR;
            
            RX1PPS = RX_RB1;   //RB1:EUSART1:RX
            RB0PPS = TX_RB0;   //RB0:EUSART1:TX
            RC0PPS = 0x00;  //Reset the PPS register in run time to change the transmission functionality to RB0 pin
            break;
        case RS485:
            TRISBbits.TRISB0 = SET;
            TRISCbits.TRISC0 = CLEAR;
            ANSELCbits.ANSC1 = CLEAR;
            ANSELBbits.ANSB1 = SET;
            
            RX1PPS = RX_RC1;                                                                   //RC1:EUSART1:RXs
            RC0PPS = TX_RC0;                                                                   //RC0:EUSART1:TX
            RB0PPS = 0x00;                                                                     //Reset the PPS register in run time to change the transmission functionality to RC0 pin
            break;
        default:
            break;
    }
    PIE1bits.RC1IE = 1;                                                                        //Enable EUSART1 Rx interrupt after switching the EUSART pin
    __delay_ms(DELAY_FOR_PIN_SWAP);
}

/*******************************************************************************
 * void EUSART1_SendString(uint8_t *dat, const uint8_t *str, uint8_t len) 
 *
 * Function to send data to terminal window
 *
 * @param Data, string, length
 * @return None
 ******************************************************************************/
void EUSART1_SendString(uint8_t *dat, uint8_t *data1, uint8_t len) 
{
    uint8_t index = 0;
    
    for (index = 0; index < strlen((char*)dat); index++) 
    {
        EUSART1_Write(dat[index]);
    }
    for (index = 0; index < len; index++) 
    {
        EUSART1_Write(data1[index]);
    }
    __delay_ms(DELAY_FOR_EUSART_COMM);
}

/*******************************************************************************
 * static void ftoa(float value_f, uint8_t *res_uc) 
 *
 * API to convert float to ascii for debug purpose
 *  
 * @param value_f: Float value, *res_us: output ascii result, n: 1 for new line
 * @return void
 ******************************************************************************/
static void ftoa(float value_f, uint8_t *res_uc) 
{
    if (value_f < 0) {
        *res_uc++ = '-';
        value_f = -value_f;
    }
    sprintf((char*) res_uc, "%lu.%02u", (long) value_f, (int) ((value_f - (long) value_f) * 100. + 0.5));
}

/*******************************************************************************
 * void EUSART_Flush(void)
 *
 * Reset the queue 
 *
 * @param None
 * @return None
 ******************************************************************************/
void EUSART_Flush(void)
{
  eusart1TxHead = 0;
  eusart1TxTail = 0;
  eusart1TxBufferRemaining = sizeof(eusart1TxBuffer);

  eusart1RxHead = 0;
  eusart1RxTail = 0;
  eusart1RxCount = 0;
}
/*******************************************************************************
 * void Timer0_UserOverflowCallback(void)
 *
 * Wrapper function to get the temperature data  
 *
 * @param None
 * @return value
 ******************************************************************************/
void Timer0_UserOverflowCallback(void) 
{
    sensorReadFlag = true;

}
