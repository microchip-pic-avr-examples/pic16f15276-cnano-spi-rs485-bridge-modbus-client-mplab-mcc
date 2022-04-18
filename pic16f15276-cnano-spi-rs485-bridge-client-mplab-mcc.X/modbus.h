/* 
 * File:   modbus.h
 * Author: I20946
 *
 * Created on November 9, 2021, 10:50 AM
 */
#include <stdint.h>

#ifndef MODBUS_H
#define	MODBUS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define LEN                              (4)                                                                                                                                                                                        
//Slave address
#define CLIENT_1                         (0x15)
#define CLIENT_2                         (0x16)

#define ERROR_FUNCTION                   (0x81)
#define MASK_LOWER_BYTE                  (0x00ff)
#define MASK_HIGHER_BYTE                 (0xff00)

//Function codes   
#define READ_HOLDING_REGISTERS           (0x03)

#define AO_TYPE_REGISTER_NUMBER          (0x9C41)
#define TEMP_SENSOR_ADDRESS              (0x9CAC)
#define PRESS_SENSOR_ADDRESS             (0x9D10)
#define ERROR_MSG_SLAVE_LENGTH           (26)
#define ERROR_MSG_CRC_LENGTH             (16)
#define ERROR_MSG_FUNCTION_LENGTH        (41)
#define ADDRESS_OF_TEMP_DATA             (TEMP_SENSOR_ADDRESS - AO_TYPE_REGISTER_NUMBER) 
#define ADDRESS_OF_PRESS_DATA            (PRESS_SENSOR_ADDRESS - AO_TYPE_REGISTER_NUMBER)
#define ERROR_MSG_SLAVE                  ("Slave address not matched.")
#define ERROR_MSG_CRC                    ("CRC not matched.")
#define ERROR_MSG_FUNCTION               ("Invalid function detected for this slave.") 

uint8_t  modbusRTUbuffer[6+LEN];

typedef enum 
{
    CLIENT_ERROR = 1,
    CRC_ERROR,
    FUNCTION_CODE_ERROR
}modbus_error_e;

typedef struct
{                                   //data streamer frame creation 
      uint8_t CRC[2];
      uint8_t data[LEN];
      uint8_t function;
	  uint16_t address;
      uint16_t length;
      uint8_t clientID;
}ModbusRTUFrame_t; 

typedef struct
{
    bool CRCMatchFlag;
    bool clientDetected;
    bool FunctionCodeDetected;
    bool SuccessMdBusRTUFrame;
    bool ReadHoldingRegisters;
}ModbusRxFlags_t; 

typedef struct
{
    uint8_t HoldRegister[LEN]; 
}RegisterData_st;

typedef union Data
{   
    uint16_t twoBytes;
    struct
    {
        uint8_t firstByte;
        uint8_t secondByte;
    }data_st;
}SplitData_u;

typedef union
{
  float sensorData;
  struct 
  {
    uint8_t data[4];
  }SensorData_st;
}SensorDataSplit_t;

void ValidateModbusQueryFrame(uint8_t dataLen);
void ReadAnalogHoldingRegisters(void);
void CreateModbusResponseFrame(void);
void CreateModbusErrorResponseFrame(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MODBUS_H */

