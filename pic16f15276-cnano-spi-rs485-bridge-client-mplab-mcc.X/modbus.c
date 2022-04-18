#include "mcc_generated_files/uart/eusart1.h"
#include "mcc_generated_files/system/clock.h"
#include "application.h"
#include "modbus.h"
#include "modbusCRC.h"
#include <string.h>

ModbusRTUFrame_t modBUSRTUFrame_t;
ModbusRxFlags_t modBUSRxFlags_t;
RegisterData_st registerData_t;

/*******************************************************************************
 * void ValidateModbusQueryFrame(uint8_t dataLen)
 *
 * API to validate and parse the host frame  
 * 
 * @param uint8_t dataLen
 * @return None
 ******************************************************************************/
void ValidateModbusQueryFrame(uint8_t dataLen) 
{
    SplitData_u splitData_u;
    uint16_t calCRC = 0;

    calCRC = ModRTU_CRC(modbusRTUbuffer, (dataLen - 1));
    modBUSRTUFrame_t.CRC[0] = (calCRC & MASK_LOWER_BYTE);
    modBUSRTUFrame_t.CRC[1] = ((calCRC & MASK_HIGHER_BYTE) >> 8);
    if (modBUSRTUFrame_t.CRC[0] == modbusRTUbuffer[dataLen - 1]) 
    {
        if (modBUSRTUFrame_t.CRC[1] == modbusRTUbuffer[dataLen]) 
        {
            modBUSRxFlags_t.CRCMatchFlag = true;
        } 
        else 
        {
            modBUSRxFlags_t.CRCMatchFlag = false;
        }
    } 
    else 
    {
        modBUSRxFlags_t.CRCMatchFlag = false;
    }
    modBUSRTUFrame_t.clientID = modbusRTUbuffer[0];
    if (modbusRTUbuffer[0] == CLIENT_1) 
    {
        modBUSRxFlags_t.clientDetected = true;
    } 
    else if (modbusRTUbuffer[0] == CLIENT_2) 
    {
        modBUSRxFlags_t.clientDetected = true;
    } 
    else 
    {
        modBUSRxFlags_t.clientDetected = false;
    }
    modBUSRTUFrame_t.function = modbusRTUbuffer[1];
    if (modBUSRTUFrame_t.function == READ_HOLDING_REGISTERS)
    {
        modBUSRxFlags_t.FunctionCodeDetected = true;
    } 
    else 
    {
        modBUSRxFlags_t.FunctionCodeDetected = false;
    }
    splitData_u.data_st.secondByte = modbusRTUbuffer[2];
    splitData_u.data_st.firstByte = modbusRTUbuffer[3];
    modBUSRTUFrame_t.address = splitData_u.twoBytes;

    splitData_u.data_st.secondByte = modbusRTUbuffer[4];
    splitData_u.data_st.firstByte = modbusRTUbuffer[5];
    modBUSRTUFrame_t.length = splitData_u.twoBytes;
    modBUSRxFlags_t.SuccessMdBusRTUFrame = true;
    if (modBUSRxFlags_t.clientDetected == false) 
    {
        modBUSRxFlags_t.SuccessMdBusRTUFrame = false;
        EUSART1_SendString((uint8_t*) "\r\nError: ", (uint8_t*) ERROR_MSG_SLAVE,ERROR_MSG_SLAVE_LENGTH);
    }
    if (modBUSRxFlags_t.CRCMatchFlag == false) 
    {
        modBUSRxFlags_t.SuccessMdBusRTUFrame = false;
        EUSART1_SendString((uint8_t*) "\r\nError: ", (uint8_t*) ERROR_MSG_CRC,ERROR_MSG_CRC_LENGTH);
    }
    if (modBUSRxFlags_t.FunctionCodeDetected == false) 
    {
        modBUSRxFlags_t.SuccessMdBusRTUFrame = false;
        EUSART1_SendString((uint8_t*) "\r\nError: ", (uint8_t*) ERROR_MSG_FUNCTION, ERROR_MSG_FUNCTION_LENGTH);
    }
    EUSART1_SendString((uint8_t*)"\r\n------------------------------",(uint8_t*)"\r\n",NEW_LINE_LENGTH); 
}
    

/*******************************************************************************
 * void ReadAnalogHoldingRegisters(void)
 *
 * API to read the holding register 
 * 
 * @param None
 * @return None
 ******************************************************************************/
void ReadAnalogHoldingRegisters(void)
{
    uint16_t dataAddress = 0;
    SensorDataSplit_t sensorDataSplit_t;

    dataAddress = (AO_TYPE_REGISTER_NUMBER + modBUSRTUFrame_t.address);
    if (dataAddress ==  TEMP_SENSOR_ADDRESS) 
    {
        modBUSRxFlags_t.ReadHoldingRegisters = true;
        sensorDataSplit_t.sensorData = GetTemperatureData();
    }
    else if (dataAddress == PRESS_SENSOR_ADDRESS) 
    {
        modBUSRxFlags_t.ReadHoldingRegisters = true;
        sensorDataSplit_t.sensorData = GetPressureData();
    }
    else 
    {
        modBUSRxFlags_t.ReadHoldingRegisters = false;
    }
    registerData_t.HoldRegister[0] = (uint8_t) sensorDataSplit_t.SensorData_st.data[0];
    registerData_t.HoldRegister[1] = (uint8_t) sensorDataSplit_t.SensorData_st.data[1]; 
    registerData_t.HoldRegister[2] = (uint8_t) sensorDataSplit_t.SensorData_st.data[2]; 
    registerData_t.HoldRegister[3] = (uint8_t) sensorDataSplit_t.SensorData_st.data[3]; 
    sensorDataSplit_t.sensorData = 0;
}

/*******************************************************************************
 * void CreateModbusResponseFrame(void)
 *
 * Function to create modbus response frame
 * 
 * @param None
 * @return None
 ******************************************************************************/
void CreateModbusResponseFrame(void)
{
    uint8_t index = 0; 
    uint16_t calCRC = 0;
    
    memset((uint8_t*)modbusRTUbuffer,(int16_t)'\0',sizeof(modbusRTUbuffer));
    modbusRTUbuffer[index++] = modBUSRTUFrame_t.clientID;
    modbusRTUbuffer[index++] = modBUSRTUFrame_t.function;
    modbusRTUbuffer[index++] = (uint8_t)(modBUSRTUFrame_t.length * 2);
    modbusRTUbuffer[index++] = registerData_t.HoldRegister[0];
    modbusRTUbuffer[index++] = registerData_t.HoldRegister[1];
    modbusRTUbuffer[index++] = registerData_t.HoldRegister[2];
    modbusRTUbuffer[index++] = registerData_t.HoldRegister[3];
    calCRC = ModRTU_CRC(modbusRTUbuffer,index);
    modbusRTUbuffer[index++] = (calCRC & MASK_LOWER_BYTE);
    modbusRTUbuffer[index++] = ((calCRC & MASK_HIGHER_BYTE) >> 8);
    modbusRTUbuffer[index] = CLEAR;
}

/*******************************************************************************
 * void CreateModbusErrorResponseFrame(void)
 *
 * Function to create error modbus frame
 * 
 * @param None
 * @return None
 ******************************************************************************/
void CreateModbusErrorResponseFrame(void) 
{
    uint8_t index = 0;
    uint16_t calCRC = 0;

    memset((uint8_t*) modbusRTUbuffer,0,sizeof (modbusRTUbuffer));
    modbusRTUbuffer[index++] = modBUSRTUFrame_t.clientID;
    modbusRTUbuffer[index++] = ERROR_FUNCTION;
    if(modBUSRxFlags_t.clientDetected == false) 
    {
       modbusRTUbuffer[index++] = CLIENT_ERROR;   
    }
    if(modBUSRxFlags_t.CRCMatchFlag == false) 
    {
       modbusRTUbuffer[index++] = CRC_ERROR;  
    }
    if(modBUSRxFlags_t.FunctionCodeDetected == false) 
    {
       modbusRTUbuffer[index++] = FUNCTION_CODE_ERROR;    
    }
    calCRC = ModRTU_CRC(modbusRTUbuffer,index);
    modbusRTUbuffer[index++] = (calCRC & MASK_LOWER_BYTE);
    modbusRTUbuffer[index++] = ((calCRC & MASK_HIGHER_BYTE) >> 8);
    modbusRTUbuffer[index] = CLEAR;
}