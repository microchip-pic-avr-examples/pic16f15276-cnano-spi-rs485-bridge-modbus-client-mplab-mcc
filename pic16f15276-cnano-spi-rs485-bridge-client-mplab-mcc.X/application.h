#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#ifndef APPLICATION_H
#define	APPLICATION_H

#define SLEEP_ON                            (0)
#define TIME_OUT                            (10)          //in minutes
#define SET                                 (true)
#define CLEAR                               (false)
#define RECEIVED                            (true)
#define NOT_RECEIVED                        (false)
#define DELAY_FOR_PIN_SWAP                  (50)
#define DELAY_FOR_EUSART_COMM               (50)
#define BUFFER_SIZE                         (32)
#define DELAY_COUNT                         (100)
#define DELAY_FOR_RESPONSE                  (1000)
#define DELAY_FOR_CLIENT_RESPONSE           (2000)

#define TEMPERATURE_ERROR                   (0)         //Temperature read ERROR handler macro
#define TEMPERATURE_DATA_LOCATION           (0)
#define PRESSURE_DATA_LOCATION              (1)

#define BASE_10                             (10)
#define NEW_LINE_LENGTH                     (2)

#define DELAY_COUNT                         (100)
#define HOST_MODBUS_FRAME_LENGTH            (6)        //length of the user information
#define RESPONSE_MODBUS_FRAME_LENGTH        (9)
#define ERROR_MODBUS_FRAME_LENGTH           (5)
#define SENSOR_DATA_LENGTH                  (7)

#define TX_RB0                              (0x05)     //CDC Tx line  
#define RX_RB1                              (0x09)     //CDC Rx line
#define TX_RC0                              (0x05)     //RS485 Tx line
#define RX_RC1                              (0x11)     //RS485 Rx line

#define DECIMAL_CONV_CONST                  (100) 

typedef enum 
{
  DATAVISUALIZER = 1, 
  RS485 =2
}EusartType_e;

typedef struct sensorDataParameters
{
  uint8_t thermoError;
  float temperatureInCelcius;
  float pressure;
}sensorData_t;

#ifdef	__cplusplus
extern "C" {
#endif
    
void Application(void);
void BMP388Init(void);
void EUSART1_SendString(uint8_t *dat, uint8_t *data1, uint8_t len);
bool ReceiveModbusRTUFrame(void);
float GetPressureData(void);
float GetTemperatureData(void);
uint8_t ReadTemperatureFromMAX31855K(float *temperatureInCelcius);
void SendModbusRTUFrame(uint8_t *dat, uint8_t len);
void Timer0_UserOverflowCallback(void);
void EUSART1PinSwap(uint8_t option);
void RS485TxEnable(void);
void RS485RxEnable(void);
void SensorDataToClientTerminal(void);

#ifdef	__cplusplus
}
#endif

#endif	/* APPLICATION_H */

