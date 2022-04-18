#include "max31855k.h"
#include "mcc_generated_files/system/clock.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/spi/mssp1.h"

static void SelectTemperatureSensor(void); 
static void DeselectTemperatureSensor(void); 

/*******************************************************************************
 * uint32_t Thermo_read_data(void) 
 *
 * API to read thermo sensor register
 *
 * @param None
 * @return uint32_t - thermo sensor register data
 ******************************************************************************/
uint32_t Thermo_read_data(void) 
{
    uint8_t buffer[4] = {0};
    uint32_t result = 0;

    SelectTemperatureSensor();
    __delay_ms(TEMP_CONV_DELAY);                      //350ms delay as per the sensor data sheet
    SPI1_BufferRead((uint8_t*)buffer,TEMPERATURE_BUFFER_LENGTH);
    DeselectTemperatureSensor();
    __delay_ms(TEMP_CONV_DELAY);
    result = buffer[0];
    result <<= 8;
    result |= buffer[1];
    result <<= 8;
    result |= buffer[2];
    result <<= 8;
    result |= buffer[3];

    return result;
}

/*******************************************************************************
 * float Thermo_get_temperature(void) 
 *
 * API to read temperature from the thermo sensor
 *
 * @param None
 * @return float - temperature sensor data 
 ******************************************************************************/
float Thermo_get_temperature(void) 
{
    uint8_t buffer[4] = {0};
    uint16_t temp_data = 0;
    float temperature = 0;

    SelectTemperatureSensor();
    __delay_ms(TEMP_DELAY);               //100ms delay as settling time after selecting the client   
    SPI1_BufferRead(buffer,TEMPERATURE_BUFFER_LENGTH);
    DeselectTemperatureSensor();
    __delay_ms(TEMP_DELAY);               //100ms delay as settling time after de-selecting the client
    temp_data = buffer[0];
    temp_data <<= 8;
    temp_data |= buffer[1];

    if (buffer[0] > SIGN_CHAR) 
    {             //Convert raw temperature data in to celsius format
        temp_data = ~temp_data;
        temperature = ( float )( ( temp_data >> 2 ) & MASK_LOWER_TWO_BITS);
        temperature *= TEMP_CAL_CONST1;             
        temperature -= ( float )( temp_data >> 4 );
        temperature += TEMP_CAL_CONST2;
    }
    else 
    {
        temperature = ( float )( ( temp_data >> 2 ) & MASK_LOWER_TWO_BITS);
        temperature *= TEMP_CAL_CONST3;
        temperature += ( float )( temp_data >> 4 );
    }

    return temperature;
}

/*******************************************************************************
 * uint8_t Thermo_check_fault(void)
 *
 * API to check the thermo click faults 
 *
 * @param None
 * @return uint8_t - error value
 ******************************************************************************/
uint8_t Thermo_check_fault(void) 
{
    uint32_t tmp;

    tmp = Thermo_read_data();
    tmp >>= FAULT_BIT;              //Read 16th bit of thermo sensor register to check the fault
    tmp &= MASK_LSB_BIT;             //Check the fault bit is one or zero

    return (uint8_t) tmp;
}

/*******************************************************************************
 * uint8_t Thermo_short_circuited_vcc(void)
 *
 * API to check the short circuit with Vcc if any
 *
 * @param None
 * @return uint8_t 
 ******************************************************************************/
uint8_t Thermo_short_circuited_vcc(void)
{
    uint32_t tmp;

    tmp = Thermo_read_data();
    tmp >>= VCC_SHORT_ERROR_BIT;               //Read 2nd bit of thermo sensor register to check the vcc short circuit status bit
    tmp &= MASK_LSB_BIT;             //Check the vcc short circuit status bit is one or zero

    return (uint8_t) tmp;
}

/*******************************************************************************
 * uint8_t Thermo_short_circuited_gnd(void)
 *
 * API to check the short circuit with gnd if any
 *
 * @param None
 * @return uint8_t 
 ******************************************************************************/
uint8_t Thermo_short_circuited_gnd(void)
{
    uint32_t tmp;

    tmp = Thermo_read_data();
    tmp >>= SHORT_CIRCUIT_GND_ERROR_BIT;                //Read 1st bit of thermo sensor register to check the gnd short circuit status bit.
    tmp &= MASK_LSB_BIT;              //Check the gnd short circuit status bit is one or zero

    return (uint8_t) tmp;
}

/*******************************************************************************
 * uint8_t Thermo_check_connections(void)
 *
 * API to check the connection errors of thermo sensor
 *
 * @param None
 * @return uint8_t 
 ******************************************************************************/
uint8_t Thermo_check_connections(void) 
{
    uint32_t tmp;

    tmp = Thermo_read_data();
    tmp &= MASK_LSB_BIT;             //Check the connection error status bit is one or zero

    return (uint8_t) tmp;
}

/*******************************************************************************
 * static void SelectTemperatureSensor(void)
 *
 * API to select thermo click as client 
 *
 * @param None
 * @return None
 ******************************************************************************/
static void SelectTemperatureSensor(void) 
{
    SS_Thermo_SetLow();
}

/*******************************************************************************
 * static void DeselectTemperatureSensor(void)
 *
 * API to de-select thermo click as client 
 *
 * @param None
 * @return None
 ******************************************************************************/
static void DeselectTemperatureSensor(void) 
{
    SS_Thermo_SetHigh();
}

