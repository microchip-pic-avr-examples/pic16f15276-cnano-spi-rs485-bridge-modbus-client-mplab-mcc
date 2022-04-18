#include "mcc_generated_files/spi/mssp1.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/system/clock.h"
#include "bmp388.h"


#define PRESSURE5_DUMMY 0


static uint8_t coefficients[22] = {0};
static pressure5_nvm_par_t   nvm_par;
static pressure5_par_coeff_t par;


// ---------------------------------------------- PRIVATE FUNCTION DECLARATIONS 

static void Pressure5_spi_write (  uint8_t reg, uint8_t *data_buf, uint8_t len );

static void Pressure5_spi_read ( uint8_t reg, uint8_t *data_buf, uint8_t len );

static void Storage_coefficient ( void );

static uint8_t WaitForData ( uint8_t tp );


/*******************************************************************************
 * void Pressure5Init(void)
 *
 * API to open the spi for read pressure sensor from pressure 5 click
 *
 * @param None
 * @return Pressure init success
 ******************************************************************************/
void Pressure5Init (void)
{
    SPI1_Open(MSSP1_DEFAULT);    
    
    SelectPressureSensor();                      //SPI Client select (Pressure sensor select)
    __delay_ms(DELAY_SENSOR_SELECT);
    DeselectPressureSensor();                    //De-select pressure sensor (as per sensor data sheet)
}


/*******************************************************************************
 * void Pressure5Default_cfg(void)
 *
 * API to configure the pressure sensor communication, resolution and method of integration
 * using pressure sensor register
 *
 * @param None
 * @return None
 ******************************************************************************/
void Pressure5Default_cfg ( void )
{
    SelectPressureSensor();
    
    Pressure5GenericWrite( PRESSURE5_REG_OSR, PRESSURE5_OSR_PRESSURE_x1 | PRESSURE5_OSR_TEMP_x1 );
    Pressure5GenericWrite( PRESSURE5_REG_ODR, PRESSURE5_ODR_200 );
    
    Pressure5GenericWrite( PRESSURE5_REG_INT_CTRL, PRESSURE5_INTCFG_INT_OUTPUT_PUSH_PULL |
                                                 PRESSURE5_INTCFG_INT_LEVEL_ACTIVE_HIGH |
                                                 PRESSURE5_INTCFG_INT_LATCH_DISABLE |
                                                 PRESSURE5_INTCFG_FIFO_WTM_DISABLE |
                                                 PRESSURE5_INTCFG_FIFO_FULL_DISABLE |
                                                 PRESSURE5_INTCFG_DATA_READY_ENABLE );

    Pressure5GenericWrite( PRESSURE5_REG_IF_CONFIG, PRESSURE5_IFCFG_SPI_4_WIRE |
                                                  PRESSURE5_IFCFG_I2C_WATCHDOG_ENABLE |
                                                  PRESSURE5_IFCFG_I2C_WATCHDOG_LONG_40ms );
                                                  
    Pressure5GenericWrite( PRESSURE5_REG_POWER_CTRL,PRESSURE5_PCTRL_PRESSURE_SENSOR_ENABLE |
                                                  PRESSURE5_PCTRL_TEMP_SENSOR_ENABLE |
                                                  PRESSURE5_PCTRL_MODE_NORMAL );
        
    Pressure5UpdateCoefficient();
    
    DeselectPressureSensor();
}

/*******************************************************************************
 * void Pressure5GenericWrite (uint8_t reg, uint8_t reg_data)
 *
 * API to write pressure sensor register using spi as per the pressure sensor module
 *
 * @param register name and data (uint8_t reg, uint8_t reg_data)
 * @return None
 ******************************************************************************/
void Pressure5GenericWrite (uint8_t reg, uint8_t reg_data)
{
    uint8_t write_data = reg_data;
    
    Pressure5_spi_write(reg,&write_data,1);
}

/*******************************************************************************
 * void Pressure5GenericRead (uint8_t reg)
 *
 * API to read pressure sensor register using spi as per the pressure sensor module
 *
 * @param uint8_t reg - read register
 * @return uint8_t return register value
 ******************************************************************************/
uint8_t Pressure5GenericRead (uint8_t reg )
{
    uint8_t read_data = 0;
    uint8_t tx_buf[2] = {0};

    tx_buf[0] = reg | MASK_MSB_BIT;
    tx_buf[1] = 1;                      // dummy data
    SelectPressureSensor();    
    SPI1_BufferWrite(tx_buf,2);
    read_data = SPI1_ByteExchange(0);   //len +1;
    DeselectPressureSensor();  
    
    return read_data;
}

/*******************************************************************************
 * void Pressure5Read (uint8_t reg, uint8_t *data_buf, uint8_t len )
 *
 * API to read pressure sensor register by writing it 
 *
 * @param uint8_t reg, uint8_t *data_buf, uint8_t len
 * @return None
 ******************************************************************************/
void Pressure5Read (uint8_t reg, uint8_t *data_buf, uint8_t len )
{
    Pressure5_spi_read(reg,data_buf,len);
}

/*******************************************************************************
 * uint8_t Pressure5GetInterruptState(void)
 *
 * API to get the interrupt pin value
 *
 * @param None
 * @return uint8_t return interrupt pin value 
 ******************************************************************************/
uint8_t Pressure5GetInterruptState ( void)
{
    return INT_Pressure_GetValue();
            
}

/*******************************************************************************
 * void Pressure5SoftwareReset(void)
 *
 * API to performs software reset of pressure click
 *
 * @param None
 * @return None
 ******************************************************************************/
void Pressure5SoftwareReset ()
{
    Pressure5GenericWrite(PRESSURE5_REG_COMMAND, PRESSURE5_CMD_SOFTWARE_RESET);
}

/*******************************************************************************
 * uint32_t Pressure5GetRawData ( uint8_t data_addr )
 *
 * API to reads raw data from pressure click. 
 *
 * @param uint8_t data_addr
 * @return uint32_t raw data 
 ******************************************************************************/
uint32_t Pressure5GetRawData ( uint8_t data_addr )
{
    uint32_t data_raw;
    
    if(data_addr == PRESSURE5_REG_TEMPERATURE_DATA_0)
    {
        WaitForData(NUMBER_OF_RETRY_1);
    }
    else
    {
        WaitForData(NUMBER_OF_RETRY_2);
    }
    
    data_raw = 0;
    data_raw = Pressure5GenericRead(data_addr + 2 );
    data_raw <<= 8;
    data_raw |=  Pressure5GenericRead(data_addr + 1 );
    data_raw <<= 8;
    data_raw |=  Pressure5GenericRead(data_addr);
    
    return data_raw;
}

/*******************************************************************************
 * void Pressure5UpdateCoefficient (void)
 *
 * API to allows you to update the calibration coefficient.
 * Coefficients must be updated on the start program.
 *
 * @param None
 * @return None
 ******************************************************************************/
void Pressure5UpdateCoefficient(void)
{ 
    Pressure5Read (SENSOR_COEFFICIENT_REG_ADDRESS,coefficients,SENSOR_COEFFICIENT_LENGTH);        //read manufacture coefficients for temperature and pressure
    Storage_coefficient( );                         //store coefficients in data memory
}

/*******************************************************************************
 * float Pressure5GetTemperatureData ( void)
 *
 * API to gets temperature in Celsius (compensation temperature value)
 *
 * @param None
 * @return float temperature value
 ******************************************************************************/
float Pressure5GetTemperatureData ( void)
{
    uint32_t temp_raw;
    float partial_data1;
    float partial_data2;
    float calib_temp;
    
    temp_raw = Pressure5GetRawData( PRESSURE5_REG_TEMPERATURE_DATA_0);
    
    partial_data1 = ( float )( temp_raw - par.t1 );
    partial_data2 = ( float )( partial_data1 * par.t2 );
    
    calib_temp = partial_data2 + ( partial_data1 * partial_data1 ) * par.t3;
    
    return calib_temp;
}

/*******************************************************************************
 * float Pressure5GetPressureData(void)
 *
 * API to gets pressure in mBar
 *
 * @param None
 * @return float - pressure sensor value in mBar
 ******************************************************************************/
float Pressure5GetPressureData(void)
{
    uint32_t pressure_raw;
    float temp_data;
    float comp_press;
    float data1;
    float data2;
    float data3;
    float data4;
    float out1;
    float out2;
    
    Pressure5Init();
    temp_data = Pressure5GetTemperatureData();
    pressure_raw = Pressure5GetRawData(PRESSURE5_REG_PRESSURE_DATA_0);
    /*Calculations as per the BMP388 datasheet for measuring pressure in mBar*/
    data1 = par.p6 * temp_data;
    data2 = par.p7 * ( temp_data * temp_data );
    data3 = par.p8 * ( temp_data * temp_data * temp_data );
    out1 = par.p5 + data1 + data2 + data3;
    
    data1 = par.p2 * temp_data;
    data2 = par.p3 * ( temp_data * temp_data );
    data3 = par.p4 * ( temp_data * temp_data * temp_data );
    out2 = ( float )pressure_raw;
    out2 *= par.p1 + data1 + data2 + data3;
    
    data1 = ( float )pressure_raw * ( float )pressure_raw;
    data2 = par.p9 + par.p10 * temp_data;
    data3 = data1 * data2;
    data4 = data3 + ( ( float )pressure_raw * ( float )pressure_raw * ( float )pressure_raw ) * par.p11;
    
    comp_press = out1 + out2 + data4;
    
    return (float)(comp_press / CONVERSION_CONST);
}


/*******************************************************************************
 * static void pressure5_spi_write (uint8_t reg, uint8_t *data_buf, uint8_t len )
 *
 * API to write pressure sensor register 
 *
 * @param uint8_t reg, uint8_t *data_buf, uint8_t len 
 * @return None
 ******************************************************************************/
static void Pressure5_spi_write (uint8_t reg, uint8_t *data_buf, uint8_t len )
{
    uint8_t tx_buf[12];
    uint8_t cnt;

    tx_buf[0] = reg;
    for ( cnt = 1; cnt <= len; cnt++ )
    {
        tx_buf[ cnt ] = data_buf[ cnt - 1 ]; 
    }
    
    SelectPressureSensor();
    SPI1_BufferWrite(tx_buf, len + 1 );
    DeselectPressureSensor(); 
}

/*******************************************************************************
 * static void pressure5_spi_read ( uint8_t reg, uint8_t *data_buf, uint8_t len )
 *
 * API to read pressure sensor register
 *
 * @param uint8_t reg, uint8_t *data_buf, uint8_t len 
 * @return None
 ******************************************************************************/
static void Pressure5_spi_read ( uint8_t reg, uint8_t *data_buf, uint8_t len )
{
    uint8_t tx_buf[2] = {0};
    uint8_t read_buf[32] = { 0 };
    uint16_t cnt;

    tx_buf[0] = reg | MASK_MSB_BIT;

    SelectPressureSensor();    
    SPI1_BufferWrite(tx_buf,2);
    SPI1_BufferRead(read_buf, len + 1);
    DeselectPressureSensor();  

    for (cnt = 0;cnt <= len;cnt++)
    {
        *data_buf = read_buf[cnt];
        data_buf++;
    }
}

/*******************************************************************************
 * static void storage_coefficient(void)
 *
 * API to store the pressure sensor coefficients
 *
 * @param None
 * @return None
 ******************************************************************************/
static void Storage_coefficient( void )
{
    //Store the coefficients to individual coefficient variables 
    // Temperature 
    /*Calculations as per the datasheet to calculate the coefficients for temperature and pressure calculations*/
    nvm_par.t1 = coefficients[1];
    nvm_par.t1 <<= 8;
    nvm_par.t1 |= coefficients[0];
    par.t1 = (float)(nvm_par.t1 / COEFFICIENT_CONSTANT_T1);
    
    nvm_par.t2 = coefficients[3];
    nvm_par.t2 <<= 8;
    nvm_par.t2 |= coefficients[2];
    par.t2 = (float)(nvm_par.t2 / COEFFICIENT_CONSTANT_T2);
    
    nvm_par.t3 = (int8_t)coefficients[4];
    par.t3 = (float)(nvm_par.t3 / COEFFICIENT_CONSTANT_T3);
    
    // Pressure
    nvm_par.p1 = coefficients[6];
    nvm_par.p1 <<= 8;
    nvm_par.p1 |= coefficients[5];
    par.p1 = (float)((nvm_par.p1 - P1_P2_CONST) / COEFFICIENT_CONSTANT_P1);
    
    nvm_par.p2 = coefficients[8];
    nvm_par.p2 <<= 8;
    nvm_par.p2 |= coefficients[7];
    par.p2 = (float)((nvm_par.p2 - P1_P2_CONST) / COEFFICIENT_CONSTANT_P2);
    
    nvm_par.p3 = (int8_t)coefficients[9];
    par.p3 = (float)(nvm_par.p3 / COEFFICIENT_CONSTANT_P3);
    
    nvm_par.p4 = (int8_t)coefficients[10];
    par.p4 = (float)(nvm_par.p4 / COEFFICIENT_CONSTANT_P4);
    
    nvm_par.p5 = (uint16_t)coefficients[12];
    nvm_par.p5 <<= 8;
    nvm_par.p5 |= (uint16_t)coefficients[11];
    par.p5 = (float)(nvm_par.p5 / COEFFICIENT_CONSTANT_P5);
    
    nvm_par.p6 = (uint16_t)coefficients[14];
    nvm_par.p6 <<= 8;
    nvm_par.p6 |= (uint16_t)coefficients[13];
    par.p6 = (float)(nvm_par.p6 / COEFFICIENT_CONSTANT_P6);
     
    nvm_par.p7 = (int8_t)coefficients[15];
    par.p7 = (float)(nvm_par.p7 / COEFFICIENT_CONSTANT_P7);
    nvm_par.p8 = (int8_t)coefficients[16];
    par.p8 = (float)(nvm_par.p8 / COEFFICIENT_CONSTANT_P8);
    
    nvm_par.p9 = (int16_t)coefficients[18];
    nvm_par.p9 <<= 8;
    nvm_par.p9 |= (int16_t)coefficients[17];
    par.p9 = (float)(nvm_par.p9 / COEFFICIENT_CONSTANT_P9);
    
    nvm_par.p10 = (int8_t)coefficients[19];
    par.p10 = (float)(nvm_par.p10 / COEFFICIENT_CONSTANT_P10);
    nvm_par.p11 = (int8_t)coefficients[20];
    par.p11 = (float)(nvm_par.p11 / COEFFICIENT_CONSTANT_P11);
}

/*******************************************************************************
 * static void WaitDataReady ( uint8_t tp )
 *
 * API to handle wait and retry of reading temperature or pressure sensor data
 *
 * @param tp
 * @return uint8_t
 ******************************************************************************/
static uint8_t WaitForData ( uint8_t tp )
{
    uint8_t status;
    uint8_t cnt = 0;
    
    do
    {
        status = Pressure5GenericRead(PRESSURE5_OSR_PRESSURE_x8); //PRESSURE5_REG_INT_STATUS
        cnt++;
        __delay_ms(RETRY_DELAY);
        if (cnt > RETRY_COUNT)
        {
            return 1;
        }
        
    }
    while ( (status & tp) == 0 );
    
    return 0;
}

/*******************************************************************************
 * void SelectPressureSensor(void)
 *
 * API to select the pressure sensor as client 
 *
 * @param None
 * @return None
 ******************************************************************************/
void SelectPressureSensor(void) 
{
    SS_Thermo_SetHigh();
    SS_Pressure_SetLow();
}

/*******************************************************************************
 * void DeselectPressureSensor(void) 
 *
 * API to de-select the pressure sensor as client 
 *
 * @param None
 * @return None
 ******************************************************************************/
void DeselectPressureSensor(void) 
{
    SS_Pressure_SetHigh();
    SS_Thermo_SetHigh();
}

