#ifndef PRESSURE5_H
#define PRESSURE5_H

#define MASK_MSB_BIT                                  (0x80)
#define SPI_MASTER_MODE_0                             (1)
#define DELAY_SENSOR_SELECT                           (500)
#define CONVERSION_CONST                              (100.0)
#define RETRY_DELAY                                   (10)
#define RETRY_COUNT                                   (20)
#define COEFFICIENT_CONSTANT_T1                       (0.00390625)
#define COEFFICIENT_CONSTANT_T2                       (1073741824.0)
#define COEFFICIENT_CONSTANT_T3                       (281474976710656.0)
#define COEFFICIENT_CONSTANT_P1                       (1048576.0)
#define COEFFICIENT_CONSTANT_P2                       (536870912.0)
#define COEFFICIENT_CONSTANT_P3                       (4294967296.0)
#define COEFFICIENT_CONSTANT_P4                       (137438953472.0)
#define COEFFICIENT_CONSTANT_P5                       (0.125)
#define COEFFICIENT_CONSTANT_P6                       (64.0)
#define COEFFICIENT_CONSTANT_P7                       (256.0)
#define COEFFICIENT_CONSTANT_P8                       (32768.0)
#define COEFFICIENT_CONSTANT_P9                       (281474976710656.0)
#define COEFFICIENT_CONSTANT_P10                      (281474976710656.0)
#define COEFFICIENT_CONSTANT_P11                      (36893488147419103232.0)
#define P1_P2_CONST                                   (16384)
#define SENSOR_COEFFICIENT_REG_ADDRESS                (0x31)
#define SENSOR_COEFFICIENT_LENGTH                     (21)
 //defgroup communication Select communication

#define PRESSURE5_MASTER_SPI                          (1)

//defgroup error_code Error Code

#define PRESSURE5_RETVAL  uint8_t

#define PRESSURE5_OK                                  (0x00)
#define PRESSURE5_INIT_ERROR                          (0xFF)
#define NUMBER_OF_RETRY_1                             (0x40)
#define NUMBER_OF_RETRY_2                             (0x20)
//defgroup general_registers General Registers

#define PRESSURE5_REG_DEVICE_ID                       (0x00)
#define PRESSURE5_REG_ERROR                           (0x02)
#define PRESSURE5_REG_PRESSURE_DATA_0                 (0x04)
#define PRESSURE5_REG_PRESSURE_DATA_1                 (0x05)
#define PRESSURE5_REG_PRESSURE_DATA_2                 (0x06)
#define PRESSURE5_REG_TEMPERATURE_DATA_0              (0x07)
#define PRESSURE5_REG_TEMPERATURE_DATA_1              (0x08)
#define PRESSURE5_REG_TEMPERATURE_DATA_2              (0x09)
#define PRESSURE5_REG_SENSOR_TIME_0                   (0x0C)
#define PRESSURE5_REG_SENSOR_TIME_1                   (0x0D)
#define PRESSURE5_REG_SENSOR_TIME_2                   (0x0E)
#define PRESSURE5_REG_EVENT                           (0x10)
#define PRESSURE5_REG_INT_STATUS                      (0x11)
#define PRESSURE5_REG_FIFO_LENGTH                     (0x13)
#define PRESSURE5_REG_FIFO_DATA                       (0x14)
#define PRESSURE5_REG_FIFO_WATERMARK_0                (0x15)
#define PRESSURE5_REG_FIFO_WATERMARK_1                (0x16)
#define PRESSURE5_REG_FIFO_CONFIG_1                   (0x17)
#define PRESSURE5_REG_FIFO_CONFIG_2                   (0x18)
#define PRESSURE5_REG_INT_CTRL                        (0x19)
#define PRESSURE5_REG_IF_CONFIG                       (0x1A)
#define PRESSURE5_REG_POWER_CTRL                      (0x1B)
#define PRESSURE5_REG_OSR                             (0x1C)
#define PRESSURE5_REG_ODR                             (0x1D)
#define PRESSURE5_REG_CONFIGURATION                   (0x1F)
#define PRESSURE5_REG_COMMAND                         (0x7E)


//defgroup fifo_config FIFO Config 

#define PRESSURE5_FCFG_FIFO_ENABLE                    (0x01)
#define PRESSURE5_FCFG_FIFO_DISABLE                   (0x00)
#define PRESSURE5_FCFG_FIFO_STOP_ON_FULL_ENABLE       (0x02)
#define PRESSURE5_FCFG_FIFO_STOP_ON_FULL_DISABLE      (0x00)
#define PRESSURE5_FCFG_FIFO_TIME_ENABLE               (0x04)
#define PRESSURE5_FCFG_FIFO_TIME_DISABLE              (0x00)
#define PRESSURE5_FCFG_FIFO_PRESSURE_STORE_ENABLE     (0x08)
#define PRESSURE5_FCFG_FIFO_PRESSURE_STORE_DISABLE    (0x00)
#define PRESSURE5_FCFG_FIFO_TEMP_STORE_ENABLE         (0x10)
#define PRESSURE5_FCFG_FIFO_TEMP_STORE_DISABLE        (0x00)

//defgroup int_config Int config 

#define PRESSURE5_INTCFG_INT_OUTPUT_PUSH_PULL         (0x00)
#define PRESSURE5_INTCFG_INT_OUTPUT_OPEN_DRAIN        (0x01)
#define PRESSURE5_INTCFG_INT_LEVEL_ACTIVE_HIGH        (0x02)
#define PRESSURE5_INTCFG_INT_LEVEL_ACTIVE_LOW         (0x00)
#define PRESSURE5_INTCFG_INT_LATCH_ENABLE             (0x04)
#define PRESSURE5_INTCFG_INT_LATCH_DISABLE            (0x00)
#define PRESSURE5_INTCFG_FIFO_WTM_ENABLE              (0x08)
#define PRESSURE5_INTCFG_FIFO_WTM_DISABLE             (0x00)
#define PRESSURE5_INTCFG_FIFO_FULL_ENABLE             (0x10)
#define PRESSURE5_INTCFG_FIFO_FULL_DISABLE            (0x00)
#define PRESSURE5_INTCFG_DATA_READY_ENABLE            (0x20)
#define PRESSURE5_INTCFG_DATA_READY_DISABLE           (0x00)

//defgroup if_config IF config 

#define PRESSURE5_IFCFG_SPI_4_WIRE                    (0x00)
#define PRESSURE5_IFCFG_SPI_3_WIRE                    (0x01)
#define PRESSURE5_IFCFG_I2C_WATCHDOG_ENABLE           (0x02)
#define PRESSURE5_IFCFG_I2C_WATCHDOG_DISABLE          (0x00)
#define PRESSURE5_IFCFG_I2C_WATCHDOG_SHORT_1p25ms     (0x00)
#define PRESSURE5_IFCFG_I2C_WATCHDOG_LONG_40ms        (0x04)

 //defgroup power_ctrl Power control registers


#define PRESSURE5_PCTRL_PRESSURE_SENSOR_DISABLE       (0x00)
#define PRESSURE5_PCTRL_PRESSURE_SENSOR_ENABLE        (0x01)
#define PRESSURE5_PCTRL_TEMP_SENSOR_DISABLE           (0x00)
#define PRESSURE5_PCTRL_TEMP_SENSOR_ENABLE            (0x02)
#define PRESSURE5_PCTRL_MODE_SLEEP                    (0x00)
#define PRESSURE5_PCTRL_MODE_FORCED                   (0x10)
#define PRESSURE5_PCTRL_MODE_NORMAL                   (0x30)

//defgroup osr_registers OSR Registers 
 
#define PRESSURE5_OSR_PRESSURE_x1                     (0x00)
#define PRESSURE5_OSR_PRESSURE_x2                     (0x01)
#define PRESSURE5_OSR_PRESSURE_x4                     (0x02)
#define PRESSURE5_OSR_PRESSURE_x8                     (0x03)
#define PRESSURE5_OSR_PRESSURE_x16                    (0x04)
#define PRESSURE5_OSR_PRESSURE_x32                    (0x05)
#define PRESSURE5_OSR_TEMP_x1                         (0x00 >> 1)
#define PRESSURE5_OSR_TEMP_x2                         (0x10 >> 1)
#define PRESSURE5_OSR_TEMP_x4                         (0x20 >> 1)
#define PRESSURE5_OSR_TEMP_x8                         (0x30 >> 1)
#define PRESSURE5_OSR_TEMP_x16                        (0x40 >> 1)
#define PRESSURE5_OSR_TEMP_x32                        (0x50 >> 1)

 //defgroup odr_registers ODR registers 

#define PRESSURE5_ODR_200                             (0x00)
#define PRESSURE5_ODR_100                             (0x01)
#define PRESSURE5_ODR_50                              (0x02)
#define PRESSURE5_ODR_25                              (0x03)
#define PRESSURE5_ODR_12p5                            (0x04)
#define PRESSURE5_ODR_6p25                            (0x05)
#define PRESSURE5_ODR_3p1                             (0x06)
#define PRESSURE5_ODR_1p5                             (0x07)
#define PRESSURE5_ODR_0p75                            (0x08)
#define PRESSURE5_ODR_0p39                            (0x09)
#define PRESSURE5_ODR_0p2                             (0x0A)
#define PRESSURE5_ODR_0p1                             (0x0B)
#define PRESSURE5_ODR_0p05                            (0x0C)
#define PRESSURE5_ODR_0p02                            (0x0D)
#define PRESSURE5_ODR_0p01                            (0x0E)
#define PRESSURE5_ODR_0p006                           (0x0F)
#define PRESSURE5_ODR_0p003                           (0x10)
#define PRESSURE5_ODR_0p0015                          (0x11)

//defgroup config_registers CONFIG Registers 

#define PRESSURE5_CFG_FILTER_COEFF_0                  (0x00)
#define PRESSURE5_CFG_FILTER_COEFF_1                  (0x01 << 1)
#define PRESSURE5_CFG_FILTER_COEFF_3                  (0x02 << 1)
#define PRESSURE5_CFG_FILTER_COEFF_7                  (0x03 << 1)
#define PRESSURE5_CFG_FILTER_COEFF_15                 (0x04 << 1)
#define PRESSURE5_CFG_FILTER_COEFF_31                 (0x05 << 1)
#define PRESSURE5_CFG_FILTER_COEFF_63                 (0x06 << 1)
#define PRESSURE5_CFG_FILTER_COEFF_127                (0x07 << 1)

//defgroup cmd_registers CMD registers 

#define PRESSURE5_CMD_EXTMODE_EN_MIDDLE               (0x34)
#define PRESSURE5_CMD_FIFO_FLUSH                      (0xB0)
#define PRESSURE5_CMD_SOFTWARE_RESET                  (0xB6)

//defgroup slave_address Slave Address 

#define PRESSURE5_DEVICE_SLAVE_ADDR_GND               (0x76)
#define PRESSURE5_DEVICE_SLAVE_ADDR_VCC               (0x77)

//defgroup data_raw Data Raw 

#define PRESSURE5_TEMPERATURE_RAW_DATA                (0x07)
#define PRESSURE5_PRESSURE_RAW_DATA                   (0x04)


//defgroup other_macros Other Macros 

#define PRESSURE5_DEVICE_ID                           (0x50)

typedef uint8_t  pressure5_select_t;

typedef struct
{

   uint32_t i2c_speed;
   uint8_t  i2c_address;

   uint32_t spi_speed;
  

   pressure5_select_t sel;

} pressure5_cfg_t;

typedef struct
{
    uint16_t t1;
    uint16_t t2;
    int8_t   t3;

    int16_t  p1;
    int16_t  p2;
    int8_t   p3;
    int8_t   p4;
    uint16_t p5;
    uint16_t p6;
    int8_t   p7;
    int8_t   p8;
    int16_t  p9;
    int8_t   p10;
    int8_t   p11;

} pressure5_nvm_par_t;


typedef struct
{
    float t1;
    float t2;
    float t3;

    float p1;
    float p2;
    float p3;
    float p4;
    float p5;
    float p6;
    float p7;
    float p8;
    float p9;
    float p10;
    float p11;

} pressure5_par_coeff_t;


#ifdef __cplusplus
extern "C"{
#endif



void Pressure5Init (void);
void Pressure5Default_cfg (void);
void Pressure5GenericWrite (uint8_t reg, uint8_t reg_data);
uint8_t Pressure5GenericRead (uint8_t reg );
void Pressure5Read (uint8_t reg, uint8_t *data_buf, uint8_t len );
uint8_t Pressure5GetInterruptState (void);
void Pressure5SoftwareReset (void);
uint32_t Pressure5GetRawData (uint8_t data_addr );
void Pressure5UpdateCoefficient ( void );
float Pressure5GetTemperatureData ( void);
float Pressure5GetPressureData(void);

void SelectPressureSensor(void);
void DeselectPressureSensor(void);


#ifdef __cplusplus
}
#endif
#endif  // _PRESSURE5_H_
