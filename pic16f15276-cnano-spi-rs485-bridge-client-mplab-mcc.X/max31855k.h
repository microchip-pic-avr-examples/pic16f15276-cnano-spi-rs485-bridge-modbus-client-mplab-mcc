#ifndef THERMO_H
#define THERMO_H

#include <stdio.h>
#include <stdint.h>

#define THERMO_OK                             (0)
#define THERMO_ERROR                          (1)

#define TEMPERATURE_BUFFER_LENGTH             (4)
#define MASK_LSB_BIT                          (0x01)
#define FAULT_BIT                             (16)
#define VCC_SHORT_ERROR_BIT                   (2)
#define SHORT_CIRCUIT_GND_ERROR_BIT           (1)
#define TEMP_DELAY                            (100)
#define TEMP_CONV_DELAY                       (350)
#define TEMP_CAL_CONST1                       (-0.25)
#define TEMP_CAL_CONST2                       (1.0)
#define TEMP_CAL_CONST3                       (0.25)
#define SIGN_CHAR                             (128)
#define MASK_LOWER_TWO_BITS                   (0x03)

#ifdef __cplusplus
extern "C"{
#endif

uint32_t Thermo_read_data (void);
float Thermo_get_temperature (void);
uint8_t Thermo_check_fault (void);
uint8_t Thermo_short_circuited_vcc (void);
uint8_t Thermo_short_circuited_gnd (void);
uint8_t Thermo_check_connections (void);

#ifdef __cplusplus
}
#endif
#endif  // _THERMO_H_

