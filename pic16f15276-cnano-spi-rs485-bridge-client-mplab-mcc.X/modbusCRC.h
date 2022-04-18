/* 
 * File:   modbusCRC.h
 * Author: I20946
 *
 * Created on November 9, 2021, 3:49 PM
 */

#include <stdint.h>

#ifndef MODBUSCRC_H
#define	MODBUSCRC_H

#ifdef	__cplusplus
extern "C" {
#endif

#define CRC16_POLYNOMIAL  (0xA001)
    
uint16_t ModRTU_CRC(uint8_t buf[], uint8_t len);

#ifdef	__cplusplus
}
#endif

#endif	/* MODBUSCRC_H */

