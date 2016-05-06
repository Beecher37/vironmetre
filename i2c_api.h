/* 
 * File:                I2C_API.h
 * Author:              Thomas Prioul
 * Comments:            I2C API for the Android Smartphone
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef I2C_API_H
#define	I2C_API_H

#include <stdbool.h>
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

bool I2C_ReadBytes(uint8_t dev_addr, uint8_t* p_buffer, uint8_t length);
bool I2C_WriteBytes(uint8_t dev_addr, uint8_t* p_buffer, uint8_t length);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* I2C_API_H */

