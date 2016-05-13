/* 
 * File:                barometer.h 
 * Author:              Thomas Prioul
 * Comments:
 * Revision history: 
 */

#ifndef BAROMETER_H
#define	BAROMETER_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdbool.h>
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#define         BMP180_ADDRESS 0x77
    
bool            BMP180_Init(void);
bool            BMP180_GetTemperature(double* T);
bool            BMP180_GetPressure(double* P, double T, uint8_t OSS);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* BAROMETER_H */

