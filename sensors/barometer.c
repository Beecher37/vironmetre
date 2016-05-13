#include <math.h>
#include <stdio.h>
#include "barometer.h"
#include "../i2c_api.h"

#define _XTAL_FREQ  8000000

#define	BMP180_REG_CONTROL 0xF4
#define	BMP180_REG_RESULT 0xF6

#define	BMP180_COMMAND_TEMPERATURE  0x2E
#define	BMP180_COMMAND_PRESSURE0    0x34
#define	BMP180_COMMAND_PRESSURE1    0x74
#define	BMP180_COMMAND_PRESSURE2    0xB4
#define	BMP180_COMMAND_PRESSURE3    0xF4

#define AC1_REG     0xAA
#define AC2_REG     0xAC
#define AC3_REG     0xAE
#define AC4_REG     0xB0
#define AC5_REG     0xB2
#define AC6_REG     0xB4
#define B1_REG      0xB6
#define B2_REG      0xB8
#define MB_REG      0xBA
#define MC_REG      0xBC
#define MD_REG      0xBE 

int16_t AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
uint16_t AC4,AC5,AC6;

double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;

bool BMP180_ReadInt16(uint8_t reg, int16_t* p_result)
{
	uint8_t data[2];

    if(I2C_WriteBytes(BMP180_ADDRESS, &reg, 1) &&
       I2C_ReadBytes(BMP180_ADDRESS, data, 2))
    {
        *p_result = (((int16_t)data[0] << 8) | ((int16_t)data[1]));
        return true;
    }
    
	return false;    
}

bool BMP180_ReadUint16(uint8_t reg, uint16_t* p_result)
{
	uint8_t data[2];

    if(I2C_WriteBytes(BMP180_ADDRESS, &reg, 1) &&
       I2C_ReadBytes(BMP180_ADDRESS, data, 2))
    {
        *p_result = (((uint16_t)data[0] << 8) | ((uint16_t)data[1]));
        return true;
    }
    
	return false;  
}

bool BMP180_Init() {
    double c3,c4,b1;
    
    if( BMP180_ReadInt16(AC1_REG, &AC1) &&
        BMP180_ReadInt16(AC2_REG, &AC2) &&
        BMP180_ReadInt16(AC3_REG, &AC3) &&
        BMP180_ReadUint16(AC4_REG, &AC4) &&
        BMP180_ReadUint16(AC5_REG, &AC5) &&
        BMP180_ReadUint16(AC6_REG, &AC6) &&
        BMP180_ReadInt16(B1_REG, &VB1) &&
        BMP180_ReadInt16(B2_REG, &VB2) &&
        BMP180_ReadInt16(MB_REG, &MB) &&
        BMP180_ReadInt16(MC_REG, &MC) &&
        BMP180_ReadInt16(MD_REG, &MD))
    {
        // Compute floating-point polynominals:
        /*
        printf("AC1: %d\n", AC1);
        printf("AC2: %d\n", AC2);
        printf("AC3: %d\n", AC3);
        printf("AC4: %u\n", AC4);
        printf("AC5: %u\n", AC5);
        printf("AC6: %u\n", AC6);
        printf("B1: %d\n", VB1);
        printf("B2: %d\n", VB2);
        printf("MB: %d\n", MB);
        printf("MC: %d\n", MC);
        printf("MD: %d\n", MD);
        */
		c3 = 160.0 * pow(2,-15) * AC3;
		c4 = pow(10,-3) * pow(2,-15) * AC4;
		b1 = pow(160,2) * pow(2,-30) * VB1;
		c5 = (pow(2,-15) / 160) * AC5;
		c6 = AC6;
		mc = (pow(2,11) / pow(160,2)) * MC;
		md = MD / 160.0;
		x0 = AC1;
		x1 = 160.0 * pow(2,-13) * AC2;
		x2 = pow(160,2) * pow(2,-25) * VB2;
		y0 = c4 * pow(2,15);
		y1 = c4 * c3;
		y2 = c4 * b1;
		p0 = (3791.0 - 8.0) / 1600.0;
		p1 = 1.0 - 7357.0 * pow(2,-20);
		p2 = 3038.0 * 100.0 * pow(2,-36);
        
        /*
        printf("b1:%f\nc3:%f\nc4:%f\nc5:%f\nc6:%f\nmc:%f\nmd:%f\nx0:%f\nx1:%f\nx2:%f\ny0:%f\ny1:%f\ny2:%f\np0:%f\np1:%f\np2:%f\n",
                b1,c3,c4,c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2);
        */
        return true;
    }
    return false;
}

bool BMP180_GetTemperature(double* T)
{
    uint8_t data[2];
    double tu, a;
    data[0] = BMP180_REG_CONTROL;
    data[1] = BMP180_COMMAND_TEMPERATURE;
    
    if (I2C_WriteBytes(BMP180_ADDRESS, data, 2))
    {
        __delay_ms(5);
        data[0] = BMP180_REG_RESULT;
        
        if(I2C_WriteBytes(BMP180_ADDRESS, data, 1) &&
           I2C_ReadBytes(BMP180_ADDRESS, data, 2))
        {
            tu = (data[0] * 256.0) + data[1];
            a = c5 * (tu - c6);
            *T = a + (mc / (a + md));        
            /*
            printf("temp\n");
            printf("data:%d\n", data[0]);
            printf("     %d\n", data[1]); 
            printf("tu:%f\n", tu);
            printf("a :%f\n", a);
            printf("T :%f\n", *T);
            */
            return true;
        }
    } 
    return false;
}

bool BMP180_GetPressure(double* P, double T, uint8_t OSS)
{
    uint8_t data[3];
    
    double pu,s,x,y,z;
    
	data[0] = BMP180_REG_CONTROL;
    switch (OSS)
	{
        default:
		case 0:
			data[1] = BMP180_COMMAND_PRESSURE0;
			data[2] = 5;
            break;
		case 1:
			data[1] = BMP180_COMMAND_PRESSURE1;
			data[2] = 8;
            break;
		case 2:
			data[1] = BMP180_COMMAND_PRESSURE2;
			data[2] = 14;
            break;
		case 3:
			data[1] = BMP180_COMMAND_PRESSURE3;
			data[2] = 26;
            break;	
	}
    
    if(I2C_WriteBytes(BMP180_ADDRESS, data, 2))
    {
        while(data[2]--)
            __delay_ms(1);
        
        data[0] = BMP180_REG_RESULT;
        
        if(I2C_WriteBytes(BMP180_ADDRESS, data, 1) &&
           I2C_ReadBytes(BMP180_ADDRESS, data, 3))
        {
            /*
            printf("data:%d\n", data[0]);
            printf("     %d\n", data[1]); 
            printf("     %d\n", data[2]); 
            */
            pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);
            s = T - 25.0;
            x = (x2 * pow(s,2)) + (x1 * s) + x0;
            y = (y2 * pow(s,2)) + (y1 * s) + y0;
            z = (pu - x) / y;
            *P = (p2 * pow(z,2)) + (p1 * z) + p0;
            
            /*
            printf("pu:%f\n", pu);
            printf("s :%f\n", s);
            printf("x,y,z:%f,%f,%f\n", x,y,z);
            printf("P :%f\n", *P);
            */
            return true;
        }
    }
    
    return false;
}
