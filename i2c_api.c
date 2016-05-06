#include "i2c_api.h"
#include "mcc_generated_files/i2c.h"
#include "mcc_generated_files/pin_manager.h"

bool I2C_ReadBytes(uint8_t dev_addr, uint8_t* p_buffer, uint8_t length)
{
    I2C_MESSAGE_STATUS status = I2C_MESSAGE_PENDING;
    uint8_t timeOut = 0;
    
    while(!I2C_MasterQueueIsEmpty());

    // this portion will read the byte from the memory location.
    timeOut = 0;
    status = I2C_MESSAGE_PENDING;
    while (status != I2C_MESSAGE_FAIL) {
        // read buffer from device
        I2C_MasterRead(p_buffer, length, dev_addr, &status);

        // wait for the message to be sent or status has changed.
        while (status == I2C_MESSAGE_PENDING && !SEN_CONN_GetValue());

        if(SEN_CONN_GetValue())
            return false;

        if (status == I2C_MESSAGE_COMPLETE)
            return true;

        // check for max retry and skip this byte
        if (timeOut == UINT8_MAX)
            break;
        else
            timeOut++;
    }

    return false;
}

bool I2C_WriteBytes(uint8_t dev_addr, uint8_t* p_buffer, uint8_t length)
{
    I2C_MESSAGE_STATUS status = I2C_MESSAGE_PENDING;
    uint8_t timeOut = 0;

    while(!I2C_MasterQueueIsEmpty());
    
    // Write reg pointer
    while(status != I2C_MESSAGE_FAIL)
    {
        // write bytes to the device
        I2C_MasterWrite(p_buffer, length, dev_addr, &status);

        // wait for the message to be sent or status has changed.
        while(status == I2C_MESSAGE_PENDING && !SEN_CONN_GetValue());

        if(SEN_CONN_GetValue())
            return false;
        
        if (status == I2C_MESSAGE_COMPLETE)
            return true;

        // check for max retry and skip this byte
        if (timeOut == UINT8_MAX)
            break;
        else
            timeOut++;
    }  
    
    return false;
}

