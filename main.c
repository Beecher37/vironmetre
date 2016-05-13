#include "mcc_generated_files/mcc.h"
#include "sensors/barometer.h"

void info_led() {
    LED_RA0_SetHigh();
    LED_RA1_SetLow();
    LED_RA2_SetHigh();

    uint8_t i = 0;
    for (i = 0; i < UINT8_MAX; i++)
        __delay_ms(2);

    LED_RA1_SetHigh();
}

void error_led() {
    LED_RA0_SetHigh();
    LED_RA1_SetHigh();
    LED_RA2_SetLow();

    uint8_t i = 0;
    for (i = 0; i < UINT8_MAX; i++)
        __delay_ms(2);

    LED_RA2_SetHigh();
}

/**
 * Trouve l'adresse du premier périphérique I2C connecté au bus.
 * @return Adresse du périphérique trouvé, ou 0 si aucun périphérique trouvé.
 */
uint8_t firstI2Cdevice() {
    uint8_t timeOut = 0;
    uint8_t addr = 0;
    uint8_t buffer = 0;

    I2C_MESSAGE_STATUS status = I2C_MESSAGE_PENDING;

    for (addr = 1; addr < 128; addr++) {
        status = I2C_MESSAGE_PENDING;

        while (status != I2C_MESSAGE_FAIL) {
            // write one byte to EEPROM (2 is the count of bytes to write)
            //I2C_MasterRead(NULL, 0, addr, &status);
            I2C_MasterRead(&buffer, 1, addr, &status);

            // wait for the message to be sent or status has changed.
            while (status == I2C_MESSAGE_PENDING && !SEN_CONN_GetValue());

            if(SEN_CONN_GetValue())
                return 0;
            
            if (status == I2C_MESSAGE_COMPLETE) {
               // printf("%x\n", buffer);
                return addr;
            }

            // check for max retry and skip this byte
            if (timeOut == 255)
                break;
            else
                timeOut++;
        }
    }

    return 0;
}

void main() {
    uint8_t i;
    double T, P;

    // Initialize the device
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowEnable();

    // Disable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighDisable();

    // Disable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowDisable();

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    // <editor-fold defaultstate="collapsed" desc="intro LEDs">

    // All LEDs off
    LED_RA0_SetHigh();
    LED_RA1_SetHigh();
    LED_RA2_SetHigh();

    // RA0 ON   
    LED_RA0_SetLow();
    for (i = 0; i < UINT8_MAX; i++)
        __delay_ms(2);
    LED_RA0_SetHigh();

    // RA1 ON   
    LED_RA1_SetLow();
    for (i = 0; i < UINT8_MAX; i++)
        __delay_ms(2);
    LED_RA1_SetHigh();

    // RA2 ON   
    LED_RA2_SetLow();
    for (i = 0; i < UINT8_MAX; i++)
        __delay_ms(2);
    LED_RA2_SetHigh();

    // All LEDs off
    LED_RA0_SetHigh();
    LED_RA1_SetHigh();
    LED_RA2_SetHigh();
    
    // </editor-fold>
    
    while (1) {
        if(!SEN_CONN_GetValue())
        { 
            switch (firstI2Cdevice()) {
                case BMP180_ADDRESS:
                    if (BMP180_Init()) {
                        info_led();

                        
                        while (!SEN_CONN_GetValue()) {
                            if (BMP180_GetTemperature(&T)) {
                                printf("%f deg\n", T);
                                if(BMP180_GetPressure(&P, T, 3))
                                    printf("%f\n\n", P);
                                else
                                    error_led();                        
                            }
                            else
                                error_led();
                           
                        }
                    }
                    else {
                        error_led();
                    }

                    break;

                default:
                    break;
            }
        }
    }
}
