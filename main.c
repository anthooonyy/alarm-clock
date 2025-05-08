 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.0
*/

/*
© [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/
#include "mcc_generated_files/system/system.h"

/*
    Main application
*/
/****FUNCTION DEFINITIONS****/
uint8_t BCDtoDecimal(uint8_t bcd);
const char* dayConversion(uint8_t binDay);
void readTime(void);

/****PCF8563 REGISTER ADDRESSES****/
#define PCF8563_ADDRESS         0x51    // address w/o read/write bit
#define PCF8563_REG_CONTROL1    0x00    // first control register
//#define PCF8563_REG_CONTROL2    0x01
#define PCF8563_REG_SECONDS     0x02    // seconds register, first time register
//#define PCF8563_REG_MINUTES     0x03
//#define PCF8563_REG_HOURS       0x04
//#define PCF8563_REG_DAYS        0x05
//#define PCF8563_REG_WEEKDAYS    0x06
//#define PCF8563_REG_MONTHS      0x07
//#define PCF8563_REG_YEARS       0x08
#define PCF8563_REG_MINALARM    0x09    // minute alarm reg, first alarm register
#define PCF8563_REG_CLKOUT      0x0D    // CLKOUT reg, control clock freq.

/****ARRAY INIT FOR I2C COMM****/
uint8_t timeDateWriteData[] = {PCF8563_REG_SECONDS, 0x00, 0x39, 0x05, 0x05, 0x03, 0x05, 0x25};
uint8_t CLKOUT_Control[] = {PCF8563_REG_CLKOUT, 0x00};
uint8_t timeDateReadData[8];
uint8_t RTCSetup[] = {PCF8563_REG_CONTROL1, 0x00, 0x00};

/****MAIN FUNCTION****/
int main(void)
{
    SYSTEM_Initialize();    // port and peripheral setup and initialization

    // Enable the Global Interrupts 
    INTERRUPT_GlobalInterruptEnable(); 

    // Disable the Global Interrupts 
    //INTERRUPT_GlobalInterruptDisable(); 

    // Enable the Peripheral Interrupts 
    INTERRUPT_PeripheralInterruptEnable(); 

    // Disable the Peripheral Interrupts 
    //INTERRUPT_PeripheralInterruptDisable(); 
    
    /****CLOCK INITIALIZATION****/
    if(I2C2_Write(PCF8563_ADDRESS, RTCSetup, 3)){
        printf("RTC Initialized\n\r");
    }
    
    if(I2C2_Write(PCF8563_ADDRESS, CLKOUT_Control, 2 )){
        printf("CLKOUT Set\n\r");
    }
    
    if(I2C2_Write(PCF8563_ADDRESS, timeDateWriteData, 8)){
        printf("Clock Programmed\n\r");
    }
    
    timeDateReadData[0]= PCF8563_ADDRESS;   //set first instance to point at 0xA3 for I2C read

    while(1)
    {
        readTime();
        LED_Toggle();
        
        __delay_ms(10);
    }    
}
/*
 * Function: BCDtoDecimal()
 * Input: unsigned 8-bit value in BCD format
 * Return: unsigned 8-bit value in decimal
 * Description:
 *      Receives a value coded in BCD format. Typically, the 4 MSB represent the
 *      tens place, and the 4 LSB represent the unit value. 
 *      Performs a bit mask to isolate the 4 MSB, then shifts those bits to 
 *      become the LSB, and multiplies that value by 10. Then it performs a 
 *      bit mask on the 4 LSB and adds that value to the previous value.
 *      And, returns the sum as a decimal value.
 */
uint8_t BCDtoDecimal(uint8_t bcd){
    return (((bcd & 0xF0) >> 4) * 10) + (bcd & 0x0F);
}

/*
 * Function: readTime()
 * Input: None
 * Return: None
 * Description:
 *      Initiates I2C read request to PCF8563 to read the 7 registers from
 *      seconds to year. Upon successful read, the output is coded in BCD.
 *      These outputs are masked and converted to decimal, where the data
 *      is displayed over serial. 
 */
void readTime(){
    if(I2C2_WriteRead(PCF8563_ADDRESS, 0x02, 1, timeDateReadData, 8)){
            printf("Read Successful \n\r");

            // block of code that sets raw data to BCD value with masking
            uint8_t seconds = timeDateReadData[1] & 0x7F; // Mask out the VL bit
            uint8_t minutes = timeDateReadData[2] & 0x7F; // Mask out unused bits
            uint8_t hours = timeDateReadData[3] & 0x3F;   // Mask out unused bits
            uint8_t days = timeDateReadData[4] & 0x3F;    // Mask out unused bits
            uint8_t weekdays = timeDateReadData[5] & 0x07; // Mask out unused bits
            uint8_t months = timeDateReadData[6] & 0x1F;  // Mask out century bit
            uint8_t years = timeDateReadData[7];          // No masking needed

            // block of code converts BCD value to decimal value
            uint8_t decSeconds = BCDtoDecimal(seconds);
            uint8_t decMinutes = BCDtoDecimal(minutes);
            uint8_t decHours = BCDtoDecimal(hours);
            uint8_t decDays = BCDtoDecimal(days);
            //const char* weekday = dayConversion(weekdays);
            uint8_t decMonths = BCDtoDecimal(months);
            uint8_t decYears = BCDtoDecimal(years);

            // serial data output for time and date
            printf("Converted Time: %02u:%02u:%02u\n\r", decHours, decMinutes, decSeconds);
            printf("Converted Date: %02u-%02u-20%02u\n\r", decDays, decMonths, decYears);

            // displays number to clock's 7-segment display
            //displayNumber(decMinutes, decHours);

        }else{
            printf("Read Unsuccessful \n\r");
        }
}