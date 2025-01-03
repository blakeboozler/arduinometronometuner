#include "lcdDriver.h"

// LCD is initially set to 8-bit mode. Reset the LCD controller to 4-bit mode before doing anything else //
void LCD_init()
{
    // wait for power up - (longer than 30ms) //
    _delay_ms(100);

    // Reset the controller to enable 4-bit mode //
    LCD_E_RS_init();    // Set E and RS pins active low for each LCD reset //

    // Reset and wait for activation //
    LCD_write_4bits(LCD_Reset);
    _delay_ms(10);

    // Set LCD to 4-bit Mode //
    LCD_write_4bits(LCD_4bit_Enable);
    _delay_us(80);  // delay must be > 39us //

    //////////////  System Reset Complete   //////////////
    /*
    *   Now operating in 4-bit mode.
    *   Must send high-nibble and low-nibble seperatly.
    *   Can now set line numbers and font size.
    *   Note: We use LCD_write_4bits() when in 8 bit mode
    *   and the LCD_write_instruction() [calls LCD_write_4bits twice]
    *   once we are in 4-bit mode.
    *   The set of instructions are found in table 7 of the datasheet for the KS0066U -> https://www.lcd-module.de/eng/pdf/zubehoer/ks0066.pdf
    */
    LCD_write_instruction(LCD_4bit_Mode);
    _delay_us(80);  // delay must be > 39us //

    LCD_write_instruction(LCD_4bit_DisplayOFF);
    _delay_us(80); //delay must be > 39us

    LCD_write_instruction(LCD_4bit_DisplayCLEAR);
    _delay_ms(80);  // delay must be > 1.53ms //

    LCD_write_instruction(LCD_4bit_entryMODE);
    _delay_us(80);  // delay must be > 39us //

    // LCD should now be initialized to operate in 4-bit mode, 2 lines, 5x8 fontsize //
    // Need to turn display back on for use //
    LCD_write_instruction(LCD_4bit_DisplayON_BL);
    _delay_us(80);  // delay must be > 39us //
}



void LCD_E_RS_init()
{
    // Set E and RS pins active low for the reset function //
    PORTB &= ~(1<<LCD_EnablePin);
    PORTB &= ~(1<<LCD_RegisterSelectPin);
}



void LCD_write_4bits(uint8_t Data)
{
    PORTA &= 0b00001111;    // clear upper nibble of PORTA //
    PORTA |= Data;          // Write data to data lines on PORTA //

    LCD_EnablePulse();      // Pulse the enable to write/read the data //
}



// Write instruction in 4-bit mode: send upper nybble then lower nybble //
void LCD_write_instruction(uint8_t Instruction)
{
    // ensure RS is low //
    LCD_E_RS_init();

    LCD_write_4bits(Instruction);       // write high nybble
    LCD_write_4bits(Instruction<<4);    // write low nybble
}



// Pulse read/write enable pin on the LCD Controller -  should be at least 230ns pulse width //
void LCD_EnablePulse(void)
{
    // set enable bit low(currently) -> high -> low //
    PORTB |= (1<<LCD_EnablePin);    // SET ENABLE HIGH //
    _delay_us(1);                   // WAIT TO ENSURE PIN HIGH //
    PORTB &= ~(1<<LCD_EnablePin);   //  SET ENABLE LOW //
     _delay_us(1);                   // WAIT TO ENSURE PIN LOW //
}



// write character to display //
void LCD_write_char(char Data)
{
    // Set up E and RS lines for data writing //
    PORTB |= (1<<LCD_RegisterSelectPin);    // Ensure RS pin is high //
    PORTB &= ~(1<<LCD_EnablePin);           // Ensure the enable pin is low //
    LCD_write_4bits(Data);                  // write the upper nybble //
    LCD_write_4bits(Data<<4);               // write lower nybble
    _delay_us(80);                          // need to wait > 43us //       
}


// write an entire line of characters to the display //
void LCD_write_line(char *Data)
{
    int i = 0;

    // Set up E and RS lines for data writing //
    PORTB |= (1<<LCD_RegisterSelectPin);    // Ensure RS pin is high //
    PORTB &= ~(1<<LCD_EnablePin);           // Ensure the enable pin is low //

    // Print all 8 characters to LCD //
    while(Data[i]!='\0')
    {
        LCD_write_4bits(Data[i]);       // write the upper nybble //
        LCD_write_4bits(Data[i]<<4);    // write lower nybble
        _delay_us(80);                  // need to wait > 43us //
        
        i++;
    } 
}



// Clear both lines of the display //
void LCD_Clear()
{
    LCD_E_RS_init();
    LCD_write_instruction(LCD_4bit_DisplayCLEAR);
    _delay_ms(10);
    //LCD_init();
}


// shift displayed data without changing DDRAM //
void LCD_Shift_Display_Right()
{   
    LCD_write_instruction(LCD_shiftRight);
    _delay_us(80);
}