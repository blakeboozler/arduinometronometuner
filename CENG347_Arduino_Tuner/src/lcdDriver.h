#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

/////   Notice  /////
/*
*   PortB is for control lines
*   PORTA (Upper Nybble) is for data lines
*/


// LCD CONTROL //
#define LCD_Reset               0b00110000  // reset LCD to put in 4-bit mode //
#define LCD_4bit_Enable         0b00100000  // Must set this before line display or fonts //
#define LCD_4bit_Mode           0b00101000  // 2-line Display, 5x8 font //
#define LCD_4bit_DisplayOFF     0b00001000  // display off //
#define LCD_4bit_DisplayON_NOBL 0b00001100  // display on - no blink //
#define LCD_4bit_DisplayON_BL   0b00001101  // display on - with blink //
#define LCD_4bit_DisplayCLEAR   0b00000001  // Clear DDRAM //
#define LCD_4bit_entryMODE      0b00000110  // set curser to write/read from left -> right //
#define LCD_4bit_cursorSET      0b10000000  // set cursor position
#define LCD_shiftRight          0b00011100  // shift display right
#define LCD_shiftLeft           0b00011000  // shift display left

// For 2 line mode //
#define LineOneStart    0x00
#define LineTwoStart    0x40    // must set DDRAM address in LCD controller for line 2 //

// Pin definitions for PORTB control lines //
#define LCD_EnablePin           1
#define LCD_RegisterSelectPin   0

// Prototypes //
void LCD_init();
void LCD_E_RS_init();
void LCD_write_4bits(uint8_t);
void LCD_EnablePulse(void);
void LCD_write_instruction(uint8_t);

void LCD_write_char(char);
void LCD_write_line(char*);
void LCD_Clear();
void LCD_Shift_Display_Right();