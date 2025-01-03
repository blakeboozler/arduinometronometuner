#ifndef ADC_H
#define ADC_H

#include <Arduino.h>
#include "arduinoFFT.h"
#include "pitches.h"


// ************************** USART ******************************** //

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (F_CPU / (USART_BAUDRATE * 16UL) - 1)

void initializeUSART();
void USART_transmit_str(const char *str);
void USART_transmit(unsigned char data);


// ************************** ADC ******************************* //

extern volatile float peakFreq;

void initADC();


// ************************** HELPERS ******************************** //

char* nearestNote(float);

#endif