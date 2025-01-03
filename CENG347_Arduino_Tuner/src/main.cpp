#include <Arduino.h>
#include <util/delay.h>
#include <pitches.h>
#include <string.h>
#include "lcdDriver.h"
#include "arduinoFFT.h"
#include "adc_USART.h"

// ********************* TUNER VARIABLES AND MENU STRINGS ********************* //
enum tuner_flag {Duration = 0, Metronome, Frequency, Nothing};

const char menu1[] = {"Please select the value you'd like to edit:\n1. Duration\n2. Tempo\n3. Frequency\n\nInput: \0"};
const char menu2[] = {"You have chosen to control duration. What length (in ms) would you like (max 5000):\nDuration: \0"};
const char menu3[] = {"You have chosen to control the tempo. What tempo would you like (40 - 500):\nTempo: \0"};
const char menu4[] = {"You have chosen to control the frequency. What note would you like (in Hz, 31 - 4978):\nFrequency: \0"};

volatile static tuner_flag mode = Nothing;
volatile bool metronomeOn = false;
volatile int duration = 2000; // Length held for a note (default 2 seconds)
volatile int tempo = 80; // tempo of metronome (default 80)
volatile int frequency = 440; // Frequency (in Hz) of a note (default 440Hz [A4])

volatile static char input[5] = {'0','0','0','0','\0'};
static uint8_t i = 0;

// Definitions for Timer2
#define AVAILABLE_TONE_PINS 1
#define USE_TIMER2
const uint8_t PROGMEM tone_pin_to_timer_PGM[] = { 2 /*, 3, 4, 5, 1, 0 */ };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255 /*, 255, 255, 255, 255, 255 */ };
volatile long timer2_toggle_count;
volatile uint8_t *timer2_pin_port;
volatile uint8_t timer2_pin_mask;

// ********************* ADC ********************* //
volatile int ADC_IR_Count = 0;

// FFT CONSTANTS //
const uint_fast16_t samples = 512; // This value MUST ALWAYS be a power of 2
const float samplingFrequency = 8500; // Reduce the sampling rate do to code delays - Determine this by experimentation

// Ideal sampling rate
// const float samplingFrequency = 9615;
// These are the input and output vectors
// vvv Input vectors receive computed results from FFT vvv
volatile float vReal[samples]; // Input Sampled Signal - after FFT contains real part of result
volatile float vImag[samples]; // Input Sample Signal - after FFT contains imaginary part of result

// FFT CLASS //
ArduinoFFT<float> FFT = ArduinoFFT<float>((float*)vReal, (float*)vImag, samples, samplingFrequency);
volatile float peakFreq = 0;
volatile bool FFTmode = false;

void initBuzzer();
const char* roundToNearest(int _frequency);
int roundToNearestFreq(int _frequency);

const char* roundToNearest(int _frequency) {
    int notes[] = {
        NOTE_B0, NOTE_C1, NOTE_CS1, NOTE_D1, NOTE_DS1, NOTE_E1, NOTE_F1, NOTE_FS1, NOTE_G1, NOTE_GS1,
        NOTE_A1, NOTE_AS1, NOTE_B1, NOTE_C2, NOTE_CS2, NOTE_D2, NOTE_DS2, NOTE_E2, NOTE_F2, NOTE_FS2,
        NOTE_G2, NOTE_GS2, NOTE_A2, NOTE_AS2, NOTE_B2, NOTE_C3, NOTE_CS3, NOTE_D3, NOTE_DS3, NOTE_E3,
        NOTE_F3, NOTE_FS3, NOTE_G3, NOTE_GS3, NOTE_A3, NOTE_AS3, NOTE_B3, NOTE_C4, NOTE_CS4, NOTE_D4,
        NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_GS4, NOTE_A4, NOTE_AS4, NOTE_B4, NOTE_C5,
        NOTE_CS5, NOTE_D5, NOTE_DS5, NOTE_E5, NOTE_F5, NOTE_FS5, NOTE_G5, NOTE_GS5, NOTE_A5, NOTE_AS5,
        NOTE_B5, NOTE_C6, NOTE_CS6, NOTE_D6, NOTE_DS6, NOTE_E6, NOTE_F6, NOTE_FS6, NOTE_G6, NOTE_GS6,
        NOTE_A6, NOTE_AS6, NOTE_B6, NOTE_C7, NOTE_CS7, NOTE_D7, NOTE_DS7, NOTE_E7, NOTE_F7, NOTE_FS7,
        NOTE_G7, NOTE_GS7, NOTE_A7, NOTE_AS7, NOTE_B7, NOTE_C8, NOTE_CS8, NOTE_D8, NOTE_DS8
    };

    const char* noteNames[] = {
        "B0", "C1", "C#1", "D1", "D#1", "E1", "F1", "F#1", "G1", "G#1",
        "A1", "A#1", "B1", "C2", "C#2", "D2", "D#2", "E2", "F2", "F#2",
        "G2", "G#2", "A2", "A#2", "B2", "C3", "C#3", "D3", "D#3", "E3",
        "F3", "F#3", "G3", "G#3", "A3", "A#3", "B3", "C4", "C#4", "D4",
        "D#4", "E4", "F4", "F#4", "G4", "G#4", "A4", "A#4", "B4", "C5",
        "C#5", "D5", "D#5", "E5", "F5", "F#5", "G5", "G#5", "A5", "A#5",
        "B5", "C6", "C#6", "D6", "D#6", "E6", "F6", "F#6", "G6", "G#6",
        "A6", "A#6", "B6", "C7", "C#7", "D7", "D#7", "E7", "F7", "F#7",
        "G7", "G#7", "A7", "A#7", "B7", "C8", "C#8", "D8", "D#8"
    };

    int nearestIndex = 0;
    int smallestDiff = abs(_frequency - notes[0]);

    for (uint8_t i = 1; i < sizeof(notes) / sizeof(notes[0]); ++i) {
        int diff = abs(_frequency - notes[i]);
        if (diff < smallestDiff) {
            smallestDiff = diff;
            nearestIndex = i;
        }
    }

    return noteNames[nearestIndex];
}

int roundToNearestFreq(int _frequency) {
    int notes[] = {
        NOTE_B0, NOTE_C1, NOTE_CS1, NOTE_D1, NOTE_DS1, NOTE_E1, NOTE_F1, NOTE_FS1, NOTE_G1, NOTE_GS1,
        NOTE_A1, NOTE_AS1, NOTE_B1, NOTE_C2, NOTE_CS2, NOTE_D2, NOTE_DS2, NOTE_E2, NOTE_F2, NOTE_FS2,
        NOTE_G2, NOTE_GS2, NOTE_A2, NOTE_AS2, NOTE_B2, NOTE_C3, NOTE_CS3, NOTE_D3, NOTE_DS3, NOTE_E3,
        NOTE_F3, NOTE_FS3, NOTE_G3, NOTE_GS3, NOTE_A3, NOTE_AS3, NOTE_B3, NOTE_C4, NOTE_CS4, NOTE_D4,
        NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_GS4, NOTE_A4, NOTE_AS4, NOTE_B4, NOTE_C5,
        NOTE_CS5, NOTE_D5, NOTE_DS5, NOTE_E5, NOTE_F5, NOTE_FS5, NOTE_G5, NOTE_GS5, NOTE_A5, NOTE_AS5,
        NOTE_B5, NOTE_C6, NOTE_CS6, NOTE_D6, NOTE_DS6, NOTE_E6, NOTE_F6, NOTE_FS6, NOTE_G6, NOTE_GS6,
        NOTE_A6, NOTE_AS6, NOTE_B6, NOTE_C7, NOTE_CS7, NOTE_D7, NOTE_DS7, NOTE_E7, NOTE_F7, NOTE_FS7,
        NOTE_G7, NOTE_GS7, NOTE_A7, NOTE_AS7, NOTE_B7, NOTE_C8, NOTE_CS8, NOTE_D8, NOTE_DS8
    };

    int nearestIndex = 0;
    int smallestDiff = abs(_frequency - notes[0]);

    for (uint8_t i = 1; i < sizeof(notes) / sizeof(notes[0]); ++i) {
        int diff = abs(_frequency - notes[i]);
        if (diff < smallestDiff) {
            smallestDiff = diff;
            nearestIndex = i;
        }
    }

    return notes[nearestIndex];
}

void initBuzzer() 
{
  // Set all PWM/buzzer pins as output
  DDRH |= 0xFF;
  DDRE |= 0xFF;
  DDRG |= 0xFF;
  DDRB |= 0xFF;

  // Enable global interrupts
  sei();

  // Set timer 0 prescale factor to 64 (For metronome)
  TCCR0B |= (1 << CS01) | (1 << CS00);

  // Enable timer 0 overflow interrupt (Metronome)
  TIMSK0 |= (1 << TOIE0);

  // Timer 2 is used for phase-corrected PWM.
  // This is used for pitch player/duration of note

  // Set timer 2 prescale factor to 64
  TCCR2B |= (1 << CS22);

  // Configure timer 2 for phase-corrected PWM (8-bit)
  TCCR2A |= (1 << WGM20);

  // Set a2d prescaler so we are inside the desired 50-200 KHz range.
  if (F_CPU >= 16000000)
  { // 16 MHz / 128 = 125 KHz
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  }

  // Enable a2d conversions
  ADCSRA |= (1 << ADEN);
}

static int8_t ourToneBegin(uint8_t _pin)
{
  int8_t _timer = -1;

  // If we're already using the pin, the timer should be configured.  
  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) 
  {
    if (tone_pins[i] == _pin) 
    {
      return pgm_read_byte(tone_pin_to_timer_PGM + i);
    }
  }
  
  // Search for an unused timer.
  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) 
  {
    if (tone_pins[i] == 255) 
    {
      tone_pins[i] = _pin;
      _timer = pgm_read_byte(tone_pin_to_timer_PGM + i);
      break;
    }
  }
  
  if (_timer != -1)
  {
    // 8 bit timer
    TCCR2A = 0;
    TCCR2B = 0;
    bitWrite(TCCR2A, WGM21, 1);
    bitWrite(TCCR2B, CS20, 1);
    timer2_pin_port = portOutputRegister(digitalPinToPort(_pin));
    timer2_pin_mask = digitalPinToBitMask(_pin);
  }

  return _timer;
}

// Frequency (in hertz) and duration (in milliseconds).
void myTone(uint8_t _pin, unsigned int frequency, unsigned long duration)
{
  uint8_t prescalarbits = 0b001;
  long toggle_count = 0;
  uint32_t ocr = 0;
  int8_t _timer;

  _timer = ourToneBegin(_pin);

  if (_timer >= 0)
  {
    // Set the pinMode as OUTPUT
    DDRH = 0xFF;
    DDRE = 0xFF;
    DDRG = 0xFF;
    DDRB = 0xFF;
    
    // if we are using an 8 bit timer, scan through prescalars to find the best fit
    if (_timer == 0 || _timer == 2)
    {
      ocr = F_CPU / frequency / 2 - 1;
      prescalarbits = 0b001;  // ck/1: same for both timers
      if (ocr > 255)
      {
        ocr = F_CPU / frequency / 2 / 8 - 1;
        prescalarbits = 0b010;  // ck/8: same for both timers

        if (_timer == 2 && ocr > 255)
        {
          ocr = F_CPU / frequency / 2 / 32 - 1;
          prescalarbits = 0b011;
        }

        if (ocr > 255)
        {
          ocr = F_CPU / frequency / 2 / 64 - 1;
          prescalarbits = _timer == 0 ? 0b011 : 0b100;

          if (_timer == 2 && ocr > 255)
          {
            ocr = F_CPU / frequency / 2 / 128 - 1;
            prescalarbits = 0b101;
          }

          if (ocr > 255)
          {
            ocr = F_CPU / frequency / 2 / 256 - 1;
            prescalarbits = _timer == 0 ? 0b100 : 0b110;
            if (ocr > 255)
            {
              // can't do any better than /1024
              ocr = F_CPU / frequency / 2 / 1024 - 1;
              prescalarbits = _timer == 0 ? 0b101 : 0b111;
            }
          }
        }
      }
      TCCR2B = (TCCR2B & 0b11111000) | prescalarbits;
    }
    else
    {
      // two choices for the 16 bit timers: ck/1 or ck/64
      ocr = F_CPU / frequency / 2 - 1;

      prescalarbits = 0b001;
      if (ocr > 0xffff)
      {
        ocr = F_CPU / frequency / 2 / 64 - 1;
        prescalarbits = 0b011;
      }
    }
    
    // Calculate the toggle count
    if (duration > 0)
    {
      toggle_count = 2 * frequency * duration / 1000;
    }
    else
    {
      toggle_count = -1;
    }

    // Set the OCR for the given timer, set the toggle count,
    // then turn on the interrupts
    OCR2A = ocr;
    timer2_toggle_count = toggle_count;
    bitWrite(TIMSK2, OCIE2A, 1);
  }
}

// This function only works properly for timer 2
void disableTimer(uint8_t _timer)
{
  TIMSK2 &= ~(1 << OCIE2A); // disable interrupt
  TCCR2A = (1 << WGM20);
  TCCR2B = (TCCR2B & 0b11111000) | (1 << CS22);
  OCR2A = 0;
}

void noTone(uint8_t _pin)
{
  int8_t _timer = -1;
  
  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) 
  {
    if (tone_pins[i] == _pin) 
    {
      _timer = pgm_read_byte(tone_pin_to_timer_PGM + i);
      tone_pins[i] = 255;
      break;
    }
  }
  
  disableTimer(_timer);

  digitalWrite(_pin, 0);
}

ISR(TIMER2_COMPA_vect)
{
  if (timer2_toggle_count != 0)
  {
    // Toggle pin
    *timer2_pin_port ^= timer2_pin_mask;

    if (timer2_toggle_count > 0)
      timer2_toggle_count--;
  }
  else
  {
    // We need to call noTone() so that the tone_pins[] entry is reset, so the
    // timer gets initialized next time we call tone().
    noTone(tone_pins[0]);
  }
}

void metronome(int tempo)
{
  // Metronome
  int metroFrequency = NOTE_C4;
  int metroDuration = 20; // 25 milliseconds

  while (true) // Unlikely this will stay; it's just to get it beeping constantly.
  {
    myTone(2, metroFrequency, metroDuration); // PH5 / Pin 8
    delay(tempo);
  }
}

void pitchPlayer()
{
  // Beep note, duration, and tempo
  frequency = NOTE_A3;
  duration = 1000; // 3 seconds
  
  myTone(2, frequency, duration); // PH5 / Pin 8
}

void PrintStats(int _duration, int _tempo, const char* _frequency)
{
  char lineOne[17];
  char lineTwo[17];
  char formatStringOne[18] = "Length:%d   Next>";
  char formatStringTwo[15] = "BPM:%d Note:%s";
  bool outputSpace = false;
  unsigned char i;
  // write lines
  sprintf(lineOne, formatStringOne, _duration);
  sprintf(lineTwo, formatStringTwo, _tempo, _frequency);
  LCD_write_instruction(LCD_4bit_cursorSET | LineOneStart);
  _delay_us(80);
  // display first line
  for (i = 0; i < 16; i++)
  {
    if (lineOne[i] == 0)
      outputSpace = true;
    if (outputSpace)
      LCD_write_char(' ');
    else
      LCD_write_char(lineOne[i]);
  }
  outputSpace = false;
  // display second line
  LCD_write_instruction(LCD_4bit_cursorSET | LineTwoStart);
  _delay_us(80);
  for (i = 0; i < 16; i++)
  {
    if (lineTwo[i] == 0)
      outputSpace = true;
    if (outputSpace)
      LCD_write_char(' ');
    else
      LCD_write_char(lineTwo[i]);
  }
}

void PrintStats2(const char *_frequency)
{
  char lineOne[17];
  char lineTwo[17];
  char formatStringOne[15] = "Heard Pitch:%s";
  char formatStringTwo[7] = "< Back";
  bool outputSpace = false;
  unsigned char i;
  // write lines
  sprintf(lineOne, formatStringOne, _frequency);
  sprintf(lineTwo, formatStringTwo);
  LCD_write_instruction(LCD_4bit_cursorSET | LineOneStart);
  _delay_us(80);
  // display first line
  for (i = 0; i < 16; i++)
  {
    if (lineOne[i] == 0)
      outputSpace = true;
    if (outputSpace)
      LCD_write_char(' ');
    else
      LCD_write_char(lineOne[i]);
  }
  outputSpace = false;
  // display second line
  LCD_write_instruction(LCD_4bit_cursorSET | LineTwoStart);
  _delay_us(80);
  for (i = 0; i < 16; i++)
  {
    if (lineTwo[i] == 0)
      outputSpace = true;
    if (outputSpace)
      LCD_write_char(' ');
    else
      LCD_write_char(lineTwo[i]);
  }
}

ISR(USART0_RX_vect)
{
  char temp;
  temp = UDR0; // GET RECIEVED DATA
  UDR0 = temp; // ECHO BACK TO TERMINAL

  switch (mode)
  {
    case Duration:
    {
      metronomeOn = false;
      if (i <= 4 && temp != '\n')
      {
        input[i] = temp;
        i++;
      }
      else if (temp == '\n') // newline is entered
      {
        input[i] = '\0';
        if(atoi((const char*)input) != 0)
          duration = atoi((const char*)input);
        mode = Nothing;
        i = 0;
        USART_transmit_str(menu1);
        myTone(2, frequency, duration);
        delay(duration);
      }
      else if (i > 4)
      {
        duration = atoi((const char*)input);
        mode = Nothing;
        i = 0;
        USART_transmit_str("\nToo many digits! Returning to menu...\n");
        USART_transmit_str(menu1);
      }
      if(duration > 5000)
        duration = 5000;
      PrintStats(duration/1000, tempo, roundToNearest(frequency));
    }
    break;

    case Metronome:
    {
      if (i <= 3 && temp != '\n')
      {
        input[i] = temp;
        i++;
      }
      else if (temp == '\n') // newline is entered
      {
        input[i] = '\0';
        if(atoi((const char*)input) != 0)
          tempo = atoi((const char*)input);
        mode = Nothing;
        i = 0;
        USART_transmit_str(menu1);
        metronomeOn = true;
      }
      else if (i > 3)
      {
        tempo = atoi((const char*)input);
        mode = Nothing;
        i = 0;
        USART_transmit_str("\nToo many digits! Returning to menu...\n");
        USART_transmit_str(menu1);
      }
      if(tempo > 500)
        tempo = 500;
      else if (tempo < 40)
        tempo = 40;
      PrintStats(duration/1000, tempo, roundToNearest(frequency));
    }
    break;

    case Frequency:
    {
      metronomeOn = false;
      if (i <= 4 && temp != '\n')
      {
        input[i] = temp;
        i++;
      }
      else if (temp == '\n') // newline is entered
      {
        input[i] = '\0';
        frequency = roundToNearestFreq(atoi((const char*)input));
        mode = Nothing;
        i = 0;
        USART_transmit_str(menu1);
        myTone(2, frequency, duration);
        delay(2000);
      }
      else if (i > 4)
      {
        frequency = roundToNearestFreq(atoi((const char*)input));
        mode = Nothing;
        i = 0;
        USART_transmit_str("\nToo many digits! Returning to menu...\n");
        USART_transmit_str(menu1);
      }
      PrintStats(duration/1000, tempo, roundToNearest(frequency));
    }
    break;

    default:
    {
      metronomeOn = false;
      // Set flag to Duration
      if (temp == '1')
      {
        mode = Duration;
        FFTmode = false;
        USART_transmit('\n'); 
        USART_transmit_str(menu2);
      }

      // Set flag to Metronome
      if (temp == '2')
      {
        mode = Metronome;
        FFTmode = false;
        USART_transmit('\n'); 
        USART_transmit_str(menu3);
      }

      // Set flag to Frequency
      if (temp == '3')
      {
        mode = Frequency;
        FFTmode = false;
        USART_transmit('\n'); 
        USART_transmit_str(menu4);
      }

      if (temp == '>')
      {
        LCD_Clear();
        FFTmode = true;
      }

      if (temp == '<')
      {
        FFTmode = false;
        LCD_Clear();
        PrintStats(duration/1000, tempo, roundToNearest(frequency));
      }
    }
    break;
  }
}

// NOTE: PARALLEL ARRAYS //
float freqArray[96] = {16.35, 17.32, 18.35, 19.45, 20.6, 21.83, 23.12, 24.5, 25.96, 27.5, 29.14, 30.87,
                      32.7, 34.65, 36.71, 38.89, 41.2, 43.65, 46.25, 49, 51.91, 55, 58.27, 61.74,
                      65.41, 69.3, 73.42, 77.78, 82.41, 87.31, 92.5, 98, 103.83, 110, 116.54, 123.47,
                      130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185, 196, 207.65, 220, 233.08, 246.94,
                      261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392, 415.3, 440, 466.16, 493.88,
                      523.25, 554.37, 587.33, 622.25, 659.25, 698.46, 739.99, 783.99, 830.61, 880, 932.33, 987.77,
                      1046.5, 1108.73, 1174.66, 1244.51, 1318.51, 1396.91, 1479.98, 1567.98, 1661.22, 1760, 1864.66, 1975.53,
                      2093, 2217.46, 2349.32, 2489, 2637, 2793.83, 2959.96, 3135.96, 3322.44, 3520, 3729.31, 3951};
char noteArray[96][5] = {"C0", "C#0", "D0", "D#0", "E0", "F0", "F#0", "G0", "G#0", "A0", "A#0", "B0",
                        "C1", "C#1", "D1", "D#1", "E1", "F1", "F#1", "G1", "G#1", "A1", "A#1", "B1",
                        "C2", "C#2", "D2", "D#2", "E2", "F2", "F#2", "G2", "G#2", "A2", "A#2", "B2",
                        "C3", "C#3", "D3", "D#3", "E3", "F3", "F#3", "G3", "G#3", "A3", "A#3", "B3",
                        "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4", "A4", "A#4", "B4",
                        "C5", "C#5", "D5", "D#5", "E5", "F5", "F#5", "G5", "G#5", "A5", "A#5", "B5",
                        "C6", "C#6", "D6", "D#6", "E6", "F6", "F#6", "G6", "G#6", "A6", "A#6", "B6",
                        "C7", "C#7", "D7", "D#7", "E7", "F7", "F#7", "G7", "G#7", "A7", "A#7", "B7"};

ISR(ADC_vect)
{
    // Store Value //
    vReal[ADC_IR_Count] = (float)ADCW;
    vImag[ADC_IR_Count] = 0;

    if (ADC_IR_Count != samples)
    {
        ADC_IR_Count++;
    }
    else
    {
      if(FFTmode)
      {
        // PERFORM FFT //	
        FFT.compute(FFTDirection::Forward);
        FFT.complexToMagnitude();
        peakFreq = FFT.majorPeak();

        // OUTPUT RESULT //
        PrintStats2(roundToNearest(peakFreq));
      }
      // RESET COUNT //
      ADC_IR_Count = 0;
    }

    // START ANOTHER CONVERSION //
    ADCSRA |= 0x40;
}

void initADC()
{
    // SET PORT F TO INPUT //
    DDRF = 0x00;

    // INITIALIZE ADC //
    // ENABLE ADC IN SINGLE CONVERSION MODE //
    // ENABLE INTERRUPTS TO USE CONVERSION COMPLETE ISR //
    // Prescale -> F_CPU/200kHz -> round -> 128 //
    // SAMPLE RATE ~125KHz/13 -> ~9600Hz //
    ADCSRA |= 0x8F;

    // USE AVCC WITH EXTERNAL CAPACITOR FOR REFERENCE //
    // LEFT ADJUSTED RESULTS //
    // USE CHANNEL AD0 - Pin F0 //
    ADMUX |= 0x40;

    sei();
}

// ************** USART ***************** //
void initializeUSART()
{
  // ENABLE RX AND TX - ENABLE RECEIVE COMPLETE INTERRUPT //
  UCSR0B |= 0x98;

  // 8-BIT FRAMES - ASYNCHRONOUS //
  UCSR0C |= 0x06;

  // SET BAUD RATE //
  UBRR0L = BAUD_PRESCALE;
  UBRR0H = (BAUD_PRESCALE >> 8);

  sei();
  _delay_ms(300);
}

// Function to transmit a character via UART
void USART_transmit(unsigned char data) 
{
  // Wait for empty transmit buffer
  while (!(UCSR0A & (1 << UDRE0)));
  // Put data into buffer, sends the data
  UDR0 = data;
}

// Function to transmit a string via UART
void USART_transmit_str(const char *str) 
{
  while (*str) 
  {
    USART_transmit(*str++);
  }
}

// ************** HELPERS ***************** //
char* nearestNote(float freq)
{
  uint8_t i, noteIdx;
  float distance = 10000;

  // Find Closest Note Frequency //
  for(i = 0; i < 96; i++)
  {
    if(abs(freq - freqArray[i]) < distance)
    {
      noteIdx = i;
      distance = abs(freq - freqArray[i]);
    }
  }

  // Return Note //
  return noteArray[noteIdx];
}

int main()
{
  DDRA = 0xF0;
  int metroFrequency = NOTE_C4;
  int metroDuration = 5; // 5 milliseconds

  initBuzzer();
  LCD_init();
  LCD_Clear();
  LCD_init();
  PrintStats(duration/1000, tempo, roundToNearest(frequency)); // Prints to LCD
  initializeUSART();
  USART_transmit_str(menu1);

  // Enable ADC //
  initADC();
  ADCSRA |= 0x40;

  while(1)
  {
    // In Metronome Mode
    while (metronomeOn)
    {
      myTone(2, metroFrequency, metroDuration); // PWM Pin 2 (can be set to any pin)
      delay(60000/tempo);
    }
  }

  return 1;
}