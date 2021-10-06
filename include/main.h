#ifndef main_h
#define main_h

#include "Arduino.h"


#include <Arduino.h>
#include <avr/wdt.h>
#include "arduinoFFT.h"
#include "flag_ship.h"


#define DEBUG_MODE      0
// motor up-down
#define BUTTON_UP_PIN     4U    
#define BUTTON_DOWN_PIN   5U
#define MOTOR_PHASE1_PIN  7U
#define MOTOR_PHASE2_PIN  8U


#define AU_PIN      A0
#define GAIN_PIN    2U
#define LED1_PIN    11U
#define LED2_PIN    12U



#define ADC0 0
#define ADC5 5

// Define the ADC channels used. ADLAR will be zero.
#define ADCH0 ((1 << REFS0) | ADC0)
#define ADCH5 ((1 << REFS0) | ADC5)


// variables
const int MAX_RESULTS = 64;

volatile int results [MAX_RESULTS];
volatile int resultNumber;

volatile int adcCurrent;
volatile int adcLastCurrent;



int ledState = LOW;
uint8_t state_det = 0;
uint8_t state_flag = 0, last_state_flag=0;
volatile uint32_t state_time = 0;
const uint32_t state_time_hold = 1000UL;
volatile uint32_t timeMils=0;
volatile uint32_t buttonMillis = 0;


// function 
void adc_deinit();
void adc_init();

void preEmphasis(int* val,int n,float s);

#if DEBUG_MODE
    void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType);
#endif

#endif