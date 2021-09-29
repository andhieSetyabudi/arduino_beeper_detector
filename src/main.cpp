#include <Arduino.h>
#include "arduinoFFT.h"
#include "flag_ship.h"


#define AU_PIN      A0
#define GAIN_PIN    4U
#define LED1_PIN    2U
#define LED2_PIN    3U

const uint8_t pin_map[] = {GAIN_PIN, LED1_PIN, LED2_PIN};

void adc_deinit();
void adc_init();

const int MAX_RESULTS = 64;

volatile int results [MAX_RESULTS];
volatile int resultNumber;


arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
void preEmphasis(int* val,int n,float s)
{
  int* tmp = (int*)malloc(n * sizeof(int));
  for(int a = 0; a<n; a++)
  {
    if(a ==0)
      tmp[a] = val[a];
    else
      tmp[a] =(int) ( (float) val[a] - (s*(float)val[a-1]));
  }
  memcpy(val,tmp,n);
  free(tmp);
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType);

// ADC complete ISR
ISR (ADC_vect)
{

  
    results[resultNumber++] = ADC;
    if(resultNumber == MAX_RESULTS)
    {
      ADCSRA = 0;  // turn off ADC
    }
} 
EMPTY_INTERRUPT (TIMER1_COMPB_vect);


flag_ship flag_state;

void setup() {
  // Serial.begin(500000);
  for(uint8_t p = 0; p<sizeof(pin_map); p++)
  {
    pinMode(pin_map[p], OUTPUT);
    digitalWrite(pin_map[p], LOW);
  }
  digitalWrite(GAIN_PIN,HIGH);
  pinMode(LED_BUILTIN, OUTPUT);

  flag_state.SetStateAndTime(HIGH);
  adc_init();
}

int ledState = LOW;
uint8_t state_det = 0;
uint8_t state_flag = 0, last_state_flag=0;
volatile uint32_t state_time = 0;
const uint32_t state_time_hold = 1000UL;
volatile uint32_t timeMils=0;
void loop() {
   
  if( resultNumber>= MAX_RESULTS )
  {
    double* vReal = (double*)malloc(MAX_RESULTS * sizeof(double));
    double* vImag = (double*)malloc(MAX_RESULTS * sizeof(double));
//     double vReal[MAX_RESULTS];
// double vImag[MAX_RESULTS];
    preEmphasis(results,MAX_RESULTS,0.97);
    for (int i = 0; i < MAX_RESULTS; i++)
    {
      vReal[i] = int8_t(results [i]); 
      vImag[i] = 0.0; 
      // Serial.println (results [i]);
    }
    resultNumber = 0;
    /* Print the results of the simulated sampling according to time */
    // Serial.println("Data:");
    // PrintVector(vReal, MAX_RESULTS, 0);
    FFT.Windowing(vReal, MAX_RESULTS, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    // Serial.println("Weighed data:");
    // PrintVector(vReal, MAX_RESULTS, 0);
    FFT.Compute(vReal, vImag, MAX_RESULTS, FFT_FORWARD); /* Compute FFT */
    // Serial.println("Computed Real values:");
    // PrintVector(vReal, MAX_RESULTS, 1);
    // Serial.println("Computed Imaginary values:");
    // PrintVector(vImag, MAX_RESULTS, 1);
    FFT.ComplexToMagnitude(vReal, vImag, MAX_RESULTS); /* Compute magnitudes */
    // Serial.println("Computed magnitudes:");
    // PrintVector(vReal, (MAX_RESULTS >> 1), 2);
    double f_, v_ ;
    FFT.MajorPeak(vReal, MAX_RESULTS, 50e+3, &f_, &v_);
    free(vImag);  free(vReal);
    
    if( (3000.f <= f_) && (f_ <= 3150.f) && (50.f <= v_) &&  (v_ <= 1900.f))
      state_det = HIGH;
    else
      state_det = LOW;
    state_flag = flag_state.updateState(state_det);
    if( last_state_flag != state_flag && (state_flag > 0) )
    {
      state_time = millis();
      last_state_flag = state_flag;
    }
      
    if( millis() - state_time >= state_time_hold )
    {
      last_state_flag = 0;
    }
    switch(last_state_flag)
    {
      case 1:
        digitalWrite(LED1_PIN, HIGH);
        break;
      case 2:
        digitalWrite(LED2_PIN, HIGH);
        break;
      default :
        digitalWrite(LED2_PIN, LOW);
        digitalWrite(LED1_PIN, LOW);
        break;
    };
    adc_deinit();
    // Serial.println("======================");
  }
  
  if(millis() - timeMils >= 1000U)
  {
    if (ledState == LOW) {
    ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(LED_BUILTIN, ledState);
    timeMils = millis();
  };
}

void adc_init()
{
   // reset Timer 1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1B = bit (CS11) | bit (WGM12);  // CTC, prescaler of 8
  TIMSK1 = bit (OCIE1B);
  OCR1A = 39;    
  OCR1B = 39;   // 20 uS - sampling frequency 50 kHz

  ADCSRA =  bit (ADEN) | bit (ADIE) | bit (ADIF);   // turn ADC on, want interrupt on completion
  ADCSRA |= bit (ADPS2);  // Prescaler of 16
//  ADCSRA |= (1 << ADPS1) | (1 << ADPS0);    // 8 prescaler for 153.8 KHz
  ADMUX = bit (REFS0) | (0 & 7);  // adc 0
  ADCSRB = bit (ADTS0) | bit (ADTS2);  // Timer/Counter1 Compare Match B
  ADCSRA |= bit (ADATE);   // turn on automatic triggering
}

void adc_deinit()
{
   ADCSRA =  bit (ADEN) | bit (ADIE) | bit (ADIF)| bit (ADPS2) | bit (ADATE);   
   // turn ADC back on
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case 0:
        abscissa = (i * 1.0);
	break;
      case 1:
        abscissa = ((i * 1.0) / 50e+3);
	break;
      case 2:
        abscissa = ((i * 1.0 * 50e+3) / MAX_RESULTS);
	break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==2)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
