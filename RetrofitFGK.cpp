
////////////////////////////////////FGK MAF I2C RTOS 2//////////////////////////////////////////////
//Pin assign RPM detection = pin 2 & 3, injection = pin 4, pump = pin 5, MAP = pin A0, TPS = pin A5
//Timer prescaler based 256
///////////////////////////////////////////INITIALIZE///////////////////////////////////////////////
//Header////////////////////////////////////////////////////////////////////////////////////////////
#include <NilRTOS.h>
#include <NilSerial.h>
#define Serial NilSerial
#include "Wire.h";
//i2C///////////////////////////////////////////////////////////////////////////////////////////////
volatile int ATtinyAddress=0x26;
volatile unsigned long marktime;
//inject////////////////////////////////////////////////////////////////////////////////////////////
const byte pullup = 2;
const byte injectPIN = 4;
volatile unsigned int injectDelayTime;
volatile unsigned long injectOnTime = 30;
volatile boolean injectOn;
volatile int alternate = 0;
volatile int lastlow = 1;
//RPM///////////////////////////////////////////////////////////////////////////////////////////////
volatile double rps;
volatile unsigned long periodz;
volatile int rpm;
//Airflow///////////////////////////////////////////////////////////////////////////////////////////
volatile double AF;
volatile int RAWMAP;
volatile double MAP;
volatile double K = 0.1;
volatile double InjSpec = 14.3625; // (4.5/1.2)*3.83
//TPS///////////////////////////////////////////////////////////////////////////////////////////////
int TPSpin = 5;
volatile double minTPS;
volatile double maxTPS;
volatile double RAWinTPS;
volatile double inTPS;
volatile double inOldTPS = 0;
volatile double outTPS = 0;
//Oil pump//////////////////////////////////////////////////////////////////////////////////////////
const byte oilPUMP = 5;

/////////////////////////////////////////////NILRTOS////////////////////////////////////////////////
//------------------------------------------------------------------------------
// Declare a stack with 64 bytes beyond context switch and interrupt needs.
NIL_WORKING_AREA(waThread1, 64);

// Declare thread function for thread 1.
NIL_THREAD(Thread1, arg) {
sInject();
  while (TRUE) 
  {
    nilThdSleep(100);
    fInject();
  }
}
//------------------------------------------------------------------------------
// Declare a stack with 64 bytes beyond context switch and interrupt needs.
NIL_WORKING_AREA(waThread2, 64);

// Declare thread function for thread 2.
NIL_THREAD(Thread2, arg) {

  while (TRUE) 
  {
    nilThdSleep(100);
    rMAP();
  }
}
//------------------------------------------------------------------------------
// Declare a stack with 64 bytes beyond context switch and interrupt needs.
NIL_WORKING_AREA(waThread3, 64);

// Declare thread function for thread 3.
NIL_THREAD(Thread3, arg) {

  while (TRUE) 
  {
    nilThdSleep(100);
    rAF(); 
  }
}
//------------------------------------------------------------------------------
NIL_THREADS_TABLE_BEGIN()
NIL_THREADS_TABLE_ENTRY(NULL, Thread1, NULL, waThread1, sizeof(waThread1))
NIL_THREADS_TABLE_ENTRY(NULL, Thread2, NULL, waThread2, sizeof(waThread2))
NIL_THREADS_TABLE_ENTRY(NULL, Thread3, NULL, waThread3, sizeof(waThread3))
NIL_THREADS_TABLE_END()
//------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  pinMode(oilPUMP, OUTPUT);
  digitalWrite(oilPUMP, HIGH);//Pump
  nilSysBegin();
  
}
//------------------------------------------------------------------------------
void loop() 
{
  nilPrintStackSizes(&Serial);
  nilPrintUnusedStack(&Serial);
  Serial.print(" RPM : ");
  Serial.print(rpm);
  Serial.print(" timeInject : ");
  Serial.println(AF);
  rRPM();
}

///////////////////////////////////////////SUBROUTINES/////////////////////////////////////////////
//i2C//////////////////////////////////////////////////////////////////////////////////////////////
void rRPM()
{
  if (millis()>marktime)
  {
    byte hb;
    byte lb;
    Wire.requestFrom(ATtinyAddress,2); 
    if (Wire.available()) 
    {
      lb=Wire.read();
      hb=Wire.read();
    }
    rpm=word(hb,lb);
    // if (rpm <= 1500)
    // {
    //   alternate = 0;
    //   lastlow = 1;
    // }
    marktime+=100;
  }
}
//inject//////////////////////////////////////////////////////////////////////////////////////////
void sInject()
{
  TCCR1A = 0;  // normal mode
  TCCR1B = 0;  // stop timer
  TIMSK1 = 0;   // cancel timer interrupt

  pinMode (injectPIN, OUTPUT);
  digitalWrite (pullup, HIGH);  // pull-up
  attachInterrupt (0, reinject, RISING);
}
ISR (TIMER1_COMPA_vect)
{
  // if currently on, turn off
  if (injectOn)
    {
    if(alternate == 1)
      {
        lastlow = 1;
        alternate = 0;
       }  
    else
      {
        lastlow = 0;
        alternate = 1;
      }
    digitalWrite (injectPIN, LOW);   // inject off
    TCCR1B = 0;                      // stop timer
    TIMSK1 = 0;                      // cancel timer interrupt
    EIFR = bit (INTF0);              // delete any pending interrupt on D2  
    attachInterrupt (0, reinject, RISING);    // re-instate interrupts for firing time
    }
  else
    // hold-off time must be up
    {
      if(alternate == 0 && lastlow == 1)
      {
      digitalWrite (injectPIN, HIGH);    // inject on
      TCCR1B = 0;                        // stop timer
      TCCR1B = bit(WGM12) | bit(CS12);   // CTC, scale to clock / 256
      OCR1A = injectOnTime;               // time before timer fires
      }
    }
  injectOn = !injectOn;  // toggle
}  
void reinject ()
{
  injectOn = false;                  // make sure flag off just in case
  TCCR1A = 0;  // normal mode
  TCCR1B = bit(WGM12) | bit(CS12);  // CTC, scale to clock / 8
  OCR1A = injectDelayTime;           // time before timer fires
  TIMSK1 = bit (OCIE1A);            // interrupt on Compare A Match
  detachInterrupt (0);   // cancel any existing falling interrupt (interrupt 0)
}
void fInject()
{   
  if (false)  // if we need to change the time, insert condition here ...
    {
    noInterrupts ();  // atomic change of the time amount
    interrupts ();
    }
} 
//Airflow///////////////////////////////////////////////////////////////////////////////////////
void rAF()
{
    rps = (rpm/60);//change RPM to frequency
    periodz = (1000/rps);//ms
    injectDelayTime = periodz*31.25;//31.25 = ((1x10-3/16000x10-9)/2)
    if (rpm <= 2500)
    {
      injectOnTime = periodz*6.25;
    } 
    else
    {
      AF=(62.5*MAP*rpm*K)/(InjSpec*157476.7782);
      injectOnTime = AF*62.5;
    }
}
//MAP//////////////////////////////////////////////////////////////////////////////////////////
void rMAP()
{
    RAWMAP = analogRead(A0);
    MAP = (1/0.045)*(((RAWMAP*5)/1023)+0.425);
    
}
//TPS//////////////////////////////////////////////////////////////////////////////////////////////
void rTPS()
{
  RAWinTPS = analogRead(TPSpin);
  inTPS = map(RAWinTPS, minTPS, maxTPS, 0, 90);
  outTPS = (0.01*inTPS) - (0.01*inOldTPS) + (0.8*outTPS);
  inTPS = inOldTPS;
}
