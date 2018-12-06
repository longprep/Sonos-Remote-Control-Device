#include <SendOnlySoftwareSerial.h>

#include <avr/sleep.h>
#include <avr/power.h>    // Power management
#include <avr/wdt.h>


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// ATMEL ATTINY 25/45/85 / ARDUINO
//
//                 +-\/-+
// Ain0 (D5) PB5  1|    |8  Vcc
// Ain3 (D3) PB3  2|    |7  PB2 (D2) Ain1
// Ain2 (D4) PB4  3|    |6  PB1 (D1) pwm1
//           GND  4|    |5  PB0 (D0) pwm0
//                 +----+

// ****** PINS USAGE ************
// unused                 // pin #1
#define PLAY_PAUSE 3      // pin #2
#define POWER_ESP  4      // pin #3
// GND                    // pin #4
// Serial TX to ESP       // pin #5
#define ESP_FLAG   1      // pin #6
#define VOLUME_UP  2      // pin #7

// double click options, +2  the value of single click button
#define NEXT_SONG 13
#define VOLUME_DOWN 12

#define NOTHING    0
#define ON true
#define OFF false

#define BUTTON_TIMEOUT 500
#define ESP_WAKE_TIME 20000


SendOnlySoftwareSerial esp(0); // TX

int button;
bool esp_on = false;
bool goToSleep = true;
unsigned long powerOnTime, delta;

void system_sleep() 
{
  cbi(ADCSRA,ADEN);                   

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
  sleep_enable();

  sleep_mode();                        

  sleep_disable();                    
  sbi(ADCSRA,ADEN);                    
}

ISR(PCINT0_vect) 
{
    // This is called only when the PIR interrupt occurs
}

void sendCommand(int c)
{
  int to=0;
  esp.print(c);
  esp.flush();
  while ((to++< 5000) && (digitalRead(ESP_FLAG) == LOW))
  {
    delay(1);
  }
}

void switchEsp(bool desiredState)
{
  if (desiredState == ON)
  {
    powerOnTime = millis();    
    if (esp_on)
      return;
    digitalWrite(POWER_ESP, HIGH);
    delay(2000);
    esp_on = true;
    while (digitalRead(ESP_FLAG) == LOW)
    {
      delay(1);
    }      
    powerOnTime = millis();  
  }
  else // OFF 
  { 
    if (!esp_on)
      return;
    digitalWrite(POWER_ESP, LOW);
    esp_on = false;
  }
}

void setup() 
{
  pinMode(POWER_ESP, OUTPUT);
  pinMode(ESP_FLAG, INPUT);
  pinMode(PLAY_PAUSE, INPUT_PULLUP);
  pinMode(VOLUME_UP, INPUT_PULLUP);

  PCMSK |= 0b00001100; //_BV(PCINT3);                   // Use PB2 & PB3 as interrupt pin for PIRs signals
  GIFR  |= _BV(PCIF);                    // clear any outstanding interrupts    
  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts     

  esp.begin(9600);      
} 


void loop() 
{
  if (goToSleep)
  {
    system_sleep();
  }
  else
  { // Check if we need to switch off the ESP or let it run
    delta = 0;
    while ((delta < ESP_WAKE_TIME) && (digitalRead(PLAY_PAUSE)==HIGH) && (digitalRead(VOLUME_UP)==HIGH))
    {
      delta = millis() - powerOnTime;
      delay(1);
    }
    if (delta >= ESP_WAKE_TIME || delta==0)
    {
      switchEsp(OFF);
      goToSleep = true;
      return;
    }
  }
  
  goToSleep = false;
  button = NOTHING;
  if (digitalRead(PLAY_PAUSE)==LOW)
    button = PLAY_PAUSE;
  if (digitalRead(VOLUME_UP) == LOW)
    button = VOLUME_UP;

  int cntr=0;
  while ((digitalRead(button)==LOW) && (cntr++ < BUTTON_TIMEOUT))
    delay(1);
  cntr = 0;
  while ((digitalRead(button)==HIGH) && (cntr++ < BUTTON_TIMEOUT))
    delay(1);

  if (cntr < BUTTON_TIMEOUT) // 
  {
    cntr = 0;
    while ((digitalRead(button) == LOW) && (cntr++ < BUTTON_TIMEOUT))
      delay(1);
    if (cntr < BUTTON_TIMEOUT)      
      button+=2; // increase the button value for double-click
    else
    { // The button is still pressed, not normal. Go back to sleep and wait for another event
      goToSleep = true;
      switchEsp(OFF);     
      return; 
    }
    
  }

  switchEsp(ON);

  if (button == PLAY_PAUSE)
    sendCommand(1);
  if (button == NEXT_SONG)
    sendCommand(2);
  if (button == VOLUME_UP)
    sendCommand(3);
  if (button == VOLUME_DOWN)
    sendCommand(4);
    
}
