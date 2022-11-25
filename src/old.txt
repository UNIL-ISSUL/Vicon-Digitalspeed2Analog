#define CIRCULAR_BUFFER_INT_SAFE
#include "CircularBuffer.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>


//speed signal pin
const byte speedPin = 0;
volatile unsigned long counter = 0;

//LED builtin state
volatile boolean state;

//Analog value
unsigned int level = 0;
//compute max_speed in pulse per ms
//motor speed is 1500 tr/min at 50Hz, the VFD drives the motor up to 100Hz -> motor speed is 3000 tr/min
//there are 26 tooth on motor pulley both interrupt on rising and falling edge
unsigned int max_pulse_s = round(1.0 * 3000 / 60 * 26 * 2);
#define BUFFER_LENGTH 2
CircularBuffer<unsigned long, BUFFER_LENGTH> pulses;

//Timer variable
unsigned int timer_us;
unsigned int computation_freq = 1;   // Hz
unsigned int refresh_us = round(1.0e6 / ( computation_freq * BUFFER_LENGTH)) ;   // 1e3: refresh value at 1000Hz

//define interupt function to count for pulses
void count();

//Adafruit 12bits DAC
Adafruit_MCP4725 dac;

void setup() {
  //start serial com
  Serial.begin(9600);
  //define pin
  pinMode(speedPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(speedPin), count, CHANGE);

  //init led state so the LED is HIGH when sensor dedect tooth
  state = !digitalRead(speedPin);

  //init dac
  if(dac.begin(0x62));
  else Serial.println("dac initialization failed");

  //init timer
  timer_us = micros();
}

void loop() {
  //write current LED state
  digitalWrite(LED_BUILTIN,state);

  //write analog value every period
  unsigned int elapsed_us = micros() - timer_us;
  if(elapsed_us >= refresh_us-2) {
    //add current counter at the end of the pulses buffer
    pulses.push(counter);
    //Compute count difference in buffer
    int count = pulses[BUFFER_LENGTH-1]-pulses[0];
    //map speed on 12 bits
    int val = round(1.0 * count * computation_freq * 4095 / 2600);    //4095 (12bits -1), 2600 is the number of pulses per second at max speed
    //send value to dac
    if (dac.setVoltage(val,false));
    else Serial.println("dac write value failed");
    //debug read values
    if(count>0) Serial.println("dac value : "+String(val)+"\tpulses : "+String(count)+"\teleapsed_us : "+String(elapsed_us));
    //update timer
    timer_us = micros();
  }
}

//ISR function call by interupt
void count() {
  //increment counter
  counter += 1;
  //swith on LED
  state = !state;
}