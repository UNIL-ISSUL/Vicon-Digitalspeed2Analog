#define CIRCULAR_BUFFER_INT_SAFE
#include "CircularBuffer.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>


//digital signal pin
const byte digitalPin = 0;
volatile unsigned int period_us = 0;
volatile unsigned int previous_micros = 0;
volatile boolean data_available = false;

//LED builtin state
volatile boolean state;

//Analog value
unsigned int level = 0;
//compute max_speed in pulse per ms
//motor speed is 1500 tr/min at 50Hz, the VFD drives the motor up to 100Hz -> motor speed is 3000 tr/min
//there are tooth on motor pulley both interrupt on rising and falling edge
unsigned int max_frequency_uHz = 1500 *1.0e6 / 60 *26;


//define interupt function to count for pulses
void on_interupt();

//Adafruit 12bits DAC
Adafruit_MCP4725 dac;

void setup() {
  //start serial com
  Serial.begin(9600);
  //define pin
  pinMode(digitalPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(digitalPin), on_interupt, RISING);

  //init led state so the LED is HIGH when sensor dedect tooth
  state = !digitalRead(digitalPin);
  //get actual micros
  previous_micros = micros();

  //init dac
  if(dac.begin(0x62));
  else Serial.println("dac initialization failed");
}

void loop() {
  //change led state if data are available
  if(data_available) {
    //write current LED state
    digitalWrite(LED_BUILTIN,state);
    //write period value to DAC
    int val = round(((1.0 / period_us) * 4095 )/ max_frequency_uHz);
    if (dac.setVoltage(val,false));
    else Serial.println("dac write value failed");
    data_available = false;
  }
}

//ISR function call by interupt
void on_interupt() {
  //increment counter
  unsigned int current_micros = micros();
  period_us = current_micros - previous_micros;
  previous_micros = current_micros;
  data_available = true;
  //swith on LED
  state = !state;
}