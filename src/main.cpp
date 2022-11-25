#define CIRCULAR_BUFFER_INT_SAFE
#include "CircularBuffer.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <EwmaT.h>


//digital signal pin
const byte digitalPin = 0;
volatile unsigned int period_us = 0;
volatile unsigned int previous_micros = 0;
volatile boolean data_available = false;
EwmaT <unsigned int> ewma(1,13); //create a filter with equivalent alpha of 0.38 (1/26) on fait la moyenne sur une tour de roue dentée.
volatile unsigned int counter = 0;

//LED builtin state
volatile boolean state;

//Analog value
unsigned int level = 0;
//compute max_speed in pulse per ms
//motor speed is 1500 tr/min at 50Hz, the VFD drives the motor up to 100Hz -> motor speed is 3000 tr/min
//there are 26 tooth on motor pulley
unsigned int max_frequency_Hz = 2.0 * 1500 / 60 * 26;
//unsigned int max_frequency_Hz = 2.0 * 1500 / 60;


//define interupt function to count for pulses
void on_interupt();

//Adafruit 12bits DAC
Adafruit_MCP4725 dac;

void setup() {
  //start serial com
  Serial.begin(9600);
  /*while(Serial.available() == 0) { 
    Serial.println("wait for input");
  }*/
  //define pin
  pinMode(digitalPin, INPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(digitalPin), on_interupt, RISING);

  //init led state so the LED is HIGH when sensor dedect tooth
  state = !digitalRead(digitalPin);
  digitalWrite(LED_BUILTIN,state);

  //get actual micros
  previous_micros = micros();

  //init dac
  if(!dac.begin(0x62)) Serial.println("dac initialization failed");
  dac.setVoltage(0,false);
  
}

void loop() {
  //change led state if data are available
  if(data_available) {
    //write current LED state
    digitalWrite(LED_BUILTIN,state);
    ////Compute Exponentially Weighted Moving Average
    unsigned int period_us_ewma = ewma.filter(period_us);
    //unsigned int period_us_ewma = period_us;
    //write period value to DAC
    int val = round(((1e6 / period_us_ewma) * 4095 ) / max_frequency_Hz);
    if (!dac.setVoltage(val,false)) Serial.println("dac write value failed");
    //else Serial.println("period us :"+String(period_us)+" 12bits value : "+String(val)+" counter : "+String(counter));
    data_available = false;
  }
}

//ISR function call by interupt
void on_interupt() {
  //update teeth counter 
  counter +=1 ;
  //if((counter % 1 )== 0) {
    //read time and compute period
    unsigned int current_micros = micros();
    period_us = current_micros - previous_micros;
    //update timer
    previous_micros = current_micros;
    //update data tag
    data_available = true;
    //swith on LED
    state = !state;
  //}
}