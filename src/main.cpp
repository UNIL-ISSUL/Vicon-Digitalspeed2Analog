#include <Arduino.h>
#include <TimerThree.h>
#include <Ewma.h>

//debug mode (1: on, 0: off) -> print data on serial port
const byte debug = 1;

//digital signal pin
const byte digitalPin = 2;  //industrial shield I0.0 / INT1
//analog output
const byte analogPin = 10;  //industrial shield A0.2
//counter for pulses
volatile unsigned int counter = 0;
//data available tag
volatile boolean data_available = false;
//degin Ewma filter
Ewma adcFilter1(0.5);

//vicon acquisition rate
const unsigned int f_vicon = 10; //Hz
//compute max_speed in pulse per ms
//motor speed is 1500 tr/min at 50Hz, the VFD drives the motor up to 100Hz -> motor speed is 3000 tr/min
//there are 26 tooth on motor pulley
unsigned int max_frequency_Hz = 2.0 * 1500 / 60 * 26;

//define interupt function to count for pulses
void on_interupt();
void on_timer();

//setup function
void setup() {
  //start serial com
  if (debug) Serial.begin(9600);
  //init digital pin
  pinMode(digitalPin, INPUT);
  //init analog pin
  pinMode(analogPin, OUTPUT);
  //init interupt
  attachInterrupt(digitalPinToInterrupt(digitalPin), on_interupt,CHANGE);
  //init timer
  Timer3.initialize(1e6/f_vicon); //5ms -> 200Hz (vicon acquisition rate) 
  Timer3.attachInterrupt(on_timer);
  //init dac
  analogWrite(analogPin,0);  
}

//loop function
void loop() {
  //Serial.println("loop"+String(data_available));
  //update frequency if data available
  if(data_available) {
    //compute frequency
    float frequency_Hz = counter * f_vicon;
    //filter freqeuncy
    float frequency_Hz_ewma = adcFilter1.filter(frequency_Hz);
    //map analog value
    //int val = map(frequency_Hz_ewma,0,max_frequency_Hz,0,255);
    int pulses_20kmh = 126;
    byte val = map(counter,0,pulses_20kmh,0,255);
    //write analog value
    analogWrite(analogPin,val);
    
    //print data on serial port
    if (debug) {
      //print counter for debug
      Serial.println(">counter:"+String(counter));
      //print period_us for teleplot
      Serial.println(">frequency_Hz:"+String(frequency_Hz));
      //print period_us_ewma for debug
      Serial.println(">frequency_Hz_ewma:"+String(frequency_Hz_ewma));
      //print val for debug
      Serial.println(">val:"+String(val));
    }
    //reset counter
    counter = 0;
    //reset data tag
    data_available = false;
  }
}

//ISR function call by interupt
void on_interupt() {
  if(true) {
    //update counter
    counter++;
  }
}

//ISR function call by timer
void on_timer() {
  //update data tag
  data_available = true;
}