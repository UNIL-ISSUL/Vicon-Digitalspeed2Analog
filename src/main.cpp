#include <Arduino.h>
//#include <Wire.h>
//#include <Adafruit_MCP4725.h>
//#include <EwmaT.h>
#include <Ewma.h>

const byte debug = 1;

//digital signal pin
const byte digitalPin = 2;  //industrial shield I0.0 / INT1
unsigned int period_us = 0;
unsigned int period_us_ewma = 0;
volatile unsigned int current_micros = 0;
unsigned int previous_micros = 0;
volatile boolean data_available = false;
//EwmaT <int> ewma(1,20); //create a filter with equivalent alpha of 0.38 (1/26) on fait la moyenne sur une tour de roue dentée.
Ewma adcFilter1(0.1);

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

//analog output
const byte analogPin = 10;  //industrial shield A0.2
//Adafruit 12bits DAC
//Adafruit_MCP4725 dac;

void setup() {
  //start serial com
  if (debug) Serial.begin(9600);
  /*while(Serial.available() == 0) { 
    Serial.println("wait for input");
  }*/
  //define pin
  //init digital pin
  pinMode(digitalPin, INPUT);
  //pinMode(LED_BUILTIN,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(digitalPin), on_interupt, RISING);
  //init analog pin
  pinMode(analogPin, OUTPUT);

  //init led state so the LED is HIGH when sensor dedect tooth
  //state = !digitalRead(digitalPin);
  //digitalWrite(LED_BUILTIN,state);

  //get actual micros
  current_micros = micros();
  previous_micros = micros();

  //init dac
  //if(!dac.begin(0x62)) Serial.println("dac initialization failed");
  //dac.setVoltage(0,false);
  analogWrite(analogPin,0);  
}
byte count_error = 0;
byte outliner = 0;
void loop() {
  //change led state if data are available
  if(data_available) {
    //compute period
    period_us = current_micros - previous_micros;
    //update timer
    previous_micros = current_micros;
    //write current LED state
    //state = !state;
    //digitalWrite(LED_BUILTIN,state);
    //compare period_us with last_period_us_ewma
    //float error = abs(period_us-period_us_temp)/period_us; 
    /*if(error > 0.2) {
      Serial.println("outliner detected");
      Serial.println(String(error));
      Serial.println(String(period_us));
      Serial.println(String(period_us_temp));
      //if period_us is 30% lower than last_period_us_ewma, outliner
      ////Compute Exponentially Weighted Moving Average
      count_error++;    }
    else {
      count_error = 0;
    }*/
    //Compute Exponentially Weighted Moving Average
    //period_us_ewma = ewma.filter(period_us);
    period_us_ewma = adcFilter1.filter(period_us);
    //write period value to DAC
    //int val = round(((1e6 / period_us_ewma) * 4095 ) / max_frequency_Hz);
    //if (!dac.setVoltage(val,false)) Serial.println("dac write value failed");
    //byte val = round(((1e6 / period_us) * 255 ) / max_frequency_Hz);
    int val;
    //check if period_us is greatear than 64000us (0.5km/h)
    if(period_us > 64000) {
      val =0;
    }
    else {
      int period_21kmh = 1515; //us
      val = constrain(round(255.0*period_21kmh/period_us_ewma),0,255);
    }
    analogWrite(analogPin,val);
    
    if (debug) {
      //print period_us for teleplot
      Serial.println(">period_us:"+String(period_us));
      //print period_us_ewma for debug
      Serial.println(">period_us_ewma:"+String(period_us_ewma));
      //print val for debug
      Serial.println(">val:"+String(val));
    }
    data_available = false;
  }
}

//ISR function call by interupt
void on_interupt() {
  if(!data_available) {
    //read time and compute period
    current_micros = micros();
  }
  //update data tag
  data_available = true;
}