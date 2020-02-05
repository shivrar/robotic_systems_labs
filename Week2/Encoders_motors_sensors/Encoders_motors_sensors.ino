#include "common_sys.h"

unsigned long last_timestamp;   // We will store a timestamp in this.
int sensor_value;             // We will store a sensor reading in this.
int count = 0;

bool led1 = false;
bool led2 = false;
bool led3 = false;


// Remember: Setup only runs once when the arduino is powered up.
void setup() {

//  pinMode(A0, INPUT );        // Setup up A0 as input to read.
  pinMode(13, OUTPUT);
  pinMode(LED_BUILTIN_RX, OUTPUT);
  pinMode(LED_BUILTIN_TX, OUTPUT);

  last_timestamp = millis();    // We set an intial timestamp value.

  if( SERIAL_ACTIVE ) 
  {

  Serial.print("An example");
  Serial.println(" of fixing the serial bug");
  };


}


// Remember: loop is called again and again.
void loop() {

    // Get how much time has passed right now.
    unsigned long time_now = millis();     

    unsigned long elapsed_time = time_now - last_timestamp;

    if( elapsed_time >= 1000 ) {
        // Since 10000ms elapsed, we overwrite our last_timestamp with the current time
        // so that another 10000ms is needed to pass.
        last_timestamp = millis();
        count++;
        led1=!led1;
        digitalWrite(13,led1);
//        Serial.println("Led3");
//        Serial.println(count);
//        Serial.println(elapsed_time);
        
    }

    if(count%3 == 0 && elapsed_time >=1000){
        led2=!led2;
        digitalWrite(LED_BUILTIN_RX,led2);
    }
    else if(count%2==0 && elapsed_time >=1000){
        led3=!led3;
        digitalWrite(LED_BUILTIN_TX,led3);
    }

  
//  last_timestamp = millis();

}
