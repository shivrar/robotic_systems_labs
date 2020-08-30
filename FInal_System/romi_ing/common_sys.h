#include <USBCore.h>    // To fix serial print behaviour bug.
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include "lineSensors.h"

u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)
#define LINE_LEFT_PIN A2
#define LINE_CENTRE_PIN A3
#define LINE_RIGHT_PIN A4
#define FORWARD LOW
#define REVERSE HIGH

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15


void setupMotorPins()
{
    // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r
  // Which of these is foward, or backward?
  digitalWrite( L_DIR_PIN, FORWARD);
  digitalWrite( R_DIR_PIN, FORWARD);
}
//TODO: Setup a sym link for the the other lib instead of copy and pasting everything

void flash_leds ()
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000); 
}

float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Function overload for variable time
void flash_leds (int delay_ms)
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_ms);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(delay_ms); 
}

void play_tone(int ledpin_no, int volume)
{
    analogWrite(ledpin_no, volume);
    delay(1);
    analogWrite(ledpin_no, volume);
}

void BangBang(LineSensor left, LineSensor centre, LineSensor right, int min_power, int max_power, byte &left_power, byte &right_power)
{
  byte sense = 0;
  sense = sense | ( (left.readCalibrated()>200 ? 1 : 0)  << 2 );
  sense = sense | ( (centre.readCalibrated()>200 ? 1 : 0)   << 1 );
  sense = sense | ( (right.readCalibrated()>200 ? 1 : 0)  << 0 );

  switch(sense){
    case 0:
      left_power = min_power;
      right_power = min_power;
    break;
    case 1:
      left_power = max_power;
      right_power = 0;
    break;
    case 3:
      left_power = max_power;
      right_power = min_power;
    break;
    case 4:
      left_power = 0;
      right_power = max_power;
    break;
    case 6:
      left_power = min_power;
      right_power = max_power;
    break;
    case 7:
      left_power = max_power;
      right_power = max_power;
    break;
    default:
    break;
  }
  
}

float weightedPower(LineSensor left, LineSensor centre, LineSensor right, int min_power, int max_power,volatile byte &left_power,volatile byte &right_power)
{
  float total = left.readCalibrated() + centre.readCalibrated() +right.readCalibrated();
  float rightM = right.readCalibrated()/(total);
  float leftM = left.readCalibrated()/(total);
  float m = leftM - rightM;  

  left_power = abs(m*max_power);
  right_power = abs(m*max_power);
  return max(min(m,1),-1);
}

float weightedPower(LineSensor left, LineSensor centre, LineSensor right, int min_power, int max_power)
{
  float total = left.readCalibrated() + centre.readCalibrated() +right.readCalibrated();
  float rightM = right.readCalibrated()/(total);
  float leftM = left.readCalibrated()/(total);
  float m = rightM - leftM;  

  return max(min(m,1),-1);
}

// Routine to setupt timer3 to run 
void setupTimer3(float hz) {
  
  // disable global interrupts
  cli();          

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // set entire TCCR3B register to 0

  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  // For a cpu clock precaler of 256:
  // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
  // Table 14.5 in manual. 
  TCCR3B = TCCR3B | (1 << CS32);
  
  
  // set compare match register to desired timer count.
  // CPU Clock  = 16000000 (16mhz).
  // Prescaler  = 256
  // Timer freq = 16000000/256 = 62500
  // We can think of this as timer3 counting up to 62500 in 1 second.
  // compare match value = 62500 / 2 (we desire 2hz).
  OCR3A = 62500.00/hz;
  
  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei(); 
  
}

// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the 
// compiler.  It automatically associates with Timer3 in
// CTC mode.

//~~~~~~~~~~~~~~~~~~~Pozyx configurations and functions~~~~~~~~~~~~~~~~~~~~
////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint16_t remote_id = 0x6000;                            // set this to the ID of the remote device
bool remote = false;                                    // set this to true to use the remote ID

boolean use_processing = false;                         // set this to true to output data for the processing sketch

const uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[num_anchors] = {0x6E2B, 0x676C, 0x670A, 0x6E22};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[num_anchors] = {0, 2737, 3651, 1541};               // anchor x-coorindates in mm
int32_t anchors_y[num_anchors] = {645, -410, 4210, 4450};                  // anchor y-coordinates in mm
int32_t heights[num_anchors] = {1214, 1913, 1853, 1775};              // anchor z-coordinates in mm

uint8_t algorithm = POZYX_POS_ALG_TRACKING;             // positioning algorithm to use. try POZYX_POS_ALG_TRACKING for fast moving objects.
uint8_t dimension  = POZYX_2_5D;                           // positioning dimension
int32_t height = 940;                                  // height of device, required in 2.5D positioning

// prints the coordinates for either humans or for processing
void printCoordinates(coordinates_t coor){
  uint16_t network_id = remote_id;
  if (network_id == NULL){
    network_id = 0;
  }
  if(!use_processing){
    Serial.print("POS ID 0x");
    Serial.print(network_id, HEX);
    Serial.print(", x(mm): ");
    Serial.print(coor.x);
    Serial.print(", y(mm): ");
    Serial.print(coor.y);
    Serial.print(", z(mm): ");
    Serial.println(coor.z);
  }else{
    Serial.print("POS,0x");
    Serial.print(network_id,HEX);
    Serial.print(",");
    Serial.print(coor.x);
    Serial.print(",");
    Serial.print(coor.y);
    Serial.print(",");
    Serial.println(coor.z);
  }
}

// error printing function for debugging
void printErrorCode(String operation){
  uint8_t error_code;
  if (remote_id == NULL){
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", local error code: 0x");
    Serial.println(error_code, HEX);
    return;
  }
  int status = Pozyx.getErrorCode(&error_code, remote_id);
  if(status == POZYX_SUCCESS){
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(" on ID 0x");
    Serial.print(remote_id, HEX);
    Serial.print(", error code: 0x");
    Serial.println(error_code, HEX);
  }else{
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", couldn't retrieve remote error code, local error: 0x");
    Serial.println(error_code, HEX);
  }
}


// print out the anchor coordinates (also required for the processing sketch)
void printCalibrationResult(){
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size, remote_id);
  Serial.print("list size: ");
  Serial.println(status*list_size);

  if(list_size == 0){
    printErrorCode("configuration");
    return;
  }

  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids, list_size, remote_id);

  Serial.println(F("Calibration result:"));
  Serial.print(F("Anchors found: "));
  Serial.println(list_size);

  coordinates_t anchor_coor;
  for(int i = 0; i < list_size; i++)
  {
    Serial.print("ANCHOR,");
    Serial.print("0x");
    Serial.print(device_ids[i], HEX);
    Serial.print(",");
    Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor, remote_id);
    Serial.print(anchor_coor.x);
    Serial.print(",");
    Serial.print(anchor_coor.y);
    Serial.print(",");
    Serial.println(anchor_coor.z);
  }
}

// function to manually set the anchor coordinates
void setAnchorsManual(){
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
    Pozyx.addDevice(anchor, remote_id);
  }
  if (num_anchors > 4){
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, remote_id);
  }
}
