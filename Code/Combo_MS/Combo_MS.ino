#include <Wire.h>
#include <AS5600.h>
#include "TCA9548.h"

// Sensor Stuff

// Hardware interupts are pins 2, 3, 18, 19, 20, 21

AS5600 pEncoder; // is channel 0 of multiplexer
AS5600 tEncoder; // is channel 1 of multiplexer

TCA9548 MP(0x70);


// red Vcc, black GND, blue SCL, green SDA, DIR

int zeroPin = 18; // Limit switch's interupt pin is digital pin 18
volatile int oTimeHit = 0; // Keeps track of when the limite was last hit
volatile float zero = 0; // Value of Transverse Encoder at limit switch
const unsigned long debounceTime = 10;

int SDAPin = 20; // I2C Pins
int I2CPin = 21;

float pan = 0; // Value of Pan Encoder
volatile float panO = 0; // Old Value of the Pan Encoder
volatile float panRots = 0; // Number of times panned around clockwise(?)
volatile bool clockWise = true; // Whether pan is in negative or positive angles
float pOffset = -117.6;

volatile int travRots = 0; // Number of times travel encoder has rotated 360 degrees
volatile float travO = 0; // Old Value of the Traverse Encoder
volatile float vel = 0;
float initTOffset = 0;

int availBytes = 0;

// Motor Stuff

#define enablePin 26                         //Define motor enable pin
#define travDirPin 23                        //Define travel & rotation stepper motor pins
#define travStepPin 22
#define rotDirPin 25
#define rotStepPin 24

#define initialDur 120 

float travTime = initialDur;  

int travelPulses = 100;

void setup() {

  // Picked this Baud Rate as that's what package demo used, not sure how essential it is so don't mess with
  
  Serial.begin(115200);

  // Motor Stuff
  
  pinMode(enablePin, INPUT);                              //Open circuit enable pin, disables motors
  pinMode(travDirPin, OUTPUT);                            //Define the travel stepper motor pins
  pinMode(travStepPin, OUTPUT);
  pinMode(rotDirPin, OUTPUT);                             //Define the rotation stepper motor pins
  pinMode(rotStepPin, OUTPUT);

  // Sensor Stuff
    
  attachInterrupt(digitalPinToInterrupt(zeroPin), hitZero, CHANGE);
  
  Wire.begin();

  if (MP.begin() == false)
  {
    Serial.println("COULD NOT CONNECT TO MP");
  }

  MP.selectChannel(0);
  if (pEncoder.begin() == false){
    Serial.println("COULD NOT CONNECT TO PAN ENCODER");
  }
  if (pEncoder.detectMagnet() == false){
    Serial.println("NO PAN MAGNET!");
  }
  if (pEncoder.magnetTooStrong() == true){
    Serial.println("PAN MAGNET TOO CLOSE!");
  }
  if (pEncoder.magnetTooWeak() == true){
    Serial.println("PAN MAGNET TOO FAR!");
  }
  pEncoder.begin(4);  //  set direction pin.
  pEncoder.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int b = pEncoder.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  MP.selectChannel(1);
  if (tEncoder.begin() == false){
    Serial.println("COULD NOT CONNECT TO TRAVEL ENCODER");
  }
  if (tEncoder.detectMagnet() == false){
    Serial.println("NO TRAVEL MAGNET!");
  }
  if (tEncoder.magnetTooStrong() == true){
    Serial.println("TRAVEL MAGNET TOO CLOSE!");
  }
  if (tEncoder.magnetTooWeak() == true){
    Serial.println("TRAVEL MAGNET TOO FAR!");
  }
  tEncoder.begin(4);  //  set direction pin.
  tEncoder.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int c = tEncoder.isConnected();
  Serial.print("Connect: ");
  Serial.println(c);

  // make sure at initial values
  pEncoder.setOffset(pOffset);

  MP.selectChannel(0);
  initTOffset = tEncoder.rawAngle()* AS5600_RAW_TO_DEGREES;
  travO = initTOffset;
  tEncoder.setOffset(initTOffset);
}

void loop() {

  delay(100);
  Serial.println(getPan());

  // Motor Code

  // Need to figure out how to merge, maybe make sensor code a func that outputs the current position
  // and have the motor code then use that?
  
  //pinMode(enablePin, OUTPUT);

  //float interval = calcInterval (travelPulses);         //Calculate the pulse interval required to move the required distance in the required time

  // put your main code here, to run repeatedly:
  //for (int i = 1; i <= travelPulses; i++)               //Pulse the motor to move the required distance in the required time
  //{
  //  digitalWrite(travStepPin, HIGH);
  //  delayMicroseconds(interval / 2);
  //  digitalWrite(travStepPin, LOW);
  //  delayMicroseconds(interval / 2);
  //}

  //delay(1000);

  //for (int i = 1; i <= travelPulses; i++)               //Pulse the motor to move the required distance in the required time
  //{
  //  digitalWrite(rotStepPin, HIGH);
  //  delayMicroseconds(interval / 2);
  //  digitalWrite(rotStepPin, LOW);
  //  delayMicroseconds(interval / 2);
  //}

}

float calcInterval (int numPulses)                                 //Calculate the interval required between pulses to achieve duration
{
  float inter = travTime * 1000000 / numPulses;
  return inter;
}

float calcRotInterval (int numPulses)                              //Calculate the interval required between pulses to achieve duration
{
  float inter = travTime * 1000 / numPulses;
  return inter;
}

float getPan(){
  // Poll Pan Encoder
  MP.selectChannel(1);
  pan = pEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;

  if ((pan < 90) and (panO > 270)){
    panRots = panRots + 1;
  }
  else if ((pan > 270) and (panO < 90)){
    panRots = panRots - 1;
  }
  panO = pan;
  pan = pan + 360*panRots;
  return pan;
}

float getTrav(){
  // Poll Travel Encoder
  // travO and travRots are stored globally as need to 
  // remember last poll to know if done a full rotation
  // and need to keep track of rotations seperate
  // to avoid double counting partial rotations
  
  MP.selectChannel(0);

  float travN = tEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;

  if ((travN < 90) and (travO > 270)){
    travRots = travRots + 1;
  }

  if ((travN > 270) and (travO < 90)){
    travRots = travRots - 1;
  }

  float totTrav = travRots + travN/360;
  travO = travN;
  return(totTrav);
}


void hitZero() { // This needs work
  // ISR, deal with hitting zero limit switch
  if (millis() - oTimeHit >= debounceTime) {
    Serial.println("Zero!");
    MP.selectChannel(0);
    zero = tEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;
    tEncoder.setOffset(zero);
    travRots = 0;
    oTimeHit = millis();
  }
}
