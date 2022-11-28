#include <Wire.h>
#include <AS5600.h>
#include "TCA9548.h"

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
float travN = 0; // Current Value of the Traverse Encoder
float travO = 0; // Old Value of the Traverse Encoder
float pOffset = -117.6;

int travRots = 0; // Number of times travel encoder has rotated 360 degrees
float totTrav = 0;
float vel = 0;
float initTOffset = 0;

int availBytes = 0;

void setup() {
  Serial.begin(115200);

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
  // put your main code here, to run repeatedly:
  delay(50);
  
  // Poll Pan Encoder
  MP.selectChannel(1);
  pan = pEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;
  Serial.print("Pan: ");
  Serial.println(pan);

  // Poll Travel Encoder
  
  MP.selectChannel(0);
  
  travN = tEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;

  if ((travN < 90) and (travO > 270)){
    travRots = travRots + 1;
  }

  if ((travN > 270) and (travO < 90)){
    travRots = travRots - 1;
  }

  totTrav = travRots + travN/360;
  
  Serial.print("Travel: ");
  Serial.println(totTrav);
  travO = travN;
}

void hitZero() { // This needs work
  // ISR, deal with hitting zero limit switch
  if (millis() - oTimeHit >= debounceTime) {
    Serial.println("Hit Zero!");
    MP.selectChannel(0);
    zero = tEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;
    tEncoder.setOffset(zero);
    travRots = 0;
    oTimeHit = millis();
  }
}
