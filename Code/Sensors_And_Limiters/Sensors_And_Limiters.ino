#include <Wire.h>
#include <AS5600.h>
#include "TCA9548.h"

// Hardware interupts are pins 2, 3, 18, 19, 20, 21

AS5600 pEncoder; // is channel 0 of multiplexer
AS5600 tEncoder; // is channel 1 of multiplexer

TCA9548 MP(0x70);


// red Vcc, black GND, blue SCL, green SDA, DIR

int zeroPin = 18; // Limit switch's interupt pin is digital pin 18
volatile int firstHit = HIGH; // Keeps track of whether going into the limit switch or out of it
volatile int oTimeHit = 0; // Keeps track of when the limite was last hit
const unsigned long debounceTime = 10;

int SDAPin = 20; // I2C Pins
int I2CPin = 21;

int pan = 0; // Value of Pan Encoder
int trav = 0; // Value of the Traverse Encoder
int zero = 0; // Value of Transverse Encoder at limit switch

int availBytes = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (MP.begin() == false)
  {
    Serial.println("COULD NOT CONNECT TO MP");
  }

  MP.selectChannel(0);
  if (pEncoder.begin() == false)
  {
    Serial.println("COULD NOT CONNECT TO PAN ENCODER");
  }
  pEncoder.begin(4);  //  set direction pin.
  pEncoder.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int b = pEncoder.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  MP.selectChannel(1);
  if (tEncoder.begin() == false)
  {
    Serial.println("COULD NOT CONNECT TO TRAVEL ENCODER");
  }
  tEncoder.begin(4);  //  set direction pin.
  tEncoder.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int c = tEncoder.isConnected();
  Serial.print("Connect: ");
  Serial.println(c);
  
  attachInterrupt(digitalPinToInterrupt(zeroPin), hitZero, CHANGE);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
  
  // Poll Pan Encoder
  MP.selectChannel(1);
  pan = pEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;
  Serial.print("Pan: ");
  Serial.println(pan);

  // Poll Travel Encoder
  
  MP.selectChannel(0);
  trav = tEncoder.rawAngle() * AS5600_RAW_TO_DEGREES - zero;
  Serial.print("Travel: ");
  Serial.println(trav);
}

void hitZero() { // This needs work
  // ISR, deal with hitting zero limit switch
  if (millis() - oTimeHit >= debounceTime) {
    Serial.println("Hit Zero Change!");
    if (firstHit == HIGH) {
      Serial.println("Going in!");
      // reverse direction, going into zero and now past it, go slow back
    } else {
      Serial.println("Heading Out!");
      // stop, now at zero position. 
      MP.selectChannel(0);
      trav = tEncoder.rawAngle() * AS5600_RAW_TO_DEGREES - zero;
      zero = 0;
    }
    firstHit = !firstHit;
    oTimeHit = millis();
  }
  
}
