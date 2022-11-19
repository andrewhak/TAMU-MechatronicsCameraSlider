#include <Wire.h>
#include <AS5600.h>

// Hardware interupts are pins 2, 3, 18, 19, 20, 21

int zeroPin = 18; // Limit switch's interupt pin is digital pin 18
volatile int firstHit = HIGH; // Keeps track of whether going into the limit switch or out of it
volatile int oTimeHit = 0; // Keeps track of when the limite was last hit
const unsigned long debounceTime = 10;

int SDAPin = 20; // I2C Pins
int I2CPin = 21;

int panEnc = 0; // Value of Pan Encoder
int travEnc = 0; // Value of the Traverse Encoder

int availBytes = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(zeroPin), hitZero, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
  
  // Poll Pan Encoder
  //Wire.requestFrom(1);
  //availBytes = Wire.available();
  println(panEnc)

  // Poll Travel Encoder
  //Wire.requestFrom(2)
  //availBytes = Wire.available();
  println(travEnc)
}

void hitZero() {
  // ISR, deal with hitting zero limit switch
  if (millis() - oTimeHit >= debounceTime) {
    Serial.println("Hit Zero Change!");
    if (firstHit == HIGH) {
      Serial.println("Going in!");
      // reverse direction, going into zero and now past it, go slow back
    } else {
      Serial.println("Heading Out!");
      // stop, now at zero position. 
      // Maybe move a few counts forward 
      // and put int a software stop to 
      // ensure doesn't hit zero again?
    }
    firstHit = !firstHit;
    oTimeHit = millis();
    travEnc = 0;
  }
  
}
