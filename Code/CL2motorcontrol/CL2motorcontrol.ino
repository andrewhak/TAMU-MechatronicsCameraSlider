#include <Wire.h>
#include <AS5600.h>
#include <TCA9548.h>

AS5600 pEncoder; // is channel 0 of multiplexer
AS5600 tEncoder; // is channel 1 of multiplexer

TCA9548 MP(0x70);

int zeroPin = 18; // Limit switch's interupt pin is digital pin 18
volatile int oTimeHit = 0; // Keeps track of when the limite was last hit
volatile int cTimeHit = 0; // Keeps track of when the limite was last hit
const unsigned long debounceTime = 10;

int SDAPin = 20; // I2C Pins
int I2CPin = 21;

volatile bool inZero = false;

float pan = 0; // Value of Pan Encoder
volatile float panO = 0; // Old Value of the Pan Encoder
volatile float panRots = 0; // Number of times panned around clockwise
volatile bool clockWise = true; // Whether pan is in negative or positive angles
float pOffset = -117.6;

volatile int travRots = 0; // Number of times travel encoder has rotated 360 degrees
volatile float travO = 0; // Old Value of the Traverse Encoder
volatile float vel = 0;
float initTOffset = 0;

unsigned long oTime = 0;
unsigned long oLoopTime = 0;


// Motor Stuff

#define enablePin 26                         //Define motor enable pin
#define travDirPin 25                        //Define travel & rotation stepper motor pins
#define travStepPin 24
#define rotDirPin 23
#define rotStepPin 22

#define initialDur 120

int i = 0;
int j = 0;

float travRotPerStep = 0.0006;
float rotDegPerStep = 0.07;


void setup() {
  // put your setup code here, to run once:
  // Picked this Baud Rate as that's what package demo used, not sure how essential it is so don't mess with

  Serial.begin(115200);

  // Motor Stuff

  pinMode(travDirPin, OUTPUT);                            //Define the travel stepper motor pins
  pinMode(travStepPin, OUTPUT);
  pinMode(rotDirPin, OUTPUT);                             //Define the rotation stepper motor pins
  pinMode(rotStepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);


  digitalWrite(travStepPin, LOW);
  digitalWrite(rotStepPin, LOW);

  // Sensor Stuff

  attachInterrupt(digitalPinToInterrupt(zeroPin), hitZero, CHANGE);

  Wire.begin();

  if (MP.begin() == false)
  {
    Serial.println("COULD NOT CONNECT TO MP");
  }

  MP.selectChannel(0);
  if (pEncoder.begin() == false) {
    Serial.println("COULD NOT CONNECT TO PAN ENCODER");
  }
  if (pEncoder.detectMagnet() == false) {
    Serial.println("NO PAN MAGNET!");
  }
  if (pEncoder.magnetTooStrong() == true) {
    Serial.println("PAN MAGNET TOO CLOSE!");
  }
  if (pEncoder.magnetTooWeak() == true) {
    Serial.println("PAN MAGNET TOO FAR!");
  }
  pEncoder.begin(4);  //  set direction pin.
  pEncoder.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int b = pEncoder.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  MP.selectChannel(1);
  if (tEncoder.begin() == false) {
    Serial.println("COULD NOT CONNECT TO TRAVEL ENCODER");
  }
  if (tEncoder.detectMagnet() == false) {
    Serial.println("NO TRAVEL MAGNET!");
  }
  if (tEncoder.magnetTooStrong() == true) {
    Serial.println("TRAVEL MAGNET TOO CLOSE!");
  }
  if (tEncoder.magnetTooWeak() == true) {
    Serial.println("TRAVEL MAGNET TOO FAR!");
  }
  tEncoder.begin(4);  //  set direction pin.
  tEncoder.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int c = tEncoder.isConnected();
  Serial.print("Connect: ");
  Serial.println(c);

  // make sure at initial values
  pEncoder.setOffset(pOffset);

  goToZero();
  Serial.println(getTrav());
}

void loop() {
  // put your main code here, to run repeatedly:
  movetospot(30, 2, 10000);
  delay(1000);
  movetospot(60, 1, 10000);

}


void movetospot(float targetRot, float targetPos, unsigned long Time) {
  Serial.println("reached func");
  float currentPos;
  float currentRot;
  float rotError;
  float posError;
  int subdivision;
  int steptime = 50;
  float travRotPerStep = 0.0006;
  float rotDegPerStep = 0.07;
  int rotStep;
  int posStep;
  subdivision = Time / steptime;
  rotError = targetRot - currentRot;
  posError = targetPos - currentPos;


  while (abs(rotError) > 0.1 or abs(posError) > 0.1) {
    currentRot = getPan();
    currentPos = getTrav();

    rotError = targetRot - currentRot;
    posError = targetPos - currentPos;


    Serial.println(rotError);
    Serial.println(posError);


    rotStep = int((rotError * (1 / rotDegPerStep)));
    posStep = int((posError * (1 / travRotPerStep)));

    Serial.println(rotStep);
    Serial.println(posStep);

//    moveMotor(rotStep, posStep, steptime);

    float interval1;
    float interval2;

    unsigned long offset1 = 0;
    unsigned long offset2 = 0;

    int curstep1 = 0;
    int curstep2 = 0;

    if (rotStep < 0) {
      digitalWrite(rotDirPin, LOW);
      rotStep = abs(rotStep);
    } else {
      digitalWrite(rotDirPin, HIGH);
    }
    if (posStep < 0) {
      digitalWrite(travDirPin, HIGH);
      posStep = abs(posStep);
    } else {
      digitalWrite(travDirPin, LOW);
    }
    interval1 = (Time / rotStep );
    interval2 = (Time / posStep );
    unsigned long currentMotor1Time = millis();
    unsigned long currentMotor2Time = millis();

    while (currentMotor1Time < steptime) {
      unsigned long currentMotor1Time = millis();
      unsigned long currentMotor2Time = millis();
      if ((currentMotor1Time - offset1) > (interval1 / 2) and curstep1 / 2 < rotStep) {
        step1();
        offset1 = currentMotor1Time;
        curstep1++;
      }

      if ((currentMotor2Time - offset2) > (interval2 / 2) and curstep2 / 2 < posStep) {
        step2();
        offset2 = currentMotor2Time;
        curstep2++;

      }

    }

  }


}

float getPan() {
  // Poll Pan Encoder
  MP.selectChannel(1);
  pan = pEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;

  if ((pan < 90) and (panO > 270)) {
    panRots = panRots + 1;
  }
  else if ((pan > 270) and (panO < 90)) {
    panRots = panRots - 1;
  }
  panO = pan;
  pan = pan + 360 * panRots;
  return pan;
}

float getTrav() {
  // Poll Travel Encoder
  // travO and travRots are stored globally as need to
  // remember last poll to know if done a full rotation
  // and need to keep track of rotations seperate
  // to avoid double counting partial rotations

  MP.selectChannel(0);

  float travN = tEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;

  if ((travN < 90) and (travO > 270)) {
    travRots = travRots + 1;
  }

  if ((travN > 270) and (travO < 90)) {
    travRots = travRots - 1;
  }

  float totTrav = travRots + travN / 360;
  travO = travN;
  return (totTrav);
}

void hitZero() { // This needs work
  // ISR, deal with hitting zero limit switch
  if (millis() - oTimeHit >= debounceTime) {
    oTimeHit = millis();
    inZero = true;
  }
}

void goToZero() {
  while (inZero == false) {
    unsigned long cTime = millis();
    moveMotor(0, -2, 6);
    if (cTime - oTime >= 50) {
      Serial.println(getTrav());
      oTime = cTime;
    }
  }
  Serial.println(getTrav());
  moveMotor(0, 100, 50);
  setZero();
}

void moveMotor(float steps1, float steps2, float timemilsec) {
  // put your main code here, to run repeatedly:
  float interval1;
  float interval2;

  unsigned long offset1 = 0;
  unsigned long offset2 = 0;

  int curstep1 = 0;
  int curstep2 = 0;

  if (steps1 < 0) {
    digitalWrite(rotDirPin, LOW);
    steps1 = abs(steps1);
  } else {
    digitalWrite(rotDirPin, HIGH);
  }
  if (steps2 < 0) {
    digitalWrite(travDirPin, HIGH);
    steps2 = abs(steps2);
  } else {
    digitalWrite(travDirPin, LOW);
  }
  interval1 = (timemilsec / steps1);
  interval2 = (timemilsec / steps2);


  while (curstep1 < steps1 or curstep2 < steps2) {
    unsigned long currentMotor1Time = millis();
    unsigned long currentMotor2Time = millis();
    if ((currentMotor1Time - offset1) > (interval1 / 2) and curstep1 / 2 < steps1) {
      step1();
      offset1 = currentMotor1Time;
      curstep1++;
    }

    if ((currentMotor2Time - offset2) > (interval2 / 2) and curstep2 / 2 < steps2) {
      step2();
      offset2 = currentMotor2Time;
      curstep2++;

    }

  }

}
void step1() {
  if (i == 0) {
    digitalWrite(rotStepPin, HIGH);
    i = i + 1;
  } else {
    digitalWrite(rotStepPin, LOW);
    i = i - 1;
  }
}

void step2() {
  if (j == 0) {
    digitalWrite(travStepPin, HIGH);
    j = j + 1;
  } else {
    digitalWrite(travStepPin, LOW);
    j = j - 1;
  }
}

void setZero() {
  MP.selectChannel(0);
  float zero = tEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;
  tEncoder.setOffset(-zero);
  travRots = 0;
  travO = 0;
}
