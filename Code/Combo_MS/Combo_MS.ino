#include <Wire.h>
#include <AS5600.h>
#include <TCA9548.h>

// Sensor Stuff

// Hardware interupts are pins 2, 3, 18, 19, 20, 21

AS5600 pEncoder; // is channel 0 of multiplexer
AS5600 tEncoder; // is channel 1 of multiplexer

TCA9548 MP(0x70);


// red Vcc, black GND, blue SCL, green SDA, DIR

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

bool travstate = false;
bool rotstate = false;

//int steps1 = 16 * 50;
//int steps2 = 16 * 25;
//
//float timemilsec = 10 * 1000;

void setup() {

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
  
  goToZero();
  Serial.println(getTrav());
}

void loop() {
  unsigned long cLoopTime = millis();

  // Motor Code

  float currentPos = getTrav();
  while (currentPos < 5.0){
    currentPos = getTrav();
    cLoopTime = millis();
    if (cLoopTime - oLoopTime >= 50){
        Serial.println(getTrav());
        oLoopTime = cLoopTime;
    }
    Serial.println(currentPos);
    moveMotor(0, 5, 15);
  }
  delay(1000);
  while (currentPos > 0.5){
    currentPos = getTrav();
    cLoopTime = millis();
    if (cLoopTime - oLoopTime >= 50){
        Serial.println(getTrav());
        oLoopTime = cLoopTime;
    }
    Serial.println(currentPos);
    moveMotor(0, -5, 15);
  }

}

void setZero(){
  Serial.println("Zero Start");
  MP.selectChannel(0);
  float zero = tEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;
  tEncoder.setOffset(-zero);
  Serial.println(zero);
  Serial.println(tEncoder.getOffset());
  Serial.println(tEncoder.rawAngle() * AS5600_RAW_TO_DEGREES);
  Serial.println("Zero End");
  travRots = 0;
  travO = 0;
}

void goToZero(){
  while (inZero == false){
    unsigned long cTime = millis();
    moveMotor(0, -2, 6);
    if (cTime - oTime >= 50){
      Serial.println(getTrav());
      oTime = cTime;
    }
  }
  Serial.println(getTrav());
  moveMotor(0, 100, 50);
  delay(50);
  setZero();
}

void moveMotor(int steps1, int steps2, unsigned long timemilsec) {
  // put your main code here, to run repeatedly:

  float interval1;
  float interval2;

  unsigned long curint1time;
  unsigned long offset1;

  unsigned long curint2time;
  unsigned long offset2;

  unsigned long elapTime;
  int curstep1 = 0;
  int curstep2 = 0;
  int maxsteps;

  int tracker = 0;
  bool step1state = false;
  bool step2state = false;


  if (steps1 < 0) {
    digitalWrite(rotDirPin, HIGH);
    steps1 = abs(steps1);
  } else {
    digitalWrite(rotDirPin, LOW);
  }
  if (steps2 < 0) {
    digitalWrite(travDirPin, HIGH);
    steps2 = abs(steps2);
  } else {
    digitalWrite(travDirPin, LOW);
  }

  if (steps1 > steps2) {
    maxsteps = steps1;
    tracker = curstep1;

  } else {
    maxsteps = steps2;
    tracker = curstep2;
  }

  interval1 = timemilsec / steps1;
  interval2 = timemilsec / steps2;

  while (!step1state or !step2state) {
    elapTime = millis();
    curint1time = elapTime - offset1;
    curint2time = elapTime - offset2;
    if (curstep1 < steps1) {
      if (curint1time > (interval1 / 2) and  rotstate == false) {
        digitalWrite(rotStepPin, HIGH);
        offset1 = elapTime;
        curstep1++;
        rotstate = true;
      }
      curint1time = elapTime - offset1;
      if (curint1time > (interval1 / 2) and rotstate == true ) {
        digitalWrite(rotStepPin, LOW);
        offset1 = elapTime;
        rotstate = false;
      }
    } else {
      step1state = true;
    }

    if (curstep2 < steps2) {
      if (curint2time > (interval2 / 2) and  travstate == false) {
        digitalWrite(travStepPin, HIGH);
        offset2 = elapTime;
        curstep2++;
        travstate = true;

      }
      curint2time = elapTime - offset2;
      if (curint2time > (interval2 / 2) and travstate == true ) {
        digitalWrite(travStepPin, LOW);
        offset2 = elapTime;
        travstate = false;
      }
    } else {
      step2state = true;
    }
  }
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
    oTimeHit = millis();
    inZero = true;
  }
}
