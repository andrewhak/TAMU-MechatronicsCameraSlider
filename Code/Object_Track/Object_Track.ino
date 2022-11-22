#include <Wire.h>
#include <AS5600.h>
#include <TCA9548.h>
#include <LiquidCrystal.h>

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

volatile float travPoints[2];
volatile float panPoints[2];

unsigned long oTime = 0;
unsigned long oLoopTime = 0;

static int pinA = 2;                        //Hardware interrupt digital pin 2
static int pinB = 3;                        //Hardware interrupt digital pin 3
volatile byte aFlag = 0;                    //Rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0;                    //Rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile int encoderPos = 0;                //Current value of encoder position, constained between limits below
volatile int prevEncoderPos = 0;            //To track whether the encoder has been turned and the display needs to update
volatile byte reading = 0;                  //Stores direct value from interrupt pin
#define encButton 6                         //Define encoder pushbutton pin
byte oldButtonState = HIGH;                 //First button state is open because of pull-up resistor
//const unsigned long debounceTime = 10;      //Debounce delay time
unsigned long buttonPressTime;              //Time button has been pressed for debounce

int encLowLim = 0;                          //Variables to store the encoder limits and increment
int encHighLim = 1;
int encIncrement = 1;
int dataInputNo = 0;
int modeSelected = 0;                       //Current operating mode (Pan, Rotate, Pan & Rotate, Track Object)

// Motor Stuff

#define enablePin 26                         //Define motor enable pin
#define travDirPin 25                        //Define travel & rotation stepper motor pins
#define travStepPin 24
#define rotDirPin 23
#define rotStepPin 22
#define initialDur 120 

bool travstate = false;
bool rotstate = false;

int i = 0;
  
int j = 0;

void setup() {

  // Picked this Baud Rate as that's what rotary encoder demo used, 
  // not sure how essential it is so don't mess with
  
  Serial.begin(115200);

  pinMode(travDirPin, OUTPUT);                            //Define the travel stepper motor pins
  pinMode(travStepPin, OUTPUT);
  pinMode(rotDirPin, OUTPUT);                             //Define the rotation stepper motor pins
  pinMode(rotStepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);


  digitalWrite(travStepPin, LOW);
  digitalWrite(rotStepPin, LOW);

  Serial.println("Motor Setup Complete");

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

  Serial.println("Sensor Setup Complete");
  
  // make sure at initial values
  pEncoder.setOffset(pOffset);
  
  goToZero();
  Serial.println(getTrav());
  Serial.println("Travel Zero'd");
  
}

void loop() {
  float ang1 = 15;
  float ang2 = 60;
  float pos1 = 1;
  float pos2 = 3; 
  float intPos = 10;
  float timer = 10;
  float currentAng = getPan();
  if (currentAng < ang1){
    while (currentAng < ang1){
      moveMotor(-200,0,10000);
      currentAng = getPan();
      Serial.print("SetUP");        
    }
  }
  else if(currentAng > ang1){
    while (currentAng > ang1){
      moveMotor(200,0,10000);
      currentAng = getPan();
      Serial.print("SetUP");    
    }
  }
  float currentPos = getTrav();
  if (currentPos < pos1){
    while (currentPos < pos1){
      moveMotor(0,1000,10000);
      currentPos = getTrav();
      Serial.print("SetUP");        
    }
  }
  float targAngle = objTrack(ang1,ang2,pos1,pos2,intPos);
  float len_ang = targAngle-ang1; 
  float len = pos2-pos1;
  boolean confirmed = false;                                //Both used to confirm button push to select mode
  boolean pressed = false;
  encoderPos = 0;      
  while (!confirmed)                                        //While the user has not confirmed the selection
  {
    moveMotor(len_ang/0.07,len/0.0006,timer);
    Serial.print("Next MOVE");
    moveMotor(-len_ang/0.07,-len/0.0006,timer);
    byte buttonState = digitalRead (encButton);
    if (buttonState != oldButtonState)
    {
      if (millis () - buttonPressTime >= debounceTime)      //Debounce button
      {
        buttonPressTime = millis ();                        //Time when button was pushed
        oldButtonState =  buttonState;                      //Remember button state for next time
        if (buttonState == LOW)
        {
          pressed = true;
          Serial.println("Button Pushed");
        }
        else
        {
          if (pressed == true)                              //Confirm the input once the button is released again
          {
            confirmed = true;
            Serial.println("STOP");
          }
        }
      }
    }
    if (encoderPos != prevEncoderPos)                       //Update the display if the encoder position has changed
    {
      break;
      prevEncoderPos = encoderPos;
    }
  }
}

float objTrack(float ang1, float ang2, float pos1, float pos2, float intPos){
  float angToMove = ang2-ang1;
  float len = abs(pos2-pos1);
  float x1 = -len / (tan(ang1)*(cos(ang2)/sin(ang2))-1); // cotan is cos on sin
  float y1 =  len / ( cos(ang1)/sin(ang1) - cos(ang2)/sin(ang2));
  float TargAngle = atan(y1 / (x1-intPos));
  return TargAngle;
}

void moveMotor(float steps1, float steps2, float timemilsec) {
  // put your main code here, to run repeatedly:
  bool travstate = false;
  bool rotstate = false;
  float interval1;
  float interval2;

  unsigned long offset1;
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

  } else {
    maxsteps = steps2;
  }
//  Serial.println(timemilsec);
  interval1 = (timemilsec / steps1);
  interval2 = (timemilsec / steps2);
//  Serial.println(interval1);
//  Serial.println(interval2);
  offset1 = 0;
  offset2 = 0;
  float currentAng=getPan();
  float currentPos=getTrav();
  while (curstep1 < steps1 or curstep2 < steps2) {
    float currentAng=getPan();
    float currentPos=getTrav();
    unsigned long currentMotor1Time = millis();
    unsigned long currentMotor2Time = millis();
    if ((currentMotor1Time - offset1) > (interval1) and curstep1 < steps1) {
      step1();
      offset1 = currentMotor1Time;
      curstep1++;
    }

    if ((currentMotor2Time - offset2) > (interval2) and curstep2 < steps2) {
      step2();
      offset2 = currentMotor2Time;
      curstep2++;

    }

  }
  Serial.print("Angle: ");
  Serial.print(currentAng);
  Serial.print("    Distance: ");
  Serial.print(currentPos);
  Serial.print("    ");;
  Serial.print(curstep1);
  Serial.println();

}
void step1() {
  if (i == 0) {
    digitalWrite(rotStepPin, HIGH);
    i = i+1;
  } else {
    digitalWrite(rotStepPin, LOW);
    i = i-1;
  }
}

void step2() {
  if (j == 0) {
    digitalWrite(travStepPin, HIGH);
    j = j+1;
  } else {
    digitalWrite(travStepPin, LOW);
    j = j-1;
  }
}

// Sensor Functions

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
  cli();
  if (millis() - oTimeHit >= debounceTime) {
    oTimeHit = millis();
    inZero = true;
  }
  sei();
}

void setZero(){
  MP.selectChannel(0);
  float zero = tEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;
  tEncoder.setOffset(-zero);
  travRots = 0;
  travO = 0;
}

void goToZero(){
  while (inZero == false){
    unsigned long cTime = millis();
    moveMotor(0, -200, 6);
    if (cTime - oTime >= 50){
      Serial.println(getTrav());
      oTime = cTime;
    }
  }
  Serial.println(getTrav());
  moveMotor(0, 100, 50);
  setZero();
}

void PinA()                                             //Rotary encoder interrupt service routine for one encoder pin
{

  cli();                                                //Stop interrupts happening before we read pin values
  reading = PINE & 0x30;                                 //Read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00110000 && aFlag)                    //Check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
  {
    if (encoderPos <= (encHighLim - encIncrement))
      encoderPos = encoderPos + encIncrement;           //Increment the encoder's position count , only when below upper limit
    else
      encoderPos = encHighLim;                          //Stop at maximum, being upper limit
    bFlag = 0;                                          //Reset flags for the next turn
    aFlag = 0;                                          //Reset flags for the next turn
  }
  else if (reading == B00010000)                        //Signal that we're expecting pinB to signal the transition to detent from free rotation
    bFlag = 1;
  sei();                                                //Restart interrupts
}

void PinB()                                             //Rotary encoder interrupt service routine for the other encoder pin
{
  cli();                                                //Stop interrupts happening before we read pin values
  reading = PINE & 0x30;                                 //Read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00110000 && bFlag)                    //Check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
  {
    if (encoderPos >= (encLowLim + encIncrement))
      encoderPos = encoderPos - encIncrement;           //Decrement the encoder's position count, only when above lower limit
    else
      encoderPos = encLowLim;                           //Stop at minimum, being lower limit
    bFlag = 0;                                          //Reset flags for the next turn
    aFlag = 0;                                          //Reset flags for the next turn
  }
  else if (reading == B00100000)                        //Signal that we're expecting pinA to signal the transition to detent from free rotation
    aFlag = 1;
  sei();                                                //Restart interrupts
}
