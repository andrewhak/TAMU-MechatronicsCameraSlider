#include <Wire.h>
#include <AS5600.h>
#include <TCA9548.h>
#include <LiquidCrystal.h>

// Sensor Stuff, Luke worked on this

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

volatile bool runTrack = false; // Whether to enable object tracking or not

volatile int travRots = 0; // Number of times travel encoder has rotated 360 degrees
volatile float travO = 0; // Old Value of the Traverse Encoder
volatile float vel = 0;
float initTOffset = 0;

volatile float travPoints[9];
volatile float panPoints[9];
volatile float timePoints[9];

volatile int numPoints;

unsigned long oTime = 0;
unsigned long oLoopTime = 0;

// Motor Stuff Luke did some refinements on, 

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

// Interface Stuff, Luke did some interface stuff building off of this

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 8, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


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

volatile int backToMenu = false;

void setup() {

  // Picked this Baud Rate as that's what rotary encoder demo used, 
  // not sure how essential it is so don't mess with (Luke did this)
  
  Serial.begin(115200);

  // Interface Stuff, Luke did some work on

  lcd.begin(16, 2); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display }

  pinMode(pinA, INPUT_PULLUP);                            //Set pinA as an input, pulled HIGH to the logic voltage
  pinMode(pinB, INPUT_PULLUP);                            //Set pinB as an input, pulled HIGH to the logic voltage
  attachInterrupt(0, PinA, RISING);                       //Set an interrupt on PinA
  attachInterrupt(1, PinB, RISING);                       //Set an interrupt on PinB
  pinMode (encButton, INPUT_PULLUP);                      //Set the encoder button as an input, pulled HIGH to the logic voltage
  pinMode(enablePin, INPUT);                              //Open circuit enable pin, disables motors
  
  lcd.clear();                                 //Clear the display

  setupMenu();

  Serial.println("LCD Setup Complete");                       //Write to serial monitor to indicate the setup function is complete

  
  // Motor Stuff
  
  pinMode(travDirPin, OUTPUT);                            //Define the travel stepper motor pins
  pinMode(travStepPin, OUTPUT);
  pinMode(rotDirPin, OUTPUT);                             //Define the rotation stepper motor pins
  pinMode(rotStepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);


  digitalWrite(travStepPin, LOW);
  digitalWrite(rotStepPin, LOW);

  Serial.println("Motor Setup Complete");

  // Sensor Stuff, Luke worked on
    
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
  Serial.println("Zero'd");
  Serial.println("Setup Done");
}

void loop() { // Luke worked on this
  // put your main code here, to run repeatedly:
  encLowLim = 0;                                            //Mode selection menu, 2 modes
  encHighLim = 1;
  encIncrement = 1;
  updateMainMenu();

  boolean confirmed = false;                                //Both used to confirm button push to select mode
  boolean pressed = false;
  backToMenu = false;
  encoderPos = 0;                                           //Encoder starts from 0, first menu option
  while (!confirmed)                                        //While the user has not confirmed the selection
  {
    byte buttonState = digitalRead (encButton);
    if (buttonState != oldButtonState)
    {
      if (millis () - buttonPressTime >= debounceTime)      //Debounce button
      {
        buttonPressTime = millis ();                        //Time when button was pushed
        oldButtonState =  buttonState;                      //Remember button state for next time
        if (buttonState == LOW)
        {
          modeSelected = encoderPos;                        //If the button is pressed, accept the current digit into the guessed code
          pressed = true;
          Serial.println("Button Pushed");
        }
        else
        {
          if (pressed == true)                              //Confirm the input once the button is released again
          {
            confirmed = true;
            Serial.println("Mode confirmed");
          }
        }
      }
    }
    if (encoderPos != prevEncoderPos)                       //Update the display if the encoder position has changed
    {
      updateMainMenu();
      prevEncoderPos = encoderPos;
    }
  }
  Serial.println("Mode selected");
  if (modeSelected == 0) {                                   //Run required mode function depending on selection
    runTrack = false;
    Serial.print(runTrack);
    Serial.println(". Run free motion");
    Motion();
  }
  else {
    runTrack = true;
    Serial.print(runTrack);
    Serial.println(". Run object track");
    Motion();
  }
}

// Interface Functions

void inputPanAndRotateData(){ // Luke worked on this
  encLowLim = 0;                                            //Mode selection menu, 2 modes
  encHighLim = 1;
  encIncrement = 1;
  int counter = 0;

  updatePickMenu(counter+1);

  boolean confirmed = false;                                //Both used to confirm button push to select mode
  boolean pressed = false;
  boolean finishedPicking = false;                          // Used to know when done inputing points
  encoderPos = 0;                                           //Encoder starts from 0, first menu option

  while (!finishedPicking){
    while (!confirmed)                                        //While the user has not confirmed the selection
    {
      byte buttonState = digitalRead (encButton);
      if (buttonState != oldButtonState)
      {
        if (millis () - buttonPressTime >= debounceTime)      //Debounce button
        {
          buttonPressTime = millis ();                        //Time when button was pushed
          oldButtonState =  buttonState;                      //Remember button state for next time
          if (buttonState == LOW)
          {
            modeSelected = encoderPos;                        //If the button is pressed, accept the current digit into the guessed code
            pressed = true;
            //Serial.println("Button Pushed");
          }
          else
          {
            if (pressed == true)                              //Confirm the input once the button is released again
            {
              confirmed = true;
              //Serial.println("Mode confirmed");
            }
          }
        }
      }
      if (encoderPos != prevEncoderPos)                       //Update the display if the encoder position has changed
      {

        updatePickMenu(counter+1);
        prevEncoderPos = encoderPos;
      }
      getTrav(); 
      getPan();
    }
    //Serial.println("Mode selected");
    if (modeSelected == 0) {                                   //Run required mode function depending on selection
      Serial.println("Picked a Point");
      confirmed = false;
      
      travPoints[counter] = getTrav(); 
      panPoints[counter]= getPan();
      pinMode(enablePin, OUTPUT);
      pickedAPoint();
      delay(1000);
      
      pickTimes(counter);
      
      pinMode(enablePin, INPUT);
      
      if (counter >= numPoints-1){
        finishedPicking = true;
        Serial.println("Done Picking");
      }
      counter ++;

      updatePickMenu(counter+1);
    }
    else {
      Serial.println("Back");
      backToMain();
      delay(1000);
      finishedPicking = true;
      backToMenu = true;
    }
  }
}

void pickTimes(int i){ // Luke worked on this
  encLowLim = 0;                                            //Mode selection menu, 50 modes
  encHighLim = 76;
  encIncrement = 1;

  boolean confirmed = false;                                //Both used to confirm button push to select mode
  boolean pressed = false;                                  // Used to know when done inputing points
  encoderPos = 0;                                           //Encoder starts from 0, first menu option
  float timeValue = 1000 + encoderPos * 250;
  updatePickTimeMenu(timeValue);
  while (!confirmed)                                        //While the user has not confirmed the selection
  {
    byte buttonState = digitalRead (encButton);
    timeValue = 1000 + encoderPos * 250;
    if (buttonState != oldButtonState)
    {
      if (millis () - buttonPressTime >= debounceTime)      //Debounce button
      {
        buttonPressTime = millis ();                        //Time when button was pushed
        oldButtonState =  buttonState;                      //Remember button state for next time
        if (buttonState == LOW)
        {
          modeSelected = encoderPos;                        //If the button is pressed, accept the current digit into the guessed code
          pressed = true;
          //Serial.println("Button Pushed");
        }
        else
        {
          if (pressed == true)                              //Confirm the input once the button is released again
          {
            confirmed = true;
            //Serial.println("Mode confirmed");
          }
        }
      }
    }
    if (encoderPos != prevEncoderPos)                       //Update the display if the encoder position has changed
    {
      updatePickTimeMenu(timeValue);
      prevEncoderPos = encoderPos;
    }
  }
  timeValue = 1000 + encoderPos * 250;
  timePoints[i] = timeValue;
  confirmed = false;
  pickedATime();
  delay(1000);
  encoderPos = 0;
}

void pickNumPoints(){ // Luke worked on this
  encLowLim = 0;                                            //Mode selection menu, 50 modes
  encHighLim = 8;
  encIncrement = 1;

  boolean confirmed = false;                                //Both used to confirm button push to select mode
  boolean pressed = false;                                  // Used to know when done inputing points
  encoderPos = 0;                                           //Encoder starts from 0, first menu option
  numPoints = encoderPos+2;
  updatePickNumMenu(numPoints);
  while (!confirmed)                                        //While the user has not confirmed the selection
  {
    byte buttonState = digitalRead (encButton);
    numPoints = encoderPos+2;
    if (buttonState != oldButtonState)
    {
      if (millis () - buttonPressTime >= debounceTime)      //Debounce button
      {
        buttonPressTime = millis ();                        //Time when button was pushed
        oldButtonState =  buttonState;                      //Remember button state for next time
        if (buttonState == LOW)
        {
          modeSelected = encoderPos;                        //If the button is pressed, accept the current digit into the guessed code
          pressed = true;
          //Serial.println("Button Pushed");
        }
        else
        {
          if (pressed == true)                              //Confirm the input once the button is released again
          {
            confirmed = true;
            //Serial.println("Mode confirmed");
          }
        }
      }
    }
    if (encoderPos != prevEncoderPos)                       //Update the display if the encoder position has changed
    {
      updatePickNumMenu(numPoints);
      prevEncoderPos = encoderPos;
    }
  }
  numPoints = encoderPos+2;
  confirmed = false;
  pickedNumPoints();
  delay(1000);
  encoderPos = 0;

  Serial.println(numPoints);
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

void setupMenu()                               //Display while setting up
{
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Setting Up");
  lcd.setCursor(2, 1);
  lcd.print("And Zeroing");
}

void backToMain()                               //Display while returning to Main Menu
{ // Luke worked on this
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Returning To");
  lcd.setCursor(2, 1);
  lcd.print("Main Menu");
}

void pickedAPoint()                               //Display after picking a point
{ // Luke worked on this
  lcd.clear();
  lcd.print("Picked a Point");
}

void pickedATime()                               //Display after picking a time
{ // Luke worked on this
  lcd.clear();
  lcd.print("Picked a Time");
}

void pickedNumPoints()                               //Display after picking number of points
{ // Luke worked on this
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Picked Num");
  lcd.setCursor(1, 1);
  lcd.print("Points");
}

void updateMainMenu()                               //Updates the display data for the main menu
{ // Luke refined this
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Free Motion");
  lcd.setCursor(2, 1);
  lcd.print("Object Track");

  int selected = 0;                                 //Stores cursor vertical position to show selected item
  if (encoderPos == 0)
    selected = 0;
  else
    selected = 1;
  lcd.setCursor(0, encoderPos);                   //Set the display cursor position
  lcd.print(">");
}


void updatePickMenu(int i)                               //Updates the display data for the pick menu
{ // Luke worked on this
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Pick Pos ");
  lcd.print(i);
  lcd.print("/");
  lcd.print(numPoints);
  lcd.setCursor(2, 1);
  lcd.print("Main Menu");

  int selected = 0;                                 //Stores cursor vertical position to show selected item
  if (encoderPos == 0)
    selected = 0;
  else
    selected = 1;
  lcd.setCursor(0, encoderPos);                   //Set the display cursor position
  lcd.print(">");
}

void updatePickTimeMenu(float timeValue)                               //Updates the display data for the pick time menu
{ // Luke worked on this
  lcd.clear();
  lcd.print("Runtime(s): ");
  lcd.print(timeValue/1000);
}

void updatePickNumMenu(float numPointsPicked)                               //Updates the display data for the pick shots menu
{ // Luke worked on this
  lcd.clear();
  lcd.print("Num Points: ");
  lcd.print(numPointsPicked);
}

void updateGateMenu()                               //Updates the display data for the pick menu
{ // Luke worked on this
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Begin");
  lcd.setCursor(2, 1);
  lcd.print("Main Menu");

  int selected = 0;                                 //Stores cursor vertical position to show selected item
  if (encoderPos == 0)
    selected = 0;
  else
    selected = 1;
  lcd.setCursor(0, encoderPos);                   //Set the display cursor position
  lcd.print(">");
}

void doneMotion()                               //Updates the display data when motion is done
{ // Luke worked on this
  lcd.clear();
  lcd.print("Done Moving");
}      


void waitForConfirmation() { // Luke worked on this
  encLowLim = 0;                                            //Mode selection menu, 2 modes
  encHighLim = 1;
  encIncrement = 1;
  updateGateMenu();

  boolean confirmed = false;                                //Both used to confirm button push to select mode
  boolean pressed = false;
  backToMenu = false;
  
  encoderPos = 0;                                           //Encoder starts from 0, first menu option
  int counter = 0;
  while (!confirmed)                                        //While the user has not confirmed the selection
  {
    byte buttonState = digitalRead (encButton);
    if (buttonState != oldButtonState)
    {
      if (millis () - buttonPressTime >= debounceTime)      //Debounce button
      {
        buttonPressTime = millis ();                        //Time when button was pushed
        oldButtonState =  buttonState;                      //Remember button state for next time
        if (buttonState == LOW)
        {
          modeSelected = encoderPos;                        //If the button is pressed, accept the current digit into the guessed code
          pressed = true;
          Serial.println("Button Pushed");
        }
        else
        {
          if (pressed == true)                              //Confirm the input once the button is released again
          {
            confirmed = true;
            Serial.println("Mode confirmed");
          }
        }
      }
    }
    if (encoderPos != prevEncoderPos)                       //Update the display if the encoder position has changed
    {
      updateGateMenu();
      prevEncoderPos = encoderPos;
    }
  }
  Serial.println("Mode selected");
  if (modeSelected == 0) {                                   //Run required mode function depending on selection
    Serial.println("Confirmed");
    confirmed = false;
    updateGateMenu();
  }
  else {
    Serial.println("Back");
    backToMain();
    delay(1000);
    backToMenu = true;
  }
}

void Motion ()                                       //Runs the pan and rotate mode sequence
{ // Luke worked on this
    pinMode(enablePin, INPUT);
    pickNumPoints();
    inputPanAndRotateData();                                   //Get user inputs for pan movement
    pinMode(enablePin, OUTPUT);
    
    float travToSteps = 800; // Rotations per Step, 0.216 Degres per Step
    float panToSteps = 13; // Degrees per Step

    if (backToMenu == false){
      for (int i = 0; i < numPoints; i = i + 1) {
        lcd.clear();
        lcd.setCursor(2, 0);
        lcd.print("Moving to");
        lcd.setCursor(2, 1);
        lcd.print("Point ");
        lcd.print(i+1);
        
        Serial.print("Moving to point ");
        Serial.println(i+1);
        
        float travCommand = travPoints[i];
        float panCommand = panPoints[i];
        int timeDelta = timePoints[i];

        Serial.print("Trav Command is ");
        Serial.println(travCommand);
        Serial.print("Pan Command is ");
        Serial.println(panCommand);
        
        float startTrav = getTrav(); 
        float startPan = getPan();

        if (runTrack){
          movetrack(panCommand, travCommand, timeDelta); // run object tracking version of move code      
        } else{
          movetospot(panCommand, travCommand, timeDelta); // run free motion version of move code
        }
  
        Serial.print("Trav Error ");
        Serial.println(abs(getTrav() - travCommand));
        Serial.print("Pan Error ");
        Serial.println(abs(getPan() - panCommand));
  
        Serial.print("At point ");
        Serial.println(i+1);
  
        if (i+1 == 1){
          waitForConfirmation();
          if (backToMenu == true){
            break;
          }
        }
      }
    }

    if (backToMenu == false){
      doneMotion();
      delay(1000);
    }
}

// Motor and Positioning Functions

void setZero(){ // Luke worked on this
  MP.selectChannel(0);
  float zero = tEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;
  tEncoder.setOffset(-zero);
  travRots = 0;
  travO = 0;
}

void goToZero(){ // Luke worked on this
  while (inZero == false){
    unsigned long cTime = millis();
    moveMotor(0, -2, 6);
    if (cTime - oTime >= 50){
      oTime = cTime;
    }
  }
  moveMotor(0, 100, 50);
  setZero();
}

float objTrack(float ang1, float ang2, float pos1, float pos2, float intPos) { // Luke did some work on this
  ang1 = radians(ang1);
  ang2 = radians(ang2);
  float len = pos2 - pos1;
  float x1 = -len / (tan(ang1) * (cos(ang2) / sin(ang2)) - 1); // cotan is cos on sin
  float y1 =  len / ( cos(ang1) / sin(ang1) - cos(ang2) / sin(ang2));
  float TargAngle = degrees(atan(y1 / (x1 - intPos)));
  float result;
  Serial.println(TargAngle);
  if (TargAngle < 0 and 0 < ang1) {
    result = TargAngle + 180;
  } else if (TargAngle > 0 and 0 > ang1) {
    result = TargAngle - 180;
  } else {
    result = TargAngle;
  }
  return result;
}

// blindly move motor w/o control

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

// move to a spot with control

void movetospot(float endRot, float endPos, unsigned long Time) { // Luke refined this

  double currentPos, currentRot, rotError1, posError1, rotError2, posError2;
  int rotStep, posStep;

  float steptime = 150;
  float travRotPerStep = (1/1600) * 2;
  float travStepPerRot = 800;
  float rotDegPerStep = (1/13) * 2;
  float rotStepPerDeg = 13/2;
  float targetPos, targetRot;
  double kpr = 3;
  double kpp = 3;
  double kir = 0.0;
  double kip = 0.0;
  double kdr = 2;
  double kdp = 2;
  unsigned long previousTime;
  unsigned long elapsedTime;
  unsigned long currentTime;
  double rotcumError, poscumError, rotDeltaError, posDeltaError, rotOut, posOut;

  float startRot = getPan();
  float startPos = getTrav();
  float startTime = millis();

  float lastPercDone = 0;
  float percDone = 0;

  bool travSlower = false;

  currentRot = startRot;
  currentPos = startPos;

  float RotSteps = abs((endRot - startRot) * rotStepPerDeg);
  float TravSteps = abs((endPos - startPos) * travStepPerRot);

  Serial.print("RotSteps ");
  Serial.println(RotSteps);
  Serial.print("TravSteps ");
  Serial.println(TravSteps);

  if (TravSteps >= RotSteps){
    travSlower = true;
  } else{
    travSlower = false;
  }

  while (abs(endRot - currentRot) >= 0.5 or abs(endPos - currentPos) >= .5/360) {
    currentTime = millis();
    elapsedTime = (double)(currentTime - previousTime);

    currentRot = getPan();
    currentPos = getTrav();

    float travPerc = (currentPos-startPos)/(endPos-startPos);
    float panPerc = (currentRot-startRot)/(endRot-startRot);

    if (currentTime - startTime <= Time){
      if (travSlower) {
        targetPos = startPos + ((currentTime-startTime)/Time) * (endPos - startPos);       
      } else{
        targetRot = startRot + ((currentTime-startTime)/Time) * (endRot - startRot);
      }

    } else{
      if (travSlower){
        targetPos = endPos;       
      } else{
        targetRot = endRot;
      }
    }

    if (travSlower){
      percDone = (currentPos-startPos)/(endPos-startPos);
      float percToGo = percDone + (percDone - lastPercDone);
      if (percToGo > 1){
        percToGo = 1;
      }
      Serial.println(percToGo);
      targetRot = startRot + (endRot - startRot)*percToGo;
    
      lastPercDone = percDone;
    } else{
      percDone = (currentRot-startRot)/(endRot-startRot);
      float percToGo = percDone + (percDone - lastPercDone);
      if (percToGo > 1){
        percToGo = 1;
      }
      Serial.println(percToGo);
      targetPos = startPos + (endPos - startPos)*percToGo;
    
      lastPercDone = percDone;
    }


    rotError1 = targetRot - currentRot;
    posError1 = targetPos - currentPos;

    rotcumError += rotError1;
    poscumError += posError1;

    rotDeltaError = (rotError1 - rotError2) / elapsedTime;
    posDeltaError = (posError1 - posError2) / elapsedTime;

    rotOut = (kir * rotcumError) + (kpr * rotError1) + (kdr * rotDeltaError);
    posOut = (kip * poscumError) + (kpp * posError1) + (kdp * posDeltaError);

    rotStep = int(rotOut * 13);
    posStep = int(posOut * 1600);

    previousTime = currentTime;
    
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

    float interval1;
    float interval2;
    unsigned long oldMotor1Time = millis();
    unsigned long oldMotor2Time = millis();
    unsigned long currentMotor1Time = millis();
    unsigned long currentMotor2Time = millis();

    interval1 = (Time / (rotStep) );
    interval2 = (Time / (posStep) );

    unsigned long offset1;
    unsigned long offset2;

    rotError2 = rotError1;
    posError2 = posError2;

    // change direction
    
    while (currentMotor1Time - oldMotor1Time < steptime) {
      currentMotor1Time = millis();
      currentMotor2Time = millis();

      if ((currentMotor1Time - offset1) > (interval1 / 2)) {
        step1();
        offset1 = currentMotor1Time;
      }

      if ((currentMotor2Time - offset2) > (interval2 / 2)) {
        step2();
        offset2 = currentMotor2Time;
      }
    }
  }
}

// move to a spot with control while tracking an object

void movetrack(float endRot, float endPos, unsigned long Time) { // Luke worked on this (just modified movetospot

  double currentPos, currentRot, rotError1, posError1, rotError2, posError2;
  int rotStep, posStep;

  float steptime = 150;
  float travRotPerStep = (1/1600) * 2;
  float travStepPerRot = 800;
  float rotDegPerStep = (1/13) * 2;
  float rotStepPerDeg = 13/2;
  float targetPos, targetRot;
  double kpr = 3; // Luke refined some constant values
  double kpp = 3;
  double kir = 0.0;  // Luke tried adding integral, but due to being position error integral causes overshoot
  double kip = 0.0;
  double kdr = 2;
  double kdp = 2;
  unsigned long previousTime;
  unsigned long elapsedTime;
  unsigned long currentTime;
  double rotcumError, poscumError, rotDeltaError, posDeltaError, rotOut, posOut;

  float startRot = getPan();
  float startPos = getTrav();
  float startTime = millis();

  float lastPercDone = 0;
  float percDone = 0;

  bool travSlower = false;

  currentRot = startRot;
  currentPos = startPos;

  float RotSteps = abs((endRot - startRot) * rotStepPerDeg);
  float TravSteps = abs((endPos - startPos) * travStepPerRot);

  Serial.print("RotSteps ");
  Serial.println(RotSteps);
  Serial.print("TravSteps ");
  Serial.println(TravSteps);

  while (abs(endRot - currentRot) >= 0.5 or abs(endPos - currentPos) >= .5/360) {
    currentTime = millis();
    elapsedTime = (double)(currentTime - previousTime);

    currentRot = getPan();
    currentPos = getTrav();

    float travPerc = (currentPos-startPos)/(endPos-startPos);

    if (currentTime - startTime <= Time){
      targetPos = startPos + ((currentTime-startTime)/Time) * (endPos - startPos);       
    } else{
      targetPos = endPos;       
    }
    
    percDone = (currentPos-startPos)/(endPos-startPos);
    
    float percToGo = percDone + (percDone - lastPercDone);
    if (percToGo > 1){
      percToGo = 1;
    }
    Serial.println(percToGo);

    float trackAngle = objTrack(startRot, endRot, startPos, endPos, percToGo*(endPos-startPos)+startPos);
    
    targetRot = trackAngle;
    
    lastPercDone = percDone;

    rotError1 = targetRot - currentRot;
    posError1 = targetPos - currentPos;

    rotcumError += rotError1;
    poscumError += posError1;

    rotDeltaError = (rotError1 - rotError2) / elapsedTime;
    posDeltaError = (posError1 - posError2) / elapsedTime;

    rotOut = (kir * rotcumError) + (kpr * rotError1) + (kdr * rotDeltaError);
    posOut = (kip * poscumError) + (kpp * posError1) + (kdp * posDeltaError);

    rotStep = int(rotOut * 13);
    posStep = int(posOut * 1600);

    previousTime = currentTime;
    
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

    float interval1;
    float interval2;
    unsigned long oldMotor1Time = millis();
    unsigned long oldMotor2Time = millis();
    unsigned long currentMotor1Time = millis();
    unsigned long currentMotor2Time = millis();

    interval1 = (Time / (rotStep) );
    interval2 = (Time / (posStep) );

    unsigned long offset1;
    unsigned long offset2;

    rotError2 = rotError1;
    posError2 = posError2;

    // change direction
    
    while (currentMotor1Time - oldMotor1Time < steptime) {
      currentMotor1Time = millis();
      currentMotor2Time = millis();

      if ((currentMotor1Time - offset1) > (interval1 / 2)) {
        step1();
        offset1 = currentMotor1Time;
      }

      if ((currentMotor2Time - offset2) > (interval2 / 2)) {
        step2();
        offset2 = currentMotor2Time;
      }
    }
  }
}

// Sensor Functions

float getPan(){  // Luke refined this
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

float getTrav(){ // Luke refined this
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

void hitZero() {  // Luke refined this
  // ISR, deal with hitting zero limit switch
  cli();
  if (millis() - oTimeHit >= debounceTime) {
    oTimeHit = millis();
    inZero = true;
  }
  sei();
}
