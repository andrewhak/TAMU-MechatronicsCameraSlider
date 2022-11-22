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
volatile float timePoints[2];

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

int i = 0;
int j = 0;

// Interface Stuff

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
  // not sure how essential it is so don't mess with
  
  Serial.begin(115200);

  // Interface Stuff

  lcd.begin(16, 2); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display }

  pinMode(pinA, INPUT_PULLUP);                            //Set pinA as an input, pulled HIGH to the logic voltage
  pinMode(pinB, INPUT_PULLUP);                            //Set pinB as an input, pulled HIGH to the logic voltage
  attachInterrupt(0, PinA, RISING);                       //Set an interrupt on PinA
  attachInterrupt(1, PinB, RISING);                       //Set an interrupt on PinB
  pinMode (encButton, INPUT_PULLUP);                      //Set the encoder button as an input, pulled HIGH to the logic voltage
  pinMode(enablePin, INPUT);                              //Open circuit enable pin, disables motors
  
  lcd.clear();                                 //Clear the display

  setupMenu();

  //  lcd.clear();
  //  lcd.setCursor(2, 0);
  //  lcd.print("Free Motion");
  //  lcd.setCursor(2, 1);
  //  lcd.print("Object Track");

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
  Serial.println("Zero'd");
  Serial.println("Setup Done");
}

void loop() {
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
    Serial.println("Run free motion");
    FreeMotion ();
  }
  else {
    Serial.println("Run object track");
    runTrack ();
  }
}

// Interface Functions

void inputPanAndRotateData(){
  encLowLim = 0;                                            //Mode selection menu, 2 modes
  encHighLim = 1;
  encIncrement = 1;
  updatePickMenu();

  boolean confirmed = false;                                //Both used to confirm button push to select mode
  boolean pressed = false;
  boolean finishedPicking = false;                          // Used to know when done inputing points
  encoderPos = 0;                                           //Encoder starts from 0, first menu option
  int counter = 0;
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
        updatePickMenu();
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
      updatePickMenu();
      
      if (counter >= 1){
        finishedPicking = true;
        Serial.println("Done Picking");
      }
      counter ++;
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

void pickTimes(int i){
  encLowLim = 0;                                            //Mode selection menu, 50 modes
  encHighLim = 76;
  encIncrement = 1;

  boolean confirmed = false;                                //Both used to confirm button push to select mode
  boolean pressed = false;                                  // Used to know when done inputing points
  encoderPos = 0;                                           //Encoder starts from 0, first menu option
  float timeValue = 1000 + encoderPos * 250;
  updatePickTimeMenu(timeValue);
  int counter = 0;
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
{
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Returning To");
  lcd.setCursor(2, 1);
  lcd.print("Main Menu");
}

void pickedAPoint()                               //Display after picking a point
{
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Picked a Point");
  lcd.setCursor(2, 1);
  lcd.print("");
}

void pickedATime()                               //Display after picking a time
{
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Picked a Time");
  lcd.setCursor(2, 1);
  lcd.print("");
}

void updateMainMenu()                               //Updates the display data for the main menu
{
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


void updatePickMenu()                               //Updates the display data for the pick menu
{
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Input Position");
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

void updatePickTimeMenu(float timeValue)                               //Updates the display data for the pick menu
{
  lcd.clear();
  lcd.print("Runtime(s): ");
  lcd.print(timeValue/1000);
}

void updateGateMenu()                               //Updates the display data for the pick menu
{
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
{
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Done Moving");
  lcd.setCursor(2, 1);
  lcd.print(" ");
}      


void waitForConfirmation() {
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

void FreeMotion ()                                       //Runs the pan and rotate mode sequence
{
    pinMode(enablePin, INPUT);
    inputPanAndRotateData ();                                   //Get user inputs for pan movement
    pinMode(enablePin, OUTPUT);
    
    float travToSteps = 800; // Rotations per Step, 0.216 Degres per Step
    float panToSteps = 13; // Degrees per Step

    if (backToMenu == false){
      for (int i = 0; i < 2; i = i + 1) {
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
  
        movetospot(panCommand, travCommand, timeDelta);
  
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

void runTrack ()                                                //Runs the object tracking mode sequence
{
 // Fill in later
}

// Motor and Positioning Functions

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
    moveMotor(0, -2, 6);
    if (cTime - oTime >= 50){
      oTime = cTime;
    }
  }
  moveMotor(0, 100, 50);
  setZero();
}

float objTrack(float ang1, float ang2, float pos1, float pos2, float intPos){
  float angToMove = ang2-ang1;
  float len = pos2-pos1;
  float x1 = -len / (tan(ang1)*(cos(ang2)/sin(ang2))-1); // cotan is cos on sin
  float y1 =  len / ( cos(ang1)/sin(ang1) - cos(ang2)/sin(ang2));
  float TargAngle = atan(y1 / (x1-intPos));
  return TargAngle;
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

void movetospot(float targetRot, float targetPos, unsigned long Time) {

  double currentPos, currentRot, rotError, posError;
  int rotStep, posStep;

  float steptime = 50;
  float travRotPerStep = 0.0006 * 2;
  float rotDegPerStep = 0.07 * 2;
  double kpr = .5;
  double kpp = 3;
  double kir = 0;
  double kip = 0;
  unsigned long previousTime;
  unsigned long elapsedTime;
  unsigned long currentTime;
  double rotcumError, poscumError, rotOut, posOut;

  rotError = targetRot - currentRot;
  posError = targetPos - currentPos;

  while (abs(rotError) >= 1 or abs(posError) >= .5) {
    currentTime = millis();
    elapsedTime = (double)(currentTime - previousTime);
    //Serial.println(elapsedTime);

    currentRot = getPan();
    currentPos = getTrav();

    rotError = targetRot - currentRot;
    posError = targetPos - currentPos;

    rotcumError += rotError * elapsedTime;
    poscumError += posError * elapsedTime;

    rotOut = (kir * rotcumError) + (kpr * rotError);
    posOut = (kip * poscumError) + (kpp * posError);
    
    //Serial.println(rotOut);
    //Serial.println(posOut);
    //    Serial.println(abs(rotError));
    //    Serial.println(abs(posError));

    rotStep = int(rotOut * 13);
    posStep = int(posOut * 1600);

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
    unsigned long currentMotor1Time = 0;
    unsigned long currentMotor2Time = 0;

    interval1 = (Time / (rotStep) );
    interval2 = (Time / (posStep) );

    unsigned long offset1;
    unsigned long offset2;

    // change direction

    previousTime = currentTime;
    while (currentMotor1Time < steptime) {
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
