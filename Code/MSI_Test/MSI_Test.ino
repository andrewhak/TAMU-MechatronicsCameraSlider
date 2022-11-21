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

// Motor Stuff

#define enablePin 26                         //Define motor enable pin
#define travDirPin 25                        //Define travel & rotation stepper motor pins
#define travStepPin 24
#define rotDirPin 23
#define rotStepPin 22
#define initialDur 120 

bool travstate = false;
bool rotstate = false;

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
  Serial.println(getTrav());
  Serial.println("Travel Zero'd");
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
    Serial.println("run free motion");
    FreeMotion ();
  }
  else {
    Serial.println("run object track");
    runTrack ();
  }
}

// Interface Functions

void inputPanAndRotateData(){
  encLowLim = 0;                                            //Mode selection menu, 2 modes
  encHighLim = 1;
  encIncrement = 1;
  updateFreePickMenu();

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
        updateFreePickMenu();
        prevEncoderPos = encoderPos;
      }
      travPoints[counter] = getTrav(); 
      panPoints[counter]= getPan();
    }
    Serial.println("Mode selected");
    if (modeSelected == 0) {                                   //Run required mode function depending on selection
      Serial.println("Picked a Point");
      confirmed = false;
      
      travPoints[counter] = getTrav(); 
      panPoints[counter]= getPan();
      counter ++;

      pickedAPoint();
      delay(1000);
      updateFreePickMenu();
            
      if (counter > 1){
        finishedPicking = true;
        Serial.println("Done Picking");
      }
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


void updateFreePickMenu()                               //Updates the display data for the pick menu
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
    // hardcode time delta as 100

    int timeDelta = 100;
    float travToSteps = 1600; // Rotations per Step, 0.216 Degres per Step
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
        
        float startTrav = getTrav(); 
        float startPan = getPan();
        
        int travSteps = int((travCommand - startTrav)*travToSteps);
        int panSteps = int((panCommand - startPan)*panToSteps);
  
        moveMotor(panSteps, travSteps, timeDelta);
  
        Serial.print("Steps per Trav Rotate ");
        Serial.println(travSteps / (getTrav() - startTrav));
        Serial.print("Steps per Pan Degree");
        Serial.println(panSteps / (getPan() - startPan));
  
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
      lcd.clear();
      lcd.setCursor(2, 0);
      lcd.print("Done Moving");
      lcd.setCursor(2, 1);
      lcd.print(" ");
  
      delay(1000);
  
      Serial.println("Done Following Path");
    }
}

void runTrack ()                                                //Runs the object tracking mode sequence
{
  //  inputTrackData ();                                             //Get user inputs for tracking movement
  //  displayStart ();
  //  display.setCursor(22,30);
  //  display.print(F("Object Tracking"));
  //  display.display();
  //  if (travelDir == 0)                                             //Set motor travel and rotate directions
  //  {
  //    digitalWrite(travDirPin, LOW);
  //    digitalWrite(rotDirPin, LOW);
  //  }
  //  else
  //  {
  //    digitalWrite(travDirPin, HIGH);
  //    digitalWrite(rotDirPin, HIGH);
  //  }
  //  int travelPulses = calcTravelPulses ();                       //Calculates the number of pulses required to move the travel distance
  //  Serial.print("Travel pulses: ");
  //  Serial.println(travelPulses);
  //  float interval = calcInterval (travelPulses);                 //Calculates the interval required to achieve the required duration
  //  Serial.print("Interval: ");
  //  Serial.println(interval);
  //  currentAngle = atan((objDist)/(travDist/2))*180/M_PI;         //Calculates the initial camera to object angle
  //  Serial.print("Current Angle: ");
  //  Serial.println(currentAngle);
  //  for (int i=1; i<=(travelPulses/2); i++)                           //Runs through movement sequence to move motors
  //  {
  //      digitalWrite(travStepPin, HIGH);
  //      boolean rotatePulse = checkRot (i);
  //      if (rotatePulse == true)
  //        digitalWrite(rotStepPin, HIGH);
  //      delayMicroseconds(interval/2);
  //      digitalWrite(travStepPin, LOW);
  //      if (rotatePulse == true)
  //        digitalWrite(rotStepPin, LOW);
  //      delayMicroseconds(interval/2);
  //      currentDist = i/pulsesPerMM;
  //      /*Serial.print("Dist: ");
  //      Serial.println(currentDist);
  //      Serial.print("Angle: ");
  //      Serial.println(currentAngle);*/
  //  }
  //  currentAngle = 90;
  //  for (int i=((travelPulses/2)+1); i<=travelPulses; i++)                           //Runs through movement sequence to move motors
  //  {
  //      digitalWrite(travStepPin, HIGH);
  //      boolean rotatePulse = checkRot (i);
  //      if (rotatePulse == true)
  //        digitalWrite(rotStepPin, HIGH);
  //      delayMicroseconds(interval/2);
  //      digitalWrite(travStepPin, LOW);
  //      if (rotatePulse == true)
  //        digitalWrite(rotStepPin, LOW);
  //      delayMicroseconds(interval/2);
  //      currentDist = i/pulsesPerMM;
  //      /*Serial.print("Dist: ");
  //      Serial.println(currentDist);
  //      Serial.print("Angle: ");
  //      Serial.println(currentAngle);*/
  //  }
  //  displayEnd();                                           //Display the end sequence and disable motors
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
      Serial.println(getTrav());
      oTime = cTime;
    }
  }
  Serial.println(getTrav());
  moveMotor(0, 100, 50);
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
    getTrav(); // keep ecounters counting. FIX THIS ISSUE OTHER WAY!
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
