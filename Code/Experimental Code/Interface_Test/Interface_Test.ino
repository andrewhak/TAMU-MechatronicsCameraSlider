#include <LiquidCrystal.h>
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
const unsigned long debounceTime = 10;      //Debounce delay time
unsigned long buttonPressTime;              //Time button has been pressed for debounce

int encLowLim = 0;                          //Variables to store the encoder limits and increment
int encHighLim = 1;
int encIncrement = 1;
int dataInputNo = 0;
int modeSelected = 0;                       //Current operating mode (Pan, Rotate, Pan & Rotate, Track Object)


#define enablePin 26                         //Define motor enable pin
#define travDirPin 23                        //Define travel & rotation stepper motor pins
#define travStepPin 22
#define rotDirPin 25
#define rotStepPin 24

void setup() {
  Serial.begin(9600);                                     //Start serial communication for debugging
  lcd.begin(16, 2); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display }

  pinMode(pinA, INPUT_PULLUP);                            //Set pinA as an input, pulled HIGH to the logic voltage
  pinMode(pinB, INPUT_PULLUP);                            //Set pinB as an input, pulled HIGH to the logic voltage
  attachInterrupt(0, PinA, RISING);                       //Set an interrupt on PinA
  attachInterrupt(1, PinB, RISING);                       //Set an interrupt on PinB
  pinMode (encButton, INPUT_PULLUP);                      //Set the encoder button as an input, pulled HIGH to the logic voltage
  pinMode(enablePin, INPUT);                              //Open circuit enable pin, disables motors
  pinMode(travDirPin, OUTPUT);                            //Define the travel stepper motor pins
  pinMode(travStepPin, OUTPUT);
  pinMode(rotDirPin, OUTPUT);                             //Define the rotation stepper motor pins
  pinMode(rotStepPin, OUTPUT);
  digitalWrite(travDirPin, HIGH);                         //Set the initial direction of motion for both motors
  digitalWrite(rotDirPin, HIGH);
  lcd.clear();                                 //Clear the display

  //  lcd.clear();
  //  lcd.setCursor(2, 0);
  //  lcd.print("Free Motion");
  //  lcd.setCursor(2, 1);
  //  lcd.print("Object Track");

  Serial.println("Setup complete");                       //Write to serial monitor to indicate the setup function is complete

}

void loop() {
  // put your main code here, to run repeatedly:
  encLowLim = 0;                                            //Mode selection menu, 4 modes
  encHighLim = 1;
  encIncrement = 1;
  updateMainMenu();

  boolean confirmed = false;                                //Both used to confirm button push to select mode
  boolean pressed = false;
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
    runTrack ();
    Serial.println("run object track");
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

void FreeMotion ()                                       //Runs the pan and rotate mode sequence
{
  //  inputPanAndRotateData ();                                   //Get user inputs for pan movement
  //  displayStart ();                                            //Display startup sequence and enable motors
  //  display.setCursor(30,30);
  //  display.print(F("Pan & Rotate"));
  //  display.display();
  //  if (travelDir == 0)                                         //Set motor travel direction
  //    digitalWrite(travDirPin, LOW);
  //  else
  //    digitalWrite(travDirPin, HIGH);
  //  if (rotDir == 0)                                            //Set motor travel direction
  //    digitalWrite(rotDirPin, HIGH);
  //  else
  //    digitalWrite(rotDirPin, LOW);
  //  int travelPulses = calcTravelPulses ();                     //Calculate the number of motor pulses required to move the travel distance
  //  Serial.print("Travel pulses: ");
  //  Serial.println(travelPulses);
  //  float interval = calcInterval (travelPulses);               //Calculate the pulse interval required to move the required distance in the required time
  //  Serial.print("Interval: ");
  //  Serial.println(interval);
  //  int rotationPulses = calcRotationPulses ();                 //Calculate the number of motor pulses required to rotate the required angle
  //  Serial.print("Rotation pulses: ");
  //  Serial.println(rotationPulses);
  //  int travelPerRotation = travelPulses/rotationPulses;        //Calculate how much the camera should pan for each rotation step
  //  for (int i=1; i<=travelPulses; i++)
  //  {
  //      digitalWrite(travStepPin, HIGH);
  //      int checkRotate = i % travelPerRotation;                //Check if a rotation step must be made
  //      if (checkRotate == 0)
  //        digitalWrite(rotStepPin, HIGH);
  //      delayMicroseconds(interval/2);
  //      digitalWrite(travStepPin, LOW);
  //      if (checkRotate == 0)
  //        digitalWrite(rotStepPin, LOW);
  //      delayMicroseconds(interval/2);
  //      /*currentDist = i/pulsesPerMM;
  //      currentAngle = i/pulsesPerDeg;
  //      Serial.print("Dist: ");
  //      Serial.println(currentDist);
  //      Serial.print("Angle: ");
  //      Serial.println(currentAngle);*/
  //  }
  //  displayEnd();                                                 //Display the end sequence and disable motors
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
