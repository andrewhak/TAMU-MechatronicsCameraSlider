#define enablePin 26                         //Define motor enable pin
#define travDirPin 23                        //Define travel & rotation stepper motor pins
#define travStepPin 22
#define rotDirPin 25
#define rotStepPin 24

#define initialDur 120 

float travTime = initialDur;  

int travelPulses = 100;
void setup() {
  // put your setup code here, to run once:

  pinMode(enablePin, INPUT);                              //Open circuit enable pin, disables motors
  pinMode(travDirPin, OUTPUT);                            //Define the travel stepper motor pins
  pinMode(travStepPin, OUTPUT);
  pinMode(rotDirPin, OUTPUT);                             //Define the rotation stepper motor pins
  pinMode(rotStepPin, OUTPUT);

}

void loop() {
  
  pinMode(enablePin, OUTPUT);

  float interval = calcInterval (travelPulses);         //Calculate the pulse interval required to move the required distance in the required time

  // put your main code here, to run repeatedly:
  for (int i = 1; i <= travelPulses; i++)               //Pulse the motor to move the required distance in the required time
  {
    digitalWrite(travStepPin, HIGH);
    delayMicroseconds(interval / 2);
    digitalWrite(travStepPin, LOW);
    delayMicroseconds(interval / 2);
  }

  delay(1000);

  for (int i = 1; i <= travelPulses; i++)               //Pulse the motor to move the required distance in the required time
  {
    digitalWrite(rotStepPin, HIGH);
    delayMicroseconds(interval / 2);
    digitalWrite(rotStepPin, LOW);
    delayMicroseconds(interval / 2);
  }




}
float calcInterval (int numPulses)                                 //Calculate the interval required between pulses to achieve duration
{
  float inter = travTime * 1000000 / numPulses;
  return inter;
}

float calcRotInterval (int numPulses)                              //Calculate the interval required between pulses to achieve duration
{
  float inter = travTime * 1000 / numPulses;
  return inter;
}
