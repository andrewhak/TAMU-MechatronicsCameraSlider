#define enablePin 26                         //Define motor enable pin
#define travDirPin 25                        //Define travel & rotation stepper motor pins
#define travStepPin 24
#define rotDirPin 23
#define rotStepPin 22



//int steps1 = 16 * 50;
//int steps2 = 16 * 25;
//
//float timemilsec = 10 * 1000;


  int i = 0;
  
  int j = 0;


void setup() {
  //Open circuit enable pin, disables motors
  Serial.begin(9600);
  pinMode(travDirPin, OUTPUT);                            //Define the travel stepper motor pins
  pinMode(travStepPin, OUTPUT);
  pinMode(rotDirPin, OUTPUT);                             //Define the rotation stepper motor pins
  pinMode(rotStepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);


  digitalWrite(travStepPin, LOW);
  digitalWrite(rotStepPin, LOW);

}
void loop() {
  moveMotor(400, 3200, 10000);
  delay(1000);
  moveMotor(-400, -3200, 10000);

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
  Serial.println(timemilsec);
  interval1 = (timemilsec / steps1);
  interval2 = (timemilsec / steps2);
  Serial.println(interval1);
  Serial.println(interval2);
  offset1 = 0;
  offset2 = 0;

  while (curstep1 < steps1 or curstep2 < steps2) {
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
