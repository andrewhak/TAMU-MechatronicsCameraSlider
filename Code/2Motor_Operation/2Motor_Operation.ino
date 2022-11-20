#define enablePin 26                         //Define motor enable pin
#define travDirPin 25                        //Define travel & rotation stepper motor pins
#define travStepPin 24
#define rotDirPin 23
#define rotStepPin 22



//int steps1 = 16 * 50;
//int steps2 = 16 * 25;
//
//float timemilsec = 10 * 1000;




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
  moveMotor(200, 50, 1000);
  delay(1000);

  moveMotor(-200, -50, 1000);

}

void moveMotor(int steps1, int steps2, unsigned long timemilsec) {
  // put your main code here, to run repeatedly:
  bool travstate = false;
  bool rotstate = false;
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
