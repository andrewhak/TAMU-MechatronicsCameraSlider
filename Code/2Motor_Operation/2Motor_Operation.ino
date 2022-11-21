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
  moveMotor(250, 200, 1000);
  delay(1000);
  moveMotor(-250, -200, 1000);

}

void moveMotor(float steps1, float steps2, float timemilsec) {
  // put your main code here, to run repeatedly:
  bool travstate = false;
  bool rotstate = false;
  float interval1;
  float interval2;

  float offset1;
  float offset2;

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
  interval1 = timemilsec / steps1;
  interval2 = timemilsec / steps2;
  Serial.println(interval1);
  Serial.println(interval2);
  offset1 = 0;
  offset2 = 0;

  while (curstep1 < steps1 or curstep2 < steps2) {
//    Serial.println("");
//    Serial.println(millis());
//    Serial.println(millis() - offset1);
//    Serial.println(interval1 / 2);
//    Serial.println(millis() - offset2);
//    Serial.println(interval2 / 2);
//    Serial.println("");

    if ((float(millis()) - float(offset1)) > ((10^3)*interval1 / 2) and  rotstate == false and curstep1 < steps1) {
      digitalWrite(rotStepPin, HIGH);
      offset1 = millis();
      curstep1++;
      rotstate = true;
    }

    if ((float(millis()) - float(offset2)) > ((10^3)*interval2 / 2) and  travstate == false and curstep2 < steps2) {
      digitalWrite(travStepPin, HIGH);
      offset2 = millis();
      curstep2++;
      travstate = true;

    }

    if ((float(millis()) - float(offset1)) > ((10^3)*interval1 / 2) and rotstate == true and curstep1 < steps1) {
      digitalWrite(rotStepPin, LOW);
      offset1 = millis();
      rotstate = false;

    }


    if ((float(millis()) - float(offset2)) > ((10^3)*interval2 / 2) and travstate == true and curstep2 < steps2) {
      digitalWrite(travStepPin, LOW);
      offset2 = millis();
      travstate = false;

    }

  }

}
