#define enablePin 26                         //Define motor enable pin
#define travDirPin 25                        //Define travel & rotation stepper motor pins
#define travStepPin 24
#define rotDirPin 23
#define rotStepPin 22

bool travstate = false;
bool rotstate = false;

int steps1 = 16 * 200;
int steps2 = 16 * 100;

float timemilsec = 1 * 1000;

float interval1 = 0;
float interval2 = 0;

unsigned long curint1time;
unsigned long offset1;

unsigned long elapTime;
int curstep1 = 0;
int maxsteps = 0;

void setup() {
  //Open circuit enable pin, disables motors
  Serial.begin(9600);
  pinMode(travDirPin, OUTPUT);                            //Define the travel stepper motor pins
  pinMode(travStepPin, OUTPUT);
  pinMode(rotDirPin, OUTPUT);                             //Define the rotation stepper motor pins
  pinMode(rotStepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  interval1 = timemilsec / steps1;
  interval2 = timemilsec / steps2;
  digitalWrite(travStepPin, LOW);
  digitalWrite(rotStepPin, LOW);
  if (steps1 > steps2) {
    maxsteps = steps1;

  } else {
    maxsteps = steps2;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (curstep1 < maxsteps) {
    elapTime = millis();
    curint1time = elapTime - offset1;
    Serial.println(curstep1);
    if (curstep1 < steps1) {
      if (curint1time > interval1 / 2 and  travstate == false) {
        digitalWrite(rotStepPin, HIGH);
        offset1 = elapTime;
        curstep1++;
        travstate = true;

      }
      if (curint1time > interval1 / 2 and travstate == true ) {
        digitalWrite(rotStepPin, LOW);
        offset1 = elapTime;
        travstate = false;
      }
    }
    
  }

}
