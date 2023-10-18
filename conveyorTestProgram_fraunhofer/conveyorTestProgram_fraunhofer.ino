
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

// Motor 1 pins
const int motor1Step = 7;
const int motor1Direction = 6;
const int motor1Enable = 8;

// Motor 2 pins
const int motor2Step = 3;
const int motor2Direction = 2;
const int motor2Enable = 4;

// Distance constants
const int distance_1 = 1200;
const int distance_2 = 1000;
const int overlap = 400;

// Acceleration constants
const float accelRate1 = 0.00004;
const float accelRate2 = 0.00006;

// Motor speeds
float motor1Speed = 0;
float motor2Speed = 0;

bool motor1Running = false;
bool motor2Running = false;

bool enabled1 = false;
bool enabled2 = false;

unsigned long start_time1;
unsigned long start_time2;

unsigned long end_time1;
unsigned long end_time2;

float actMotor1Speed = 0;
float actMotor2Speed = 0;

int lastTrigger = 0;



// Function prototypes
void setup();
void loop();
void logic(int delay);
void controlMotor1(float targetSpeed);
void controlMotor2(float targetSpeed);

void setup() {

  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);

  sensor.init();
  sensor.setTimeout(50);
  sensor.startContinuous();

  pinMode(motor1Step, OUTPUT);
  pinMode(motor1Direction, OUTPUT);
  pinMode(motor1Enable, OUTPUT);
  pinMode(motor2Step, OUTPUT);
  pinMode(motor2Direction, OUTPUT);
  pinMode(motor2Enable, OUTPUT);
}

void logic(int delay) {

  static unsigned long int waitingSince = 0;
  if (millis() - waitingSince >= delay) {
    int distanceSensor = sensor.readRangeContinuousMillimeters();

    if (distanceSensor < 85) {
      start_time1 = millis();
      end_time1 = start_time1 + distance_1;
      end_time2 = end_time1 + distance_2;
    }

    if (!enabled1 && end_time1 > millis()) {
      motor1Speed = 0.2;
      start_time2 = start_time1 + distance_1 - overlap;
    }

    if (start_time2 <= millis()) {
      if (!enabled2 && end_time2 > millis()) {
        motor2Speed = 0.4;
      }
    }

    if (end_time1 <= millis()) {
      motor1Speed = 0;
    }

    if (end_time2 <= millis()) {
      motor2Speed = 0;
    }

    waitingSince = waitingSince + delay;
  }
}

void controlM1(float targetSpeed) {

  if (targetSpeed != 0.0)
    enabled1 = true;
  else {
    enabled1 = false;
    if (actMotor1Speed < 0.0000001) {
      digitalWrite(motor1Enable, LOW);
      motor1Running = false;
    }
  }

  if (actMotor1Speed < targetSpeed) {
    actMotor1Speed = min(actMotor1Speed + accelRate1, targetSpeed);
  } else if (actMotor1Speed > targetSpeed) {
    actMotor1Speed = max(actMotor1Speed - accelRate1, targetSpeed);
  }
  unsigned long delay = 98UL / actMotor1Speed;

  /*Serial.print(actMotor1Speed);
  Serial.print(" : ");
  Serial.print(delay);
  Serial.print(" : ");
  Serial.print(digitalRead(motor1Enable));
  Serial.print(" : ");*/

  static enum { OFF,
                ON,
                DISABLED } state = DISABLED;
  static unsigned long int waitingSince = 0;
  switch (state) {
    case OFF:
      if (micros() - waitingSince >= delay) {
        digitalWrite(motor1Step, LOW);
        waitingSince = waitingSince + delay;
        state = ON;
      }
      break;

    case ON:
      if (motor1Running) {
        if (micros() - waitingSince >= delay) {
          digitalWrite(motor1Step, HIGH);
          waitingSince = waitingSince + delay;
          state = OFF;
        }
      } else {
        state = DISABLED;
      }
      break;

    case DISABLED:
      if (enabled1) {
        //Serial.println("-----------------ON---------------");
        digitalWrite(motor1Enable, HIGH);
        state = ON;
        motor1Running = true;
        waitingSince = micros();
      }
      break;
  }
}

void controlM2(float targetSpeed) {

  if (targetSpeed != 0.0)
    enabled2 = true;
  else {
    enabled2 = false;
    if (actMotor2Speed < 0.0000001) {
      digitalWrite(motor2Enable, LOW);
      motor2Running = false;
    }
  }

  if (actMotor2Speed < targetSpeed) {
    actMotor2Speed = min(actMotor2Speed + accelRate2, targetSpeed);
  } else if (actMotor2Speed > targetSpeed) {
    actMotor2Speed = max(actMotor2Speed - accelRate2, targetSpeed);
  }
  unsigned long delay = 98UL / actMotor2Speed;

  /*Serial.print(actMotor2Speed);
  Serial.print(" : ");
  Serial.println(delay);*/

  static enum { OFF,
                ON,
                DISABLED } state = DISABLED;
  static unsigned long int waitingSince = 0;
  switch (state) {
    case OFF:
      if (micros() - waitingSince >= delay) {
        digitalWrite(motor2Step, LOW);
        waitingSince = waitingSince + delay;
        state = ON;
      }
      break;

    case ON:
      if (motor2Running) {
        if (micros() - waitingSince >= delay) {
          digitalWrite(motor2Step, HIGH);
          waitingSince = waitingSince + delay;
          state = OFF;
        }
      } else {
        state = DISABLED;
      }
      break;

    case DISABLED:
      if (enabled2) {
        digitalWrite(motor2Enable, HIGH);
        state = ON;
        motor2Running = true;
        waitingSince = micros();
      }
      break;
  }
}

void loop() {
  //logic(200);
  if (millis() > 375000) {
    motor1Speed = 0;
    motor2Speed = 0;
  } else if (millis() > 255000) {
    motor1Speed = 0.4;
    motor2Speed = 0.4;
  } else if (millis() > 135000) {
    motor1Speed = 0;
    motor2Speed = 0.4;
  } else if (millis() > 15000) {
    motor1Speed = 0.4;
    motor2Speed = 0;
  } else {
    motor1Speed = 0;
    motor2Speed = 0;
  }


  controlM1(motor1Speed);
  controlM2(motor2Speed);
  /*Serial.print(millis());
  Serial.print(" : ");
  Serial.print(micros());
  Serial.print(" : ");
  Serial.println(end_time1);*/
}
