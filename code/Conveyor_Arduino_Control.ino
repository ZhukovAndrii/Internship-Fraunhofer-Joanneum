#include <Wire.h>
#include <VL53L0X.h>

// Define sensor instance
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
  // Initialize I2C communication and Serial communication
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);

  // Initialize the TOF sensor
  sensor.init();
  sensor.setTimeout(50);
  sensor.startContinuous();

  // Set motor pins as output
  pinMode(motor1Step, OUTPUT);
  pinMode(motor1Direction, OUTPUT);
  pinMode(motor1Enable, OUTPUT);
  pinMode(motor2Step, OUTPUT);
  pinMode(motor2Direction, OUTPUT);
  pinMode(motor2Enable, OUTPUT);
}

void logic(int delay) {
  static unsigned long int waitingSince = 0;

  // Perform logic based on time intervals
  if (millis() - waitingSince >= delay) {
    int distanceSensor = sensor.readRangeContinuousMillimeters();

    // If the distance measured by the sensor is less than 85mm, initiate motor movement
    if (distanceSensor < 85) {
      start_time1 = millis();
      end_time1 = start_time1 + distance_1;
      end_time2 = end_time1 + distance_2;
    }

    // Control motor 1 based on specific conditions
    if (!enabled1 && end_time1 > millis()) {
      motor1Speed = 0.2;
      start_time2 = start_time1 + distance_1 - overlap;
    }

    // Control motor 2 based on specific conditions
    if (start_time2 <= millis()) {
      if (!enabled2 && end_time2 > millis()) {
        motor2Speed = 0.4;
      }
    }

    // Stop motor 1 when required duration is reached
    if (end_time1 <= millis()) {
      motor1Speed = 0;
    }

    // Stop motor 2 when required duration is reached
    if (end_time2 <= millis()) {
      motor2Speed = 0;
    }

    waitingSince = waitingSince + delay;
  }
}

void controlM1(float targetSpeed) {

  // Check if motor 1 should be enabled or disabled based on the target speed
  if (targetSpeed != 0.0)
    enabled1 = true;
  else {
    enabled1 = false;
    if (actMotor1Speed < 0.0000001) {
      digitalWrite(motor1Enable, LOW);
      motor1Running = false;
    }
  }

  // Adjust motor 1 speed based on the target speed and acceleration rate
  if (actMotor1Speed < targetSpeed) {
    actMotor1Speed = min(actMotor1Speed + accelRate1, targetSpeed);
  } else if (actMotor1Speed > targetSpeed) {
    actMotor1Speed = max(actMotor1Speed - accelRate1, targetSpeed);
  }

  // Calculate the delay based on the current motor 1 speed
  unsigned long delay = 98UL / actMotor1Speed;

  // State machine to control motor 1 movement
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
        // Enable motor 1 movement
        digitalWrite(motor1Enable, HIGH);
        state = ON;
        motor1Running = true;
        waitingSince = micros();
      }
      break;
  }
}

void controlM2(float targetSpeed) {

  // Check if motor 2 should be enabled or disabled based on the target speed
  if (targetSpeed != 0.0)
    enabled2 = true;
  else {
    enabled2 = false;
    if (actMotor2Speed < 0.0000001) {
      digitalWrite(motor2Enable, LOW);
      motor2Running = false;
    }
  }

  // Adjust motor 2 speed based on the target speed and acceleration rate
  if (actMotor2Speed < targetSpeed) {
    actMotor2Speed = min(actMotor2Speed + accelRate2, targetSpeed);
  } else if (actMotor2Speed > targetSpeed) {
    actMotor2Speed = max(actMotor2Speed - accelRate2, targetSpeed);
  }
  
  // Calculate the delay based on the current motor 2 speed
  unsigned long delay = 98UL / actMotor2Speed;

  // State machine to control motor 2 movement
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
        // Enable motor 2 movement
        digitalWrite(motor2Enable, HIGH);
        state = ON;
        motor2Running = true;
        waitingSince = micros();
      }
      break;
  }
}

void loop() {

  // Call the logic function to control motor movement
  logic(200);

  // Control motor 1 and motor 2 based on their respective speeds
  controlM1(motor1Speed);
  controlM2(motor2Speed);

}