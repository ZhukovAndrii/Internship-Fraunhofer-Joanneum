#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>


// Define the chip select pin for the SD card module
#define cardSelect 4
// File object to handle the log file
File logfile;
// Filename for the log file
char* filename;


// Create Adafruit_BME280 instances for two sensors
Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
// Create Adafruit_MPU6050 instances for two sensors
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

// Variables to store sensor data
float temp1;
float temp2;
float ax1, ay1, az1, ax2, ay2, az2;
float vol1, vol2;
float cur1, cur2, cur3, cur4, cur5;

// Counter to keep track of measurements
long count = 0;

// Define analog input pins for voltage and current measurements
const int voltage1 = A0;
const int voltage2 = A1;
const int current1 = A3;
const int current2 = A2;
const int current3 = A4;
const int current4 = A5;
const int current5 = 9;

// Encoder related variables
const int encoderPin = 5;               // Encoder output A connected to digital pin 2
const float wheelCircumference = 0.19;  // Circumference of the measuring wheel in meters
const int pulsesPerRevolution = 2000;   // Number of encoder pulses per revolution
int encoderCount = 0;                   // Counter to store the encoder position
unsigned long prevTime = 0;             // Time of the previous encoder position change
float speed = 0.0;                      // Variable to store the calculated speed in meters per second (m/s)
float timeDiff;
float pulsesPerSecond;

// Function prototypes
void setup();
void loop();
void error(uint8_t errno);
void readFilename();
int getMeasurementTime();
void updateEncoder();

void setup() {

  // Initialize I2C communication and Serial communication
  Wire.begin();
  Wire.setClock(400000);

  // Set pins
  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(voltage1, INPUT);
  pinMode(voltage2, INPUT);
  pinMode(current1, INPUT);
  pinMode(current2, INPUT);
  pinMode(current3, INPUT);
  pinMode(current4, INPUT);
  pinMode(current5, INPUT);
  Serial.begin(9600);

  // Initialize BME280 and MPU6050 sensors
  if (!bme1.begin(0x76))
    error(10);
  if (!bme2.begin(0x77))
    error(11);
  if (!mpu1.begin(0x68))
    error(12);
  if (!mpu2.begin(0x69))
    error(13);

  // Initialize SD card
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
    error(2);
  }

  // Read filename from the user and create the log file
  readFilename();


  // Open the log file and write the header
  logfile = SD.open(filename, FILE_WRITE);
  logfile.println("#; CPU time in ms; temp1 in C; temp2 in C; ax1 in m/s^2; ay1 in m/s^2; az1 in m/s^2; ax2 in m/s^2; ay2 in m/s^2; az2 in m/s^2; volt1; volt2; cur1; cur2; cur3; cur4; cur5; speed in m/s");
  if (!logfile) {
    Serial.print("Couldnt create ");
    Serial.println(filename);
    error(3);
  }
  logfile.close();
  Serial.print("Writing to: ");
  Serial.println(filename);

  // Set up encoder pin and interrupt
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), updateEncoder, RISING);
}

void loop() {

  // Open the log file for writing
  logfile = SD.open(filename, FILE_WRITE);
  if (logfile) {
    digitalWrite(13, HIGH);
    int duration = getMeasurementTime();
    long start = millis();
    prevTime = start;
    digitalWrite(8, HIGH);
    while (millis() - start < duration) {
      // Read sensor data and update encoder information
      //-------------TEMP---------------
      temp1 = bme1.readTemperature();
      temp2 = bme2.readTemperature();
      //-------------ACCEL----------------
      sensors_event_t a, g, temp;
      mpu1.getEvent(&a, &g, &temp);
      ax1 = a.acceleration.x;
      ay1 = a.acceleration.y;
      az1 = a.acceleration.z;
      mpu2.getEvent(&a, &g, &temp);
      ax2 = a.acceleration.x;
      ay2 = a.acceleration.y;
      az2 = a.acceleration.z;
      //-------------ENCODER---------------
      timeDiff = millis() - prevTime;                                        // Time difference in milliseconds
      pulsesPerSecond = encoderCount / (timeDiff / 1000);                    // Convert time difference to seconds
      speed = (pulsesPerSecond * wheelCircumference) / pulsesPerRevolution;  // Calculate the distance covered in one second (speed)
      // Reset the encoderCount and update the previous time for the next calculation
      encoderCount = 0;
      prevTime = millis();

      //--------------ANALOG---------------
      vol1 = analogRead(voltage1);
      vol2 = analogRead(voltage2);
      cur1 = analogRead(current1);
      cur2 = analogRead(current2);
      cur3 = analogRead(current3);
      cur4 = analogRead(current4);
      cur5 = analogRead(current5);


      // Write data to the log file
      logfile.print(++count);
      logfile.print("; ");
      logfile.print(millis());
      logfile.print("; ");
      logfile.print(temp1, 4);
      logfile.print("; ");
      logfile.print(temp2, 4);
      logfile.print("; ");
      logfile.print(ax1, 4);
      logfile.print("; ");
      logfile.print(ay1, 4);
      logfile.print("; ");
      logfile.print(az1, 4);
      logfile.print("; ");
      logfile.print(ax2, 4);
      logfile.print("; ");
      logfile.print(ay2, 4);
      logfile.print("; ");
      logfile.print(az2, 4);
      logfile.print("; ");
      logfile.print(vol1, 4);
      logfile.print("; ");
      logfile.print(vol2, 4);
      logfile.print("; ");
      logfile.print(cur1, 4);
      logfile.print("; ");
      logfile.print(cur2, 4);
      logfile.print("; ");
      logfile.print(cur3, 4);
      logfile.print("; ");
      logfile.print(cur4, 4);
      logfile.print("; ");
      logfile.print(cur5, 4);
      logfile.print("; ");
      logfile.println(speed, 4);
    }
    logfile.close();
    digitalWrite(8, LOW);
    digitalWrite(13, LOW);
    Serial.println("Done");
    while (1) {}  // Infinite loop to stop the program after logging data
  }
}


// Function to handle error conditions and blink out an error code
void error(uint8_t errno) {
  Serial.println("--------------------------------------------------------");
  switch (errno) {
    case 2:
      Serial.println("SD-Error");
      break;
    case 3:
      Serial.println("File-Error");
      break;
    case 10:
      Serial.println("BME1-ERROR");
      break;
    case 11:
      Serial.println("BME2-ERROR");
      break;
    case 12:
      Serial.println("MPU1-ERROR");
      break;
    case 13:
      Serial.println("MPU2-ERROR");
      break;
  }
  // Blink out the error code
  while (1) {
    uint8_t i;
    for (i = 0; i < errno; i++) {
      digitalWrite(13, HIGH);
      delay(60);
      digitalWrite(13, LOW);
      delay(60);
    }
    for (i = errno; i < 10; i++) {
      delay(120);
    }
  }
}

// Function to read the filename from the user through the Serial monitor
void readFilename() {
  Serial.println("Please enter the filename (without extension):");
  while (!Serial.available()) {
    // Wait for user input
  }
  delay(100);           // Wait a bit more in case the user sends multiple characters
  int bufferSize = 50;  // You can adjust this to set the maximum allowed filename length
  char input[bufferSize];
  int i = 0;
  while (Serial.available() && i < bufferSize - 1) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {  // Stop reading when Enter key is pressed
      input[i] = '\0';             // Null-terminate the input
      break;
    }
    input[i] = c;
    i++;
  }
  filename = new char[i + 5];  // Allocate memory for the filename (+5 for the ".csv" and null terminator)
  strcpy(filename, input);
  strcat(filename, ".csv");  // Concatenate the ".csv" extension to the filename
  Serial.print("Filename created: ");
  Serial.println(filename);

  // Clear any remaining characters in the Serial buffer
  while (Serial.available()) {
    Serial.read();
  }
}

// Function to get the measurement time from the user through the Serial monitor
int getMeasurementTime() {
  Serial.println("Please enter the measurement time in milliseconds:");
  while (!Serial.available()) {
    // Wait for user input
  }
  delay(100);  // Wait a bit more in case the user sends multiple characters
  int bufferSize = 20;
  char input[bufferSize];
  int i = 0;
  while (Serial.available() && i < bufferSize - 1) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {  // Stop reading when Enter key is pressed
      input[i] = '\0';             // Null-terminate the input
      break;
    }
    input[i] = c;
    i++;
  }
  int measurementTime = atoi(input);  // Convert the input to an integer
  Serial.print("Starting measurement for ");
  Serial.print(measurementTime);
  Serial.println("ms.");
  return measurementTime;
}

// Interrupt service routine to update the encoder count
void updateEncoder() {
  // Increment the encoderCount for each pulse received
  encoderCount++;
}