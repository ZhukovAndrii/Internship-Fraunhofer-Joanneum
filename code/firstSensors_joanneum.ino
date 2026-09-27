#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>

#define cardSelect 4
File logfile;
char* filename;


Adafruit_BME280 bme1;  // I2C
Adafruit_BME280 bme2;  // I2C
// Create a MPU6050 object
Adafruit_MPU6050 mpu;

float temp1;
float temp2;
float ax, ay, az;
long count = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
  Serial.begin(9600);
  delay(5000);
  bme1.begin(0x76);
  bme2.begin(0x77);
  mpu.begin();

  if (!SD.begin(cardSelect)) {
    Serial.println("-------------------------");
    Serial.println("-------------------------");
    Serial.println("-------------------------");
    Serial.println("-------------------------");
    Serial.println("-------------------------");
    Serial.println("-------------------------");
    Serial.println("-------------------------");
    Serial.println("-------------------------");
    Serial.println("Card init. failed!");
    while (1) {}
  }

  readFilename();  // Call the function to get the filename from the console


  logfile = SD.open(filename, FILE_WRITE);
  logfile.println("#; CPU time in ms; temp1 in °C; temp2 in °C; ax in m/s^2; ay in m/s^2; az in m/s^2");
  if (!logfile) {
    Serial.print("Couldnt create ");
    Serial.println(filename);
  }
  logfile.close();
}

void loop() {

  logfile = SD.open(filename, FILE_WRITE);
  if (logfile) {
    digitalWrite(13, HIGH);
    digitalWrite(8, HIGH);
    int duration = getMeasurementTime();
    long start = millis();
    while (millis() - start < duration) {
      temp1 = bme1.readTemperature();
      temp2 = bme2.readTemperature();
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      ax = a.acceleration.x;
      ay = a.acceleration.y;
      az = a.acceleration.z;
      //Serial.print("Temperature1 = ");
      /*Serial.print(temp1);
        Serial.print("  ");
        //Serial.println(" °C");
        //Serial.println("---------------");
        //Serial.print("Temperature2 = ");
        Serial.print(temp2);
        //Serial.println(" °C");

        Serial.print("  ");
        Serial.print(ax);
        Serial.print("  ");
        Serial.print(ay);
        Serial.print("  ");
        Serial.println(az);*/
      //delay(100);

      //digitalWrite(8, HIGH);
      logfile.print(++count);
      logfile.print("; ");
      logfile.print(millis());
      logfile.print("; ");
      logfile.print(temp1);
      logfile.print("; ");
      logfile.print(temp2);
      logfile.print("; ");
      logfile.print(ax);
      logfile.print("; ");
      logfile.print(ay);
      logfile.print("; ");
      logfile.println(az);
    }
    logfile.close();
    digitalWrite(8, LOW);
    digitalWrite(13, LOW);
    Serial.println("Done");
    while (1) {}
  }
  //digitalWrite(8, LOW);
}
void readFilename() {
  Serial.println("Please enter the filename (without extension):");
  while (!Serial.available()) {
    // Wait for user input
  }
  delay(100);           // Wait a bit more in case the user sends multiple characters
  int bufferSize = 20;  // You can adjust this to set the maximum allowed filename length
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
  filename = new char[i + 5]; // Allocate memory for the filename (+5 for the ".csv" and null terminator)
  strcpy(filename, input);
  strcat(filename, ".csv"); // Concatenate the ".csv" extension to the filename
  Serial.print("Filename created: ");
  Serial.println(filename);

  // Clear any remaining characters in the Serial buffer
  while (Serial.available()) {
    Serial.read();
  }
}



int getMeasurementTime() {
  Serial.println("Please enter the measurement time in milliseconds:");
  while (!Serial.available()) {
    // Wait for user input
  }
  delay(100); // Wait a bit more in case the user sends multiple characters
  int bufferSize = 20;
  char input[bufferSize];
  int i = 0;
  while (Serial.available() && i < bufferSize - 1) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { // Stop reading when Enter key is pressed
      input[i] = '\0'; // Null-terminate the input
      break;
    }
    input[i] = c;
    i++;
  }
  int measurementTime = atoi(input); // Convert the input to an integer
  Serial.print("Starting measurement for ");
  Serial.print(measurementTime);
  Serial.println("ms.");
  return measurementTime;
}

