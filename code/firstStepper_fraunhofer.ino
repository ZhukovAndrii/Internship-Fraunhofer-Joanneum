// Define pin connections & motor's steps per revolution
const int dirPin = 4;
const int stepPin = 3;
const int stepsPerRevolution = 10;
const int sleep = 2;

void setup()
{
  Serial.begin(9600);
	// Declare pins as Outputs
	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);
	pinMode(sleep, OUTPUT);
}

void loop()
{
	// Set motor direction clockwise
	digitalWrite(dirPin, HIGH);
	digitalWrite(sleep, HIGH);

	// Spin motor slowly
  Serial.println("slow");
	for(int x = 0; x < stepsPerRevolution; x++)
	{
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(20000);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(20000);
	}
	digitalWrite(sleep, LOW);
	delay(1000); // Wait a second
	
	// Set motor direction counterclockwise
	digitalWrite(dirPin, LOW);
	digitalWrite(sleep, HIGH);

	// Spin motor quickly
  Serial.println("fast");
	for(int x = 0; x < stepsPerRevolution; x++)
	{
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(20000);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(20000);
	}
	digitalWrite(sleep, LOW);
	delay(100); // Wait a second*/
}