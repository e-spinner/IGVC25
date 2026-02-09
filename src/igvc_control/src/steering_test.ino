#include <Stepper.h>

// Stepper motor variables
int stepsPerRevolution = 2048; // number of steps per revolution for this stepper motor
int motSpeed = 17; // speed of stepper motor
int numSteps; // number of steps we want motor to rotate
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

int steeringInput = 3;

// variables for single drive motor
int motor1Dir1 = 4; // motor1 forward rotation
int motor1Dir2 = 5; // motor1 backward rotation
int motor1SpeedPin = 6; // pwm pin for speed control of the drive motor
int motor1SpeedSP = 255; // motor speed set point for drive motor 0-255

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // set baud rate to 9600
  while(!Serial){
    ; // wait for serial port to connect
  }
  myStepper.setSpeed(motSpeed); // stepper initialization
  pinMode(steeringInput, INPUT); // how much we need to turn by
  pinMode(motor1Dir1, OUTPUT); // motor1 forward motor rotation
  pinMode(motor1Dir2, OUTPUT); // motor1 backward motor rotation
  pinMode(motor1SpeedPin, OUTPUT); // motor1 speed
}

const char TERMINATOR = '|';

void loop(){
  // put your main code here, to run repeatedly:

  // stepper will be replaced by dc motor and encoder
  myStepper.step(numSteps);

  // motor forward direction
  digitalWrite(motor1Dir1, HIGH); 
  digitalWrite(motor1Dir2, LOW);
  analogWrite(motor1SpeedPin, motor1SpeedSP); // motor speed
    
  delay(3000);

/*

// gets an angle via serial cable from jetson
// converts angle to number of steps for the stepper motor

  if(Serial.available() > 0){
    String commandFromJetson = Serial.readStringUntil(TERMINATOR);

    //confirm
    String ackMsg = "Hello Jetson. This is what I got from you: " + commandFromJetson; // String(messageBuffer);

    Serial.print(ackMsg);
  }
  delay(500);

  if (Serial.available()) { // Check if data is available
    char receivedChar = Serial.read(); // Read the incoming byte
    Serial.print("Received: ");
    Serial.println(receivedChar); // Print the received data
  }
*/
}
