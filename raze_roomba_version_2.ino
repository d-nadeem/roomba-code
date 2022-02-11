#include <NewPing.h>// might need to delete if issue 


#include "NewPing.h"

//sensor 1
#define TRIGGER_PIN_1  12
#define ECHO_PIN_1     13

//sensor 2
#define TRIGGER_PIN_2  10
#define ECHO_PIN_2     11

//sensor 3
#define TRIGGER_PIN_3  8
#define ECHO_PIN_3     9


//max distance
#define MAX_DISTANCE 400


// pins for the motor left side
#define ENABLE_L 7
#define dir1L 6
#define dir2L 5

// pins for the motor right side
#define ENABLE_R 4
#define dir1R 3
#define dir2R 2

//for distance 1
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
float duration1; // Stores First HC-SR04 pulse duration value
float distance1; // Stores calculated distance in cm for First Sensor

//check here for errors
//for distance 2
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
float duration2; // Stores First HC-SR04 pulse duration value
float distance2; // Stores calculated distance in cm for First Sensor
int iterations = 5;

//for distance 3
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE);
float duration3; // Stores First HC-SR04 pulse duration value
float distance3; // Stores calculated distance in cm for First Sensor

void setup() {
  //---set pin direction
  pinMode(ENABLE_L, OUTPUT); //for motor left side
  pinMode(dir1L, OUTPUT);
  pinMode(dir2L, OUTPUT);

  pinMode(ENABLE_R, OUTPUT); //for motor right side
  pinMode(dir1R, OUTPUT);
  pinMode(dir2R, OUTPUT);

  Serial.begin (9600);
}

void loop() {


  // Measure duration for first sensor
  duration1 = sonar1.ping_median(iterations);
  distance1 = (duration1 / 2) * 0.0343;

  // Measure duration for second sensor
  duration2 = sonar2.ping_median(iterations);
  distance2 = (duration2 / 2) * 0.0343;


  // to enable the left motor to max speed
  analogWrite(ENABLE_L, 255);

  // to enable the right motor to max speed
  analogWrite(ENABLE_R, 255);


  //activates the motor for the left side
  digitalWrite(dir1L, HIGH);
  digitalWrite(dir2L, LOW);

  //activates the motor for the right side
  digitalWrite(dir1R, HIGH);
  digitalWrite(dir2R, LOW);


  Serial.print("Distance 1: ");
  // if statements for the first sensor determines the output
  if (distance1 >= 400 || distance1 <= 2) {
    Serial.print("Out of range");
    Serial.println("");
  }
  else {
    Serial.print(distance1);
    Serial.print(" cm ");
    Serial.println("");
  }



  Serial.print("Distance 2: ");
  // if statements for the second sensor determines the output
  if (distance2 >= 400 || distance2 <= 2) {
    Serial.print("Out of range");
    Serial.println("");
  }
  else {
    Serial.print(distance2);
    Serial.print(" cm ");
    Serial.println("");
  }


  Serial.print("Distance 3: ");
  // if statements for the second sensor determines the output
  if (distance3 >= 400 || distance3 <= 2) {
    Serial.print("Out of range");
    Serial.println("");
  }
  else {
    Serial.print(distance3);
    Serial.print(" cm ");
    Serial.println("");
  }




  //to turn left
  if (distance1 < 10 && distance1 < distance2 && distance1 < distance3) {
    // issue is right here the distance is not updating so code is stuck
    //turns off the right wheel so the left wheel can turn the roomba to the left
    digitalWrite(dir1R, HIGH);
    digitalWrite(dir2R, HIGH);
    delay(200);
  }

  // to turn right
  else if (distance2 < 10 && distance2 < distance1 && distance2 < distance3) {
    // issue is right here the distance is not updating so code is stuck
    //turns off the left wheel so the right wheel can turn the roomba to the right
    digitalWrite(dir1L, HIGH);
    digitalWrite(dir2L, HIGH);
    delay(200);
  }



  // to rotate the device 180 degrees
  else if (distance3 < 10 && distance3 < distance1 && distance3 < distance2) {
    // issue is right here the distance is not updating so code is stuck
    //turns off the left wheel so the right wheel can turn the roomba to the right

    //allows to reverse from the wall
    //left motor
    digitalWrite(dir1L, LOW);
    digitalWrite(dir2L, HIGH);
    //right motor
    digitalWrite(dir1R, LOW);
    digitalWrite(dir2R, HIGH);
    delay(5000);

    //lets the device turn cw to 180 degrees
    digitalWrite(dir1L, HIGH);
    digitalWrite(dir2L, LOW);

    //stops the right wheel to let the device rotate
    digitalWrite(dir1R, HIGH);
    digitalWrite(dir2R, HIGH);
    delay (10000);// gives 10 seconds to allow the device to rotate cw to 180 degrees
  }



  //keeps the roomba moving straight
  // flip the directions if the wheels are not moving in the same direction
  else {
    //left motor
    digitalWrite(dir1L, HIGH);
    digitalWrite(dir2L, LOW);
    //right motor
    digitalWrite(dir1L, HIGH);
    digitalWrite(dir2L, LOW);
  }


}
