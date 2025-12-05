/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

 #include <Arduino.h>
 #include <ESP32Servo.h>        //Servo motor library. This is standard library
 #include <NewPing.h>        //Ultrasonic sensor function library. You must install this library
 
 //our L298N control pins
 const int LeftMotorForward = 17;
 const int LeftMotorBackward = 16;
 const int RightMotorForward = 19;
 const int RightMotorBackward = 18;
 const int enableRightMotor = 12;
 const int enableLeftMotor = 13;
 
 //sensor pins
 #define trig_pin 14 //analog input 1
 #define echo_pin 21 //analog input 2
 
 #define maximum_distance 200
 boolean goesForward = false;
 int distance = 100;
 
 NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
 Servo servo_motor; //our servo name
 Servo servo_motor1; //our servo name
 Servo servo_motor2; //our servo name

 #define MAX_MOTOR_SPEED 150

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

int distanceRight = 0;
int distanceLeft = 0;

QueueHandle_t  q = NULL;
 
 int readPing(){
   delay(70);
   int cm = sonar.ping_cm();
   if (cm==0){
     cm=250;
   }
   return cm;
 }
 
 
 void moveStop(){
   
   digitalWrite(RightMotorForward, LOW);
   digitalWrite(LeftMotorForward, LOW);
   digitalWrite(RightMotorBackward, LOW);
   digitalWrite(LeftMotorBackward, LOW);

   int r = 0;
   int l = 0;

   ledcWrite(rightMotorPWMSpeedChannel, abs(r));
   ledcWrite(leftMotorPWMSpeedChannel, abs(l));  
 }
 
 void moveForward(){
 
   if(!goesForward){
 
     goesForward=true;
     
     digitalWrite(LeftMotorForward, LOW);
     digitalWrite(RightMotorForward, LOW);
   
     digitalWrite(LeftMotorBackward, HIGH);
     digitalWrite(RightMotorBackward, HIGH); 

     ledcWrite(rightMotorPWMSpeedChannel, abs(MAX_MOTOR_SPEED));
     ledcWrite(leftMotorPWMSpeedChannel, abs(MAX_MOTOR_SPEED));  
   }
 }
 
 void moveBackward(){
 
   goesForward=false;
 
   digitalWrite(LeftMotorBackward, LOW);
   digitalWrite(RightMotorBackward, LOW);
   
   digitalWrite(LeftMotorForward, HIGH);
   digitalWrite(RightMotorForward, HIGH);

   int r = -MAX_MOTOR_SPEED;
   int l = -MAX_MOTOR_SPEED;

   ledcWrite(rightMotorPWMSpeedChannel, abs(r));
   ledcWrite(leftMotorPWMSpeedChannel, abs(l));  

    
   
 }
 
 void turnRight(){
 
   digitalWrite(LeftMotorForward, HIGH);
   digitalWrite(RightMotorBackward, HIGH);
   
   digitalWrite(LeftMotorBackward, LOW);
   digitalWrite(RightMotorForward, LOW);

   int r = -255;
   int l = 255;

   ledcWrite(rightMotorPWMSpeedChannel, abs(r));
   ledcWrite(leftMotorPWMSpeedChannel, abs(l));  
   
   delay(250);
   
   digitalWrite(LeftMotorForward, HIGH);
   digitalWrite(RightMotorForward, HIGH);
   
   digitalWrite(LeftMotorBackward, LOW);
   digitalWrite(RightMotorBackward, LOW);
  
  
   r = MAX_MOTOR_SPEED;
   l = MAX_MOTOR_SPEED;

   ledcWrite(rightMotorPWMSpeedChannel, abs(r));
   ledcWrite(leftMotorPWMSpeedChannel, abs(l));  
   
 }
 
 void turnLeft(){
 
   digitalWrite(LeftMotorBackward, HIGH);
   digitalWrite(RightMotorForward, HIGH);
   
   digitalWrite(LeftMotorForward, LOW);
   digitalWrite(RightMotorBackward, LOW);

   int r = 255;
   int l = -255;

   ledcWrite(rightMotorPWMSpeedChannel, abs(r));
   ledcWrite(leftMotorPWMSpeedChannel, abs(l));  
 
   delay(250);
   
   digitalWrite(LeftMotorForward, HIGH);
   digitalWrite(RightMotorForward, HIGH);
   
   digitalWrite(LeftMotorBackward, LOW);
   digitalWrite(RightMotorBackward, LOW);

   r = MAX_MOTOR_SPEED;
   l = MAX_MOTOR_SPEED;

   ledcWrite(rightMotorPWMSpeedChannel, abs(r));
   ledcWrite(leftMotorPWMSpeedChannel, abs(l));  
 }
 
 int lookRight(){  
   servo_motor.write(0);
   delay(500);
   int distance = readPing();
   delay(100);
   servo_motor.write(90);
   return distance;
 }
 
 int lookLeft(){
   servo_motor.write(180);
   delay(500);
   int distance = readPing();
   delay(100);
   servo_motor.write(90);
   return distance;
   delay(100);
 }
 
 void TaskMotorCam(void * parameter) {
  while(1) {
    servo_motor1.write(20);
    vTaskDelay(pdMS_TO_TICKS(200)); // Delay
    servo_motor1.write(50);
    vTaskDelay(pdMS_TO_TICKS(200)); // Delay

    servo_motor2.write(120);
    vTaskDelay(pdMS_TO_TICKS(200)); // Delay
    servo_motor2.write(180);
    vTaskDelay(pdMS_TO_TICKS(200)); // Delay

    vTaskDelay(pdMS_TO_TICKS(10000)); // Delay
  }
}
void TaskUltrasonic(void * parameter) {
  if (q == NULL) {
    Serial.println(F("Queue in Measurement task is not ready"));
    return;
  }
  while(1) {
    int distances = readPing();
    if (distances <= 45) {
      moveStop();
      xQueueSend(q, (void *)&distances, (TickType_t )0);
      vTaskDelay(pdMS_TO_TICKS(3000)); // Delay
    }else{
      moveForward();
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // Delay
  }
}

void TaskMotorControl(void * parameter) {
  int distances;
  if (q == NULL) {
    Serial.println(F("Queue in HTTP socket task is not ready"));
    return;
  }
  while(1) {
    xQueueReceive(q, &distances, portMAX_DELAY); 
    distanceRight = 0;
    distanceLeft = 0;
    vTaskDelay(pdMS_TO_TICKS(50)); // Delay
    
    moveBackward();
    vTaskDelay(pdMS_TO_TICKS(200)); // Delay
    moveStop();
    vTaskDelay(pdMS_TO_TICKS(300)); // Delay

    servo_motor.write(0);
    vTaskDelay(pdMS_TO_TICKS(300)); // Delay
    distanceRight = readPing();
    vTaskDelay(pdMS_TO_TICKS(300)); // Delay
    servo_motor.write(90);

    //distanceRight = lookRight();
    vTaskDelay(pdMS_TO_TICKS(300)); // Delay

    servo_motor.write(180);
    vTaskDelay(pdMS_TO_TICKS(300)); // Delay
    distanceLeft = readPing();
    vTaskDelay(pdMS_TO_TICKS(300)); // Delay
    servo_motor.write(90);
    //distanceLeft = lookLeft();
    //vTaskDelay(pdMS_TO_TICKS(300)); // Delay

    if (distanceRight >= distanceLeft){
      turnRight();
      moveStop();
    }
    else{
      turnLeft();
      moveStop();
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // Check more frequently if needed
  }
}
 void setup(){
 
   pinMode(RightMotorForward, OUTPUT);
   pinMode(LeftMotorForward, OUTPUT);
   pinMode(LeftMotorBackward, OUTPUT);
   pinMode(RightMotorBackward, OUTPUT);
   
   servo_motor.attach(33); //our servo pin
   servo_motor1.attach(25); //our servo pin
   servo_motor2.attach(26); //our servo pin

   //Set up PWM for speed
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);  
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel); 
  q = xQueueCreate(20, sizeof(int));
  xTaskCreate(TaskMotorCam, "TaskMotorCam", 2048, NULL, 1, NULL);
  xTaskCreate(TaskUltrasonic, "UltrasonicSensorTask", 2048, NULL, 1, NULL);
  xTaskCreate(TaskMotorControl, "MotorControlTask", 2048, NULL, 1, NULL);

 
   servo_motor.write(90);
   delay(2000);
   distance = readPing();
   delay(100);
   distance = readPing();
   delay(100);
   distance = readPing();
   delay(100);
   distance = readPing();
   delay(100);
 }

 void loop(){
 
 
 }
