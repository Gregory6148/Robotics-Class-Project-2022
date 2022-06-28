// Robotics Project 2022
// Controls a robot using a gripper and a line follower sensor

#include <Servo.h> 

//Pins
//Motor Pins
int LMOTOR = P2_5;
int LMOTORD = P2_4;
int RMOTOR = P1_7;
int RMOTORD = P1_6;

//Servo Pins
int OCServo = P2_2;
int UDServo = P2_1;

//Servo Initialization
Servo OCser; 
Servo UDser; 
int SerO = 15;
int SerC = 55;
int SerCC = 60;
int SerU = 125;
int SerD = 100;

//Button Pins
int BUT1 = P1_0;
int BUT2 = P1_5;

//LF Sensors
int DET = P2_0;
int LFS = P1_4;

//Constants
int MPOW = 150; //Motor Duty Cycle (0-255)

//Constants for Line Following Function
  int lasterr = 0;
  float Kp = 0.15;
  float Ki = 0;
  float Kd = 2;
  int P,I,D;
  int error;
  int sped;
  int bsped = 150;

//Function that Picks the can
void Pick()
{
  for(int ang = SerO; ang < SerC; ang++)  
  {                                  
    OCser.write(ang);               
    delay(10);                   
  } 
  for(int ang = SerD; ang < SerU; ang++)  
  {                                  
    UDser.write(ang);               
    delay(10);                   
  } 
}

//Function that Resets can
void Place()
{
  for(int ang = SerU; ang > SerD; ang--)  
  {                                  
    UDser.write(ang);
    delay(10);
  }
  for(int ang = SerC; ang > SerO; ang--)
  {                                  
    OCser.write(ang);               
    delay(10);                 
  }
}

//Function resetting position
void Reset()
{
  //Positions gripper
  OCser.write(SerO);
  UDser.write(SerD);
  
  //Turn off Motors and set them to FW
  digitalWrite(LMOTORD,LOW);
  digitalWrite(RMOTORD,LOW);
  analogWrite(LMOTOR,0);
  analogWrite(RMOTOR,0);
}

//Function that moves robot in reverse for specified ms
void Bck(int ms)
{
  //Reverses movement direction
  digitalWrite(LMOTORD,HIGH);
  digitalWrite(RMOTORD,HIGH);
  analogWrite(LMOTOR,60);
  analogWrite(RMOTOR,60);
  delay(ms);

  //Resets and stops motors
  digitalWrite(LMOTORD,LOW);
  digitalWrite(RMOTORD,LOW);
  analogWrite(LMOTOR,0);
  analogWrite(RMOTOR,0);
}

void Flip()
{
  digitalWrite(LMOTORD,LOW);
  digitalWrite(RMOTORD,HIGH);
  analogWrite(LMOTOR,200);
  analogWrite(RMOTOR,255-200);
  delay(100);
  while(analogRead(LFS) < 600);
  digitalWrite(RMOTORD,LOW);
  analogWrite(LMOTOR,0);
  analogWrite(RMOTOR,0);
}

void LineFollow()
{
  while(digitalRead(DET) == 0)
  {
    error = 600 - analogRead(LFS);
    P = error;
    I = I + error;
    D = error - lasterr;
    lasterr = error;
    sped = P*Kp + I*Ki + D*Kd;
    analogWrite(LMOTOR,bsped - sped);
    analogWrite(RMOTOR,bsped + sped);
  }
  analogWrite(LMOTOR,0);
  analogWrite(RMOTOR,0);
}

void setup()
{
  //Usb serial monitor for debugging
  Serial.begin(9600);
  
  //Sets Pin Mode for every pin
  pinMode(LMOTOR,OUTPUT);
  pinMode(LMOTORD,OUTPUT);
  pinMode(RMOTOR,OUTPUT);
  pinMode(RMOTORD,OUTPUT);
  
  OCser.attach(OCServo); 
  UDser.attach(UDServo); 
  
  pinMode(BUT1,INPUT);
  pinMode(BUT2,INPUT);
  pinMode(DET,INPUT);
  
  Reset();
}

//Main Program
void loop()
{
  //Checks For Button Presses
  if(digitalRead(BUT1) == HIGH)
  {
    delay(5000);
    LineFollow();
    Pick();
    Flip();
    delay(500);
    LineFollow();
    Flip();
    Place();
  }
  else if(digitalRead(BUT2) == HIGH)
  {
    delay(5000);
    Pick();
    LineFollow();
    Place();
    Bck(150);
    Flip();
    delay(500);
    LineFollow();
    Flip();
  }
  delay(1000);
}

