#include "I2Cdev.h"
#include "MPU6050.h"
//#include <IRremote.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
const int ENA = 3;
const int ENB = 6;
const int IN1 = 2;
const int IN2 = 4;
const int IN3 = 5;
const int IN4 = 7;

unsigned long currentTime;
unsigned long timeNow;
unsigned long lastTime;
int output;
double errSum, lastErr;
float cycleTime = 0.004;//in seconds
double kp = 2.0;
double ki = 0.30;
double kd = 3.0;
double accelData;
double error;
double dErr;
double accelTotal;
double gyroData;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int sumgx, sumgy, sumgz;
int sumax, sumay, sumaz;
int ayTotal;
float delayTimeCorrect;

static int setpoint = 0;
static int input = 0;
static int lastInput = 0;

#define OUTPUT_READABLE_ACCELGYRO

void setup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  accelgyro.initialize();
  for (int i = 1; i < 33; i = i + 1)
  {
    accelgyro.getRotation(&gx, &gy, &gz);
    sumgx = sumgx + gx;
  }
  sumgx = sumgx / 32;
  for (int i = 1; i < 33; i = i + 1)
  {
    accelgyro.getAcceleration(&ax, &ay, &az);
    sumay = sumay + ay;
  }
  sumay = sumay / 32;
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT); 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);   
}

void loop()
{
  currentTime = micros();
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelData = (ay-sumay)>>2;// / by 4
  accelTotal = accelData + setpoint;
  gyroData = (gx-sumgx)>>1;// / by 2
  input = (0.98*((lastInput)+((gyroData)*cycleTime)))+(0.02*(accelTotal));
  /*
   * the above takes the last reading of gyro (lastInput), and then takes the current 
   * reading * cycle time (to work out a turning movement in a time frame)
   * and then adds the two together. this works out how much it has turned since last time
   * this is then multiplied by 98% and then added to 2% of the current accel reading.
   */
  calcOutput();
  MotorL298();
  serialInput();
  serialOutput();
  lastErr = error;
  lastInput = input;
  correctTime();
}
void correctTime()
{
  timeNow = micros();
  delayTimeCorrect = 4000-(timeNow - currentTime);
  delayMicroseconds(delayTimeCorrect);
}  

void calcOutput()
{
  error = setpoint - input;
  error = error;
  errSum = errSum + (error * cycleTime); //intergral part sum of errors past
  double dInput = input - lastInput;
  output = (kp * error) + (ki * errSum) - (kd * dInput);
}
  
void MotorL298()
{
  output = (output / 2);
  if (output > 255)
  {
    output = 255;
  }
  if (output < -255)
  {
    output = -255;
  }
  if (output < 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, (output*-1));
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, (output*-1));
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, output);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, output);
    
  }
}

void serialInput()
{
  if (Serial.available() > 0)
  {
    char ch = Serial.read();
    if (ch == '4')
    {
      kp = kp + 0.01;
    }
    else if (ch == '1')
    {
      kp = kp - 0.01;
    }
    else if (ch == '5')
    {
      ki = ki + 0.01;
    }
    else if (ch == '2')
    {
      ki = ki - 0.01;
    }
    else if (ch == '6')
    {
      kd = kd + 0.01;
    }
    else if (ch == '3')
    {
      kd = kd - 0.01;
    }
    else if (ch == 'y')
    {
      //neutral1 = neutral1 + 1;
    }
    else if (ch == 'h')
    {
      //neutral1 = neutral1 - 1;      
    }
    else if (ch == 'u')
    {
      //neutral2 = neutral2 + 1;
    }
    else if (ch == 'j')
    {
      //neutral2 = neutral2 - 1;      
    }
    else if (ch == 'd')
    {
      //moveData = moveData + 1;
    }
    else if (ch == 'c')
    {
     // moveData = moveData - 1;
    }
  }  
}

void serialOutput()
{
  static int k = 0;
  if (k == 0)
  {
    Serial.print("P ");
    Serial.print(kp);
  }
  else if (k == 1)
  {
    Serial.print("  I ");
    Serial.print(ki);
  }  
  else if (k == 2)
  {
    Serial.print("  D ");
    Serial.print(kd);
  }
  else if (k == 3)  
  {
    Serial.print("  Raw= ");
    Serial.print(accelTotal);
  }
  else if (k == 4)  
  {
    Serial.print("  in = ");
    Serial.print(input);

  }  
  else if (k == 5)  
  {
    Serial.print("  out = ");
    Serial.print(output);
  }  
  else if (k == 6)  
  {
    Serial.print(" time = ");
    Serial.print(delayTimeCorrect);
  }  
  else if (k == 7)  
  {
    Serial.print("  now= ");
    Serial.print(timeNow);
  }  
  else if (k == 8)  
  {
    Serial.print("  ?= ");
    //Serial.print(delayTimeCorrect);
  }  
  k = k + 1;
  if (k == 7)
  {
    k = 0;
    Serial.println();
  }  
}  

