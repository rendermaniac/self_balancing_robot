// See https://www.instructables.com/Arduino-Self-Balancing-Robot-1/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <LSM303.h>
#include <PID_v1.h>

#define RAD2DEG 57.2958
#define MOTOR_COMPENSATION 0.745

unsigned long samplerate = 2.0;
float interval = samplerate / 1000.0;

float gyro_offset;
sensors_event_t gyro_event; 

float last_pitch = 0.0;

unsigned long now, previous;

double angle = 0.0;
double power = 0.0;
int drive = 0;

double setpoint = 79.0;
int motor_bite = 100;
bool start = false;

double kP = 60; // 50
double kI = 850.0; // 730.0;
double kD = 2.8; // 2.3;

// Motor A
int enA = 9; // blue
int in1 = 8; // yellow
int in2 = 7; // red

// Motor B
int enB = 3; // orange
int in3 = 5; // brown
int in4 = 4; // green

LSM303 accel;
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
PID myPID(&angle, &power, &setpoint, kP, kI, kD, DIRECT);

void blink_LED(int times, int delayTime)
{
  for(int i=0; i<times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delayTime);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delayTime);
  }
}

float calibrate_gyro(Adafruit_L3GD20_Unified *gyro, int calibration_samples)
{
  float offset = 0.0;
  sensors_event_t event;

  for (uint16_t sample = 0; sample < calibration_samples; sample++) {
    gyro->getEvent(&event);
    offset += event.gyro.y - 0.0;
    delay(10);
  }
  return offset / calibration_samples;
}

void start_motor()
{
  // Turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void forward(int car_speed)
{
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW); 

  // if (car_speed < 50)
    analogWrite(enA, int(float(car_speed)));
  // else
  //   analogWrite(enA, int(MOTOR_COMPENSATION * float(car_speed)));
  analogWrite(enB, car_speed);
}

void backward(int car_speed)
{
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 

  // if (car_speed < 50)
    analogWrite(enA, int(float(car_speed)));
  // else
  //   analogWrite(enA, int(MOTOR_COMPENSATION * float(car_speed)));
  analogWrite(enB, car_speed);
}

void setup(void) 
{
  Serial.begin(115200);
  Serial.print("\n starting up \n");
  
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  gyro.enableAutoRange(true);
  gyro.begin();

  accel.init();
  accel.enableDefault();

  start_motor();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(samplerate);

  delay(5000);
  Serial.print("starting calibration \n");
  gyro_offset = calibrate_gyro(&gyro, 1000);

  Serial.print("initialized \n");
  blink_LED(3, 200);
  previous = millis(); 
}

void loop(void) 
{
  now = millis();

  if (now - previous >= samplerate)
  {
    previous = millis();
    gyro.getEvent(&gyro_event);
    angle += RAD2DEG * (gyro_event.gyro.y - gyro_offset) * interval; // pitch
    // Serial.print("angle:"+String(angle)+",drive:"+String(drive)+"\n");
  }

  if (angle >= setpoint && !start) {
      Serial.print("PID started \n");
      start = true;
  }

  if (start)
  {
    myPID.Compute();

    drive = round(abs(power));
    if (drive > 255) drive = 255;

    if (power >= 0) {
      forward(drive);
    } else if (power < 0) {
      backward(drive);
    }
  }

}