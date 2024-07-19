// See https://www.instructables.com/Arduino-Self-Balancing-Robot-1/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <LSM303.h>
#include <PID_v1.h>

#define EPSILON 0.000001
#define RAD2DEG 57.2958
#define NORMALIZE_1G 16735.0

#define ALPHA 0.001

#define MIN_DRIVE_SIGNAL 0
#define BITE_DRIVE_SIGNAL 40
#define MAX_DRIVE_SIGNAL 255

unsigned long samplerate = 8.0;
double interval = samplerate / 1000.0;

sensors_event_t gyro_event;
double gyro_offset = 0.0;
double gyro_compensation = 0.94; // 0.9562;

double acc_offset = 0.0;
double ax = 0.0;
double ay = 0.0;
double az = 0.0;

unsigned long now, previous;

double angle = 0.0;
double power = 0.0;
double setpoint_stationary = 79.5;
double setpoint = setpoint_stationary;
double abortpoint = 50.0;
bool startPID = false;
bool abortPID = false;

double kP = 18; // 20;
double kI = 100;
double kD = 0.3;

int offsetA = 0;
int offsetB = 0;

double compensationA = 1.0;
double compensationB = 1.0;

// ESP32
int rx = 10; // brown
int tx = 11; // white

SoftwareSerial esp32Serial =  SoftwareSerial(rx, tx);

uint8_t syncHeader[] = {0xFA, 0xCE};

struct __attribute__((packed, aligned(1))) data
{
  int16_t x;
  int16_t y;
  char mode;
  char dpad;
};

data d;

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
PID myPID(&angle, &power, &setpoint, kP, kI, kD, P_ON_M, DIRECT);

int trig = 12; // purple
int echo = 6; // grey
int setpoint_distance = 0.0;
double distance = 0.0;

void blink_LED(int times, int delayTime)
{
  for(int i=0; i<times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delayTime);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delayTime);
  }
}

double get_gyro_anglerate()
{
  gyro.getEvent(&gyro_event);
  return gyro_compensation * RAD2DEG * (gyro_event.gyro.x - gyro_offset);
}

double calibrate_gyro(int calibration_samples)
{
  double offset = 0.0;
  for (int sample = 0; sample < calibration_samples; sample++) {
    gyro.getEvent(&gyro_event);
    offset += double(gyro_event.gyro.x);
    delay(10);
  }
  return offset / calibration_samples;
}

void get_accel_data(double& x, double& y, double& z, LSM303 *accelerometer)
{
  accelerometer->read();
  x = accelerometer->a.x / NORMALIZE_1G; 
  y = accelerometer->a.y / NORMALIZE_1G;
  z = accelerometer->a.z / NORMALIZE_1G;
}

double get_accel_angle(double x, double z)
{
  double angle = RAD2DEG * asin(x / 1.0);
  if (z > 0.0) angle = 180.0 - angle;
  if (z == 0.0) angle = 90.0;
  return angle;
}

double calibrate_accel(LSM303 *accelerometer, int calibration_samples)
{
  double offset = 0.0;

  double axx = 0.0;
  double ayy = 0.0;
  double azz = 0.0;

  for (uint16_t sample = 0; sample < calibration_samples; sample++) {
    get_accel_data(axx, ayy, azz, accelerometer);
    offset += get_accel_angle(axx, azz);
    delay(10);
  }
  return offset / calibration_samples;
}

double get_distance()
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH);
  return duration * 0.034 / 2;
}

double compute_complementary_angle(double comp_angle, double d_gyro_angle, double acc_angle, double alpha)
{
    if (!isnan(comp_angle) && !isnan(d_gyro_angle) && !isnan(acc_angle))
      return (acc_angle * alpha) + ((1.0 - alpha) * (comp_angle + d_gyro_angle));
    else
      return comp_angle;
}

void stop_motors()
{
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void drive_motor(int drive, int h1, int h2, int en)
{
  if (drive >= 0) {
    digitalWrite(h1, HIGH);   // forwards
    digitalWrite(h2, LOW);
  } else {
    digitalWrite(h1, LOW); // backwards
    digitalWrite(h2, HIGH); 
  }

  int speed = abs(drive);
  speed += BITE_DRIVE_SIGNAL;
  speed = max(speed, MIN_DRIVE_SIGNAL);
  speed = min(speed, MAX_DRIVE_SIGNAL);
  analogWrite(en, speed);
}

void drive_motorA(int drive)
{
  drive_motor(drive, in1, in2, enA);
}

void drive_motorB(int drive)
{
  drive_motor(drive, in3, in4, enB);
}

void printData() {
  String data;
  // data += ",P:"  + String(kP);
  // data += ",I:"  + String(kI);
  // data += ",D:"  + String(kD);

  // data += ",compensationA:"  + String(compensationA);
  // data += ",compensationB:" + String(compensationB);

  // data += ",x:90.0";
  // data += ",s:"  + String(setpoint);
  // data += ",a:" + String(angle);

  data += ",ds:" + String(setpoint_distance);
  data += ",d:" + String(distance);

  // data += ",oa:"  + String(offsetA);
  // data += ",ob:" + String(offsetB);

  // data += ",started:" + String(startPID);
  // data += ",aborted:" + String(abortPID);
  Serial.println(data);
}

void setup(void) 
{
  Serial.begin(115200);
  esp32Serial.begin(9600);
  Serial.println("\n starting up");
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on until ready

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  gyro.enableAutoRange(true);
  gyro.begin();

  // accel.init();
  // accel.enableDefault();

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(samplerate);

  delay(5000);
  Serial.println("starting calibration");
  gyro_offset = calibrate_gyro(1000);
  // acc_offset = calibrate_accel(&accel, 1000);

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

    distance = get_distance();
    double gyro_angle_difference = get_gyro_anglerate() * interval;

    if (abs(setpoint_distance - distance) < EPSILON)
      angle = setpoint;

    // get_accel_data(ax, ay, az, &accel);
    // double accel_angle = get_accel_angle(ax, az) - acc_offset;

    angle += gyro_angle_difference;
    // angle = compute_complementary_angle(angle, gyro_angle_difference, accel_angle, ALPHA);
    // Serial.println("min:70.0,max:100.0,setpoint:"+String(setpoint)+",angle:"+String(angle));

    if (angle >= setpoint && !startPID && !abortPID) {
        setpoint_distance = distance;
        Serial.print("PID started \n");
        startPID = true;
    }

    if (abs(setpoint - angle) > abortpoint && startPID && !abortPID) {
        Serial.print("Aborting! \n");
        startPID = false;
        abortPID = true;
        stop_motors();
    }

    if (esp32Serial.available() >= (sizeof(syncHeader) + sizeof(data)) ) {
      
      uint16_t sync = (uint16_t)esp32Serial.read() << 8 | esp32Serial.read();

      if (sync == 0xFACE) {
        byte buffer[sizeof(data)];
        esp32Serial.readBytes((byte*)&buffer, sizeof(data));
        memcpy(&d, &buffer, sizeof(data));

        // forwards -y backwards +y
        setpoint = setpoint_stationary - (d.y / 150);
        offsetA = d.x / 16.0;
        offsetB = -d.x / 16.0;

        if (d.mode == 'y') {
          if (d.dpad == 'u') kP += 1;
          if (d.dpad == 'd') kP -= 1;
          kP = max(kP, 0.0);
        }

        if (d.mode == 'x') {
          if (d.dpad == 'u') kI += 10;
          if (d.dpad == 'd') kI -= 10;
          kI = max(kI, 0.0);
        }

        if (d.mode == 'b') {
          if (d.dpad == 'u') kD += 0.1;
          if (d.dpad == 'd') kD -= 0.1;
          kD = max(kD, 0.0);
        }

        if (d.mode == 'a') {
          if (d.dpad == 'u') setpoint_stationary += 0.1;
          if (d.dpad == 'd') setpoint_stationary -= 0.1;
          if (d.dpad == 'l') compensationA += 0.1;
          if (d.dpad == 'r') compensationB += 0.1;
        }

        myPID.SetTunings(kP, kI, kD);
      }
    }

    // printData();

    if (startPID)
    { 
      myPID.Compute();

      drive_motorA(int(power * compensationA) + offsetA);
      drive_motorB(int(power * compensationB) + offsetB);
    }
  }

}