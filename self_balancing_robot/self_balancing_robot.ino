// See https://www.instructables.com/Arduino-Self-Balancing-Robot-1/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <LSM303.h>
#include <PID_v1.h>

#define RAD2DEG 57.2958
#define NORMALIZE_1G 16735.0

#define ALPHA 0.001

#define MOTOR_COMPENSATION 1.01

#define MODE_M 0
#define MODE_P 1
#define MODE_I 2
#define MODE_D 3

#define SETPOINT_STATIONARY 84.0
#define SETPOINT_FORWARDS (SETPOINT_STATIONARY + 5.0)
#define SETPOINT_BACKWARDS (SETPOINT_STATIONARY - 5.0)

unsigned long samplerate = 2.0;
float interval = samplerate / 1000.0;

float gyro_offset = 0.0;

float acc_offset = 0.0;
float ax = 0.0;
float ay = 0.0;
float az = 0.0;

unsigned long now, previous;

double angle = 0.0;
double power = 0.0;
int drive = 0;

double setpoint = SETPOINT_STATIONARY;
double abortpoint = 30.0;
bool startPID = false;
bool abortPID = false;

double kP = 40.0;
double kI = 250.0;
double kD = 1.1;

int mode = MODE_M;

int OffsetLeft = 0;
int OffsetRight = 0;

// ESP32
int rx = 11; // brown
int tx = 12; // white

SoftwareSerial esp32Serial =  SoftwareSerial(rx, tx);

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

void blink_LED(int times, int delayTime)
{
  for(int i=0; i<times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delayTime);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delayTime);
  }
}

float calibrate_gyro(Adafruit_L3GD20_Unified *gyroscope, int calibration_samples)
{
  float offset = 0.0;
  sensors_event_t event;

  for (uint16_t sample = 0; sample < calibration_samples; sample++) {
    gyroscope->getEvent(&event);
    offset += float(event.gyro.x);
    delay(10);
  }
  return offset / calibration_samples;
}

void get_accel_data(float& x, float& y, float& z, LSM303 *accelerometer)
{
  accelerometer->read();
  x = accelerometer->a.x / NORMALIZE_1G; 
  y = accelerometer->a.y / NORMALIZE_1G;
  z = accelerometer->a.z / NORMALIZE_1G;
}

float get_accel_angle(float x, float z)
{
  float angle = RAD2DEG * asin(x / 1.0);
  if (z > 0.0) angle = 180.0 - angle;
  if (z == 0.0) angle = 90.0;
  return angle;
}

float calibrate_accel(LSM303 *accelerometer, int calibration_samples)
{
  float offset = 0.0;

  float axx = 0.0;
  float ayy = 0.0;
  float azz = 0.0;

  for (uint16_t sample = 0; sample < calibration_samples; sample++) {
    get_accel_data(axx, ayy, azz, accelerometer);
    offset += get_accel_angle(axx, azz);
    delay(10);
  }
  return offset / calibration_samples;
}

float compute_complementary_angle(float comp_angle, float d_gyro_angle, float acc_angle, float alpha)
{
    if (!isnan(comp_angle) && !isnan(d_gyro_angle) && !isnan(acc_angle))
      return (acc_angle * alpha) + ((1.0 - alpha) * (comp_angle + d_gyro_angle));
    else
      return comp_angle;
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

void stop_motor()
{
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void forward(int car_speed, int OffsetLeft, int OffsetRight)
{  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW); 

  analogWrite(enA, car_speed * MOTOR_COMPENSATION + OffsetLeft);
  analogWrite(enB, car_speed + OffsetRight);
}

void backward(int car_speed, int OffsetLeft, int OffsetRight)
{ 
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 

  analogWrite(enA, car_speed * MOTOR_COMPENSATION + OffsetLeft);
  analogWrite(enB, car_speed + OffsetRight);
}

void printData() {
  String data;
  data += "mode:" + String(mode);
  data += ",P:"  + String(kP);
  data += ",I:"  + String(kI);
  data += ",D:"  + String(kD);
  data += ",setpoint:"  + String(setpoint);
  data += ",OffsetLeft:"  + String(OffsetLeft);
  data += ",OffsetRight:" + String(OffsetRight);
  data += ",angle:" + String(angle);
  data += ",started:" + String(startPID);
  data += ",aborted:" + String(abortPID);
  Serial.println(data);
}

void setup(void) 
{
  Serial.begin(115200);
  esp32Serial.begin(115200);
  Serial.println("\n starting up");
  
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
  Serial.println("starting calibration");
  gyro_offset = calibrate_gyro(&gyro, 1000);
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

    sensors_event_t event;
    gyro.getEvent(&event);
    float gyro_angle_difference = RAD2DEG * ((event.gyro.x - gyro_offset) * interval);

    // get_accel_data(ax, ay, az, &accel);
    // float accel_angle = get_accel_angle(ax, az) - acc_offset;

    angle += gyro_angle_difference;
    // angle = compute_complementary_angle(angle, gyro_angle_difference, accel_angle, ALPHA);

    // Serial.println("min:70.0,max:100.0,setpoint:"+String(setpoint)+",angle:"+String(angle));

  }

  if (angle >= setpoint && !startPID && !abortPID) {
      Serial.print("PID started \n");
      startPID = true;
  }

  if (abs(setpoint - angle) > abortpoint && startPID && !abortPID) {
      Serial.print("Aborting! \n");
      startPID = false;
      abortPID = true;
  }

  if (esp32Serial.available() > 0) {
    char command = esp32Serial.read();

    // if (command == 'Y') {
    //   mode = MODE_P;
    // } else if (command == 'X') {
    //   mode = MODE_I;
    // } else if (command == 'A') {
    //   mode = MODE_D;
    // } else if (command == 'B') {
    //   mode = MODE_M;
    // }

    // if (mode == MODE_P) {
    //   if (command == 'U') {
    //     kP += 1.0;
    //   } else if (command == 'D') {
    //     kP -= 1.0;
    //   }
    // }

    // if (mode == MODE_I) {
    //   if (command == 'U') {
    //     kI += 10.0;
    //   } else if (command == 'D') {
    //     kI -= 10.0;
    //   }
    // }

    // if (mode == MODE_D) {
    //   if (command == 'U') {
    //     kD += 0.1;
    //   } else if (command == 'D') {
    //     kD -= 0.1;
    //   }
    // }

    if (true) { // mode == MODE_M
      if (command == 'U') {
        setpoint = SETPOINT_FORWARDS; // min(setpoint + 0.5, SETPOINT_FORWARDS);
        Serial.println("forwards setpoint "+String(setpoint));
      } else if (command == 'D') {
        setpoint = SETPOINT_BACKWARDS; // max(setpoint - 0.5, SETPOINT_BACKWARDS);
        Serial.println("backwards setpoint "+String(setpoint));
      } else if (command == 'L') {
        OffsetLeft -= 2;
        OffsetRight += 2;
      } else if (command == 'R') {
        OffsetLeft += 2;
        OffsetRight -= -2;
      } else if (command == 'N') {
        float diff = setpoint - SETPOINT_STATIONARY;
        setpoint -= 0.1 * diff;
        // setpoint = SETPOINT_STATIONARY;
        Serial.println("stationary setpoint "+String(setpoint));
        OffsetLeft = 0;
        OffsetRight = 0;
      }
    }

  myPID.SetTunings(kP, kI, kD);
  // printData();
  }

  if (startPID)
  { 
    myPID.Compute();

    drive = round(abs(power));
    if (drive > 255) drive = 255;

    if (power >= 0) {
      forward(drive, OffsetLeft, OffsetRight);
    } else if (power < 0) {
      backward(drive, OffsetLeft, OffsetRight);
    }
  } else {
    stop_motor();
  }

}