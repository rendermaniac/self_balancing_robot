#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <LSM303.h>

#define RAD2DEG 57.2958
#define NORMALIZE_1G 16735.0

#define ALPHA 0.02

unsigned long samplerate = 5;
float interval = samplerate / 1000.0;

float gyro_offset = 0.0;
float dx = 0.0;
float ga = 0.0;
sensors_event_t event;

float acc_offset = 0.0;
float aa = 0.0;
float ax = 0.0;
float ay = 0.0;
float az = 0.0;

float ca = 0.0;

unsigned long now, previous;

LSM303 accel;
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

float calibrate_gyro(Adafruit_L3GD20_Unified *gyro, int calibration_samples)
{
  float offset = 0.0;
  sensors_event_t event;

  for (uint16_t sample = 0; sample < calibration_samples; sample++) {
    gyro->getEvent(&event);
    offset += event.gyro.x - 0.0;
    delay(10);
  }
  return offset / calibration_samples;
}

void get_accel_data(float& ax, float& ay, float& az, LSM303 *accel)
{
  accel->read();
  ax = accel->a.x / NORMALIZE_1G; 
  ay = accel->a.y / NORMALIZE_1G;
  az = accel->a.z / NORMALIZE_1G;
}

float get_accel_angle(float x, float z)
{
  float angle = RAD2DEG * asin(x / 1.0);
  if (z > 0.0) angle = 180.0 - angle;
  if (z == 0.0) angle = 90.0;
  return angle;
}

float calibrate_accel(LSM303 *accel, int calibration_samples)
{
  float offset = 0.0;

  float axx = 0.0;
  float ayy = 0.0;
  float azz = 0.0;

  for (uint16_t sample = 0; sample < calibration_samples; sample++) {
    get_accel_data(axx, ayy, azz, accel);
    offset += get_accel_angle(axx, azz);
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

void setup(void) 
{
  Serial.begin(115200);
  gyro.enableAutoRange(true);
  gyro.begin();

  accel.init();
  accel.enableDefault();

  Serial.println("starting calibration");
  delay(5000);
  gyro_offset = calibrate_gyro(&gyro, 1000);
  acc_offset = calibrate_accel(&accel, 1000);
  Serial.println("finished calibration");
  previous = millis(); 
}

void loop(void) 
{
  now = millis();

  if (now - previous >= samplerate)
  {
    previous = millis();
    
    gyro.getEvent(&event);
    float dx = RAD2DEG * (event.gyro.x - gyro_offset) * interval;

    ga += dx;

    get_accel_data(ax, ay, az, &accel);
    aa = get_accel_angle(ax, az) - acc_offset;

    ca = compute_complementary_angle(ca, dx, aa, ALPHA);

    Serial.print("gyro:"+String(ga)+",");
    Serial.print("accel:"+String(aa)+",");
    Serial.print("comp:"+String(ca)+",");
    Serial.print("min:0.0,max:90.0");
    Serial.print("\n");
  }
}