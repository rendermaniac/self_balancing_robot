#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

#define RAD2DEG 57.2958

unsigned long samplerate = 5;
float interval = samplerate / 1000.0;

float gyro_offset = 0.0;
float y = 0.0;
sensors_event_t event;

unsigned long now, previous;

Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

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

void setup(void) 
{
  Serial.begin(115200);
  gyro.enableAutoRange(true);
  gyro.begin();

  delay(5000);
  gyro_offset = calibrate_gyro(&gyro, 1000);
  previous = millis(); 
}

void loop(void) 
{
  now = millis();

  if (now - previous >= samplerate)
  {
    previous = millis(); 
    gyro.getEvent(&event);

    y += RAD2DEG * (event.gyro.y - gyro_offset) * interval;
    Serial.print("Y:"+String(y)+"\n");
  }
}