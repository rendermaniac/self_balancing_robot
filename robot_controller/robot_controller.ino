#include <Bluepad32.h>

#define LED_BUILTIN 33
#define ALPHA 0.02

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];
int count = 0;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == nullptr) {
      myGamepads[i] = gp;
      foundEmptySlot = true;
      Serial.println("Gamepad connected!");
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
        "CALLBACK: Gamepad connected, but could not found empty slot");
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == gp) {
      Serial.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
      myGamepads[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
        "CALLBACK: Gamepad disconnected, but not found in myGamepads");
  }
}

uint8_t syncHeader[] = {0xFA, 0xCE};

struct __attribute__((packed, aligned(1))) data
{
  int16_t x;
  int16_t y;
  char mode;
  char dpad;
};

data d;
data previous;

void setup() {
  Serial.begin(9600);
  pinMode (LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("intialising ESP32");

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.forgetBluetoothKeys();
}

void loop() {

  BP32.update();

  GamepadPtr myGamepad = myGamepads[0];
  if (myGamepad && myGamepad->isConnected()) {

    digitalWrite(LED_BUILTIN, HIGH);

    int x = myGamepad->axisX();
    if (x > -15 && x < 15) x = 0;

    int y = myGamepad->axisY();
    if (y > -15 && y < 15) y = 0;

    d.x = (ALPHA * previous.x) + ((1.0 - ALPHA) * x);
    d.y = (ALPHA * previous.y) + ((1.0 - ALPHA) * y);

    uint8_t dpad = myGamepad->dpad();  
    if (dpad == DPAD_UP) d.dpad = 'u';
    else if (dpad == DPAD_DOWN) d.dpad = 'd';
    else if (dpad == DPAD_LEFT) d.dpad = 'l';
    else if (dpad == DPAD_RIGHT) d.dpad = 'r';
    else d.dpad = ' ';

    if (myGamepad->x()) d.mode = 'x';
    if (myGamepad->y()) d.mode = 'y';
    if (myGamepad->b()) d.mode = 'b';
    if (myGamepad->a()) d.mode = 'a';


    if (d.x != previous.x || d.y != previous.y || d.mode != previous.mode || d.dpad != previous.dpad)
    {
  
      // Serial.print("x:");
      // Serial.print(d.x);
      // Serial.print(",y:");
      // Serial.print(d.y);
      // Serial.print(",mode:");
      // Serial.print(d.mode);
      // Serial.print("\n");
  
      // write header
      Serial.write(syncHeader[0]);
      Serial.write(syncHeader[1]);

      // write data
      byte buffer[sizeof(data)];
      memcpy(&buffer, &d, sizeof(data));

      Serial.write(buffer, sizeof(data));
      digitalWrite(LED_BUILTIN, LOW);
      
      previous = d;
    }

    if (count == 0) {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  delay(100); // debounce

  count += 1;
  if (count >= 20) {
    count = 0;
  }

}
