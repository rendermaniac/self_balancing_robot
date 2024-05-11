#include <Bluepad32.h>

#define LED_BUILTIN 33

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

int x = 0;
int y = 0;
char Pos[10];
int count = 0;

bool updateNeutral = false;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == nullptr) {
      myGamepads[i] = gp;
      foundEmptySlot = true;
      Serial.println("Gamepad connected");
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

struct data
{
  int x;
  int y;
};

data d;
data previous;

void setup() {
  Serial.begin(115200);
  pinMode (LED_BUILTIN, OUTPUT);
  Serial.println("intialising ESP32");

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.forgetBluetoothKeys();
}

void loop() {

  BP32.update();

  GamepadPtr myGamepad = myGamepads[0];
  if (myGamepad && myGamepad->isConnected()) {

    if (myGamepad->x()) {
      // Serial.print("X");
      digitalWrite(LED_BUILTIN, LOW);
    }

    if (myGamepad->y()) {
      digitalWrite(LED_BUILTIN, LOW);
      // Serial.print("Y");
    }

    if (myGamepad->b()) {
      digitalWrite(LED_BUILTIN, LOW);
      // Serial.print("B");
    }

    if (myGamepad->a()) {
      digitalWrite(LED_BUILTIN, LOW);
      // Serial.print("A");
    }

    // x = myGamepad->axisX(); // (-511 - 512) left X Axis   
    // if (x < -200) {
    //   digitalWrite(LED_BUILTIN, LOW);
    //   Serial.print("L"); // left
    //   updateNeutral = true;
    // } else if (x > 200) {
    //   digitalWrite(LED_BUILTIN, LOW);
    //   Serial.print("R"); // right
    //   updateNeutral = true;
    // }

    // y = myGamepad->axisY(); // (-511 - 512) left Y axis
    // if (y < -200) {
    //   digitalWrite(LED_BUILTIN, LOW);
    //   Serial.print("U"); // forward
    //   updateNeutral = true;
    // } else if (y > 200) {
    //   digitalWrite(LED_BUILTIN, LOW);
    //   Serial.print("D"); // backward
    //   updateNeutral = true;
    // }

    // if (x < 30 && x > -30 && y < 30 && y > -30 && updateNeutral) {
    //   digitalWrite(LED_BUILTIN, LOW);
    //   Serial.print("N"); // neutral 
    //   updateNeutral = false;   
    // }
    // Serial.println("x: "+String(x)+"y: "+String(y));

    d.x = myGamepad->axisX();
    d.y = myGamepad->axisY();

    if (abs(d.x - previous.x) > 20 || abs(d.y - previous.y) > 20 ) {
      Serial.write((byte*)&d, sizeof(d));
      digitalWrite(LED_BUILTIN, LOW);
      previous = d;
    }
  }

   if (count == 0) {
    digitalWrite(LED_BUILTIN, LOW);
  }

  delay(100); // debounce

  digitalWrite(LED_BUILTIN, HIGH);

  count += 1;
  if (count >= 20) {
    count = 0;
  }

}
