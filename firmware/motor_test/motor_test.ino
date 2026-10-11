// Bring-up test for the reject arm: moves the stepper only on command over USB serial.
//
// Wiring (TB6600-type driver, ENA not connected = always enabled):
//   PUL+ -> D2   PUL- -> GND
//   DIR+ -> D3   DIR- -> GND
//
// Commands (115200 baud, newline-terminated):
//   f<N>   move N steps forward        b<N>   move N steps back
//   v<N>   set max speed (steps/s)     a<N>   set acceleration (steps/s^2)
//   p      print current position      z      zero the position here
#include <AccelStepper.h>

const int STEP_PIN = 2;
const int DIR_PIN = 3;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
String line;

void setup() {
  Serial.begin(115200);
  stepper.setMinPulseWidth(5);  // TB6600 needs >= 2.5 us pulses
  stepper.setMaxSpeed(800);
  stepper.setAcceleration(1600);
  Serial.println(F("motor_test ready: f<N> b<N> v<N> a<N> p z"));
}

void handle(const String& cmd) {
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);
  long n = cmd.substring(1).toInt();
  if (c == 'f' || c == 'b') {
    long target = stepper.currentPosition() + (c == 'f' ? n : -n);
    unsigned long start = millis();
    stepper.moveTo(target);
    stepper.runToPosition();
    Serial.print(F("done pos="));
    Serial.print(stepper.currentPosition());
    Serial.print(F(" ms="));
    Serial.println(millis() - start);
  } else if (c == 'v') {
    stepper.setMaxSpeed(n);
    Serial.print(F("speed=")); Serial.println(n);
  } else if (c == 'a') {
    stepper.setAcceleration(n);
    Serial.print(F("accel=")); Serial.println(n);
  } else if (c == 'p') {
    Serial.print(F("pos=")); Serial.println(stepper.currentPosition());
  } else if (c == 'z') {
    stepper.setCurrentPosition(0);
    Serial.println(F("pos=0"));
  } else {
    Serial.println(F("unknown command"));
  }
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      line.trim();
      handle(line);
      line = "";
    } else {
      line += ch;
    }
  }
}
