// Reject arm as a Modbus RTU slave (design spec §7): id 1, 115200 baud 8N1 over the USB serial.
// The handshake, watchdog and fault logic live in RejectController (unit-tested on the PC in
// tests/firmware); this file only wires it to the Modbus registers and the stepper.
//
// Wiring (TB6600-type driver, ENA not connected = always enabled):
//   PUL+ -> D2   PUL- -> GND
//   DIR+ -> D3   DIR- -> GND
//
// Libraries: AccelStepper, ModbusRTUSlave (3.x; pulls in ModbusADU, ModbusRTUComm, ModbusSlaveLogic).
//
// Everything here is non-blocking: moves run one step per loop(), so the slave keeps answering
// polls and beating its heartbeat during a strike. Each Modbus request still holds loop() for
// ~4-5 ms (receive + frame gap + reply), during which no steps are taken; the master polls
// every 50 ms, so a strike runs a little slower than the set speed, but the position stays exact.
#include <AccelStepper.h>
#include <ModbusRTUSlave.h>

#include "RejectController.h"

const int STEP_PIN = 2;
const int DIR_PIN = 3;
const uint8_t SLAVE_ID = 1;
const unsigned long BAUD = 115200;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
ModbusRTUSlave modbus(Serial);
Registers regs;

class StepperArm : public Actuator {
 public:
  long position() override { return stepper.currentPosition(); }
  void moveTo(long target) override { stepper.moveTo(target); }
  bool moving() override { return stepper.distanceToGo() != 0; }
  // setCurrentPosition() also zeroes the speed and sets the target to here: an immediate stop
  void halt() override { stepper.setCurrentPosition(stepper.currentPosition()); }
  void configure(float maxSpeed, float acceleration) override {
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(acceleration);
  }
};

StepperArm arm;
RejectController controller(regs, arm);

void setup() {
  stepper.setMinPulseWidth(5);  // TB6600 needs >= 2.5 us pulses
  modbus.configureCoils(regs.coils, reg::COIL_COUNT);
  modbus.configureDiscreteInputs(regs.discreteInputs, reg::DISCRETE_COUNT);
  modbus.configureHoldingRegisters(regs.holding, reg::HOLDING_COUNT);
  modbus.configureInputRegisters(regs.input, reg::INPUT_COUNT);
  Serial.begin(BAUD);
  modbus.begin(SLAVE_ID, BAUD);
  controller.begin(millis());
}

void loop() {
  modbus.poll();
  controller.update(millis());
  stepper.run();
}
