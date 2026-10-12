// Reject-arm logic behind the Modbus registers (design spec §7).
//
// Free of Arduino calls, so the same file runs on the Mega and in the PC unit tests
// (tests/firmware). reject_cell.ino only wires it to ModbusRTUSlave and AccelStepper.
//
// Reject handshake:  master: READY? -> REQ_REJECT=1
//                    slave:  BUSY=1, strike out, return home, BUSY=0, DONE=1
//                    master: DONE  -> REQ_REJECT=0
//                    slave:  DONE=0 -> READY
// Watchdog: master heartbeat unchanged for 1 s -> stop the motor, FAULT_CODE=COMM_LOST.
#pragma once
#include <stdint.h>

namespace reg {
// coils, written by the master
enum Coil : uint8_t { REQ_REJECT, RESET_FAULT, COIL_COUNT };
// discrete inputs, read by the master
enum DiscreteInput : uint8_t { READY, BUSY, DONE, FAULT, DISCRETE_COUNT };
// holding registers, written by the master
enum Holding : uint8_t { MASTER_HEARTBEAT, STRIKE_STEPS, MAX_SPEED, ACCELERATION, HOLDING_COUNT };
// input registers, read by the master
enum Input : uint8_t { SLAVE_HEARTBEAT, FAULT_CODE, CYCLE_COUNT, INPUT_COUNT };
}  // namespace reg

enum FaultCode : uint16_t { FAULT_NONE = 0, FAULT_COMM_LOST = 1, FAULT_MOVE_TIMEOUT = 2, FAULT_NOT_HOME = 3 };

// The arrays ModbusRTUSlave serves; the controller reads and writes them directly.
struct Registers {
  bool coils[reg::COIL_COUNT];
  bool discreteInputs[reg::DISCRETE_COUNT];
  uint16_t holding[reg::HOLDING_COUNT];
  uint16_t input[reg::INPUT_COUNT];
};

// The motor as the controller sees it. Moves are non-blocking: the sketch steps the motor in loop().
class Actuator {
 public:
  virtual long position() = 0;
  virtual void moveTo(long target) = 0;
  virtual bool moving() = 0;
  virtual void halt() = 0;  // stop at once, no deceleration
  virtual void configure(float maxSpeed, float acceleration) = 0;
};

// One flag that survives a reset (EEPROM on the Mega): the stick is away from home. Home is the
// power-on position, so a restart while away would silently move home; the flag catches that.
class Persist {
 public:
  virtual bool awayFromHome() = 0;
  virtual void setAwayFromHome(bool away) = 0;
};

class RejectController {
 public:
  static const uint32_t HEARTBEAT_MS = 100;      // slave heartbeat period
  static const uint32_t WATCHDOG_MS = 1000;      // master heartbeat must change within this
  // stroke timeout = ideal stroke time (trapezoidal speed profile) x STROKE_SLACK + STROKE_MARGIN_MS;
  // the slack covers the ~25 % of loop() the Modbus requests take. cell/actuator.py mirrors this.
  static constexpr float STROKE_SLACK = 1.5f;
  static const uint32_t STROKE_MARGIN_MS = 500;
  static uint32_t strokeTimeoutMs(long steps, float speed, float accel);

  // defaults until the master writes its own; 3200 steps/rev, so 800 = a quarter turn
  static const int16_t DEFAULT_STRIKE_STEPS = 800;
  static const uint16_t DEFAULT_MAX_SPEED = 2400;      // steps/s
  static const uint16_t DEFAULT_ACCELERATION = 12000;  // steps/s^2

  enum State : uint8_t { WAIT_MASTER, IDLE, STRIKE_OUT, STRIKE_BACK, DONE, RETURNING, FAULTED };

  RejectController(Registers& regs, Actuator& arm, Persist& persist)
      : regs_(regs), arm_(arm), persist_(persist) {}

  // Power-on: resets all state. The power-on position is home (no limit switch), unless the flag says the stick was away.
  void begin(uint32_t now);
  // Call every loop(), after modbus.poll().
  void update(uint32_t now);

  State state() const { return state_; }
  long home() const { return home_; }

 private:
  void startStroke(long target, State next, uint32_t now);
  void fault(FaultCode code);
  void publish();

  Registers& regs_;
  Actuator& arm_;
  Persist& persist_;
  State state_ = WAIT_MASTER;
  long home_ = 0;
  long strikeTarget_ = 0;
  uint32_t strokeStart_ = 0;
  uint32_t moveTimeoutMs_ = 0;
  uint32_t lastBeat_ = 0;
  uint16_t lastMasterBeat_ = 0;
  uint32_t masterBeatAt_ = 0;
  bool masterSeen_ = false;
  bool requestArmed_ = false;  // REQ_REJECT seen low since the last cycle: one strike per request
  bool prevReset_ = false;
  uint16_t faultCode_ = FAULT_NONE;
  uint16_t cycleCount_ = 0;
};
