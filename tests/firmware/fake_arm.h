// A simulated stepper for the PC tests: moves toward its target at max speed (no ramps)
// as simulated time advances, and can be stalled to provoke a move timeout.
#pragma once
#include "../../firmware/reject_cell/RejectController.h"

class FakeArm : public Actuator {
 public:
  long position() override { return pos_; }
  void moveTo(long target) override { target_ = target; }
  bool moving() override { return pos_ != target_; }
  void halt() override {
    target_ = pos_;
    halts++;
  }
  void configure(float maxSpeed, float acceleration) override {
    speed = maxSpeed;
    accel = acceleration;
  }

  // advance one millisecond
  void tick() {
    if (stalled || pos_ == target_) return;
    credit_ += speed / 1000.0f;
    while (credit_ >= 1.0f && pos_ != target_) {
      pos_ += target_ > pos_ ? 1 : -1;
      credit_ -= 1.0f;
      if (pos_ > maxPos) maxPos = pos_;
      if (pos_ < minPos) minPos = pos_;
    }
    if (pos_ == target_) credit_ = 0;
  }

  // the Mega restarts: AccelStepper counts from 0 again, wherever the stick physically is
  void reboot() {
    pos_ = target_ = 0;
    credit_ = 0;
  }

  long target() const { return target_; }

  float speed = 1000;
  float accel = 0;
  bool stalled = false;
  int halts = 0;
  long maxPos = 0;
  long minPos = 0;

 private:
  long pos_ = 0;
  long target_ = 0;
  float credit_ = 0;
};

// The EEPROM flag, in memory; counts real writes (EEPROM.update skips unchanged values)
struct FakePersist : public Persist {
  bool away = false;
  int writes = 0;
  bool awayFromHome() override { return away; }
  void setAwayFromHome(bool v) override {
    if (v != away) writes++;
    away = v;
  }
};

// Controller + registers + arm on a simulated millisecond clock, with the master's heartbeat.
struct Rig {
  Registers regs{};
  FakeArm arm;
  FakePersist persist;
  RejectController ctrl{regs, arm, persist};
  uint32_t now;
  bool masterBeating = true;

  explicit Rig(uint32_t start = 0) : now(start) { ctrl.begin(now); }

  // advance `ms` milliseconds; the master bumps its heartbeat every 100 ms while beating
  void run(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
      now++;
      if (masterBeating && now % 100 == 0) regs.holding[reg::MASTER_HEARTBEAT]++;
      arm.tick();
      ctrl.update(now);
    }
  }

  bool ready() const { return regs.discreteInputs[reg::READY]; }
  bool busy() const { return regs.discreteInputs[reg::BUSY]; }
  bool done() const { return regs.discreteInputs[reg::DONE]; }
  bool faulted() const { return regs.discreteInputs[reg::FAULT]; }
  uint16_t faultCode() const { return regs.input[reg::FAULT_CODE]; }
  uint16_t cycles() const { return regs.input[reg::CYCLE_COUNT]; }
  void setRequest(bool on) { regs.coils[reg::REQ_REJECT] = on; }
  void setReset(bool on) { regs.coils[reg::RESET_FAULT] = on; }
};
