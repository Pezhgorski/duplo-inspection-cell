// Unit tests for the slave side: RejectController on a simulated clock and arm.
// Built and run by tests/test_firmware.py (any C++11 compiler: g++ / clang++).
#include <cstdio>

#include "fake_arm.h"

static int failures = 0;
static const char* current = "";

#define CHECK(cond)                                                                   \
  do {                                                                                \
    if (!(cond)) {                                                                    \
      std::printf("FAIL %s (%s:%d): %s\n", current, __FILE__, __LINE__, #cond);       \
      failures++;                                                                     \
    }                                                                                 \
  } while (0)

// run until DONE (or give up after `limit` ms); returns the ms it took
static uint32_t runUntilDone(Rig& rig, uint32_t limit = 5000) {
  uint32_t t = 0;
  while (!rig.done() && t < limit) {
    rig.run(1);
    t++;
  }
  return t;
}

static void test_not_ready_until_master_heartbeat() {
  Rig rig;
  rig.masterBeating = false;
  rig.run(5000);
  CHECK(!rig.ready());
  CHECK(!rig.faulted());  // no master yet is not a lost master
  rig.masterBeating = true;
  rig.run(100);
  CHECK(rig.ready());
}

static void test_defaults_written_to_holding_registers() {
  Rig rig;
  CHECK((int16_t)rig.regs.holding[reg::STRIKE_STEPS] == 800);
  CHECK(rig.regs.holding[reg::MAX_SPEED] == 2400);
  CHECK(rig.regs.holding[reg::ACCELERATION] == 12000);
}

static void test_slave_heartbeat_counts_every_100ms() {
  Rig rig;
  uint16_t start = rig.regs.input[reg::SLAVE_HEARTBEAT];
  rig.run(1000);
  CHECK((uint16_t)(rig.regs.input[reg::SLAVE_HEARTBEAT] - start) == 10);
}

static void test_full_reject_cycle() {
  Rig rig;
  rig.run(100);
  CHECK(rig.ready());
  rig.setRequest(true);
  rig.run(1);
  CHECK(rig.busy());
  CHECK(!rig.ready());
  CHECK(rig.arm.target() == 800);
  CHECK(rig.arm.speed == 2400);
  CHECK(rig.arm.accel == 12000);
  runUntilDone(rig);
  CHECK(rig.done());
  CHECK(!rig.busy());
  CHECK(rig.arm.maxPos == 800);      // went all the way out
  CHECK(rig.arm.position() == 0);    // and is back home before DONE
  CHECK(rig.cycles() == 1);
  rig.run(500);
  CHECK(rig.done());                 // DONE holds until the master drops the request
  rig.setRequest(false);
  rig.run(1);
  CHECK(!rig.done());
  CHECK(rig.ready());
}

static void test_held_request_strikes_only_once() {
  Rig rig;
  rig.run(100);
  rig.setRequest(true);
  runUntilDone(rig);
  rig.run(3000);
  CHECK(rig.cycles() == 1);
  CHECK(!rig.ready());
  rig.setRequest(false);
  rig.run(10);
  rig.setRequest(true);
  rig.run(1);
  CHECK(rig.busy());                 // a fresh request starts the next cycle
  runUntilDone(rig);
  CHECK(rig.cycles() == 2);
}

static void test_strike_parameters_from_master() {
  Rig rig;
  rig.regs.holding[reg::STRIKE_STEPS] = (uint16_t)(int16_t)-600;  // reverse direction
  rig.regs.holding[reg::MAX_SPEED] = 1200;
  rig.run(100);
  rig.setRequest(true);
  runUntilDone(rig);
  CHECK(rig.arm.minPos == -600);
  CHECK(rig.arm.maxPos == 0);
  CHECK(rig.arm.speed == 1200);
  CHECK(rig.arm.position() == 0);
}

// a strike at full speed (~0.67 s on FakeArm, which has no ramps) is shorter than the watchdog, so slow it down to catch it mid-move
static void slowStrike(Rig& rig) { rig.regs.holding[reg::MAX_SPEED] = 400; }  // 800 steps = 2 s out

static void test_watchdog_stops_motor_mid_strike() {
  Rig rig;
  slowStrike(rig);
  rig.run(100);
  rig.setRequest(true);
  rig.run(150);
  rig.masterBeating = false;
  rig.run(1100);
  CHECK(rig.faulted());
  CHECK(rig.faultCode() == FAULT_COMM_LOST);
  CHECK(!rig.busy());
  CHECK(!rig.ready());
  CHECK(rig.arm.halts == 1);
  CHECK(!rig.arm.moving());
  CHECK(rig.arm.position() != 0);    // stopped where it was, not driven anywhere
}

static void test_watchdog_while_idle() {
  Rig rig;
  rig.run(500);
  rig.masterBeating = false;
  rig.run(900);
  CHECK(!rig.faulted());             // 1 s grace
  rig.run(300);
  CHECK(rig.faulted());
  CHECK(rig.faultCode() == FAULT_COMM_LOST);
}

static void test_reset_returns_home_then_ready() {
  Rig rig;
  slowStrike(rig);
  rig.run(100);
  rig.setRequest(true);
  rig.run(150);
  rig.masterBeating = false;
  rig.run(1100);
  long stoppedAt = rig.arm.position();
  CHECK(rig.faulted());
  // master is back, drops the stale request and pulses RESET_FAULT
  rig.masterBeating = true;
  rig.setRequest(false);
  rig.run(100);
  CHECK(rig.faulted());              // no reset yet: faults latch
  rig.setReset(true);
  rig.run(1);
  rig.setReset(false);
  CHECK(!rig.faulted());
  CHECK(rig.faultCode() == FAULT_NONE);
  CHECK(!rig.ready());               // still returning
  CHECK(rig.arm.target() == 0);
  rig.run(2500);
  CHECK(rig.ready());
  CHECK(rig.arm.position() == 0);
  CHECK(stoppedAt != 0);
  CHECK(rig.cycles() == 0);          // the interrupted strike is not counted
}

static void test_reset_needs_a_rising_edge() {
  Rig rig;
  rig.run(100);
  rig.setReset(true);                // held high while there is no fault: ignored
  rig.run(100);
  rig.masterBeating = false;
  rig.run(1100);
  CHECK(rig.faulted());
  rig.masterBeating = true;
  rig.run(200);
  CHECK(rig.faulted());              // still high, no new edge
  rig.setReset(false);
  rig.run(1);
  rig.setReset(true);
  rig.run(1);
  CHECK(!rig.faulted());
}

static void test_reset_ignored_while_master_silent() {
  Rig rig;
  rig.run(100);
  rig.masterBeating = false;
  rig.run(1100);
  rig.setReset(true);
  rig.run(1);
  CHECK(rig.faulted());
}

static void test_request_during_fault_does_not_strike_after_reset() {
  Rig rig;
  rig.run(100);
  rig.masterBeating = false;
  rig.run(1100);
  rig.masterBeating = true;
  rig.setRequest(true);              // stale request still set when the fault is cleared
  rig.run(100);
  rig.setReset(true);
  rig.run(1);
  rig.run(100);
  CHECK(rig.ready());
  CHECK(!rig.busy());
  CHECK(rig.arm.maxPos == 0);
  rig.setRequest(false);
  rig.run(1);
  rig.setRequest(true);
  rig.run(1);
  CHECK(rig.busy());
}

static void test_stalled_motor_times_out() {
  Rig rig;
  rig.run(100);
  rig.arm.stalled = true;
  rig.setRequest(true);
  rig.run(1250);
  CHECK(rig.busy());
  rig.run(100);
  CHECK(rig.faulted());
  CHECK(rig.faultCode() == FAULT_MOVE_TIMEOUT);
  CHECK(rig.arm.halts == 1);
}

static void test_stall_on_the_way_back_times_out() {
  Rig rig;
  rig.run(100);
  rig.setRequest(true);
  rig.run(400);                      // out (333 ms) and on the way back
  CHECK(rig.ctrl.state() == RejectController::STRIKE_BACK);
  rig.arm.stalled = true;
  rig.run(2600);
  CHECK(rig.faultCode() == FAULT_MOVE_TIMEOUT);
  CHECK(rig.cycles() == 0);
}

static void test_survives_millis_wraparound() {
  Rig rig(0xFFFFFF00u);              // millis() wraps 256 ms from now
  rig.run(100);
  rig.setRequest(true);
  uint32_t took = runUntilDone(rig);
  CHECK(rig.done());
  CHECK(took < 1000);
  CHECK(!rig.faulted());
  rig.setRequest(false);
  rig.run(2000);
  CHECK(rig.ready());
}

static void test_stroke_timeout_follows_the_settings() {
  uint32_t fast = RejectController::strokeTimeoutMs(800, 2400, 12000);
  CHECK(fast >= 1290 && fast <= 1310);  // 1.5 x 0.533 s + 0.5 s
  CHECK(RejectController::strokeTimeoutMs(-800, 2400, 12000) == fast);
  CHECK(RejectController::strokeTimeoutMs(800, 300, 12000) > 4000);
  CHECK(RejectController::strokeTimeoutMs(800, 0, 0) > 0);  // bad registers: no division by zero
}

static void test_slow_strike_is_not_a_move_timeout() {
  Rig rig;
  rig.regs.holding[reg::MAX_SPEED] = 300;  // 2.7 s per stroke: over the old fixed 2.5 s
  rig.run(100);
  rig.setRequest(true);
  runUntilDone(rig, 8000);
  CHECK(rig.done());
  CHECK(!rig.faulted());
  CHECK(rig.cycles() == 1);
}

int main() {
  struct { const char* name; void (*fn)(); } tests[] = {
      {"not_ready_until_master_heartbeat", test_not_ready_until_master_heartbeat},
      {"defaults_written_to_holding_registers", test_defaults_written_to_holding_registers},
      {"slave_heartbeat_counts_every_100ms", test_slave_heartbeat_counts_every_100ms},
      {"full_reject_cycle", test_full_reject_cycle},
      {"held_request_strikes_only_once", test_held_request_strikes_only_once},
      {"strike_parameters_from_master", test_strike_parameters_from_master},
      {"watchdog_stops_motor_mid_strike", test_watchdog_stops_motor_mid_strike},
      {"watchdog_while_idle", test_watchdog_while_idle},
      {"reset_returns_home_then_ready", test_reset_returns_home_then_ready},
      {"reset_needs_a_rising_edge", test_reset_needs_a_rising_edge},
      {"reset_ignored_while_master_silent", test_reset_ignored_while_master_silent},
      {"request_during_fault_does_not_strike_after_reset", test_request_during_fault_does_not_strike_after_reset},
      {"stalled_motor_times_out", test_stalled_motor_times_out},
      {"stall_on_the_way_back_times_out", test_stall_on_the_way_back_times_out},
      {"survives_millis_wraparound", test_survives_millis_wraparound},
      {"stroke_timeout_follows_the_settings", test_stroke_timeout_follows_the_settings},
      {"slow_strike_is_not_a_move_timeout", test_slow_strike_is_not_a_move_timeout},
  };
  for (auto& t : tests) {
    current = t.name;
    int before = failures;
    t.fn();
    std::printf("%s %s\n", failures == before ? "ok  " : "FAIL", t.name);
  }
  std::printf("%d check(s) failed\n", failures);
  return failures ? 1 : 0;
}
