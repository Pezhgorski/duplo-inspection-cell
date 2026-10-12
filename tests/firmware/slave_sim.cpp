// The slave firmware's logic as a shared library, so the Python master can be tested against it
// (tests/test_firmware.py, via ctypes). Time is simulated in milliseconds.
#include "fake_arm.h"

struct Sim {
  Registers regs{};
  FakeArm arm;
  FakePersist persist;
  RejectController ctrl{regs, arm, persist};
  uint32_t now = 0;
  Sim() { ctrl.begin(now); }
};

#if defined(_WIN32)
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C"
#endif

API Sim* sim_new() { return new Sim(); }
API void sim_free(Sim* s) { delete s; }

// advance time, stepping the arm and running the controller once per millisecond
API void sim_advance(Sim* s, uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    s->now++;
    s->arm.tick();
    s->ctrl.update(s->now);
  }
}

// one loop() pass with no time passing: what the slave does right after answering a request
API void sim_update(Sim* s) { s->ctrl.update(s->now); }

API int sim_coil(Sim* s, int a) { return s->regs.coils[a]; }
API void sim_set_coil(Sim* s, int a, int v) { s->regs.coils[a] = v != 0; }
API int sim_discrete(Sim* s, int a) { return s->regs.discreteInputs[a]; }
API int sim_holding(Sim* s, int a) { return s->regs.holding[a]; }
API void sim_set_holding(Sim* s, int a, int v) { s->regs.holding[a] = (uint16_t)v; }
API int sim_input(Sim* s, int a) { return s->regs.input[a]; }

API long sim_position(Sim* s) { return s->arm.position(); }
API long sim_max_position(Sim* s) { return s->arm.maxPos; }
API long sim_min_position(Sim* s) { return s->arm.minPos; }
API void sim_set_stalled(Sim* s, int stalled) { s->arm.stalled = stalled != 0; }
API int sim_state(Sim* s) { return s->ctrl.state(); }

API uint32_t sim_stroke_timeout_ms(int steps, int speed, int accel) {
  return RejectController::strokeTimeoutMs(steps, (float)speed, (float)accel);
}

// power-cycle the Mega: registers and the step counter start from zero, the EEPROM flag survives
API void sim_reboot(Sim* s) {
  s->regs = Registers{};
  s->arm.reboot();
  s->ctrl.begin(s->now);
}
API int sim_away(Sim* s) { return s->persist.away; }
