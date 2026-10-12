#include "RejectController.h"

#include <math.h>

uint32_t RejectController::strokeTimeoutMs(long steps, float speed, float accel) {
  const float s = (float)(steps < 0 ? -steps : steps);
  if (speed < 1) speed = 1;  // a zero register must not divide by zero
  if (accel < 1) accel = 1;
  const float ramp = speed * speed / accel;  // steps spent speeding up and slowing down from full speed
  const float t = s >= ramp ? s / speed + speed / accel : 2 * sqrtf(s / accel);
  return (uint32_t)(STROKE_SLACK * t * 1000.0f) + STROKE_MARGIN_MS;
}

void RejectController::begin(uint32_t now) {
  for (bool& c : regs_.coils) c = false;
  regs_.holding[reg::MASTER_HEARTBEAT] = 0;
  regs_.holding[reg::STRIKE_STEPS] = (uint16_t)DEFAULT_STRIKE_STEPS;
  regs_.holding[reg::MAX_SPEED] = DEFAULT_MAX_SPEED;
  regs_.holding[reg::ACCELERATION] = DEFAULT_ACCELERATION;
  regs_.input[reg::SLAVE_HEARTBEAT] = 0;
  home_ = arm_.position();
  strikeTarget_ = home_;
  strokeStart_ = now;
  moveTimeoutMs_ = 0;
  lastBeat_ = now;
  lastMasterBeat_ = 0;
  masterBeatAt_ = now;
  masterSeen_ = false;
  requestArmed_ = false;
  prevReset_ = false;
  cycleCount_ = 0;
  faultCode_ = FAULT_NONE;
  state_ = WAIT_MASTER;
  if (persist_.awayFromHome()) {  // restarted mid-stroke: this "home" is wherever the stick stopped
    faultCode_ = FAULT_NOT_HOME;
    state_ = FAULTED;
  }
  publish();
}

void RejectController::update(uint32_t now) {
  // unsigned differences below stay correct when millis() wraps after 49 days
  if (now - lastBeat_ >= HEARTBEAT_MS) {
    regs_.input[reg::SLAVE_HEARTBEAT]++;
    lastBeat_ = now;
  }
  if (regs_.holding[reg::MASTER_HEARTBEAT] != lastMasterBeat_) {
    lastMasterBeat_ = regs_.holding[reg::MASTER_HEARTBEAT];
    masterBeatAt_ = now;
    masterSeen_ = true;
  }
  const bool masterAlive = masterSeen_ && now - masterBeatAt_ < WATCHDOG_MS;
  const bool request = regs_.coils[reg::REQ_REJECT];
  const bool resetEdge = regs_.coils[reg::RESET_FAULT] && !prevReset_;
  prevReset_ = regs_.coils[reg::RESET_FAULT];
  if (!request) requestArmed_ = true;

  // before the first master heartbeat there is nobody to lose; after it, silence is a fault
  if (masterSeen_ && !masterAlive && state_ != WAIT_MASTER && state_ != FAULTED) fault(FAULT_COMM_LOST);

  switch (state_) {
    case WAIT_MASTER:
      if (masterAlive) state_ = IDLE;
      break;
    case IDLE:
      if (request && requestArmed_) {
        requestArmed_ = false;
        arm_.configure(regs_.holding[reg::MAX_SPEED], regs_.holding[reg::ACCELERATION]);
        // signed: a negative value strikes in the other direction
        strikeTarget_ = home_ + (int16_t)regs_.holding[reg::STRIKE_STEPS];
        persist_.setAwayFromHome(true);
        startStroke(strikeTarget_, STRIKE_OUT, now);
      }
      break;
    case STRIKE_OUT:
    case STRIKE_BACK:
    case RETURNING: {
      const long target = state_ == STRIKE_OUT ? strikeTarget_ : home_;
      if (!arm_.moving() && arm_.position() == target) {
        if (state_ == STRIKE_OUT) {
          startStroke(home_, STRIKE_BACK, now);
        } else if (state_ == STRIKE_BACK) {
          persist_.setAwayFromHome(false);
          cycleCount_++;
          state_ = DONE;
        } else {
          persist_.setAwayFromHome(false);
          state_ = IDLE;
        }
      } else if (now - strokeStart_ > moveTimeoutMs_) {
        fault(FAULT_MOVE_TIMEOUT);
      }
      break;
    }
    case DONE:
      if (!request) state_ = IDLE;
      break;
    case FAULTED:
      if (resetEdge && masterAlive) {
        faultCode_ = FAULT_NONE;
        requestArmed_ = !request;  // a request raised before the reset is stale: it must drop first
        if (arm_.position() == home_) {
          persist_.setAwayFromHome(false);  // after NOT_HOME the operator vouches that the stick is home
          state_ = IDLE;
        } else {
          arm_.configure(regs_.holding[reg::MAX_SPEED], regs_.holding[reg::ACCELERATION]);
          startStroke(home_, RETURNING, now);
        }
      }
      break;
  }
  publish();
}

void RejectController::startStroke(long target, State next, uint32_t now) {
  moveTimeoutMs_ = strokeTimeoutMs(target - arm_.position(), regs_.holding[reg::MAX_SPEED],
                                   regs_.holding[reg::ACCELERATION]);
  arm_.moveTo(target);
  strokeStart_ = now;
  state_ = next;
}

void RejectController::fault(FaultCode code) {
  arm_.halt();
  faultCode_ = code;
  state_ = FAULTED;
}

void RejectController::publish() {
  regs_.discreteInputs[reg::READY] = state_ == IDLE;
  regs_.discreteInputs[reg::BUSY] = state_ == STRIKE_OUT || state_ == STRIKE_BACK;
  regs_.discreteInputs[reg::DONE] = state_ == DONE;
  regs_.discreteInputs[reg::FAULT] = state_ == FAULTED;
  regs_.input[reg::FAULT_CODE] = faultCode_;
  regs_.input[reg::CYCLE_COUNT] = cycleCount_;
}
