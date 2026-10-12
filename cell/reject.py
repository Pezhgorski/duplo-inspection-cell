"""When to strike in real-time mode, and whether the strike worked (design spec §5, VERIFY).

ARMED    --NG on a part that has settled (SETTLE_FRAMES frames, < SETTLE_PX)-->  strike
STRIKING --arm reports DONE-->  VERIFY;   --arm fails-->  FAILED
VERIFY   --after VERIFY_FRAMES: part gone, moved > half its length or turned-->  CLEARING
         --otherwise-->  FAILED (REJECT_FAILED: the camera says the part did not move)
any state: a hand (crowded / too_large) in the box holds the strike and pauses VERIFY
CLEARING / FAILED  --plate box empty for EMPTY_FRAMES frames-->  ARMED

A part is struck at most once: the trigger re-arms only when the plate box is seen empty.
"""
import math

SETTLE_FRAMES = 5
SETTLE_PX = 2.0
VERIFY_FRAMES = 5
TURNED_DEG = 15.0
EMPTY_FRAMES = 3  # consecutive empty frames before re-arming: one bad frame must not allow a second strike


def _angle_diff(a: float, b: float) -> float:
    d = abs(a - b) % 180.0  # a rectangle looks the same turned by 180
    return min(d, 180.0 - d)


class RejectTrigger:
    def __init__(self):
        self.state = "ARMED"
        self.message = ""
        self._history = []   # recent NG part centers while armed
        self._struck = None  # the part as it was when struck
        self._job = None
        self._verify_frames = 0
        self._empty_frames = 0

    def update(self, status: str, verdict: str | None, part, actuator, crowded: bool = False) -> None:
        """One camera frame. status: find_part() status; verdict: "OK"/"NG" when inspected;
        part: the located Part or None; actuator: anything with request_reject() -> job | None;
        crowded: a hand-sized blob is in the box besides the part (Search.crowded)."""
        if self.state == "ARMED":
            if crowded:
                self._history = []
                self.message = "hand in the plate box: strike held"
            elif status == "found" and verdict == "NG":
                self._history = (self._history + [part.center])[-SETTLE_FRAMES:]
                if len(self._history) == SETTLE_FRAMES and self._settled():
                    job = actuator.request_reject()
                    if job is None:
                        self.message = "NG part, but the arm is not ready"
                    else:
                        self._job, self._struck = job, part
                        self._go("STRIKING", "striking")
            else:
                self._history = []
                self.message = ""
        elif self.state == "STRIKING":
            if self._job.state == "done":
                self._verify_frames = 0
                self._go("VERIFY", "verifying the reject")
            elif self._job.state == "failed":
                self._go("FAILED", f"REJECT NOT DONE: {self._job.error}")
        elif self.state == "VERIFY":
            if crowded or status == "too_large":
                self._verify_frames = 0  # a hand hides or moves the part: judge once it has gone
                self.message = "hand in the plate box: waiting to verify"
                return
            self._verify_frames += 1
            if self._verify_frames >= VERIFY_FRAMES:  # let the part come to rest first
                if self._moved(status, part):
                    self._go("CLEARING", "reject verified, remove the part")
                else:
                    self._go("FAILED", "REJECT_FAILED: the part did not move")
        else:  # CLEARING or FAILED
            self._empty_frames = self._empty_frames + 1 if status == "empty" else 0
            if self._empty_frames >= EMPTY_FRAMES:
                self._go("ARMED", "")

    def _settled(self) -> bool:
        x0, y0 = self._history[0]
        return all(math.hypot(x - x0, y - y0) < SETTLE_PX for x, y in self._history)

    def _moved(self, status: str, part) -> bool:
        if status != "found":
            return True  # gone, or pushed against the box edge
        before = self._struck
        shift = math.hypot(part.center[0] - before.center[0], part.center[1] - before.center[1])
        return shift > before.length / 2 or _angle_diff(part.angle, before.angle) > TURNED_DEG

    def _go(self, state: str, message: str):
        self.state, self.message = state, message
        self._history = []
        self._empty_frames = 0
