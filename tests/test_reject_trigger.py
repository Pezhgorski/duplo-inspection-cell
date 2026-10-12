"""When the cell strikes, and how the camera verifies the strike (cell/reject.py)."""
from dataclasses import dataclass

from cell.actuator import RejectJob
from cell.reject import EMPTY_FRAMES, SETTLE_FRAMES, VERIFY_FRAMES, RejectTrigger


@dataclass
class Part:  # the fields of cell.geometry.Part the trigger uses
    center: tuple[float, float]
    length: float = 170.0
    width: float = 85.0
    angle: float = 0.0


class FakeActuator:
    def __init__(self, ready=True):
        self.ready = ready
        self.jobs = []

    def request_reject(self):
        if not self.ready:
            return None
        self.jobs.append(RejectJob())
        return self.jobs[-1]


def frames(trigger, actuator, n, status="found", verdict="NG", part=None):
    for _ in range(n):
        trigger.update(status, verdict, part, actuator)


def strike(trigger, actuator, part):
    frames(trigger, actuator, SETTLE_FRAMES, part=part)
    assert trigger.state == "STRIKING"
    actuator.jobs[-1]._finish()
    frames(trigger, actuator, 1, part=part)
    assert trigger.state == "VERIFY"


def test_strikes_only_a_settled_ng_part():
    t, arm = RejectTrigger(), FakeActuator()
    frames(t, arm, SETTLE_FRAMES - 1, part=Part((300, 200)))
    assert not arm.jobs
    frames(t, arm, 1, part=Part((300, 200)))
    assert len(arm.jobs) == 1 and t.state == "STRIKING"


def test_moving_part_is_not_struck():
    t, arm = RejectTrigger(), FakeActuator()
    for x in range(20):  # slides 5 px per frame
        t.update("found", "NG", Part((300 + 5 * x, 200)), arm)
    assert not arm.jobs


def test_ok_part_is_never_struck():
    t, arm = RejectTrigger(), FakeActuator()
    frames(t, arm, 50, verdict="OK", part=Part((300, 200)))
    assert not arm.jobs


def test_one_ok_frame_restarts_settling():
    t, arm = RejectTrigger(), FakeActuator()
    frames(t, arm, SETTLE_FRAMES - 1, part=Part((300, 200)))
    frames(t, arm, 1, verdict="OK", part=Part((300, 200)))
    frames(t, arm, SETTLE_FRAMES - 1, part=Part((300, 200)))
    assert not arm.jobs


def test_arm_not_ready_is_reported_and_retried():
    t, arm = RejectTrigger(), FakeActuator(ready=False)
    frames(t, arm, SETTLE_FRAMES, part=Part((300, 200)))
    assert t.state == "ARMED" and "not ready" in t.message
    arm.ready = True
    frames(t, arm, SETTLE_FRAMES, part=Part((300, 200)))
    assert t.state == "STRIKING"


def test_part_knocked_away_is_verified_and_struck_once():
    t, arm = RejectTrigger(), FakeActuator()
    strike(t, arm, Part((300, 200)))
    frames(t, arm, VERIFY_FRAMES, status="empty", verdict=None)
    assert t.state == "CLEARING"
    frames(t, arm, EMPTY_FRAMES, status="empty", verdict=None)
    assert t.state == "ARMED"
    assert len(arm.jobs) == 1


def test_part_pushed_but_still_in_box_waits_for_removal():
    t, arm = RejectTrigger(), FakeActuator()
    strike(t, arm, Part((300, 200)))
    moved = Part((300 + 100, 200))  # more than half its length
    frames(t, arm, VERIFY_FRAMES, part=moved)
    assert t.state == "CLEARING" and "verified" in t.message
    frames(t, arm, 30, part=moved)  # still NG and settled, but already struck
    assert len(arm.jobs) == 1
    frames(t, arm, EMPTY_FRAMES, status="empty", verdict=None)
    assert t.state == "ARMED"


def test_turned_part_counts_as_moved():
    t, arm = RejectTrigger(), FakeActuator()
    strike(t, arm, Part((300, 200), angle=10))
    frames(t, arm, VERIFY_FRAMES, part=Part((305, 200), angle=40))
    assert t.state == "CLEARING"


def test_angle_wraps_at_180():
    t, arm = RejectTrigger(), FakeActuator()
    strike(t, arm, Part((300, 200), angle=-89))
    frames(t, arm, VERIFY_FRAMES, part=Part((300, 200), angle=89))  # the same pose
    assert t.state == "FAILED"


def test_unmoved_part_is_reject_failed():
    t, arm = RejectTrigger(), FakeActuator()
    strike(t, arm, Part((300, 200)))
    frames(t, arm, VERIFY_FRAMES, part=Part((310, 205)))
    assert t.state == "FAILED" and t.message.startswith("REJECT_FAILED")
    frames(t, arm, 30, part=Part((310, 205)))
    assert len(arm.jobs) == 1  # no automatic second strike
    frames(t, arm, EMPTY_FRAMES, status="empty", verdict=None)
    assert t.state == "ARMED"


def test_arm_failure_is_reported():
    t, arm = RejectTrigger(), FakeActuator()
    frames(t, arm, SETTLE_FRAMES, part=Part((300, 200)))
    arm.jobs[-1]._finish("ACTUATOR_TIMEOUT: no BUSY within 0.3 s")
    frames(t, arm, 1, part=Part((300, 200)))
    assert t.state == "FAILED" and "ACTUATOR_TIMEOUT" in t.message


def test_hand_in_the_box_does_not_rearm():
    t, arm = RejectTrigger(), FakeActuator()
    strike(t, arm, Part((300, 200)))
    frames(t, arm, VERIFY_FRAMES, part=Part((450, 200)))
    frames(t, arm, 5, status="outside", verdict=None)  # operator's hand reaching in
    assert t.state == "CLEARING"


def test_hand_next_to_an_ng_part_holds_the_strike():
    t, arm = RejectTrigger(), FakeActuator()
    for _ in range(3 * SETTLE_FRAMES):
        t.update("found", "NG", Part((300, 200)), arm, crowded=True)
    assert not arm.jobs
    assert "hand" in t.message
    frames(t, arm, SETTLE_FRAMES, part=Part((300, 200)))  # hand gone: settles again, then strikes
    assert len(arm.jobs) == 1


def test_hand_during_verify_is_not_a_verified_reject():
    t, arm = RejectTrigger(), FakeActuator()
    strike(t, arm, Part((300, 200)))
    frames(t, arm, 2 * VERIFY_FRAMES, status="too_large", verdict=None)
    assert t.state == "VERIFY"
    frames(t, arm, VERIFY_FRAMES, part=Part((300, 200)))  # hand gone, the part never moved
    assert t.state == "FAILED"


def test_one_empty_frame_does_not_rearm():
    t, arm = RejectTrigger(), FakeActuator()
    strike(t, arm, Part((300, 200)))
    frames(t, arm, VERIFY_FRAMES, part=Part((310, 205)))  # REJECT_FAILED
    frames(t, arm, 1, status="empty", verdict=None)       # one bad frame from find_part
    frames(t, arm, 30, part=Part((310, 205)))
    assert t.state == "FAILED"
    assert len(arm.jobs) == 1
