"""Simulated time for the actuator tests: the master and the slave share one clock."""


class SimClock:
    def __init__(self, start: float = 100.0):
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, seconds: float):
        self.t += seconds


def run(master, clock, seconds: float, period: float = 0.05, slave_advance=None, until=None):
    """Step the master every `period` s for `seconds` (or until `until()` is true).
    slave_advance(dt) lets a simulated slave run the same time."""
    for _ in range(round(seconds / period)):
        clock.advance(period)
        if slave_advance:
            slave_advance(period)
        master.step()
        if until and until():
            return True
    return False
