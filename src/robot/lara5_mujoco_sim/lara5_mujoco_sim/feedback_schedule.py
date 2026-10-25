"""Wall-time publication gate, called only after a completed physics step."""

import math


class FeedbackSchedule:
    def __init__(self, rate=20.):
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('Feedback rate must be finite and positive')
        self.period = 1./rate
        self.last = None

    def due(self, now):
        if self.last is None or now-self.last >= self.period-1e-9:
            self.last = now
            return True
        return False
