"""Encoder-only teaching data. Caller serializes access using worker.state_lock."""
import math


class TeachingRecording:
    def __init__(self):
        self.samples = []
        self.points = []
        self.active = False
        self.last_stamp = None
        self.origin = None
        self.note = "No recording"

    def start(self):
        # A new run requires explicit clearing, so data is never silently lost.
        if self.samples:
            raise ValueError("Export or clear the existing recording before starting a new run")
        self.active = True
        self.origin = self.last_stamp = None
        self.note = "Recording encoder positions (up to 100 Hz)"

    def stop(self):
        if self.active:
            self.note = "Recording stopped"
        self.active = False

    def observe(self, stamp, q):
        if not self.active:
            return
        if not math.isfinite(stamp) or not all(math.isfinite(x) for x in q):
            self.stop()
            self.note = "Stopped: invalid encoder sample"
            return
        if self.last_stamp is not None:
            if stamp < self.last_stamp:
                self.stop()
                self.note = "Stopped: encoder clock moved backwards"
                return
            if stamp == self.last_stamp:
                return
            if stamp - self.last_stamp > 0.25:
                self.stop()
                self.note = "Stopped: encoder feedback gap"
                return
            if stamp - self.last_stamp < 0.01 - 1e-7:
                return
        if self.origin is None:
            self.origin = stamp
        self.samples.append({"time_s": stamp - self.origin, "position_rad": list(q)})
        self.last_stamp = stamp
        if len(self.samples) >= 60000:
            self.stop()
            self.note = "Stopped: 60000 sample limit reached"

    def document(self):
        return {"format": "lite6_encoder_teaching_v1", "joint_names": [f"joint{i}" for i in range(1, 7)],
                "time_source": "joint_states.header.stamp", "positions_unit": "rad",
                "reference_frame": "joint space", "requires_retiming_and_validation": True,
                "points": [dict(p, position_rad=list(p["position_rad"])) for p in self.points],
                "samples": [dict(p, position_rad=list(p["position_rad"])) for p in self.samples]}
