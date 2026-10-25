"""The run report: one line per reference object, returned as the Restore action's report_json."""

from __future__ import annotations

import json
import math


class Report:
    def __init__(self):
        self.objects = {}  # reference id -> fields
        self.events = []

    def _entry(self, reference_id, label):
        return self.objects.setdefault(reference_id, {"label": label, "attempts": 0, "outcome": "", "notes": []})

    def attempt(self, reference_id, label):
        self._entry(reference_id, label)["attempts"] += 1

    def attempts(self, reference_id):
        return self.objects.get(reference_id, {}).get("attempts", 0)

    def note(self, reference_id, label, text):
        self._entry(reference_id, label)["notes"].append(text)
        self.events.append(f"{label}: {text}")

    def finish(self, diff, reference):
        """Final outcome of every reference object from the last diff."""
        labels = {o.id: o.label for o in reference.objects}
        for cur_id, ref_id in diff.in_place:
            e = self._entry(ref_id, labels.get(ref_id, "unknown"))
            e["outcome"] = "restored" if e["attempts"] else "in place"
        for task in diff.tasks:
            e = self._entry(task.reference_id, task.label)
            e["outcome"] = "not restored"
            e["dxy_mm"] = round(1000 * task.dxy, 1)
            e["dyaw_deg"] = round(math.degrees(task.dyaw), 1)
        for r in diff.refused:
            self._entry(r.reference_id, r.label)["outcome"] = f"refused: {r.reason}"
        for ref_id in diff.missing:
            self._entry(ref_id, labels.get(ref_id, "unknown"))["outcome"] = "missing"

    def counts(self):
        out = {}
        for e in self.objects.values():
            key = e["outcome"].split(":")[0] or "pending"
            out[key] = out.get(key, 0) + 1
        return out

    def summary(self):
        return ", ".join(f"{n} {k}" for k, n in sorted(self.counts().items())) or "nothing to do"

    def to_json(self):
        return json.dumps({"objects": self.objects, "events": self.events[-50:]}, indent=1)
