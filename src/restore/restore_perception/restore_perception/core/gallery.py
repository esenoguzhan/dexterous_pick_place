"""Sections 5.7, 7.1 and 8.3: the object gallery and matching against it.

    gallery/<name>/view_XX.png     masked crops, as DINOv2 saw them
                   embeddings.npy  one row per view
                   histograms.npy  one HSV histogram per view
                   meta.json       {"views": [{"file", "up_side", "camera", "size_m", "added"}]}

The test bench writes the same layout without histograms.npy and size_m; histograms are then computed
from the stored crops, so views registered on the bench can be used here.
"""

from __future__ import annotations

import json
import re
import shutil
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

from .embed import crop_histogram, histogram_similarity

_NAME_RE = re.compile(r"[^a-z0-9_-]+")


def clean_name(name):
    return _NAME_RE.sub("_", name.strip().lower()).strip("_")


def size_fits(measured, reference, tolerance):
    """Section 5.7: every dimension within tolerance, in any axis order (an object may lie on a side)."""
    m, r = np.sort(np.asarray(measured))[::-1], np.sort(np.asarray(reference))[::-1]
    return bool(np.all(np.abs(m - r) <= tolerance * r))


class Gallery:
    def __init__(self, root):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self._objects = {}  # name -> {"embeddings", "histograms", "meta"}
        for path in sorted(self.root.glob("*/embeddings.npy")):
            self._objects[path.parent.name] = self._load(path.parent)

    def _load(self, folder):
        embeddings = np.load(folder / "embeddings.npy").astype(np.float32)
        meta_path = folder / "meta.json"
        meta = json.loads(meta_path.read_text()) if meta_path.exists() else {"views": []}
        hist_path = folder / "histograms.npy"
        if hist_path.exists() and len(np.load(hist_path)) == len(embeddings):
            histograms = np.load(hist_path)
        else:
            histograms = []
            for i, view in enumerate(meta["views"][:len(embeddings)]):
                crop = cv2.imread(str(folder / view.get("file", f"view_{i:02d}.png")))
                histograms.append(None if crop is None else crop_histogram(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)))
            histograms = np.stack(histograms) if histograms and all(h is not None for h in histograms) else None
        return {"embeddings": embeddings, "histograms": histograms, "meta": meta}

    def names(self):
        with self.lock:
            return sorted(self._objects)

    def list(self):
        with self.lock:
            return [{"name": name, "views": len(item["embeddings"]),
                     "up_sides": sorted({v.get("up_side", "") for v in item["meta"]["views"]} - {""})}
                    for name, item in sorted(self._objects.items())]

    def measured_size(self, name):
        """Median size over the views registered with a size, or None."""
        with self.lock:
            item = self._objects.get(name)
            sizes = [v["size_m"] for v in item["meta"]["views"] if v.get("size_m")] if item else []
        return np.median(np.array(sizes), axis=0).tolist() if sizes else None

    def add(self, name, crops, embeddings, histograms, size_m=None, up_side="", cameras=None):
        """Append views of one object; returns (clean name, number of stored views)."""
        name = clean_name(name)
        if not name:
            raise ValueError("object name is empty")
        if not len(crops):
            raise ValueError("no view to store")
        with self.lock:
            folder = self.root / name
            folder.mkdir(parents=True, exist_ok=True)
            item = self._objects.get(name) or {"embeddings": np.zeros((0, embeddings.shape[1]), np.float32),
                                               "histograms": np.zeros((0, histograms.shape[1]), np.float32),
                                               "meta": {"views": []}}
            if item["histograms"] is None:
                item["histograms"] = np.zeros((len(item["embeddings"]), histograms.shape[1]), np.float32)
            for i, crop in enumerate(crops):
                filename = f"view_{len(item['meta']['views']):02d}.png"
                cv2.imwrite(str(folder / filename), cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
                item["meta"]["views"].append({
                    "file": filename, "up_side": up_side, "added": time.time(),
                    "camera": (cameras or [""] * len(crops))[i],
                    "size_m": None if size_m is None else [round(float(v), 4) for v in size_m]})
            item["embeddings"] = np.vstack([item["embeddings"], embeddings]).astype(np.float32)
            item["histograms"] = np.vstack([item["histograms"], histograms]).astype(np.float32)
            np.save(folder / "embeddings.npy", item["embeddings"])
            np.save(folder / "histograms.npy", item["histograms"])
            (folder / "meta.json").write_text(json.dumps(item["meta"], indent=2))
            self._objects[name] = item
            return name, len(item["embeddings"])

    def remove(self, name):
        with self.lock:
            if name not in self._objects:
                return False
            shutil.rmtree(self.root / name, ignore_errors=True)
            del self._objects[name]
            return True

    def match(self, embeddings, histograms, sizes, objects, cfg):
        """Section 5.7: name every detection, each gallery object at most once.

        embeddings: n x d, histograms: n x b (rows may be None), sizes: n x 3 measured sizes (rows may
        be None when depth gave no size).
        objects: objects.yaml entries; their size_m (else the registered size) is the size check.
        """
        n = len(embeddings)
        unknown = {"label": "unknown", "score": 0.0, "candidate": None, "margin": 0.0, "view": None,
                   "up_side": "", "size_ok": None, "by_histogram": False, "top": []}
        with self.lock:
            names = sorted(self._objects)
            items = [self._objects[k] for k in names]
        if not names or n == 0:
            return [dict(unknown) for _ in range(n)]
        per_view = [embeddings @ item["embeddings"].T for item in items]  # n x views, per object
        scores = np.stack([s.max(axis=1) for s in per_view], axis=1)
        best_view = np.stack([s.argmax(axis=1) for s in per_view], axis=1)
        rows, cols = linear_sum_assignment(1.0 - scores)
        assigned = dict(zip(rows.tolist(), cols.tolist()))

        results = []
        for i in range(n):
            order = np.argsort(-scores[i])
            top = [{"name": names[k], "score": round(float(scores[i, k]), 3)} for k in order[:3]]
            if i not in assigned:
                results.append({**unknown, "candidate": names[int(order[0])], "score": round(float(scores[i, order[0]]), 3),
                                "top": top})
                continue
            j = assigned[i]
            score = float(scores[i, j])
            others = [k for k in order if k != j]
            margin = score - (float(scores[i, others[0]]) if others else 0.0)

            reference = (objects.get(names[j]) or {}).get("size_m") or self.measured_size(names[j])
            reliable = (objects.get(names[j]) or {}).get("depth_reliable", True)
            if reference is None or not reliable or sizes[i] is None:
                size_ok = None  # no check: nothing to compare, or depth is known to be poor on this object
            else:
                size_ok = size_fits(sizes[i], reference, cfg["size_tolerance"])

            by_histogram = False
            if score >= cfg["min_score"] and margin < cfg["min_margin"] and others and histograms[i] is not None:
                rival = others[0]
                if items[j]["histograms"] is not None and items[rival]["histograms"] is not None:
                    own = max(histogram_similarity(histograms[i], h) for h in items[j]["histograms"])
                    other = max(histogram_similarity(histograms[i], h) for h in items[rival]["histograms"])
                    by_histogram = own - other >= cfg["histogram_margin"]

            accepted = (score >= cfg["min_score"] and (margin >= cfg["min_margin"] or by_histogram)
                        and size_ok is not False)
            view = int(best_view[i, j])
            views = items[j]["meta"]["views"]
            results.append({
                "label": names[j] if accepted else "unknown", "score": round(score, 3), "candidate": names[j],
                "margin": round(margin, 3), "view": view,
                "up_side": views[view].get("up_side", "") if accepted and view < len(views) else "",
                "size_ok": size_ok, "by_histogram": by_histogram, "top": top})
        return results
