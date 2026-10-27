"""Scoring of estimates against the simulator's ground truth (no ROS)."""

import math

import pytest

from restore_perception.core import sim_truth as st

TRUTH = [
    {"name": "cube", "position": [0.1, 0.2, 0.925], "size_m": [0.05, 0.05, 0.05],
     "long_axis_yaw_rad": math.radians(20), "yaw_symmetry": 4},
    {"name": "bar", "position": [0.4, 0.2, 0.915], "size_m": [0.10, 0.04, 0.03],
     "long_axis_yaw_rad": math.radians(-35), "yaw_symmetry": 2},
    {"name": "can", "position": [0.1, -0.2, 0.92], "size_m": [0.08, 0.08, 0.04],
     "long_axis_yaw_rad": 0.0, "yaw_symmetry": "continuous"},
]


def est(id_, position, yaw_deg, size, **extra):
    return {"id": id_, "label": "unknown", "position": position, "yaw": math.radians(yaw_deg), "size": size,
            "confidence": 0.9, "cameras_seen": 1, "depth_reliable": True, **extra}


def test_yaw_error_respects_symmetry():
    assert st.yaw_error_deg(math.radians(110), math.radians(20), 4) == pytest.approx(0.0)
    assert st.yaw_error_deg(math.radians(50), math.radians(20), 4) == pytest.approx(30.0)
    assert st.yaw_error_deg(math.radians(145), math.radians(-35), 2) == pytest.approx(0.0)
    assert st.yaw_error_deg(math.radians(-170), math.radians(5), 2) == pytest.approx(5.0)
    assert st.yaw_error_deg(1.0, 2.0, "continuous") is None


def test_yaw_from_quaternion_and_size_error():
    assert math.degrees(st.yaw_from_quaternion(0, 0, math.sin(0.3), math.cos(0.3))) == pytest.approx(math.degrees(0.6))
    assert st.size_error_mm([0.04, 0.052, 0.101], [0.10, 0.04, 0.03]) == pytest.approx([1.0, 12.0, 10.0])


def test_matching_is_one_to_one_and_nearest():
    estimates = [est("obj_00", [0.405, 0.21, 0.92], 148, [0.102, 0.041, 0.031]),
                 est("obj_01", [0.11, 0.19, 0.93], 112, [0.05, 0.05, 0.049]),
                 est("obj_02", [0.13, 0.22, 0.93], 0, [0.02, 0.02, 0.02]),   # a second blob near the cube
                 est("obj_03", [0.9, 0.9, 0.9], 0, [0.05, 0.05, 0.05])]       # far away: unmatched
    scored = st.score_capture(TRUTH, estimates, radius=0.08)
    by_name = {r["name"]: r for r in scored["rows"]}
    assert set(by_name) == {"cube", "bar"} and scored["missed"] == ["can"]
    assert [e["id"] for e in scored["extra"]] == ["obj_02", "obj_03"]
    assert scored["extra"][1]["position"] == [0.9, 0.9, 0.9] and scored["extra"][1]["size_mm"] == pytest.approx([50, 50, 50])
    assert by_name["cube"]["estimate_id"] == "obj_01"
    assert by_name["cube"]["dxy_mm"] == pytest.approx(math.hypot(10, 10))
    assert by_name["cube"]["yaw_error_deg"] == pytest.approx(2.0)
    assert by_name["bar"]["yaw_error_deg"] == pytest.approx(3.0)
    assert by_name["bar"]["size_error_mm"] == pytest.approx([2.0, 1.0, 1.0])


def test_summary_and_report():
    estimates = [est("obj_00", [0.1, 0.2, 0.93], 20, [0.05, 0.05, 0.05]),
                 est("obj_01", [0.1, -0.2, 0.92], 0, [0.081, 0.08, 0.04])]
    captures = [st.score_capture(TRUTH, estimates, 0.08), st.score_capture(TRUTH, estimates[:1], 0.08)]
    summary = st.summarize(captures, [t["name"] for t in TRUTH])
    assert summary["objects"]["cube"]["detected"] == 2 and summary["objects"]["can"]["detected"] == 1
    assert summary["objects"]["bar"]["detected"] == 0 and summary["objects"]["bar"]["xy_mean_mm"] is None
    assert summary["overall"]["expected"] == 6 and summary["overall"]["detected"] == 3
    assert summary["objects"]["cube"]["z_mean_mm"] == pytest.approx(5.0)
    report = st.markdown_report(summary, captures, {"radius": 0.08, "cameras": [1], "models": "off"})
    assert "| cube | 2/2 |" in report and "Missed: bar" in report
