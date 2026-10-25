import sys
from pathlib import Path

# Import restore_task and restore_perception from the source tree, so the tests run without colcon.
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "restore_perception"))
