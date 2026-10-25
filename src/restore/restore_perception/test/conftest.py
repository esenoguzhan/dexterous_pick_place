import sys
from pathlib import Path

# Import the package from the source tree, so the tests also run without colcon.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
