"""Put this directory on sys.path so `import depth_cli` works under pytest.

tools/ is not a Python package -- these are standalone scripts run directly,
not installed by colcon.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
