import os
import sys

# Allow running the tests straight from the source tree (no colcon install):
# both this package and washbot_planning must be importable.
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO_ROOT = os.path.dirname(_PKG_ROOT)
for path in (_PKG_ROOT, os.path.join(_REPO_ROOT, 'washbot_planning')):
    if path not in sys.path:
        sys.path.insert(0, path)
