import os
import sys

# Ensure the inner ``perception_tower`` package is importable without installing
# the ROS package. The ament package directory (parent of this test dir) must be
# on sys.path so ``import perception_tower`` finds ``<ament>/perception_tower``.
ament_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ament_dir not in sys.path:
    sys.path.insert(0, ament_dir)
