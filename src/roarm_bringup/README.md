# roarm_bringup

This package provides a single launch file to bring up the ROARM system,
including robot description, MoveIt, IK solver, sensor reader, and driver.

## Usage

```bash
ros2 launch roarm_bringup bringup.launch.py use_driver:=true

