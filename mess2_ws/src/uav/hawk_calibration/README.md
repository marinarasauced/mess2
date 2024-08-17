# hawk_calibration

## Overview
The `hawk_calibration` package provides a method for generating the `calibration.yaml` files for ACE Lab Hawk reconfigurable aerial sensors in ROS2. It consists of an action server and client for collecting measurements from the VICON motion capture system and writing a calibration quaternion. The `calibration.yaml` file is automatically written to `mess2/agents/AGENT_NAME/calibration.yaml`.

**Note**
Since the `vicon_driver` reads `mess2/agents/` for `calibration.yaml` files statically, before calibrating, **you MUST delete the `calibration.yaml` file for the current AGENT_NAME**.
