# NightHawk : Active Illumination Control [ISER 25']

<div align="left">
    <a href="https://github.com/droneslab/NH"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/droneslab/NH"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
</div>

[![Project Page](https://img.shields.io/badge/Project%20Page-NightHawk-blue)](https://droneslab.github.io/NH)

**This is a work in progress. The code is not yet ready for use.**

![NightHawk](./NH-Implx.png)
---

## Features

- 🎯 **Bayesian Optimization** for camera exposure and lighting control  
- 🔍 **Feature-based utility metric** to evaluate image utility  
- 🤖 **ROS2 Integration**  
- 🧠 **Event-triggered recursive optimization pipeline**  
- 🧪 **Field-tested** on a legged robot in culvert environments  


---

## Directory Structure

```
NH/
├── camera_control/              # Camera hardware drivers
│   ├── auto_shutter.py
│   └── flir_control.py
│
├── light_control/              # Illumination control logic
│   └── light_meter.py
│
├── teensy_meter_driver/        # Firmware for Teensy-based light meter
│   └── teensy_meter_driver.ino
│
├── nighthawk_ros/              # Main ROS package
│   ├── config/
│   │   └── params.yml
│   ├── launch/
│   │   └── nighthawk_ros_launch.py
│   ├── nighthawk_ros/
│   │   ├── __init__.py
│   │   ├── image_writer.py
│   │   ├── light_control_ros.py
│   │   ├── nighthawk_ros.py
│   │   ├── nighthawk_score_node.py
│   │   └── score.py
│   └── resource/
│       └── nighthawk_ros
│
├── utils/                      # Frontend/processing utilities
│   ├── exposure-scaler.ts
│   ├── state-process.ts
│   └── NightHawk.json
│
├── setup.py, setup.cfg         # Python package configuration
├── package.xml                 # ROS package metadata
├── .gitignore
└── README.md
```

---

## Installation

### Prerequisites

- ROS2 (Humble Recommended)
- Python 3.8+  
- Teensyduino (for Teensy microcontroller)  
- FLIR camera SDK (if applicable)

### Install Dependencies

If using Python:

Coming soong!

<!-- ```bash
cd NH
pip install -e .
``` -->

If using ROS:


```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Hardware Integration

- **Camera**: Controlled via `camera_control/flir_control.py`  
- **Light Meter**: Teensy driver in `teensy_meter_driver.ino`  
- **Lighting System**: Managed via `light_control/light_meter.py`  

Ensure proper connection to the robot’s onboard hardware before launching the pipeline.

---

## Usage

#### Launch NightHawk

```bash
ros2 launch nighthawk_ros nighthawk_ros_launch.py
```

#### Run Individual Nodes

```bash
ros2 run nighthawk_ros shared_memory_writer # Subscribes to image and writes to shared memory

ros2 run nighthawk_ros nighthawk_score_node # Subscribes to image and publishes Mfeat score (based on R2D2)

ros2 run nighthawk_ros light_control_ros # Runs LED light control node

ros2 run nighthawk_ros nighthawk_ros # Runs main nighthawk node for active illumination control

# Optional 

ros2 run foxglove_bridge foxglove_bridge --ros-args -p topic_whitelist:='["/flir_camera/image_raw", "/flir_camera/meta", "/led_driver/value", "/light_control_ros/transition_event", "/nighthawk/optimal", "/nighthawk/score", "/nighthawk/state", "/parameter_events", "/rosout"]' # Foxglove visualization

```

<!-- ### Run Optimizer Standalone

```bash
python nighthawk_ros/nighthawk_ros.py
``` -->

---



## License

MIT License

---

## Contact

For inquiries or collaborations, please reach out to:  
**[Yash Turkar]** – [yashturk@buffalo.edu]
