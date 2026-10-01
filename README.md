# 🤖 Autonomous Rover — Environment Mapping & Hazard Detection

> An RL-driven autonomous ground rover for disaster-response reconnaissance: real-time SLAM mapping, multi-model hazard/victim detection, adaptive path planning, and live telemetry — built to sweep unknown environments and guide people to safety.

![Status](https://img.shields.io/badge/status-active%20development-yellow)
![Python](https://img.shields.io/badge/python-3.10%2B-blue)
![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%205-c51a4a)

---

## Overview

This project explores how a low-cost ground rover can autonomously sweep a disaster-affected area, build a live map of its surroundings, detect hazards and people in real time, and compute safe evacuation routes — all while giving a human operator full visibility through a live telemetry dashboard.

The rover combines LIDAR-based SLAM, two parallel object-detection models, gas sensing, and reinforcement-learning-driven navigation into a single pipeline, running on a Raspberry Pi 5.

## Key Features

- 🗺️ **Real-time SLAM mapping** — LIDAR-based mapping (BreezySLAM) builds a live map of the environment as the rover sweeps, overlaid with a hazard risk heatmap.
- 🎯 **Dual-model hazard & victim detection** — a modified YOLOv8n (custom multi-task risk-score head + reweighted loss, tuned to prioritize human detection) runs alongside a separate YOLOv9t camera model for redundancy and broader hazard coverage.
- 🔥 **Gas hazard sensing** — an onboard MQ-2 sensor flags fire/smoke/gas hazards that aren't visually detectable.
- 🧭 **Adaptive evacuation routing** — a modified A* search runs over the live SLAM map to compute the safest, shortest path out of a hazard zone for any detected person, paired with generated voice guidance.
- 🧠 **RL-driven navigation** — reinforcement learning adapts the rover's sweep path on the fly as new hazards are discovered, instead of following a fixed route.
- 📡 **Live telemetry dashboard** — a Flask dashboard streams live position, sensor readings, and detections for human oversight.
- 🛟 **Sensor-fusion robustness** — HC-SR04 ultrasonic sensors, a Pi Camera, and an MPU9250 IMU are fused across both autonomous and manual modes, so navigation stays stable even if an individual sensor is degraded.

## Tech Stack

| Layer | Technology |
|---|---|
| Compute | Raspberry Pi 5 |
| Mapping | BreezySLAM (LIDAR-based SLAM) |
| Object Detection | YOLOv8n (custom risk-score head) · YOLOv9t |
| Path Planning | Modified A* search |
| Navigation | Reinforcement Learning |
| Sensors | HC-SR04 ultrasonic · Pi Camera · MPU9250 IMU · MQ-2 gas sensor |
| Telemetry / Dashboard | Flask |
| Language | Python |

## System Architecture

```
                         ┌─────────────────────┐
                         │     LIDAR + SLAM     │
                         │    (BreezySLAM)      │
                         └──────────┬───────────┘
                                    │ live map
                                    ▼
        ┌───────────────────────────────────────────────┐
        │                 Perception Layer                │
        │  ┌───────────────┐        ┌───────────────┐    │
        │  │ YOLOv8n (mod.) │        │   YOLOv9t     │    │
        │  │ risk-score head│        │  camera model │    │
        │  └───────┬───────┘        └───────┬───────┘    │
        │          │     ┌───────────────┐  │            │
        │          └────▶│  MQ-2 sensor  │◀─┘            │
        │                │ (fire/smoke/  │                │
        │                │  gas hazard)  │                │
        │                └───────┬───────┘                │
        └────────────────────────┼────────────────────────┘
                                  ▼
                     ┌────────────────────────┐
                     │  Hazard Risk Heatmap    │
                     │   overlaid on SLAM map  │
                     └────────────┬────────────┘
                                  ▼
        ┌─────────────────────────────────────────────┐
        │             Planning & Navigation             │
        │  Modified A* evacuation routing (+ voice)     │
        │  RL-based adaptive sweep path planning        │
        └────────────────────┬──────────────────────────┘
                              ▼
        ┌─────────────────────────────────────────────┐
        │         Sensor Fusion (Autonomous/Manual)      │
        │   HC-SR04 ultrasonic · Pi Camera · MPU9250 IMU │
        └────────────────────┬──────────────────────────┘
                              ▼
                   ┌─────────────────────┐
                   │  Flask Telemetry     │
                   │  Dashboard (live)    │
                   └─────────────────────┘
```

## How It Works

1. **Mapping** — The rover's LIDAR feeds BreezySLAM, which builds a real-time occupancy map of the environment as the rover moves.
2. **Detection** — Each camera frame is run through the modified YOLOv8n model (for human/victim detection with an added risk-score head) and the separate YOLOv9t model in parallel. The MQ-2 sensor continuously checks for gas/smoke/fire signatures the cameras can't see.
3. **Risk mapping** — Detections are projected onto the live SLAM map as a hazard risk heatmap, so risk is tied to actual physical location, not just a single camera frame.
4. **Routing** — When a person is detected inside a hazard zone, a modified A* search computes the shortest path to safety on the live map, and the system issues generated voice guidance to direct them out.
5. **Adaptive sweeping** — While no one is in immediate danger, an RL policy adjusts the rover's area-sweep path in real time to prioritize unexplored or newly-flagged hazardous regions.
6. **Fusion & fallback** — Ultrasonic, camera, and IMU readings are fused continuously; if one sensor degrades or drops out, the rover falls back on the others to keep navigating safely, in both autonomous and manual-override modes.
7. **Oversight** — Every reading, position update, and detection streams to a Flask-based dashboard so a human operator can monitor the sweep live.

## Project Status

🚧 Active development. Core perception (YOLO detection + SLAM mapping), path planning, and the telemetry dashboard are in progress; see [Roadmap](#roadmap) below.

## Roadmap

- [x] LIDAR-based SLAM mapping
- [x] Modified YOLOv8n with risk-score prediction head
- [x] Secondary YOLOv9t detection model
- [x] MQ-2 gas hazard sensing
- [x] Modified A* evacuation path planning
- [x] RL-based adaptive sweep navigation
- [x] Flask telemetry dashboard
- [ ] Multi-rover coordination
- [ ] Field testing in physical disaster-simulation environments
- [ ] Voice guidance language/accessibility expansion

## Getting Started

```bash
# Clone the repository
git clone https://github.com/HemashreeVI/Rover.git
cd Rover

# Install dependencies
pip install -r requirements.txt

# Run the telemetry dashboard
python dashboard/app.py

# Run the main rover control loop
python main.py
```

> Hardware setup (Raspberry Pi 5, LIDAR, camera, and sensor wiring) is documented in `/docs/hardware_setup.md`.

## Author

**V I Hemashree** ([@HemashreeVI](https://github.com/HemashreeVI))
B.Tech Computer Science (Cyber-Physical Systems), VIT Chennai

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.
