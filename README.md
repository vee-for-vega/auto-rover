# Rover — Autonomous 1/10-Scale Vehicle

A 1/10-scale autonomous ground vehicle built to drive outdoor sidewalks and
neighborhood streets, developed on **ROS 2 Humble** with a **C++-first** codebase.
This repository tracks the system as it is built up module by module — from ROS 2
fundamentals through perception to full autonomous navigation.

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-ready-2496ED?logo=docker&logoColor=white)

---

## 0. Overview

The rover drives autonomously from point A to point B outdoors. During testing, a
human spotter follows on a bicycle and can stop the vehicle at any time.

|  |  |
|---|---|
| **Platform** | Traxxas Slash 2WD chassis + NVIDIA Jetson Orin Nano |
| **Phase 1 — current** | Point A → B using onboard perception only (no GPS) |
| **Phase 2 — planned** | RTK-GPS waypoint navigation |
| **Codebase** | C++ first; Python only for prototyping and scripts |

---

## 1. Current State

The repository currently contains one ROS 2 package, **`rover_bringup`**, which
implements the **operational-state (arm / disarm) safety layer** in simulation.

### Nodes

| Node | Type | Responsibility |
|---|---|---|
| `odom_publisher` | Topic publisher | Publishes mock wheel odometry on `/odom/raw`. Drives a circular path when **armed**; freezes at zero velocity when **disarmed** (but keeps publishing). Speed (`linear_vel`, `angular_vel`) and `publish_rate_hz` are ROS 2 parameters, tunable at launch or runtime. |
| `odom_subscriber` | Topic subscriber | Diagnostic logger — echoes received position and heading. |
| `safety_status_service` | Service + publisher | `/rover/set_armed` service toggles the armed state and publishes it (latched) on `/rover/armed`. |

### How it is wired

```
safety_status_service ──/rover/armed (latched)──▶ odom_publisher ──/odom/raw──▶ odom_subscriber
   (arm / disarm authority)                        (gates its own motion)         (diagnostics)
```

Calling `/rover/set_armed {data: false}` flips the published state; the publisher
sees it and freezes the vehicle in place while continuing to publish at 20 Hz.
This models a safety operator enabling or disabling autonomy. Disarming commands
**zero velocity** rather than going silent — silence is reserved as a failure
signal for the watchdog (see below).

### Safety architecture (design intent)

The final vehicle separates **three independent stop mechanisms**. Conflating them
is a common design error; keeping them distinct is the point.

| Layer | Trigger | Reaction time | Status |
|---|---|---|---|
| **Operational state** (arm / disarm) | Deliberate operator command | seconds | Built |
| **Emergency stop** | Perception detects an obstacle in the path | milliseconds | Module 5 |
| **Failsafe watchdog** (dead man's switch) | Safety heartbeat **disappears** | milliseconds | Module 4 |

The watchdog is the critical layer: the vehicle is permitted to move **only while a
continuous safety heartbeat is present**, and it stops the instant that heartbeat
is lost — whether from a deliberate stop, a dropped radio link, or a software
crash. The heartbeat is produced automatically by the controller connection, so
no button needs to be held.

### Current limitations (intentional — addressed in later modules)

- **Simulation only.** Odometry is synthetic; no hardware drivers are connected yet.
- **The arm/disarm layer fails open.** If `safety_status_service` stops, the
  publisher keeps running on its last cached state. The *stop-on-silence* watchdog
  that closes this gap is **Module 4**.
- **No perception-based stopping** yet — **Module 5**.
- **No transform tree, launch files, or visualization** yet — **Module 1.5–1.7**.

---

## 2. The Stack

### Hardware

| Subsystem | Component | Role |
|---|---|---|
| Chassis | Traxxas Slash 2WD | Drivetrain (rear-wheel drive) |
| Compute | NVIDIA Jetson Orin Nano 8GB | Onboard compute |
| Motor control | Flipsky FSESC 6.7 (VESC-based) | Motor controller; UART/CAN telemetry |
| LiDAR | RPLIDAR S2L (360° DTOF) | Primary 360° obstacle detection |
| Camera | Intel RealSense D435if | Active-stereo depth + IMU |
| GPS | SparkFun ZED-F9P RTK | cm-level positioning (Phase 2) |
| Safety input | Logitech F710 gamepad | Manual override / emergency stop |

### Software

| Layer | Choice |
|---|---|
| OS | Ubuntu 22.04 |
| Middleware | ROS 2 Humble |
| DDS | Eclipse CycloneDDS |
| Language | C++ (primary); Python (scripts / prototyping) |
| State estimation | `robot_localization` EKF (planned) |
| Navigation | Nav2 (planned — Phase 2) |
| Dev environment | Docker (`osrf/ros:humble-desktop-full`), colcon; RViz2 / Gazebo via XQuartz |

### How data flows

Every sensor follows the same pattern — a hardware driver publishes onto a topic,
and downstream nodes subscribe to it:

```
VESC   ──/odom/raw──▶ EKF ──/odometry/filtered──▶ Nav2 ──/cmd_vel──▶ VESC
LiDAR  ──/scan───────▶ perception / costmaps
camera ──/image,/depth──▶ perception
```

Nodes are deliberately decoupled: each can be restarted independently, and they
affect one another **only** through the topics, services, and actions that wire
them together.

---

## 3. Roadmap

| Module | Focus | Status |
|---|---|---|
| 1 | ROS 2 fundamentals (C++): topics, services, actions, parameters, TF2, launch, RViz2 | In progress |
| 2 | Hardware bringup & driver integration | Planned |
| 3 | State estimation — EKF (from scratch + `robot_localization`) | Planned |
| 4 | Safety architecture — dead man's switch, watchdog, failsafes | Planned |
| 5a | Perception — segmentation & depth | Planned |
| 5b | Perception — 3D object detection & tracking | Planned |
| 5c | Perception — LiDAR–camera fusion | Planned |
| 6 | Simulation & fault injection (Gazebo) | Planned |
| 7 | Navigation & waypoint following (Nav2) | Planned |
| 8 | System integration & field testing | Planned |

---

## Repository layout

```
rover/
├── src/
│   └── rover_bringup/            # ROS 2 package: odometry + operational-state safety
│       ├── src/
│       │   ├── odom_publisher.cpp
│       │   ├── odom_subscriber.cpp
│       │   └── safety_status_service.cpp
│       ├── CMakeLists.txt
│       └── package.xml
├── docker/
│   ├── Dockerfile
│   └── run_ros2.sh
├── .gitignore
└── README.md
```

> `build/`, `install/`, and `log/` are colcon-generated and are intentionally
> excluded from version control (see `.gitignore`). They are recreated by
> `colcon build`.

---

## Build & run

**1. Start the container**

```bash
./docker/run_ros2.sh
```

**2. Build the workspace (inside the container)**

```bash
cd /home/ubuntu/rover_ws
colcon build --packages-select rover_bringup
source install/setup.bash
```

**3. Run the arm/disarm demo**

Open separate terminals into the running container with
`docker exec -it rover_ros2 bash`, sourcing `/opt/ros/humble/setup.bash` and
`install/setup.bash` in each:

```bash
# Terminal 1 — safety authority
ros2 run rover_bringup safety_status_service

# Terminal 2 — the vehicle (starts DISARMED, sits still)
ros2 run rover_bringup odom_publisher

# Terminal 3 — watch position
ros2 topic echo /odom/raw --field pose.pose.position

# Terminal 4 — drive the experiment
ros2 service call /rover/set_armed std_srvs/srv/SetBool "{data: true}"    # moves
ros2 service call /rover/set_armed std_srvs/srv/SetBool "{data: false}"   # freezes
```

When armed, the position advances along a circular path; when disarmed it holds
position while still publishing at 20 Hz.

---

## License

Not yet licensed — all rights reserved by the author pending a license decision.

## Acknowledgments

Developed with [Claude](https://claude.ai) (Anthropic) as a pair-programmer and
learning tutor for ROS 2 and C++ during the build.
