# QGroundControl + Gazebo + ArduCopter SITL + Offboard Python Control

This project demonstrates a complete software-in-the-loop (SITL) drone simulation pipeline using:
* Gazebo → physics-based simulation environment \
* ArduPilot (ArduCopter SITL) → flight controller firmware \
* QGroundControl (QGC) → ground control station  \
* Python (pymavlink) → offboard control, mission logic, and safety behaviors  

The system enables realistic autonomous flight testing without (drone) hardware, including mission execution, geofencing, failsafes, and optional collision avoidance logic.

# Learning Objectives
This system teaches:
* MAVLink communication and multi-endpoint routing
* SITL-based drone testing workflows
* Autonomous mission generation using geometry
* Safety-critical behaviors (geofence, failsafe)
* Integration of offboard control with onboard autopilot
* Logging and replay for debugging and validation

## System Architecture

Gazebo (world + vehicle physics) \
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;        ↓   \
ArduCopter SITL (flight controller) \
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;        ↓  \
&emsp;&emsp;&emsp;&emsp;&emsp;   MAVLink (UDP) \
&emsp;&emsp;&emsp;&emsp;&emsp;      ↙   &emsp;    ↘    \
QGroundControl &emsp;  Python Offboard Script \
   (GCS)   &emsp; &emsp;&emsp;&emsp;&emsp;     (custom autonomy logic)


## Software Installation


## Simulation Execution Workflow

### Terminal 1 

```
gazebo --verbose worlds/iris_arducopter_runway.world
```

### Terminal 2 

```
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris \
  --out=udp:127.0.0.1:14540 \
  --out=udp:127.0.0.1:14550
```
* Runs ArduPilot firmware in simulation
* Sends MAVLink telemetry to:
  * 14550 → QGroundControl
  * 14540 → Python offboard script

### Terminal 3 — QGroundControl
```
./QGroundControl
```
* Connects automatically via UDP
* Used for:
  * Vehicle monitoring
  * Mode switching (GUIDED, AUTO)
  * Visual telemetry (GPS, heading, etc.)

Note: Set vehicle to GUIDED mode in QGC top before running the script.

### Terminal 4 — Python Offboard Script
```
python3 offb_ardu_w_geo_multi_wpt.py \
  --bearing {initial_heading_deg} \
  --conn udp:127.0.0.1:14540 \
  --baud 57600 \
  --avoid
```

# Offboard Script Overview

The Python script (offb_ardu_w_geo_multi_wpt.py) implements a mission-level autonomy layer on top of ArduPilot.

## Core Responsibilities

### 1. Initialization
* Waits for MAVLink heartbeat
* Retrieves home GPS position
* Confirms GPS lock (3D fix)

### 2. Safety Configuration (Kill Envelope)
The script programs key safety features into ArduPilot:

* Geofence
  * Circular boundary (e.g., 18 m radius)
  * Max altitude limit
  * Action: LAND on breach
* GCS Failsafe
  * Triggered on MAVLink loss
  * Action: LAND

> This creates a software-defined safety envelope

### 3. Guided Takeoff
* Switches to GUIDED mode
* Arms vehicle
* Executes takeoff to ~5 meters

### 4. Mission Generation (Geometric Waypoints)
Waypoints are generated relative to:
```
home position + initial bearing
```
Example mission:
* WP1 → inside geofence
* WP2 → outside geofence (intentional breach)
* RTL → fallback

This enables controlled testing of:
* geofence behavior
* failsafe responses

### 5. Mission Execution
* Uploads mission via MAVLink
* Switches to AUTO mode
* Monitors execution in real time

### 6. Real-Time Monitoring & Logging

The script logs:
* STATUSTEXT messages (fence, failsafe events)
* flight modes
* GPS position
* system state transitions

All events are saved in:
```
JSONL (flight_events.jsonl)
```
This allows:
* replay
* debugging
* post-flight analysis

### 7. Collision Avoidance Hooks (Optional)

The script includes a state-machine-based avoidance framework:

States:
```
CRUISE → HOLD → BACKUP → YAW → RESUME
```
Inputs:
* distance sensors (MAVLink DISTANCE_SENSOR / OBSTACLE_DISTANCE)

Actions:
* stop
* reverse
* yaw away
* resume mission

Note: Currently implemented as hooks for extension, not full control override

## Replay & Analysis Tool

The script supports log replay:
```
python3 offb_ardu_w_geo_multi_wpt.py --replay
```
Provides summary of:
* fence breaches
* failsafe triggers
* landing events
* mode transitions

## Key Insight
* This setup mirrors real-world autonomy stacks:
* Autopilot (ArduPilot) = low-level flight control  
* Offboard script = mission intelligence + safety logic  
* QGC = operator interface

# Summary

You’ve built a modular drone autonomy testbed that enables:
* safe experimentation
* repeatable mission testing
* extensible autonomy (e.g., avoidance, perception)
—all without hardware.
