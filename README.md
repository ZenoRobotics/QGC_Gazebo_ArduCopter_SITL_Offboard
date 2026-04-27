# ZenoRobotics Series: “Build a Drone Autonomy Stack from Scratch (No Hardware Required)”

## Big Picture

Simulation → Control → Safety → Autonomy → Perception → Real-World Bridge

# Video Series (YouTube) Structure

## Video 1 - Autonomous Drone Simulation with ArduPilot + Gazebo 
### Focus:
* Show the full system running
* Highlight:
  * takeoff
  * mission execution
  * geofence breach → Switch to RTL Execution
* Quick architecture explanation

## Video 2 - Video 2 – How to Setup ArduPilot SITL + Gazebo + QGroundControl (Step-by-Step)
### Overview

In this video, we set up a complete drone simulation environment using ArduPilot SITL, Gazebo, and QGroundControl. By the end, you will have a fully working virtual drone system ready for autonomous control.

### Setup Steps
1. Install ArduPilot
* Clone the ArduPilot repository
* Install required dependencies
* Set up the environment
    
2. Launch Gazebo Simulation
* Start the Gazebo world
* Load the Iris quadcopter model
* Verify simulation is running
    
3. Start ArduCopter SITL
* Run sim_vehicle.py with Gazebo support
* Enable MAVLink output streams

```
--out=udp:127.0.0.1:14540  # Python script
--out=udp:127.0.0.1:14550  # QGroundControl
```
4. Connect QGroundControl
* Launch QGroundControl
* Verify automatic connection
* Check telemetry (GPS, heading, altitude)
* Set vehicle mode to GUIDED

6. Understanding UDP Communication (Key Concept)
* ArduPilot sends MAVLink data over UDP
* Each port corresponds to a different consumer:
* 14550 → QGroundControl (monitoring)
* 14540 → Python script (control)

=> This allows multiple programs to interact with the drone simultaneously.

### Expected Result
* Gazebo simulation running
* ArduPilot SITL active
* QGroundControl connected and displaying telemetry
* System ready for offboard Python control
   
## Video 3 - Controlling a Drone with Python (MAVLink Offboard Explained)
### Content:
* Walk through your script:
  * heartbeat
  * connection
  * modes
  * commands
* Show:
  * takeoff
  * waypoint logic

## Video 4 - How Drone Safety Actually Works (Geofence + Failsafe Explained)
### Content:
* Geofence:
  * radius
  * altitude
* Failsafe:
  * MAVLink loss
* Show:
  * intentional breach
  * LAND trigger

## Video 5 - Analyzing Drone Missions with Logs (Replay + Debugging)
### Content:
* Show:
  * JSONL logs
  * replay mode
* Explain:
  * STATUSTEXT
  * mode transitions
* Example:
  * “Here’s exactly when the fence triggered”

## Video 6 - Adding Obstacle Avoidance to a Drone (State Machine Approach)
### Content:
* Your avoidance states:
  * HOLD
  * BACKUP
  * YAW
* Explain:
  * how sensors plug in
* Show:
  * simulated behavior (even mocked)

## Video 7 - How to Program Drone Missions with Math (Bearing + Waypoints)
### Content:
* Explain:
  * bearing → north/east conversion
  * lat/lon offsets
* Show:
  * dynamic waypoint generation

## Video 8 - From Simulation to Real Drone (What Actually Changes)
### Content:
* What stays the same:
  * MAVLink
  * logic
* What changes:
  * timing
  * sensors
  * noise

## Video 9 - Building a DIY Multispectral Drone (Low Cost vs Commercial)
### Content:
* Connect my earlier work:
  * OV9281 cameras
  * filters
* Explain:
  * spectral imaging basics

# QGroundControl + Gazebo + ArduCopter SITL + Offboard Python Control

This project demonstrates a complete software-in-the-loop (SITL) drone simulation pipeline using:
* Gazebo → physics-based simulation environment 
* ArduPilot (ArduCopter SITL) → flight controller firmware 
* QGroundControl (QGC) → ground control station  
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

https://ardupilot.org/dev/docs/sitl-with-gazebo.html \
https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

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
