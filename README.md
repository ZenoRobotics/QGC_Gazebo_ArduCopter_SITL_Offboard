# QGroundControl + Gazebo + ArduCopter SITL + Offboard Python Control

This project demonstrates a complete software-in-the-loop (SITL) drone simulation pipeline using:

* Gazebo → physics-based simulation environment \
* ArduPilot (ArduCopter SITL) → flight controller firmware \
* QGroundControl (QGC) → ground control station  \
* Python (pymavlink) → offboard control, mission logic, and safety behaviors  

The system enables realistic autonomous flight testing without (drone) hardware, including mission execution, geofencing, failsafes, and optional collision avoidance logic.

## System Architecture

Gazebo (world + vehicle physics) \
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;        ↓   \
ArduCopter SITL (flight controller) \
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;        ↓  \
&emsp;&emsp;&emsp;&emsp;&emsp;   MAVLink (UDP) \
&emsp;&emsp;&emsp;&emsp;&emsp;      ↙   &emsp;    ↘    \
QGroundControl &emsp;  Python Offboard Script \
   (GCS)   &emsp; &emsp;&emsp;&emsp;&emsp;     (custom autonomy logic)

   
