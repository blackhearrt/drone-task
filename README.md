# ArduPilot Test Task

Python script for controlling ArduPilot SITL copter via DroneKit.

## Current progress

- Mission Planner SITL launched
- DroneKit connection established
- STABILIZE mode confirmed
- Vehicle arming tested
- RC override test completed

## Connection

- tcp:127.0.0.1:5762

## Dependencies

- dronekit
- pymavlink
- future


In the provided task description, SIM_WIND_TURB_FREQ was requested. In the current SITL/Copter parameter set this parameter was not available, so I used SIM_WIND_TC = 0.2 as the closest active parameter controlling wind variation timing