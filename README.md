# ArduPilot SITL Flight Task

Python mission script for ArduPilot Copter SITL.

## What the script does

- Connects to the simulator
- Verifies the start position is near Point A
- Takes off to mission altitude
- Flies from Point A to Point B
- Keeps a fixed yaw during the mission
- Lands near Point B
- Writes logs to file

## Mission coordinates

- Point A / Start: `50.450739, 30.461242`
- Point B / Target: `50.443326, 30.448078`
- Mission altitude: `300 m`

## Mission Planner setup

Before running the script, start the simulation in Mission Planner and set the home position near Point A.

Recommended home command:

```
--home=50.450739,30.461242,100,0
```

### Run order

- Start Mission Planner simulation
- Confirm the home marker is near Point A
- Run the Python script

## Dependencies

```python
dronekit
pymavlink
future
```

## Connection

The script uses the following connection string:

```bash
tcp:127.0.0.1:5762
```

### If your simulation uses a different port, update the value in main.py.

##Logging

Each run creates two files in the logs/ folder:

flight_YYYYMMDD_HHMMSS.log
flight_YYYYMMDD_HHMMSS.csv

The text log is useful for reading the full mission flow.
The CSV log is useful for compact analysis of position, altitude, yaw, and distance over time.

Safety checks

The script includes:

start-position validation near Point A
timeout checks for mission stages
abort if the vehicle moves too far away from the mission path
automatic RTL on failure
RC override cleanup
Result format
Code on GitHub
Screen recording of the script launch and flight
Link to the recording in Google Drive

---
```md
## Result

The mission was successfully completed in SITL:
- takeoff
- flight to target
- landing near Point B
- logs saved to file
```

### In the provided task description, SIM_WIND_TURB_FREQ was requested.
### In the current SITL/Copter parameter set this parameter was not available, so I used SIM_WIND_TC = 0.2 as the closest active parameter controlling wind variation timing.
