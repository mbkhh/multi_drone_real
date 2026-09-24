# swarm_station_decenter

Interactive station for `swarm_single_decenter`. All drones publish directly
to the station; there is no elected leader.

Commands without a target go to all drones. Prefix a command with a numeric ID
or `drone ID` to address only one vehicle:

```text
status
arm
takeoff
mission
land

all arm
all takeoff 3
all mission waypoints_decentralized_36x34.yaml
all land

start_detection return_land 32,29
start_detection report 32,29
stop_detection

2 arm
2 takeoff 3
2 mission
2 move 10 12 2
2 yaw -22.5
2 land
```

`status` prints every drone whose status was received in the last five
seconds, including position, control state, arming/Offboard state, mission
progress, and the peer IDs that drone currently sees.

The normal three-drone mission sequence is:

```text
status
all arm
status
all takeoff 2
all mission
```

Wait for `status` to show all drones armed and in Offboard before takeoff, and
confirm takeoff before starting the mission.

Vision commands are always sent to all drones. In `return_land` mode, a target
event from any unchanged `swarm_vision` node makes every armed Offboard drone
return at its current height to its own configured initial position and then
start its existing controlled landing independently.
