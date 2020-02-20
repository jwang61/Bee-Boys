# Flying

## Flying procedure

1. Ensure that an `RX` connection is made with the drone
2. Turn off the safety switch by clicking the red flashing button on the GPS
3. Put drone into `POSITION` mode by switching the 3rd switch down
4. Arm the drone by switching the 4th switch down
5. Drone is now controllable
6. Once drone is landed, switch down the first switch to kill the drone

## Running MAVROS and QGroundControl

```
	<arg name="fcu_url" default="/dev/ttyUSB0:57600" />
	<arg name="gcs_url" default="udp://@localhost:14550" />
```


1. Launch QGroundControl
2. Ensure that QGroundControl is not looking at Sik Radio
3. run `roslaunch mavros px4.launch`