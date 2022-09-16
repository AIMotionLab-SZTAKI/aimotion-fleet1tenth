# aimotion-fleet1tenth
ROS workspace containing code running on the controller computer.
## scripts
### `config`
Store all the configuration files needed. For further information check out the commented config files.
### `fleet1tenthpy`
Python package providing tools for teleoperation & control of multiple F1/10 vehicles running simultaneously 
## `src` - ROS packages
### `start`
Package contaning all the necessary launch files to start the system
### `keyboard_operator`
Package used to send control commands to the vehicle from the keyboard
> Deprecated in favor of `fleeth1tenthpy`'s `keyboard_control()`
### `drive_bridge_msgs`
Message definition for the comminucation with the vehicle's `drive_bridge` package 
### `vehicle_state_msgs`
Message definition for the vehicle's state
### `vehicle_state_logger`
Packege for the logging of the vehicle's state
### `vesc_msgs`
Message definition of the vehicle's onboard VESC motor controller ROS interface. Used to query the vehicle's state.