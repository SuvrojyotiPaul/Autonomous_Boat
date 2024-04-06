# Autonomous_Boat

## Overview

This project aims to develop an autonomous boat capable of navigating and performing tasks with minimal human intervention. Utilizing ROS2 for communication and coordination among various components, the boat can autonomously navigate through water bodies, making it suitable for research, environmental monitoring. This README provides detailed information on the project's components, including ROS2 nodes for command publishing, gamepad control, GPS data handling, and servo control.

## Repository Contents

- **Command Publishers**: Scripts for publishing linear, angular, and string commands.
- **Controller Interfaces**: Gamepad and keyboard interfaces for manual control.
- **GPS Data Handling**: Nodes for publishing and simulating GPS data.
- **Servo Control**: Nodes for controlling servo motors based on different inputs.


## Nodes and Scripts

### Command Publishing Nodes

#### `command_publisher_linear&angular_inputs.py`

Serves as a ROS2 node designed for publishing linear and angular velocity commands. It's pivotal for controlling the autonomous boat's movement, allowing for dynamic adjustments to its speed and direction. 

Creates a publisher that posts messages of type Twist to the cmd_vel topic. It takes user input for linear and angular velocities, constructs a Twist message, and publishes it. This allows for real-time control of the boat's propulsion and steering.

**Running the Node:**

```sh
ros2 run my_package command_publisher_linear&angular_inputs.py
```
When prompted, enter the linear and angular velocities in the format <linear> <angular> (e.g., 1.5 0.2).


#### `command_publisher_stringCommand_inputs.py`

Designed to publish string-based commands to control servo mechanisms or other actuated components on the autonomous boat. It facilitates an interface for issuing high-level commands such as "left", "right", and "stop".

Establishes a publisher for sending messages of type String to the servo_commands topic.Implements a simple interface for real-time command input and validation.

**Running the Node:**

```sh
ros2 run my_package command_publisher_stringCommand_inputs.py
```

Input commands ("left", "right", "stop") as prompted to control the boat's direction.


### Control Interfaces

#### `gamepad_controller.py`

Integrates a gamepad into the ROS2 environment for the autonomous boat project, enabling manual control over the boat's movement through gamepad inputs.This ROS2 node subscribes to joystick inputs and translates them into linear and angular velocities and controls the motootrs in differntial mode.

Subscribes to the joy topic to receive joystick messages of type Joy from sensor_msgs and publishes velocity commands as Twist messages on the cmd_vel topic. The joy_callback fuction converts joystick axis movements into linear and angular velocities.

**Running the Node:**

```sh
ros2 run my_package gamepad_controller.py
```
Control the boat using the gamepad wihc is connected ot the raspberry pi usb port. I used a logitech f310.


#### `keyboard_command_publisher.py`

This ROS2 node enables manual control of the autonomous boat's movement through keyboard inputs. It translates **arrow key** presses into linear and angular velocity commands, publishing them as Twist messages.

Listens for arrow key presses to determine the desired movement direction and speed, converting these inputs into linear and angular velocities.


**Running the Node:**

```sh
ros2 run my_package keyboard_command_publisher.py
```
Use the arrow keys to control the boat's movement; press ESC to exit the program.


### GPS Data Handling

#### `gps_data_publisher.py`

This ROS2 node is designed for publishing GPS data, simulating real-world navigational inputs for the autonomous boat. It reads GPS coordinates from a CSV file and publishes them as NavSatFix messages.

Utilizes a timer to periodically publish GPS data as NavSatFix messages on the gps_data topic, simulating the receipt of real GPS data.
This was done to enable the development and testing of navigational algorithms without the need for live GPS data. It uses NavSatFix messages from sensor_msgs to represent GPS coordinates, facilitating integration with ROS2 navigation stacks.

**Running the Node:**

```sh
ros2 run my_package gps_data_publisher.py
```
Once you run it the node will read and publish GPS data from the CSV, logging the published coordinates.


### Servo Control Nodes

#### `servo_control_gps_data.py`

This ROS2 node that implements GPS-based navigation for the autonomous boat, controlling servo motors based on GPS waypoints. It subscribes to GPS data and uses this information to navigate towards predefined waypoints, adjusting the two motors to steer the boat accordingly.

**GPS Waypoint Navigation**: The node subscribes to gps_data topics for real-time GPS coordinates and navigates the boat towards a series of predefined waypoints.
**Servo Control**: Utilizes the gpiozero library to control servo motors attached to the boat, steering it based on the calculated bearing towards the next waypoint.
**Bearing Calculation**: Implements a function to calculate the bearing between the current location and the next waypoint, adjusting the boat's direction to follow this bearing.
**Movement Strategies**: Includes several methods to control the boat's movement, such as moving forward, turning slightly/sharply to the left or right, and stopping the servos.

**Running the Node:**

```sh
ros2 run my_package servo_control_gps_data.py
```
Once you run it the node will begin navigating towards the predefined waypoints, logging progress and steering the boat using the servo motors.


#### `servo_control_linear&angular_&_arrowkeys_inputs.py`

This ROS2 subscriber node designed to control servo motors based on linear and angular velocity commands received via the cmd_vel topic. This allows for the dynamic adjustment of the boat's direction and speed in response to real-time inputs, such as those from a keyboard . 

The node subscribes to the cmd_vel topic and translates the received Twist messages into servo movements, enabling precise control over the boat's movement. Contains a callback function that processes linear and angular velocities, calculating the appropriate speed for each servo to achieve the desired movement. 
Implements logic to convert the velocity commands into servo values, considering both the linear and angular components to calculate the differential drive speeds.This method allows for a wide range of movements, including sharp turns and straight movement.

**Running the Node:**

```sh
ros2 run my_package servo_control_linear&angular_&_arrowkeys_inputs.py
```
Dont forget to run the publihser node along with it (command_publisher_linear&angular_inputs.py or keyboard_command_publisher.py).
This will publish Twist messages to the cmd_vel topic to control the boat manually.


#### `servo_control_stringCommand_inputs.py`

This ROS2 subscriber node specifically designed for servo control based on string commands received through the servo_commands topic. This interface allows for straightforward command issuance such as "left", "right", and "stop", directly controlling the boat's movement or orientation with simple instructions.
The node subscribes to servo_commands, listening for incoming string messages that dictate servo actions.It demonstrates an effective method of translating user-friendly commands into complex hardware operations.

**Running the Node:**

```sh
ros2 run my_package servo_control_stringCommand_inputs.py
```
Dont forget to run the publihser node along with it (command_publisher_stringCommand_inputs.py).
This will publish string commands to the servo_commands topic (e.g., "left", "right", "stop") to control the direction or halt the boat's movement.
 

## Mock Data Generator

### `mock_gpsdata_generator.ipynb`

A Jupyter notebook for generating mock GPS data. Open and run this notebook in Jupyter to create CSV files with GPS data for testing.

Does the following:
Interpolates points between each waypoint to simulate gradual movement.
Generates timestamps for each mock GPS data point, simulating a 5-second interval between data points.
Writes the mock GPS data to a CSV file named mock_gps_data.csv.
By running this script, it will create a CSV file with mock GPS data representing the defined path. You can then feed this data into your navigation system to simulate movement along the path and observe how your differential steering responds to the variety of maneuvers required by the path.

