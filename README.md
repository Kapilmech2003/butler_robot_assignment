# Butler Robot



## Installation
Clone the Repository & Build the Package

```bash
mkdir butler_robot
mkdir butler_robot/src
cd butler_robot/src
git clone https://github.com/Kapilmech2003/butler_robot_assignment.git
cd ..
colcon build
source install/setup.bash
```
## Export the TURTLEBOT3_MODEL and the GAZEBO path
```bash
export TURTLEBOT3_MODEL=burger

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/butler_robot/src/turtlebot3_gazebo/models
```
## Run the Gazebo and Navigation Stack
Start the Simulation Environment
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Start Navigation
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```
Run the Butler Robot Navigation Script
```bash
ros2 run butler_robot butler_navigation
```
## 📌 Task Execution Scenarios

1️⃣ Simple Food Delivery

The robot moves from home → kitchen → table.

After delivering food, it returns to home.

✅ No confirmation required. 

[senario_1](https://drive.google.com/file/d/1QwYb0c15ushAJMW0O8XUPT4Mamdo2WxD/view?usp=drive_link)

2️⃣ Confirmation-Based Delivery

The robot waits for confirmation at the kitchen & table.

If no one confirms, the robot returns home after a timeout.

[senario_2](https://drive.google.com/file/d/1vw5WRbhClEbJ2FXoAOgPbiy844hCCFJ-/view?usp=drive_link)

3️⃣ Handling Timeouts

If the kitchen confirmation is missing, the robot returns home.

If the table confirmation is missing, the robot goes to the kitchen before returning home.

[senario_3](https://drive.google.com/file/d/13z8vXckVLyVjkRMzzcegCV9O2tsICkJd/view?usp=drive_link)

4️⃣ Handling Order Cancellations

If canceled while going to a table, the robot returns to the kitchen first, then goes home.

If canceled while going to the kitchen, the robot returns home directly.

[senario_4](https://drive.google.com/file/d/13z8vXckVLyVjkRMzzcegCV9O2tsICkJd/view?usp=drive_link)

5️⃣ Multi-Table Delivery

The robot moves home → kitchen → multiple tables → home.

All tables receive their orders before returning home.

[senario_5](https://drive.google.com/file/d/1IU9ERwKu4yevEcObNuYkVoNf66UxecJf/view?usp=drive_link)

6️⃣ Skipping Unconfirmed Tables

If a table does not confirm the order, the robot skips to the next table.

After the last table, it returns to the kitchen first before going home.

[senario_6](https://drive.google.com/file/d/1RFFYVUVHXbYdNCSkISm_ZOQqzoYAgd-h/view?usp=drive_link)

7️⃣ Handling Table Order Cancellations

If one table cancels, the robot skips it and continues deliveries.

After the last table, it returns to the kitchen before going home.

[senario_7](https://drive.google.com/file/d/1RFFYVUVHXbYdNCSkISm_ZOQqzoYAgd-h/view?usp=drive_link)


