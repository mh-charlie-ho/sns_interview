# SNS_Kitchen

**Dependency**
- ros2 version: humble

**Install & Build**
```
git clone https://github.com/mh-charlie-ho/sns_kitchen.git
cd sns_kitchen
colcon build
source ./install/setup.bash
```

## View the .srv setting for Q2.
``` bash
# sns_msgs/ConveyorAction.srv
ros2 interface show sns_msgs/srv/ConveyorAction
```
``` bash
# sns_msgs/RobotAction.srv
ros2 interface show sns_msgs/srv/RobotAction
``` 

## View the result of Q3.
The first terminal to launch the conveyor, robot, heat sensor, and stove server.
You will see the server statment.

``` bash
ros2 launch task_planning machine_server_launch.py
```

The second terminal for user interface
```bash
ros2 run task_planning console
```
You can modify the parameter file to control the variables in the task.
Go to `<root path>/src/task_planning/config/params.yaml`
- ingredient_id: 3
  -  option: 1, 2, 3
- robot_position: [5.0, 8.0]
  -  range(float): -10.0~10.0
- stove_level: "Max"
  - option: OFF, Min, Med, Max
- keep_temperature_sec: 5.0
  - range(float): [0, inf)
  
After modification, use the command to update the parameters
```
ros2 param load /console <root path>/src/task_planning/config/params.yaml
```

## View the result of Q4.
Launch the machine
``` bash
ros2 launch task_planning machine_server_launch.py
```
Call the server
``` bash
 ros2 service call /robot_service sns_msgs/srv/RobotAction "{x: 2, y: 2}"
```
Monitor the joint command
``` bash
 ros2 topic echo /joint_cmd
```
Monitor the joint state
``` bash
 ros2 topic echo /joint_state
```

## View the result of Q5.

