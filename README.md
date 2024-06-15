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
- ingredient_id: 1
- robot_position: [12.3, 20.8]
- stove_level: "Max"
- keep_temperature_sec: 20
  
After modification, use the command to update the parameters
```
ros2 param load /console <root path>/src/task_planning/config/params.yaml
```

## View the result of Q4.
