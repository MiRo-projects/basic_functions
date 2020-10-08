# Basic Functions

Basic interface and functions for MiRo, to avoid re-inventing the wheel with every MiRo project.

For development of standard operations such as movement, turning (eventually using the kinematic chain), sensation, etc; new MiRo projects should be able to refer to a standard function here rather than having to re-code the same basic interactions anew each time.

## Usage
### MiRo-ROS interface

The MiRo-ROS interface allows you to easily interface with the robot without re-implementing the ROS subscriber / publisher commands for each project. Getting MiRo moving and illuminated is as easy as:
```
import miro_ros_interface as mri
miro_pub = mri.MiRoPublishers()
miro_pub.pub_cmd_vel_ms(left=0.2, right=0.2)
miro_pub.pub_illum(all=0xFF00FFFF)
```
Sensor data can be read with:
```
import miro_ros_interface as mri
miro_sen = mri.MiRoSensors()
print(miro_sen.light)
print(miro_sen.cliff)
``

See the [MiRo-E ROS technical interface](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Interfaces_ROS) document for more information on available ROS data.
