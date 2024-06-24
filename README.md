## Description
ROS2 node to process point cloud data from a radar system and calculate the corresponding body frame velocity. This process involves performing non-linear optimization using the point cloud data as inputs. To ensure accuracy, outliers are managed through the application of robust error models.

The node has been tested with the Smartmicro DRVEGRD 152 Front Radar and was designed for integration with the MicroStrain CV7-INS, but can applied for general use.

## Building from Source
1. Install [ROS2](https://docs.ros.org/en/humble/Installation.html) and [Create a Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
2. Clone the Repository into your workspace:
   ```
   git clone https://github.com/microstrain-robotics/radar_velocity_estimation.git --recursive
   ```
3. Install rosdeps for all the packages: `rosdep install --from-paths ~/your_workspace/src -i -r -y`
4. Build your workspace
   ```
   cd ~/your_workspace
   colcon build
   ```
   
## Running the node
```
source /your_workspace/install/setup.bash
ros2 run radar_velocity_estimation_node radar_velocity_estimation_node
```

## ROS Interfaces
| Topic                        | Type                                     | Description                                                                   |
|------------------------------|------------------------------------------|-------------------------------------------------------------------------------|
| `/smart_radar/can_targets_0`     | sensor_msgs/PointCloud2                  | Input 4D radar point cloud with speed                                         |
| `/cv7_ins/ext/velocity_body` | geometry_msgs/TwistWithCovarianceStamped | Output estimated bodyframe velocity with covariance                           |
| `/radar_velocity_viz`        | geometry_msgs/TwistStamped               | Output estimated bodyframe velocity, primarily used for visualization in RViz |

## License
MIT License
