## Description
ROS2 node to process point cloud data from a radar system and calculate the corresponding body frame velocity. This process involves performing non-linear optimization using the point cloud data as inputs. To ensure accuracy, outliers are managed through the application of robust error models.

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
5. Source ROS2 and the Workspace
   ```
   source /opt/ros/humble/setup.bash
   source /your_workspace/install/setup.bash
   ```
   The source commands need to be run on every terminal before launching the node or these lines can just be added to the .bashrc

## Running the node
```
ros2 run radar_velocity_estimation_node radar_velocity_estimation_node
```
The Node was made for the Smartmicro DRVEGRD 152 Front Radar and the CV7_INS,
  * It Subscribes to `/smart_radar/targets_0` containing `sensor_msgs::msg::PointCloud2` messages
  * It Publishes `geometry_msgs::msg::TwistWithCovarianceStamped` to `/cv7_ins/ext/velocity_body`

The Topics subscribed/published to can be changed by modifying the file: `/radar_velocity_estimation_node/include/radar_velocity_estimation_node/topics.h`

## License
MIT License
