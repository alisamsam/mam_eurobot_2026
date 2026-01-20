# Commandes ROS 2 Eurobot 2026

## 1. Build & Launch

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch mam_eurobot_2026 arena.launch.py
ros2 launch mam_eurobot_2026 combined.launch.py
ros2 launch mam_eurobot_2026 arena.launch.py launch_rviz:=true
ros2 launch mam_eurobot_2026 arena.launch.py gazebo_headless:=true launch_rviz:=true
```

### Options de lancement modulaires (Optimisation)
- **Lidar seul :**
  ```bash
  ros2 launch mam_eurobot_2026 arena.launch.py launch_lidar:=true
  ```
- **SLAM :**
  ```bash
  ros2 launch mam_eurobot_2026 arena.launch.py launch_slam:=true launch_lidar:=true
  ```
- **Caméras :**
  ```bash
  ros2 launch mam_eurobot_2026 arena.launch.py launch_camera:=true
  ```
- **Tout activer :**
  ```bash
  ros2 launch mam_eurobot_2026 arena.launch.py launch_lidar:=true launch_camera:=true launch_slam:=true launch_rviz:=true
  ```
- **Lidar, SLAM, RViz, Headless :**
  ```bash
  ros2 launch mam_eurobot_2026 arena.launch.py launch_lidar:=true launch_slam:=true launch_rviz:=true gazebo_headless:=true
  ```

## 2. Gazebo Model Management

```bash
ign model --remove simple_robot
ign service -s /world/eurobot_2026_arena/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 300 \
  --req 'sdf_filename:"/workspaces/mam_eurobot_2026/models/simple_robot", name:"model"'

ros2 run ros_gz_sim create \
  -file /workspaces/mam_eurobot_2026/models/simple_robot/model.sdf \
  -name simple_robot -x 0 -y 0 -z 0.05

ign service -s /world/eurobot_2026_arena/remove \
  --reqtype ignition.msgs.Entity \
  --reptype ignition.msgs.Boolean \
  --timeout 300 \
  --req 'name:"simple_robot", type:MODEL'

# Crates
ign service -s /world/eurobot_2026_arena/remove \
  --reqtype ignition.msgs.Entity \
  --reptype ignition.msgs.Boolean \
  --timeout 300 \
  --req 'name:"crate", type:MODEL'

ros2 run ros_gz_sim create \
  -file /workspaces/mam_eurobot_2026/models/crate/model.sdf \
  -name crate -x 0.3 -y 0.3 -z 0.05

ros2 run ros_gz_sim create \
  -file /workspaces/mam_eurobot_2026/models/crate/crate_aruco.sdf \
  -name crate2 -x 0.4 -y 0.3 -z 0.05
```

## 3. Bridges utiles (caméra, lidar, TF, etc.)

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image

ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
  /model/simple_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V \
  --ros-args -r /lidar:=/scan

ros2 run ros_gz_bridge parameter_bridge \
  /model/simple_robot/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V
```

## 4. Notes

- ROS spawn objects at their center of mass:

  (+Y) <--- X ---> (-Y)
   ^                   v
  +X                 -X

- Rotation order: -R (roll), -P (pitch), -Y (yaw)
- RViz parfois bugs :
  ```bash
  rm -rf ~/.rviz2
  ```
- Rentrer dans le terminal :
  ```powershell
  docker exec -it quizzical_einstein bash (windows vscode)
  ```
- lancer Keyctrl via powershell
  /usr/local/python/current/bin/python /workspaces/mam_eurobot_2026/mam_eurobot_2
026/key_ctrl.py

- goal pose
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}
  '
  
  implement the imu to control the drifting 
  import mecanum wheels visual 

  use a wrapped camera technic, changer l'orientation de l'output image avant de proceder a la detection avec opencv 

  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: -0.1, y: -0.410, z: 0.0}, orientation: {w: 1.0}}}}"
  

  for the next session i need to implement the gripper, make a video of crate catching, and stategy, 