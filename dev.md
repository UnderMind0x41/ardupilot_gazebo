# Arducopter gazebo simulation

## Task 1
We've arducopter simulation with gazebo. Default map with default drone has camera gimbal and video transmition via gstreamer. Can you add gimbal and video streamer to agrodrones which runs via scripts/start_sim.sh?
If so, please, do this.
Check running and make sure, that there are no any errors and issues. 


## Task 2
To our iris agrodrone (2 drones) add additional camera. This camera should be static, look to the bottom side and should have 130 degrees FOV.
FPS should be at least 10. Show me gstreamer picture from each camera.

Later we will use this camera as presicion landing using Apriltag, but it's later.


## Task 3 - Add AprilTag detection to Arducopter gazebo simulation
We've arducopter simulation with gazebo. There are two bases with Apriltags for presicion landing.
It seems like we need to implement ROS2 (maybe with MAVROS) to our system. There are some projects for AprilTag ROS2 implementation (https://github.com/christianrauch/apriltag_ros).
Can you implement ROS2 into my system for AprilTag detection? It's better to place ROS2-base not into gazebo folder, so we can use that ROS2 base in the our real system later.
Do we need to create ROS2 folder in the root folder? If so, do this. Also make ROS2 works together with our apriltag and show if detect works well.