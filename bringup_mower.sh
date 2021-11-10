rosrun mapping_explorer set_datum
rosbag record /tf_static /tf /imu/data /husky_velocity_controller/odom /outdoor_waypoint_nav/odometry/filtered \
/outdoor_waypoint_nav/odometry/filtered_map /gps/heading /gps/qual /gps/time_reference /gps/vel \
/husky_velocity_controller/cmd_vel /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/gps /imu_filter/rpy/filtered /imu/mag