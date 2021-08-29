```
rosrun capture_depth.py
python calib_extrinsic.py

rosrun check_depth.py
```

rosbag record /amcl_pose /tf /outdoor_waypoint_nav/odometry/filtered_map


### github util
```
echo "# mapping_explorer" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M master
git remote add origin git@github.com:wowyunDBL/mapping_explorer.git
git push -u origin master
``` 

### shape file in gps(lat, lon)
[!image](https://github.com/wowyunDBL/mapping_explorer/blob/master/image/RGBD-point_cloud.png)