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
git branch -M main
git remote add origin git@github.com:wowyunDBL/mapping_explorer.git
git push -u origin main
``` 
