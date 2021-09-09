# mapping_explorer
## INSTALLATION
This is a ROS package. Compile it with catkin.

```sh
$ pip install -r requirements.txt
```

```sh
rosrun capture_depth.py
python calib_extrinsic.py

rosrun check_depth.py
```

## Template for commit mdg
What/How
[New]
[Fix]
[Enhancement]


## REFERENCE
[find closest dist](https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/) are a powerful concept. They allow you to, find the *closest point in the dataset*.

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
![image](https://github.com/wowyunDBL/mapping_explorer/blob/master/image/RGBD-point_cloud.png)



### teach you how to write argument
https://www.pyimagesearch.com/2021/02/22/opencv-connected-component-labeling-and-analysis/
