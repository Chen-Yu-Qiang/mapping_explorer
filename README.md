```sh
$ pip install -r requirements.txt
```


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
![image](https://github.com/wowyunDBL/mapping_explorer/blob/master/image/RGBD-point_cloud.png)

https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/

### teach you how to write argument
https://www.pyimagesearch.com/2021/02/22/opencv-connected-component-labeling-and-analysis/
