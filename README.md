# mapping_explorer
## INSTALLATION
```sh
$ pip install -r requirements.txt
```

```sh
rosrun capture_depth.py
python calib_extrinsic.py

rosrun check_depth.py
```

This is a ROS package. Compile it with catkin.
```sh
mkdir build
cd build
cmake ..
make

```
## Template for commit mdg
What/How  
[New]
[Fix]
[Modi]

## file_list
```
.
├── demo.md  
├── demo.sh
├── jupyterNotebook
│   ├── Automatic_tree_detection.ipynb
│   ├── demo_test_loc_series(deletable).ipynb
│   ├── image_util.ipynb
│   ├── spatio_transform_util.ipynb
│   ├── Untitled1.ipynb
│   └── Untitled.ipynb
├── launch
│   ├── rosbagPlayer_w_tf_static_template.launch  # play rosbag with complementary tf tree
│   └── tf_reframe.launch
├── library
│   ├── loadData_utils.py
│   ├── map_utils.py
│   ├── plot_utils.py
│   └── rosbag_utils.py
├── release-notes.md
├── requirements.txt
├── scripts
│   ├── calib_extrinsic.py
│   ├── capture_depth.py
│   ├── cb_pose.py
│   ├── cb_topic2npy.py
│   ├── check_depth.py
│   ├── find_trunk_center_inaframe.py
│   ├── landmark_matcher.py
│   ├── mapping_landmark.py
│   ├── plot_landmark.py
│   └── tf_remove_frames.py
└── src
    ├── depth_2_1dpcl_node.cpp
    └── test.cpp
```

## REFERENCE
[find closest dist](https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/) are a powerful concept. They allow you to, find the *closest point in the dataset*.


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
