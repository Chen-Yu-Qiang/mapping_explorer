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
│   ├── icp_svd.ipynb # center of trunk match iteration (quite same as Automatic_xxx)
│   ├── image_util.ipynb
│   ├── spatio_transform_util.ipynb # point cloud distance test
│   ├── ground_subtr_npy.ipynb # using height/depth mask and calcu moment of obj and dist
│   ├── low_pass.ipynb  # moving average to extract ground
│   ├── transformation_height.ipynb  # equal height line and pitch offset for height
│   ├── transformation_pose.ipynb # icp for trunk skin
│   └── histogram_of_trunk.ipynb  # find grass and trunk histogram
│
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
│   ├── tf_remove_frames.py
│   └── rosmsg_depth_template.py  #publish depth.npy to rosmsg
└── src
    ├── depth_2_1dpcl_node.cpp
    └── test.cpp
```

## REFERENCE
[find closest dist](https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/) are a powerful concept. They allow you to, find the *closest point in the dataset*.



### shape file in gps(lat, lon)
![image](https://github.com/wowyunDBL/mapping_explorer/blob/master/image/RGBD-point_cloud.png)



### teach you how to write argument
https://www.pyimagesearch.com/2021/02/22/opencv-connected-component-labeling-and-analysis/
