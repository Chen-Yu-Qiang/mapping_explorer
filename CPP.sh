roslaunch depth_app hector_mower.launch
python3 /home/ncslaber/mapping_node/mapping_ws/src/mapping_explorer/scripts/test.py
python3 /home/ncslaber/cpp/path_planning.py
python3 /home/ncslaber/cpp/plot_pickle.py
