# laser_mapping

## Brief introduction
Building laser map offline with submap-based structure (only finish access to laser point now).




## Dependence
* Ubuntu16.04, ros kinetic
* libpointmatcher
* libpointmatcher_ros
* ros-kinetic-gps-common 
* GPS msgs: [rtk-gps](https://github.com/ThomasRobot/rtk_gps.git) and poslvx

Part of the depandences without link can be found [here](https://github.com/syywh/laser_mapping_tools.git). Installation of **libpointmatcher** can be found [here](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/CompilationUbuntu.md). Our code is developed based its earlier version, you can utilize the package in this [repository](https://github.com/syywh/laser_mapping_tools.git) to avoid the failure of compilation. **libpointmatcher_ros**, **rtk-gps** and **poslvx** are ros packages and can be placed within a ros workspace and compiled by `catkin_make`. **ros-kinetic-gps-common** can be installed by
```
sudo apt-get install ros-kinetic-gps-common
```

## Compile
```
catkin_make -j3
```

## Usage
Ajust the parameters in launch file launch/laser_mapping_with_bag.launch and run 
```
roslaunch laser_mapping laser_mapping_with_bag.launch
```

parameters that must adjust:
* **bagname**: the absolute location of the bag
* **cloud_in**: the msg name of the laser points in the bag (type: sensor_msgs/PointCloud2)
* **MapSavingFile**: absolute saving location of the mapping result that contains the topological relations of the submap
* **FramesSavingPath**: absolute saving location of the mapping result that contains the mappoints of each submap

As we construct the laser map under submap-based structure, the results are saved topologically. We record the topological tree of the constructed map in `MapSavingFile`. Each node in the tree has a unique id and its relative transformation to the neighbouring nodes. Another xml file with a suffix of `_frames.xml` will be generated within the same directory to record the relative transformation of each laser frame with respect to the frame's reference submap. The local mappoints of the submap can be referred according to the id in `FramesSavingPath`.

## Citation
If you want to use this code in your papers, we'll be happy if you can cite

      @inproceedings{ding2018multi,
        title={Multi-session map construction in outdoor dynamic environment},
        author={Ding, Xiaqing and Wang, Yue and Yin, Huan and Tang, Li and Xiong, Rong},
        booktitle={2018 IEEE International Conference on Real-time Computing and Robotics (RCAR)},
        pages={384--389},
        year={2018},
        organization={IEEE}
      }
  
  If you want to use this topologic map structure in your navigation, we hope this paper could give you some inspiration
  
      @article{tang2019topological,
      title={Topological local-metric framework for mobile robots navigation: a long term perspective},
      author={Tang, Li and Wang, Yue and Ding, Xiaqing and Yin, Huan and Xiong, Rong and Huang, Shoudong},
      journal={Autonomous Robots},
      volume={43},
      number={1},
      pages={197--211},
      year={2019},
      publisher={Springer}
    }
