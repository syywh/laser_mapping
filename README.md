# laser_mapping

## Brief introduction
building laser map offline
--only finish access to laser point now

## Dependence
* Ubuntu16.04, ros kinetic
* libpointmatcher
* libpointmatcher_ros
* ros-kinetic-gps-common
* GPS msgs: [rtk-gps](https://github.com/ThomasRobot/rtk_gps.git) and [poslvx](on the way)



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
* bagname: the absolute location of the bag
* cloud_in: the msg name of the laser points in the bag (type: sensor_msgs/PointCloud2)
* MapSavingFile: absolute saving location of the mapping result that contains the topological relations of the submap
* FramesSavingPath: absolute saving location of the mapping result that contains the mappoints of each submap

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
