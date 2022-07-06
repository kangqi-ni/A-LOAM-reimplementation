# A-LOAM Reimplementation

This project reimplements A-LOAM with more modularized design and clearer logic but also serves as a learning process to understand A-LOAM.

This reimplementation has the same performance with the original A-LOAM.

<img src="https://github.com/kangqi-ni/A-LOAM-reimplementation/blob/master/src/A-LOAM-reimplementation/picture/kitti_data.png"/>

Any rosbag that is compatible with A-LOAM can be used to run this algorithm. The configuration including topic names and parameters can be modified in config/aloam.yaml.

The dependencies are the same as A-LOAM, which are detailed in CMakeLists.txt.

References:

[1] Tong Qin, Shaozu Cao. (2020). A-LOAM: Advanced Implementation of LOAM. https://github.com/HKUST-Aerial-Robotics/A-LOAM. 

[2] Qian Ren. (2020). Localization in Auto Driving. https://github.com/Little-Potato-1990/localization_in_auto_driving. 
