# Direct Stereo Visual Odometry
Copyright (c) <2018> <Jiawei Mo, Junaed Sattar>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Dependencies
- [ROS](http://www.ros.org/)

- [g2o](https://github.com/RainerKuemmerle/g2o)

# Installation
```
cd ~/catkin_ws/src
git clone https://github.umn.edu/moxxx066/dsvo.git
cd ~/catkin_ws
catkin_make
```

# Usage
Calibrate cameras using [kalibr](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration)

Follow [sample.launch](https://github.umn.edu/moxxx066/dsvo/blob/master/launch/sample.launch)

# Published Topics
/cam_pose (geometry_msgs/PoseWithCovarianceStamped)

/points (sensor_msgs/PointCloud2)

# Output files under .ros
vo.txt: generated odometry (time 0:dsvo;stereo_match x y z)

time.txt: computational time consumption, check [time_format.txt](https://github.umn.edu/moxxx066/dsvo/blob/master/test/time_format.txt) for details

# Parameters
BLUR_SZ: blur size

BLUR_VAR: blur value

FEATURE_BLOCKS: divide image to blocks and detect feature on each block

FEATURE_THRES: feature threshold

FEATURES_PER_CELL: number of features per block

FEATURE_OF_PYMD: number of pyramid used to track new points

PROP_PYMD: number of pyramid used to propagate camera pose

PROP_MAX_STEP: max optimization step for camera pose propagation

PROP_PROJ_DIST: max projection dist for points to be inlier during camera pose propagation

PROP_PTS_RATIO: min ratio for successful camera pose propagation

PROP_MIN_POINTS: min inliers for successful camera pose propagation

KF_DIST: min dist for new keyframe

BA_INLIER_THRES: min ratio for successful bundle adjustment

BA_REPROJ_DIST: max projection dist for points to be inlier during bundle adjustment

BA_MAX_STEP: max optimization step for bundle adjustment

SCALE_PYMD: number of pyramid used to optimize scale

SCALE_MAX_STEP: max optimization step for scale optimization
