# Imu Filtering ROS Package

This ROS package reads data from an IMU message, filters it, and then integrates it to obtain Pose
**This has been tested on ROS Melodic.**

## Setting up

You must clone this repository as `imu_filtering` into your catkin workspace:

```bash
https://github.com/olayasturias/imu_filtering.git
```

Currently, the package `tf2_ros` is not working in ROS Melodic with Python3.
Clone the `geometry2` package and build it in your workspace with Python3:

```bash
https://github.com/ros/geometry2
```

Dependencies:

```bash
pip3 install numpy scipy PyWavelets statsmodels
```


## Compiling

You **must** compile this package before being able to run it. You can do so
by running:

```bash
catkin build
```

from the root of your workspace.

## Running

To run, simply run:

```bash
roslaunch imu_filtering imu_to_position.launch imu_topic:=</imu/topic> wavelet_mode:=<wav_mode> wavelet_level:=<level>
```

`imu_topic`, `wavelet_mode` and `wavelet_level` are run-time ROS launch arguments:
/
-   `imu_topic`: topic where your imu message is published, default: `/mavros/imu/data`.
-   `wavelet_mode`: wavelet mode for filtering, default: `db38`.
-   `wavelet_level`: wavelet level for filtering, default: `6`.


The `imu_filter` node will output to the following ROS topics:

-   `/imu_filtered`: `Imu` message. Filtered Imu data.
-   `/imu_pose`: `Pose` message. Pose obtained from integrating filtered imu data.
-   `/unfiltered_imu_pose`: `Pose` message. Pose from the unfiltered imu data, so you can compare how much it improves ;)

## Plotting

If you want to see the results, for example for x axis, run in your terminal:

```bash
rqt_plot /mavros/imu/data/linear_acceleration/x /imu_filtered/linear_acceleration/x /imu_pose/positn/x /unfiltered_imu_pose/position/x

```
