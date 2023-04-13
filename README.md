# Pointcloud distance and pose tracking
Here is two pkg in my project, some of the code can just run with another pkg *realsense-ros*. So before you test, please put these two pkg in *src* of your workspace and with *realsense-ros* together.
## Operating environment and necessary installation
* Ubuntu 22.04 + ROS Humble
* Realsense D435 camera + Intelrealsense SDK + [Intelrealsense Wrapper](https://github.com/IntelRealSense/realsense-ros)
* [Nuitrack install](https://github.com/3DiVi/nuitrack-sdk) + for [python beta](https://github.com/3DiVi/nuitrack-sdk/blob/master/PythonNuitrack-beta/README.MD)
* [InfluxDB-client](https://www.influxdata.com/blog/getting-started-python-influxdb/) for python *(and you also need to have your own InfluxDB account)*
## Finished Tasks
### pointcloud_test
this pkg (min-distance-get-pub.cpp) can get pointcloud from D435 camera and calculate min-distance between pointcloud and another point, then publish this distance and this point position as a topic *minimum_distance_topic*
**this pkg can only run with realsense-ros together**
#### run
In workspace, Terminal 1:
'<ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true>'
Terminal 2:
'<ros2 run pointcloud_test min_distance_get>'
### influxdb_test
this pkg realize:
* subscribe min-distance of pointcloud and send these Info to InfluxDB (min-distance-sub-inDB.py)
* use Nuitrack to get keypoints position (xyz) and send them to InfluxDB, here right hand position was used as example (nuitrack-to-InfluxDB.py)
* use Nuitrack and Opencv to track right hand position and every 300 frame to get a heat map, this heat map color indicates how long the right hand stayed in this position during this period. And at same time right hand position are send to InfluxDB (heatmap_test.py)
#### run
min-distance-sub-inDB.py **must run with min-distance-get-pub.cpp**
Terminal 3:
'<ros2 run influxdb_test min_distance_sub_inDB>'

nuitrack-to-InfluxDB.py and heatmap-test.py can run direct, as long as Nuitrack is installed


