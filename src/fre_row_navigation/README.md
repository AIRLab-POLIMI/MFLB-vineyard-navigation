# FRE Row Navigation

<p float="left" align="middle"> 
  <img src="https://kamaro-engineering.de/wp-content/uploads/2015/03/Kamaro_Logo-1.png" width="250" style="margin: 10px;">
</p>
<p align="middle">
  <a href="https://clang.llvm.org/docs/ClangFormat.html"><img src="https://img.shields.io/badge/clang--format-Google-blue" alt="Code style: clang-format --style=Google"/></a>
  <a href="https://www.gnu.org/licenses/gpl-3.0"><img src="https://img.shields.io/badge/License-GPLv3-blue.svg" alt="License: GPL v3"/></a>
</p>
<p align="middle">
  This package contains the code used by Kamaro Engineering e.V. for navigating within a crop row at the Field Robot Event (FRE). The version published herein is a polished version of the code that won Task 1 of the <a href="https://web.archive.org/web/20210805203539/https://www.fieldrobot.com/event/index.php/2021/06/08/betegeuze-nova-wins-basic-navigation-task/">virtual FRE 2021</a>.
</p>
<p align="middle">
  This package assumes you are working on <b>ROS Noetic</b>.
</p>

## Working principle
The only sensor used for traversing the row is the laser scanner. For driving through the rows we search the filtered point cloud for an obstacle-free cone of configurable length, usually 1 - 2 m, in the filtered laser scan, beginning from the center (driving direction of robot). This gives us a line in the direction of the row. We shift this line to the left/right by an amount calculated from the ratio of the distances to the plants on the left/right. We clamp the distances to a maximum value to not be too confused by missing plants. The distances are calculated by growing a rectangle on the side of the robot until more than three points are inside it. This helps to avoid taking shortcuts through the plants in curves. We then use a PID controller to steer onto the point on this line that is 1m in front of the robot.

![Image of the visualization of this node in rviz](row_nav.png)

This algorithm steers the robot reactively and requires no concept of a field map. Notably, it controls the robot based on two different measures and controlled variables:

* the angular offset (controlling the robot orientation)
* the offset to the crop rows on both sides

Our main robot (Beteigeuze) for which this algorithm was originally designed, has two steerable axes and can therefore control both variables independently. For the simulated Jackal robot (and for our other Robot "Dschubba", which took part in FRE2019), which both use differential driving, we use an adapted version of the algorithm that compensates the side offset by adding an additional angle to the direction computed by the angular offset.

<small>Note: The above description was originally published in the proceedings document for 2021's Field Robot Event.</small>

## How to use
This package contains one node, `crawl_row_node`, that implements the algorithm described above.

### Needed inputs:
* a front-mounted LIDAR, available at
    * `/lidar_front` (as a `sensor_msgs/LaserScan` message)
    * `/lidar_front_cloud_filtered` (as a `sensor_msgs/PointCloud2` message, please see the [http://wiki.ros.org/laser_geometry](`laser_geometry`) and [http://wiki.ros.org/laser_filters](`laser_filters`) packages for converting the data)
    * `/odometry/filtered` containing an `nav_msgs/Odometry` message
    * `/tf` (for visualization, a `base_link` frame should be present)
    * a `/crawl_row` action [(type definition in this package)](action/CrawlRow.action)
        * publishing a goal on this action will start the row navigation
        * the action will complete when the end of the row is detected, and this node will become inactive again

### Outputs
* `/row_navigation/cmd_vel` (`geometry_msgs/Twist` message for controlling a robot with differential drive)
* `/crawl_row/viz` (`visualization_msgs/Marker` for RViz, showing the inner workings of the algorithm)

# License [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
The code in this repository is (c) by Kamaro Engineering e.V., subject to the file [LICENSE](LICENSE).
