# Mushr Global Planner
By: Max Thompson  
Edited by: Matt Schmittle

## Tutorial:
Checkout our tutorial for setup/usage [here]()!

## Setup:
- Install ompl from source [here](https://ompl.kavrakilab.org/installation.html) with python bindings
    - If you have a segfault with CastXML on Ubuntu 16.04 as I did, install the binaries from [here](https://data.kitware.com/#collection/57b5c9e58d777f126827f5a1/folder/57b5de948d777f10f2696370). You will need to put the contents of `.../castxml/bin` into `/usr/local/bin/` and `.../castxml/share` into `/usr/local/share`. Also, uninstall your current version of cast xml `sudo apt remove castxml` 
    - make sure you have the [following](https://ompl.kavrakilab.org/installPyPlusPlus.html) installed
- Clone this repo into `.../catkin_ws/src/`:   
    `git clone https://github.com/thompsonmax/mushr_global_planner.git`
- Make and source your workspace. (Assuming source commands in .bashrc)
    `cd ~/catkin_ws && catkin_make && source ~/.bashrc`

## Demo
We have a pre-built demo that is easy to use just run the following:  
`roslaunch mushr_global_planner demo.launch`  
Then open rviz  
`rviz`  
After a minute, if you subscribe to the `/mushr_global_planner_result` topic you should see a red path.

## How the planner works
The planner is essentially a wrapper around OMPL doing Dubin’s path planning using BIT*. The input map is used to get the bounds of the state space and for determining if a sampled state is invalid (either by being out of bounds or within an obstacle). The planner uses the optimization objective of minimizing the path length of the solution. The planner then converts the solution path to a non-ompl format (list of tuples) and returns it to the user. The path is filled in with interpolated points via solutionPath.interpolate to create a dense path, but this can be tuned to vary the density of the output path.

#### Publishers
Topic | Type | Description
------|------|------------
`/mushr_global_planner_start`|[geometry_msgs/Pose](http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html)| Start position
`/mushr_global_planner_goal`|[geometry_msgs/Pose](http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html)|Goal position
`/mushr_global_planner_result`|[geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)| Planned path of poses in world coordinates from start to goal

#### Services
Topic | Type | Description
------|------|------------
`/mushr_global_planner`|[mushr_global_planner/MushrGlobalPlanner](srv/MushrGlobalPlanner.srv)| Calls for a path to be created with a given start, goal, turning radius, and planning time
