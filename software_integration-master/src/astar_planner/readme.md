# Running the astar_planner

```sh
$ roslaunch odometry_agent odometry_agent.launch is_four_wd:=true is_simulation:=true
$ morse run fourwd fourwd/default.py
$ rosrun rviz rviz 
$ rosrun astar_planner astarPlanner.py
#---- wait until the odometry info has been received by the node ----
$ rosservice call /get_astar_path 
#---- input the goal position where there are no obstacles, it will throw out an error if it is not the case ------
```
