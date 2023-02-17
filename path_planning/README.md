# The package has two executables:

1. ros_node 

1. env_node 

# RVIZ parameters: 

1. Frame_id = "/map" 

1. marker_topic = "path_planner_rrt" 

# Instructions:  

1. Open terminal and type 

```
roscore 
```

2. Open new terminal and go to the the root of your catkin workspace 

```
catkin_make 
source ./devel/setup.bash 
rosrun path_planner env_node 
```

3. open new terminal 

```
rviz
```

4. add a marker and change marker topic to "path_planner_rrt" 

5. Open new terminal
```
rosrun path_planner rrt_node 
```

