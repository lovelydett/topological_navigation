# topological_navigation
A ROS pkg to create and navigate on topological map in auto-driving.

## Topological map file format  
[N: number of points]  
[X: resolution 1:X(cm)]  
[N lines of id to coord:   
point_id x y z]  
[N lines of adj list:  
point_id n nb1 nb2 ... nbn]  

## Build topological map
```
rosrun topological_navigation topological_build_map
```
- press 'a' to add current point and an undirected edge <cur_point, last_point> into map  
- press 's' to save current map  

## Navigate through the topological map
```
rosrun topological_navigation topological_nav
```
- input a on-map destination coord (x, y)  
- node finds a shortest path on topological map, publishing middle points orderly as goal point, leading the robot towards final destination.

## Todo
1. configurations and arguments issues  
2. user-friendly way of interaction  
