# topological_navigation
A ROS pkg to create and navigate on topological map in auto-driving.

## Topological map file format  
[N: number of points]  
[X: resolution 1:X(cm)]  
N lines of id to coord: [point_id x y z]  
N lines of adj list: [point_id n nb1 nb2 ... nbn]  

## Build topological map
```
roslaunch topological_navigation build_map.launch
```
- press 'a' to add current point and an undirected edge <cur_point, last_point> into map  
- press 's' to save current map  

## Navigate through the topological map
```
roslaunch topological_navigation navigation.launch
```
- input an on-map destination coord (x, y)  
- node will find a shortest path on topological map, publishing middle points orderly as goals, leading the robot towards final destination.

## Arguments  
Modify arguments such as resolution, threshold(the range each point covers) and topic names in launch files. Be careful of the scope of ros parameters.

## Todo
1. a user-friendly way of interaction
2. topological points pruning and combining
