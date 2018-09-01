## RRT Implementation
### This is a repo for RRT implementation on ROS.

#### struct
##### msg
* ellipsoid: define the ellipsoid with a center point, an angle and the length of the axises
* point: 2D point
* position: define the pose of a certain ellipsoid (robot) pose with a center point and an angle in [0, \pi)
* trajectory: a position array + a model
* pointsArray: a position array + the model's name, used to describe an area (arena, obstacles, robot)
* world: arena, robot and a obstacle array with a size, all use ellipsoid model

##### srv
* ellipsoid_points: convert position to pointsArray
* collision_detect: use a simple but imprecise method to estimate

#### rviz config
##### Global Optional
* Fixed Frame = "/root"
##### Marker
* arena
* obstacle0
* obstacle1
* robot
* trajectory

#### running note
after 'catkin_make' && 'source devel/setup.bash'
* run the server 
```
rosrun rrt_implement ellipsoid_point_gen_srv
rosrun rrt_implement collision_detection_srv
```
* run the rrt planner
```
rosrun rrt_implement rrtPlanner
```
* run the publisher
```
rosrun rrt_implement environment_pub
rosrun rrt_implement trajectory_pub
```
* run rviz
```
rosrun rviz rviz
```




