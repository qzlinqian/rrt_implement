## RRT Implementation
### This is a repo for RRT implementation on ROS.

#### TO-DO list
##### initial config
- [x] package definition & configuration

##### algorithm itself
- [x] create basic structure of tree and node
- [x] complete insert & remove function
- [x] grow the tree
- [ ] collision checking (FCL)


##### world
- [x] environment
- [ ] graph -- too much to display

##### final work
- [ ] visualize in rviz
- [x] trajectory

#### package construct
##### msg
* ellipsoid: define the ellipsoid with a center point, an angle and the length of the axises
* point: 2D point
* position: define the pose of a certain ellipsoid (robot) pose with a center point and an angle
* trajectory: a position array + a model
* pointsArray: a position array + the model's name, used to describe an area (arena, obstacles, robot)
* world: arena, robot and a obstacle array with a size, all use ellipsoid model

##### srv
* ellipsoid_points: convert position to pointsArray   // need to complete
* collision_check   // need to complete
