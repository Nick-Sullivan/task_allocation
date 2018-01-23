# centralised_auction
ROS package for multi robot task allocation. Given a series of task and robot locations, allocates the tasks to robots using sequential single-item auction.

## 1. Nodes
### 1.1 centralised_auction_node
#### 1.1.1 Published Topics
`prefix#/tasks` ([`WaypointArray`](https://github.com/Nick-Sullivan/waypoint_follower/blob/master/waypoint_follower_msgs/msg/WaypointArray.msg))

The tasks to be completed, in order. `prefix` is a given parameter, and `#` is the robot ID.

#### 1.1.2 Parameters
`~prefix` (`string`, default: `robot`)

The published topic prefix.

`~tasks` (`array`, default: `[]`)

The positions (x,y,z) and orientations (x,y,z,w) of the tasks.

`~robots` (`array`, default: `[]`)

The positions (x,y,z) of the robots.
