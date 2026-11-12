## General usage

The `navigator` is a ROS node that implements a path planner for
navigation and is supposed to be used in conjunction with the
'operator' node. Beside this, the whole functionality is provided by
the RobotNavigator class, which is exported by this package and can be
used from within other nodes as well.

Interaction with the Navigator is implemented using the
[actionlib](https://index.ros.org/p/actionlib) interface. This allows
other nodes (e.g. some high level mission control) to start actions like
goal navigation or exploration, monitor the progress and eventually
cancel an action in progress. The following four actions are available:

- MoveToPosition2D
- !StartMapping
- !StartExploration
- Localize

## Example

See the [Navigator Tutorial](https://index.ros.org/p/nav2d) for an
example how to setup the Navigator in Stage with a given map of the
environment.

## ROS Interface

### navigator

The `navigator` takes a map and the robot's position in this map to
generate a path to a goal point and send motion commands to the
`operator`.

### set_goal_client

Send a goal to the navigator via action interface.

### get_map_client

Initialize mapping (drive forward and turn) via action interface.

### explore_client

Start exploration via action interface.
