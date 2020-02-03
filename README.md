# cmd_vel mux

## About

ROS 2 package for selecting among a number of incoming geometry_msg/msg/Twist messages, choosing the highest
priority one to republish on the output topic.  It will automatically dislodge streams that are lower
priority or that stop publishing for any reason.  The stream currently in use is published on the
"active" topic.

## Published Topics
* `/active` (`std_msgs/msg/String`) - A latched topic.  Publishes the "name" field of the currently active `geometry_msgs/msg/Twist` stream (see Parameters below), or "idle" if nothing is being a published.
* `/cmd_vel` (`geometry_msgs/msg/Twist`) - The current `geometry_msgs/msg/Twist` message.  The values from the current highest-priority `geometry_msgs/msg/Twist` are republished here without change.

## Subscribed Topics
* `/any` (`geometry_msgs/msg/Twist`) - One topic of type `geometry_msgs/msg/Twist` is subscribed to based on the `topic` parameter (see Parameters below).  The data from the highest-priority of these inputs is used as the output `/cmd_vel`.

## Parameters
* `name` (list of strings) - The "name" of each of the input `geometry_msgs/msg/Twist` topics.  This is what will be published on the `active` topic when the currently active publisher changes.
* `topics` (list of strings) - The "topic" corresponding to each of the input `geometry_msgs/msg/Twist` topics.  This is what will be subscribed to.  The length of this list must match the `name` list above.
* `timeout` (list of doubles) - The "timeout" of each of the input `geometry_msgs/msg/Twist` topics.  If no data is received on the input topic for this amount of time, the input will be automatically disabled.  The length of this list must match the `name` list above.
* `priority` (list of integers) - The "priority" of each of the input `geometry_msgs/msg/Twist` topics.  The higher the number, the higher the priority.  Higher priority topics will dislodge lower priority topics and start publishing to the output topic automatically.  The length of this list must match the `name` list above.
* `short_desc` (list of strings) - The "short description" of each of the input `geometry_msgs/msg/Twist` topics.  This is informational only.  The length of this list must match the `name` list above.
