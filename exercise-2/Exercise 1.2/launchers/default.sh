#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
# roscore & # needed if compiling locally
# sleep 5

# Launch the nodes manually
# dt-exec rosrun my_package my_publisher_node.py
# dt-exec rosrun my_package my_subscriber_node.py

# Launch the nodes using the launcher
# roslaunch my_package multiple_nodes.launch # the one that doesn't care about namespaces
roslaunch my_package multiple_nodes.launch veh:=$VEHICLE_NAME

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
