##############################################################################
# Tree
##############################################################################

# Leaves of the tree
Behaviour[] behaviours

# Whether the status of any single behaviour or the tree graph itself changed
bool changed

##############################################################################
# Blackboard
##############################################################################

diagnostic_msgs/KeyValue[] blackboard_on_visited_path
ActivityItem[] blackboard_activity

##############################################################################
# Other
##############################################################################

Statistics statistics
