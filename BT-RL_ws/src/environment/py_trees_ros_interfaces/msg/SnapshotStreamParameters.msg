##############################################################################
#
# Configuration for a snapshot stream.
#
#    blackboard_data: publish blackboard variables on the visited path
#    blackboard_activity: enable and publish blackboard activity in the last tick
#    snapshot_period: period between snapshots (use /inf to only publish on tree status changes)
#
##############################################################################

float64 snapshot_period
bool blackboard_data
bool blackboard_activity

