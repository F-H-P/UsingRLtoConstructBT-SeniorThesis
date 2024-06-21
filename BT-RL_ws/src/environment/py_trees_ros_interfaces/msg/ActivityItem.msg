##############################################################################
#
# A single blackboard activity item
#
#    key: the active blackboard key
#    client_name : name of the blackboard client accessing the key
#    activity_type: type of operation attempted on the ky
#    previous_value: if the key was modified, stores the previous value of the key
#    current_value: if the key was modified, stores the current value of the key
#
##############################################################################

# Activity types
string READ = "READ"
string INITIALISED = "INITIALISED"
string WRITE = "WRITE"
string ACCESSED = "ACCESSED"
string ACCESS_DENIED = "ACCESS_DENIED"
string NO_KEY = "NO_KEY"
string NO_OVERWRITE = "NO_OVERWRITE"
string UNSET = "UNSET"

string key
string client_name
string activity_type
string previous_value
string current_value
