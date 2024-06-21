# Possible behaviour states - these must match py_trees.common.Status
uint8 INVALID = 1
uint8 RUNNING = 2
uint8 SUCCESS = 3
uint8 FAILURE = 4

# Possible types of behaviour
uint8 UNKNOWN_TYPE = 0
uint8 BEHAVIOUR = 1
uint8 SEQUENCE = 2
uint8 SELECTOR = 3
uint8 PARALLEL = 4
uint8 CHOOSER = 5
uint8 DECORATOR = 6

# Blackbox Level - these must match py_trees.common.BlackBoxLevel
uint8 BLACKBOX_LEVEL_DETAIL = 1
uint8 BLACKBOX_LEVEL_COMPONENT = 2
uint8 BLACKBOX_LEVEL_BIG_PICTURE = 3
uint8 BLACKBOX_LEVEL_NOT_A_BLACKBOX = 4

# Key Access Levels - these must match py_trees.common.Access
string BLACKBOARD_ACCESS_READ = "r"
string BLACKBOARD_ACCESS_WRITE = "w"
string BLACKBOARD_ACCESS_EXCLUSIVE_WRITE = "x"

# Name given to the behaviour on construction
string name

# Name of the class of this behaviour
string class_name

# The ID of this behaviour
unique_identifier_msgs/UUID own_id

# The ID of the parent behaviour
unique_identifier_msgs/UUID parent_id

# ID of the tip of this behaviour. If the tip is the behaviour itself or None,
# this is the zero-filled UUID
unique_identifier_msgs/UUID tip_id

# The IDs of children of this behaviour, if any
unique_identifier_msgs/UUID[] child_ids

# The ID of the current child (in case of a composite).
# If None, this is the zero-filled UUID.
unique_identifier_msgs/UUID current_child_id

# A composite, sequence, selector, plain behaviour, etc.
uint8 type

# Additional detail about this behaviour, useful for debugging. Example: Parallel policy
string additional_detail

# Blackbox level (for visualisation purposes)
uint8 blackbox_level

# The current state of execution. Can be invalid, running, success or failure
uint8 status

# The message attached to the current execution state
string message

# Whether this node was part of the last tick
bool is_active

# Blackboard keys and access level registered by blackboard clients of this behaviour
KeyValue[] blackboard_access
