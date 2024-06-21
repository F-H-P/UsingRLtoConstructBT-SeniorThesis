import json

safety_ensure = {
    "fallback":
    {
        "name":"Target Closed",
        "child":[
            {
                
            }
        ]
    }
    
}

bt_json = {
    "fallback":
    {
        "name":"Safety Ensure",
        "child":[
            {
                "NavigationToPose":
                {
                    "Pose":[0.0, 0.0, 0.0],
                },
                "GripperOpen":
                {
                    "Pose":[0.0, 0.0, 0.0],
                },
                "CreateTrajectory":
                {
                    "Pose":[0.0, 0.0, 0.0],
                }
            }
        ]
    },
    
}

test = {
    "type": "selector",
    "children": [
        {
            "type": "sequence",
            "children": [
                {
                    "type": "condition",
                    "name": "Is enemy near?"
                },
                {
                    "type": "action",
                    "name": "Run away"
                }
            ]
        },
        {
            "type": "action",
            "name": "Wander around"
        }
    ]
}
behavior_tree = {
    "type": "selector",
    "children": [
        {
            "type": "sequence",
            "children": [
                {
                    "type": "condition",
                    "name": "Is enemy near?"
                },
                {
                    "type": "action",
                    "name": "Run away"
                }
            ]
        },
        {
            "type": "action",
            "name": "Wander around"
        },
        test
    ]
}


print(behavior_tree)
# print(bt_json)
# 
# print(json.dumps(bt_json, indent=1))