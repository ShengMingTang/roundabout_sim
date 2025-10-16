# Roundabout Simulation Project
Simulation of cars running on a multi-lane roundabout.
## Rules
1. There are 3 actions
    1. Stop: stay still in the current tick
    1. Straight: go straight
    1. Switch(i32): Switch inward (switching to higher index lane) or outward (switching to lower index lane). Switching is conducted radially.
## Collision Check
1. Switch collision: the switching car is switching to next lane, called the target point, and the target point falls onto the arc that is going to be occupied by another one going straight with some update time. One of them will be rejected according to *switch_policy*
1. Straight collision: one car going straight and is going to collide (overlap) with another one in front of the former. The former's straight action will be rejected or truncated (not allowed to advance that much) if the advancing distance is too small
# Configuration
``` json
{
    "n_inter": 2, // number of entrance/exit evenly spaced at a roundabout, 0th is placed at theta 0

    // radius of lanes
    // index 0 means the outermost one, values must be in decreasing order
    "r_lanes": [1.0],
    "switch_policy": "StraightFirst|SwitchFirst", // when cars are about to collide with each other, specify which can go
    "tick": 0.1, // simulation granularity
    "init": {
        "0": { // id
            "dst": 1, // destination
            "vel": 1.0, // velocity
            "lane": 0, // initial lane, index to r_lanes
            "theta": 0.0 // in radian
        }
    }
}
```
# Demo
``` $ cargo run -- <path_to_json> ``` for running one configuration
# Todo
1. Generate large random testcase -> Update too small
2. Draw roundabout, cars (with action)