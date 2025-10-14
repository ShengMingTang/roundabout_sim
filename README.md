# Roundabout Simulation Project
Simulation of cars running on a multi-lane roundabout.
# Configuration
``` json
{
    "n_inter": 2, // number of entrance/exit evenly spaced at a roundabout, 0th is placed at theta 0
    "n_cars": 1, // number of cars

    // radius of lanes
    // index 0 means the outermost one, values must be in decreasing order
    "r_lanes": [1.0],

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