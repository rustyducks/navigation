#ifndef PARAMETERS_H
#define PARAMETERS_H

#define IS_HOLONOMIC false
#define MAX_LINEAR_ACCELERATION 50.         // mm/s²
#define MAX_LINEAR_SPEED 200.               // mm/s
#define MAX_ROTATIONAL_ACCELERATION 0.5     // rad/s²
#define MAX_ROTATIONAL_SPEED 1.8            // rad/s
#define ADMITTED_LINEAR_POSITION_ERROR 5.   // mm
#define ADMITTED_ANGLE_POSITION_ERROR 0.05  // rad

#define LINEAR_CONTROL_STOP_DISTANCE_FACTOR 3

#define PURE_PURSUIT_LOOKAHEAD_DISTANCE 50.  // mm

#endif /* PARAMETERS_H */