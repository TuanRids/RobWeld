#pragma once
#include "YMConnect/YMConnect.h"
struct Motion
{
    ControlGroupId groupId{};
    DOUBLE64 speed{}; //Speed of the motion.  This value is a percentage (0.00 - 100.00) for JointMotion. For LinearMotion and CircularMotion, the value is in mm/sec (0.0 - 1500.0)
    MotionAccelDecel attributes{}; //The acceleration and deceleration ratios for the motion.
    MoveInterpolationType interpolationType{ MoveInterpolationType::NoneSelected }; //The interpolation type of the motion.
    PositionData position{}; //The position data of the motion.

    Motion() = default;

    Motion(ControlGroupId groupId, const PositionData& destinationCoords, DOUBLE64 speed, const MotionAccelDecel& attributes = MotionAccelDecel())
        : groupId(groupId),
        speed(speed),
        attributes(attributes),
        position(destinationCoords)
    {}
};
