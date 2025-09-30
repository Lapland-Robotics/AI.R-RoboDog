#ifndef WAYPOINT_H
#define WAYPOINT_H

#include "common/mathTypes.h"
#include "common/enumClass.h"
#include "utils/kinematic.h"

class Waypoint {
    public:
        // Construct waypoint from leg poses in joint space
        Waypoint(Vec12 targetQ, float t)
            : _targetPose(targetQ), _travelTime(t) {}

        Waypoint(Vec34 targetFeetPose, float t)
            : _travelTime(t) {
            _targetPose = FeetPoses2Q(targetFeetPose);
        }

        Vec12 getTargetQ() const { return _targetPose; }
        float getMotorTargetQ(int motorID) const { return _targetPose(motorID); }
        float getTravelTime() const { return _travelTime; }

    private:
        Vec12 _targetPose;
        float _travelTime; // seconds
};

#endif // WAYPOINT_H