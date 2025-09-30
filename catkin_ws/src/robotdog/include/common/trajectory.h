#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "common/waypoint.h"
#include <vector>

class Trajectory {
public:
    void addWaypoint(const Waypoint& wp) {
        waypoints.push_back(wp);
    }

    void addWaypoint(Waypoint&& wp) {
        waypoints.push_back(std::move(wp));
    }

    size_t size() const {
        return waypoints.size();
    }

    const Waypoint& operator[](size_t idx) const {
        return waypoints[idx];
    }

    Waypoint& operator[](size_t idx) {
        return waypoints[idx];
    }

private:
    std::vector<Waypoint> waypoints;
};

#endif //TRAJECTORY_H