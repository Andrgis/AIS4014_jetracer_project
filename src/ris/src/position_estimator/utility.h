//
// Created by jetson on 08/04/25.
//

#ifndef UTILITY_H
#define UTILITY_H

#include <geometry_msgs/Vector3.h>

namespace position_estimator{

struct Pose2D {
    double x;
    double y;
    double theta;
};

class DeadReckoning {
public:
    DeadReckoning();

    void update(double& linear_accel_x, double& linear_accel_y, double angular_velocity_z, double dt);

    Pose2D getPose() const;

private:
    Pose2D pose_;
    double velocity_x_;
    double velocity_y_;
};

}  // namespace jetracer_nav

#endif //UTILITY_H
