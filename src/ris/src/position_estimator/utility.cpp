#include "utility.h"
#include <cmath>

namespace position_estimator {

DeadReckoning::DeadReckoning() // Initial values
    : velocity_x_(0.0), velocity_y_(0.0) {
    pose_.x = 0.0;
    pose_.y = 0.0;
    pose_.theta = 0.0;
}

void DeadReckoning::update(double& linear_accel_x, double& linear_accel_y, double angular_velocity_z, double dt) {
    velocity_x_ += linear_accel_x * dt;̈́
    velocity_y_ += linear_accel_y * dt;
    pose_.theta += angular_velocity_z * dt;

    pose_.x += velocity_x_ * dt;
    pose_.y += velocity_y_ * dt;
}


Pose2D DeadReckoning::getPose() const {
      return pose_;
}

}