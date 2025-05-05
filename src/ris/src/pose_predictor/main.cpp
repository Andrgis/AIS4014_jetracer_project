#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "KalmanFilter.h"
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

class OdomFuseNode
{
public:
  OdomFuseNode()
    : nh_(),
      odom_sub_(nh_, "/odom_raw", 1),
      imu_sub_(nh_, "/imu", 1),
      sync_(odom_sub_, imu_sub_, 10),
      initialized_(false)
  {
    // Set up your matrices
    const int n = 8, m = 8;
    A_.resize(n,n);
    C_.resize(m,n);
    C_.setZero();
    for(int i=0;i<8;++i) C_(i,i) = 1.0;
    Q_ = Eigen::MatrixXd::Identity(n,n)*0.01;
    R_ = Eigen::MatrixXd::Identity(m,m)*0.1;
    P_ = Eigen::MatrixXd::Identity(n,n)*1.0;

    // dummy dt; will be overwritten on first update
    kf_ = std::make_unique<KalmanFilter>(0.01, A_, C_, Q_, R_, P_);

    sync_.registerCallback(boost::bind(&OdomFuseNode::callback, this, _1, _2));
    fused_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom_kalman", 10);
  }

private:
  void callback(const nav_msgs::OdometryConstPtr& odom,
                const sensor_msgs::ImuConstPtr& imu)
  {

    // 2) Compute dt
    ros::Time now = odom->header.stamp;
    double dt;
    if (!initialized_) {
      last_time_ = now;
      // initialize state vector [pos, vel, acc]
      Eigen::VectorXd x0(8);
      x0 << odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->twist.twist.linear.x,
            odom->twist.twist.linear.y,
            imu->linear_acceleration.x,
            imu->linear_acceleration.y,
            tf::getYaw(odom->pose.pose.orientation),
            imu->angular_velocity.z;
      kf_->init(now.toSec(), x0);
      initialized_ = true;
      return;
    } else {
      dt = (now - last_time_).toSec();
      last_time_ = now;
    }

    // 3) Build A matrix for this dt
    // Inside your per‐step update, after computing dt:
    A_.setIdentity();

    // position from velocity and accel
    A_(0,2) = dt;     A_(1,3) = dt;
    A_(0,4) = 0.5*dt*dt;  A_(1,5) = 0.5*dt*dt;
    A_(2,4) = dt;     A_(3,5) = dt;

    // yaw from angular rate
    A_(6,7) = dt;

    // 7) extract yaw from odom quaternion:
    geometry_msgs::Quaternion q_msg = odom->pose.pose.orientation;
    tf::Quaternion   q_tf(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll,pitch,yaw;
    tf::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);

    // 8) get angular rate from IMU:
    double wz = imu->angular_velocity.z;

    // 9) form the 8×1 measurement vector
    Eigen::VectorXd y(8);
    y <<
      odom->pose.pose.position.x,
      odom->pose.pose.position.y,
      odom->twist.twist.linear.x,
      odom->twist.twist.linear.y,
      imu->linear_acceleration.x,
      imu->linear_acceleration.y,
      yaw,
      wz;

    // 5) Update filter
    kf_->update(y, dt, A_);
    Eigen::VectorXd x_hat = kf_->state();

    // 6) Publish fused odometry
    nav_msgs::Odometry fused;
    fused.header.stamp    = now;
    fused.header.frame_id = "odom";
    fused.child_frame_id  = odom->child_frame_id;

    fused.pose.pose.position.x = x_hat(0);
    fused.pose.pose.position.y = x_hat(1);

    // orientation: keep yaw from raw odom (or compute from IMU if you wish)
    fused.pose.pose.orientation = odom->pose.pose.orientation;

    fused.twist.twist.linear.x  = x_hat(2);
    fused.twist.twist.linear.y  = x_hat(3);

    fused.pose.pose.orientation = tf::createQuaternionMsgFromYaw(x_hat(6));
    fused.twist.twist.angular.z   = x_hat(7);

    fused_pub_.publish(fused);



  }

  ros::NodeHandle nh_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::Imu>      imu_sub_;
  message_filters::TimeSynchronizer<
      nav_msgs::Odometry, sensor_msgs::Imu>           sync_;

  ros::Publisher fused_pub_;
  std::unique_ptr<KalmanFilter> kf_;

  Eigen::MatrixXd A_, C_, Q_, R_, P_;
  ros::Time last_time_;
  bool initialized_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekf_fuse_node");
  OdomFuseNode node;
  ros::spin();
  return 0;
}
