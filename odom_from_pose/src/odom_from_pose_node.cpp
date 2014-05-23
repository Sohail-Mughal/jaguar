
#include <ros/ros.h>
#include <tf/transform_datatypes.h> // for transorming quaternion to fixed axis angles (roll/pich/yaw)
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> // msg to be read from SLAM (hector_slam)
#include <geometry_msgs/Vector3.h>




class OdometryFromPose
{

public:
  OdometryFromPose()
  {
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_testing", 50);
    odom_unfiltered_pub_ = nh.advertise<nav_msgs::Odometry>("odom_testing_unfiltered", 50);
    rpy_pub_ = nh.advertise<geometry_msgs::Vector3>("rpy_angles", 50);
    pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("poseupdate",50,
                                                                       &OdometryFromPose::poseCallback,this);
    last_time_ = ros::Time::now();

    previous_x_ = 0;
    previous_y_ = 0;
    previous_z_= 0;
    previous_yaw_ = 0;


    // alpha beta filter initialization

    alpha_ = 0.8;
    beta_ = 0.4;

    x_abFilter_ = 0 ; y_abFilter_ = 0 ; yaw_abFilter_ = 0;
    vx_abFilter_ = 0; vy_abFilter_ = 0; vyaw_abFilter_= 0;


  }

private:
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  ros::NodeHandle nh;
  ros::Publisher odom_pub_, odom_unfiltered_pub_, rpy_pub_;
  ros::Subscriber pose_sub_;
  ros::Time last_time_;
  double previous_x_, previous_y_, previous_z_, previous_yaw_ ;
  double alpha_, beta_, x_abFilter_, y_abFilter_ , vx_abFilter_, vy_abFilter_ , yaw_abFilter_ , vyaw_abFilter_;
};


void OdometryFromPose::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

  //Variables

  nav_msgs::Odometry odom;
  tf::Quaternion quat;
  geometry_msgs::Vector3 rpy;
  ros::Time current_time;

  double dt; // time interval since last loop
  double roll, pitch, yaw_raw; // angles
  double  x_raw, y_raw , x_predicted, y_predicted, yaw_predicted, residual_x , residual_y , residual_yaw;
  double  vx_raw, vy_raw, v , VelocityYaw_raw;


  // transform quaternion to fixed axis angles

  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
  tf::Matrix3x3 m(quat);  // transform this quaternion to a rotation matrix
  m.getRPY(roll, pitch, yaw_raw); //use the accessor getRPY on this matrix.


  // Store and publih the raw/unfiltered angles (pub for testing reasons)
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw_raw;
  rpy_pub_.publish(rpy);

  //yaw_raw = fabs(yaw_raw); creates problems // for avoiding in anglular vel calculation the jump from 3.12 to -3.12 rad/s



  // Calculate raw/unfiltered position and linear velocity

  current_time = msg->header.stamp; // the time that the pose msg was published
  dt = current_time.toSec() - last_time_.toSec();
  x_raw = msg->pose.pose.position.x;
  y_raw = msg->pose.pose.position.y;
  vx_raw = (x_raw - previous_x_) / dt; // velocity x component
  vy_raw = (y_raw - previous_y_) / dt; // velocity y component


  // apply alpha beta filter

  x_predicted = x_abFilter_ + (dt * vx_abFilter_) ;
  residual_x = x_raw - x_predicted;

  x_abFilter_ = x_predicted + (alpha_ * residual_x) ; // the filter calculated values
  vx_abFilter_ += ( beta_ * residual_x ) / dt ;


  y_predicted = y_abFilter_ + (dt * vy_abFilter_) ;
  residual_y = y_raw - y_predicted;

  y_abFilter_ = y_predicted + (alpha_ * residual_y) ; // the filter calculated values
  vy_abFilter_ += ( beta_ * residual_y ) / dt ;

  v = sqrt(vx_abFilter_*vx_abFilter_ + vy_abFilter_*vy_abFilter_); // velocity magnitude v = (vx,vy) = sqrt( vx^2 + vy^2)

  yaw_predicted = yaw_abFilter_ + (dt * vyaw_abFilter_);
  residual_yaw = yaw_raw - yaw_predicted;

  yaw_abFilter_ = yaw_predicted + (alpha_ * residual_yaw);
  vyaw_abFilter_ += ( beta_ * residual_yaw ) / dt ;




  //curr_yaw = tf::getYaw(msg->pose.pose.orientation);
  VelocityYaw_raw = (yaw_raw - previous_yaw_)/ dt;

  // Store the current values as previous for the next loop
  last_time_ = current_time;
  previous_x_ = msg->pose.pose.position.x;
  previous_y_ = msg->pose.pose.position.y;
  previous_z_ = msg->pose.pose.position.z;
  previous_yaw_ = yaw_raw;




  // publish the ros::odometry msg

  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "map"; // ref to pose
  odom.child_frame_id = "base_link"; // ref to twist msg

  odom.pose.pose.position.x = x_abFilter_;
  odom.pose.pose.position.y = y_abFilter_;
  odom.pose.pose.position.z = msg->pose.pose.position.z; // since we are 2D we are not caring about z

  odom.twist.twist.linear.x = v;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;

  odom.twist.twist.angular.z = vyaw_abFilter_;

  odom_pub_.publish(odom);


  // publish the unfiltered msg for testing
  v = sqrt(vx_raw*vx_raw + vy_raw * vy_raw);
  odom.pose.pose = msg->pose.pose;
  odom.twist.twist.linear.x = v;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;

  odom.twist.twist.angular.z = VelocityYaw_raw;

  odom_unfiltered_pub_.publish(odom);

}



int main(int argc, char** argv)
{

  ros::init(argc, argv, "odom_from_pose_node");
  OdometryFromPose odometry_obj;

  ros::spin();
  return(0);
}
