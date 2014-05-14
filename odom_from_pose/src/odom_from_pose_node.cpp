
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
    rpy_pub_ = nh.advertise<geometry_msgs::Vector3>("rpy_angles", 50);
    pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("poseupdate",50,
                                                                       &OdometryFromPose::poseCallback,this);
    last_time_ = ros::Time::now();

    previous_x_ = 0;
    previous_y_ = 0;
    previous_z_= 0;
    previous_yaw_ = 0;
  }

private:
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  ros::NodeHandle nh;
  ros::Publisher odom_pub_, rpy_pub_;
  ros::Subscriber pose_sub_;
  ros::Time last_time_;
  double previous_x_, previous_y_, previous_z_, previous_yaw_ ;
};


void OdometryFromPose::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  nav_msgs::Odometry odom;
  tf::Quaternion quat;
  geometry_msgs::Vector3 rpy;
  ros::Time current_time;

  double roll, pitch, yaw, dt;


  // transform quaternion to fixed axis angles

  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);

  // First we have to turn this quaternion to a rotation matrix and then use the accessor getRPY on this matrix.
  tf::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);

  // Store and publih the angles
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;
  rpy_pub_.publish(rpy);

  //set the position to odometry msg
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "map";
  odom.pose.pose = msg->pose.pose;


  // calculate veocity by calcualting derivatives
  current_time = ros::Time::now();
  dt = current_time.toSec() - last_time_.toSec();

  //odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = (msg->pose.pose.position.x - previous_x_) / dt;
  odom.twist.twist.linear.y = (msg->pose.pose.position.y - previous_y_) / dt;
  odom.twist.twist.linear.z = (msg->pose.pose.position.z - previous_z_) / dt;

  //curr_yaw = tf::getYaw(msg->pose.pose.orientation);
  odom.twist.twist.angular.z = (yaw - previous_yaw_)/ dt;



  odom_pub_.publish(odom); // publish the complete odometry msg


  // Store the current values as previous for the next loop
  last_time_ = current_time;
  previous_x_ = msg->pose.pose.position.x;
  previous_y_ = msg->pose.pose.position.y;
  previous_z_ = msg->pose.pose.position.z;
  previous_yaw_ = yaw;

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_from_pose_node");
  OdometryFromPose odometry_obj;
  ROS_INFO("odometry_from_pose_node initialised");
  ros::Rate r(10); // 10 hz

  while (ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }


}
