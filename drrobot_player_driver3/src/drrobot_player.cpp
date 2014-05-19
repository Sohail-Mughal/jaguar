/*!
 *  drrobot_player_driver3
 *  Copyright (c) 2011, Dr Robot Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!

@mainpage
  drrobot_player_driver3 is a driver for motion control system on I90/Sentinel3/Hawk/H20/X80SV/Jaguar series mobile robot, available from
<a href="http://www.drrobot.com">Dr Robot </a>.
<hr>

@section usage Usage
@par     After start roscore, you need load robot configuration file to parameter server first.
          For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
@verbatim
$ drrobot_player_driver3
@endverbatim

<hr>
@section topic ROS topics

Subscribes to (name/type):
- @b "cmd_vel"/Twist : velocity commands to differentially drive the robot.
- @b will develop other command subscribles in future, such as servo control.

Publishes to (name / type):
-@b drrobot_motor: will publish MotionInfoArray Message. Please referee the message file.
-@b drrobot_powerinfo: will publish PowerInfo Message. Please referee the message file.
-@b drrobot_ir: will publish RangeArray Message for IR sensor, and transform AD value from DrRobotMotionSensorDriver to distance value in unit meter. Please referee the message file.
-@b drrobot_sonar: will publish RangeArray Message for ultrasonic sensor, and transform value from DrRobotMotionSensorDriver to distance value in unit meter. Please referee the message file.
-@b drrobot_standardsensor: will publish StandardardSensor Message. Please referee the message file.
-@b drrobot_customsensor: will publish CustomSensor Message. Please referee the message file. Not available for standard I90/Sentinel3/Hawk/H20/X80SV robot

<hr>

@section parameters ROS parameters, please read yaml file

- @b RobotCommMethod (string) : Robot communication method, normally is "Network".
- @b RobotID (string) : specify the robot ID
- @b RobotBaseIP (string) : robot main WiFi module IP address in dot format, default is "192.168.0.201".
- @b RobotPortNum (string) : socket port number first serial port, and as default the value increased by one will be second port number.
- @b RobotSerialPort (int) : specify the serial port name if you choose serial communication in RobotCommMethod, default /dev/ttyS0"
- @b RobotType (string) : specify the robot type, now should in list: I90, Sentinel3, Hawk_H20, Jaguar, X80SV
- @b MotorDir (int) : specify the motor control direction
- @b WheelRadius (double) : wheel radius
- @b WheelDistance (double) : the distance between two driving wheels
- @b EncoderCircleCnt (int) : one circle encoder count
- @b MinSpeed (double) : minimum speed, unit is m/s.
- @b MaxSpeed (double) : maximum speed, unit is m/s.
- @b enable_ir (bool)  : Whether to enable sonar range sensors. Default: true.
- @b enable_sonar (bool)  : Whether to enable IR range sensors. Default: true.
 */

#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <drrobot_player_driver3/MotorInfo.h>
#include <drrobot_player_driver3/MotorInfoArray.h>
#include <drrobot_player_driver3/RangeArray.h>
#include <drrobot_player_driver3/Range.h>
#include <drrobot_player_driver3/PowerInfo.h>
#include <drrobot_player_driver3/StandardSensor.h>
#include <drrobot_player_driver3/CustomSensor.h>

#include <DrRobotMotionSensorDriver.hpp>

#define MOTOR_NUM       6
#define IR_NUM          10
#define US_NUM          6
#define WHEEL_BASE_CM   40.0 
#define TICKS_PER_CM    3.45  //3.45 based upon the diameter of the wheel including the track, 4.52 based upon the diamater of the wheel excluding the track. 3.45 works best inside the lab, 4.52 works best on the carpet outside
#define PI              3.14159265359
#define ENCODER_MIN     0
#define ENCODER_MAX     32767
#define ENCODER_THRESHOLD        100


using namespace std;
using namespace DrRobot_MotionSensorDriver;

class DrRobotPlayerNode
{
public:

    ros::NodeHandle node_;

    tf::TransformBroadcaster tf_broad;

    ros::Publisher motorInfo_pub;
    ros::Publisher powerInfo_pub;
    ros::Publisher ir_pub;
    ros::Publisher sonar_pub;
    ros::Publisher standardSensor_pub;
    ros::Publisher customSensor_pub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;

    geometry_msgs::Quaternion odom_quat;
    long prev_encoder_left, prev_encoder_right, current_encoder_left, current_encoder_right;
    long delta_encoder_left, delta_encoder_right;
    double left_cm, right_cm;
    ros::Time current_time, last_time;
    double delta_time, distance_cm, x_pos, y_pos, theta_pos;
    int first_run;

    ros::Subscriber cmd_vel_sub_;
    std::string robot_prefix_;

    DrRobotPlayerNode()
    {
	//NodeHandle getParam() currently does not work
        ros::NodeHandle private_nh("~");

        robotID_ = "drobot1";
        private_nh.getParam("RobotID",robotID_);
        ROS_INFO("I get ROBOT_ID: [%s]", robotID_.c_str());

        robotType_ = "Jaguar";
        private_nh.getParam("RobotType",robotType_);
        ROS_INFO("I get ROBOT_Type: [%s]", robotType_.c_str());

        robotCommMethod_ = "Network";
        private_nh.getParam("RobotCommMethod",robotCommMethod_);
        ROS_INFO("I get ROBOT_CommMethod: [%s]", robotCommMethod_.c_str());

        robotIP_ = "192.168.0.60";
        private_nh.getParam("RobotBaseIP",robotIP_);
        ROS_INFO("I get ROBOT_IP: [%s]", robotIP_.c_str());

        commPortNum_ = 10001;
        private_nh.getParam("RobotPortNum",commPortNum_);
        ROS_INFO("I get ROBOT_PortNum: [%d]", commPortNum_);

        robotSerialPort_ = "/dev/ttyS0";
        private_nh.getParam("RobotSerialPort",robotSerialPort_);
        ROS_INFO("I get ROBOT_SerialPort: [%s]", robotSerialPort_.c_str());

        enable_ir_ = false;
        private_nh.getParam("Enable_IR", enable_ir_);
        if (enable_ir_)
          ROS_INFO("I get Enable_IR: true");
        else
          ROS_INFO("I get Enable_IR: false");


        enable_sonar_ = false;
        private_nh.getParam("Enable_US", enable_sonar_);
        if (enable_sonar_)
          ROS_INFO("I get Enable_US: true");
        else
          ROS_INFO("I get Enable_US: false");

        motorDir_ = 1;
        private_nh.getParam("MotorDir", motorDir_);
        ROS_INFO("I get MotorDir: [%d]", motorDir_);

        wheelRadius_ = 0.0835;
        private_nh.getParam("WheelRadius", wheelRadius_);
        ROS_INFO("I get Wheel Radius: [%f]", wheelRadius_);

        wheelDis_ = 0.305;
        private_nh.getParam("WheelDistance", wheelDis_);
        ROS_INFO("I get Wheel Distance: [%f]", wheelDis_);

        minSpeed_ = 0.1;
        private_nh.getParam("MinSpeed", minSpeed_);
        ROS_INFO("I get Min Speed: [%f]", minSpeed_);

        maxSpeed_ = 1.0;
        private_nh.getParam("MaxSpeed", maxSpeed_);
        ROS_INFO("I get Max Speed: [%f]", maxSpeed_);

        encoderOneCircleCnt_ = 800;
        private_nh.getParam("EncoderCircleCnt", encoderOneCircleCnt_);
        ROS_INFO("I get Encoder One Circle Count: [%d]", encoderOneCircleCnt_);

        if (robotCommMethod_ == "Network")
        {
          robotConfig1_.commMethod = Network;
          robotConfig2_.commMethod = Network;
        }
        else
        {
          robotConfig1_.commMethod = Serial;
          robotConfig2_.commMethod = Serial;
        }

        if (robotType_ == "Jaguar")
        {
          robotConfig1_.boardType = Jaguar;
        }
        else if(robotType_ == "I90")
        {
          robotConfig1_.boardType = I90_Power;
          robotConfig2_.boardType = I90_Motion;
        }
        else if (robotType_ == "Sentinel3")
        {
          robotConfig1_.boardType = Sentinel3_Power;
          robotConfig2_.boardType = Sentinel3_Motion;
        }
        else if (robotType_ == "Hawk_H20")
        {
          robotConfig1_.boardType = Hawk_H20_Power;
          robotConfig2_.boardType = Hawk_H20_Motion;
        }
        else if(robotType_ == "X80SV")
        {
          robotConfig1_.boardType = X80SV;
        }

        robotConfig1_.portNum = commPortNum_;
        robotConfig2_.portNum = commPortNum_ + 1;

      //  strcat(robotConfig1_.robotIP,robotIP_.c_str());
	  strcpy(robotConfig1_.robotIP,robotIP_.c_str());
      //  strcat(robotConfig2_.robotIP,robotIP_.c_str());
	  strcpy(robotConfig2_.robotIP,robotIP_.c_str());

      //  strcat(robotConfig1_.serialPortName,robotSerialPort_.c_str());
	  strcpy(robotConfig1_.serialPortName,robotSerialPort_.c_str());
      //  strcat(robotConfig2_.serialPortName,robotSerialPort_.c_str());
	  strcpy(robotConfig2_.serialPortName,robotSerialPort_.c_str());
        //create publishers for sensor data information
        motorInfo_pub = node_.advertise<drrobot_player_driver3::MotorInfoArray>("drrobot_motor", 1);
        powerInfo_pub = node_.advertise<drrobot_player_driver3::PowerInfo>("drrobot_powerinfo", 1);
        if (enable_ir_) { ir_pub = node_.advertise<drrobot_player_driver3::RangeArray>("drrobot_ir", 1); }
        if (enable_sonar_) { sonar_pub = node_.advertise<drrobot_player_driver3::RangeArray>("drrobot_sonar",1); }
        standardSensor_pub = node_.advertise<drrobot_player_driver3::StandardSensor>("drrobot_standardsensor", 1);
        customSensor_pub = node_.advertise<drrobot_player_driver3::CustomSensor>("drrobot_customsensor", 1);
        odom_pub = node_.advertise<nav_msgs::Odometry>("odom", 50);
        
        
 
        current_time = ros::Time::now();
        last_time = ros::Time::now();

        drrobotPowerDriver_ = new DrRobotMotionSensorDriver();
        drrobotMotionDriver_ = new DrRobotMotionSensorDriver();
        if (  (robotType_ == "Jaguar") )
        {
          drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
        }
        else
        {
          drrobotPowerDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
          drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig2_);
        }
        cntNum_ = 0;
        left_cm = 0.0;
        right_cm = 0.0;
        x_pos = 0.0;
        y_pos = 0.0;
        theta_pos = 0.0;
        first_run = 0;

    }

    ~DrRobotPlayerNode()
    {
    }

    int start()
    {

      int res = -1;
      if (  (robotType_ == "Jaguar"))
      {
        res = drrobotMotionDriver_->openNetwork(robotConfig1_.robotIP,robotConfig1_.portNum);
	if (res == 0)
	{
		ROS_INFO("open port number at: [%d]", robotConfig1_.portNum);
	}
	else
	{
		ROS_INFO("could not open network connection to [%s,%d]",  robotConfig1_.robotIP,robotConfig1_.portNum);
		ROS_INFO("error code [%d]",  res);
	}

      }
      else
      {
        drrobotMotionDriver_->openNetwork(robotConfig2_.robotIP,robotConfig2_.portNum);
        drrobotPowerDriver_->openNetwork(robotConfig1_.robotIP,robotConfig1_.portNum);

      }

      cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("drrobot_cmd_vel", 1, boost::bind(&DrRobotPlayerNode::cmdVelReceived, this, _1));
        

      return(0);
    }

    int stop()
    {
        int status = 0;
         drrobotMotionDriver_->close();
        drrobotPowerDriver_->close();
        usleep(1000000);
        return(status);
    }

    void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
      double g_vel = cmd_vel->linear.x;
      double t_vel = cmd_vel->angular.z;
      if (robotConfig1_.boardType != Jaguar)
      {
        double leftWheel = (2 * g_vel - t_vel* wheelDis_) / (2 * wheelRadius_);
        double rightWheel = (t_vel* wheelDis_ + 2 * g_vel) / (2 * wheelRadius_);

        int leftWheelCmd = motorDir_ * leftWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
        int rightWheelCmd = - motorDir_ * rightWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
        ROS_INFO("Received control command: [%d, %d]", leftWheelCmd,rightWheelCmd);
        drrobotMotionDriver_->sendMotorCtrlAllCmd(Velocity,leftWheelCmd, rightWheelCmd,NOCONTROL,NOCONTROL, NOCONTROL,NOCONTROL);
      }
      else
      {
         int forwardPWM = -motorDir_ * g_vel * 16384 + 16384;
         int turnPWM = -motorDir_ * t_vel * 16384 + 16384;
         if (forwardPWM > 32767) forwardPWM = 32767;
         if (forwardPWM < 0) forwardPWM = 0;
         if (turnPWM > 32767) turnPWM = 32767;
         if (turnPWM < 0) turnPWM = 0;
         //ROS_INFO("Received control command: [%d, %d]", forwardPWM,turnPWM);
         drrobotMotionDriver_->sendMotorCtrlAllCmd(PWM,NOCONTROL,NOCONTROL,NOCONTROL,forwardPWM,turnPWM, NOCONTROL);
      }

    }

    void doUpdate()
    {
        current_time = ros::Time::now();
        delta_time = (current_time - last_time).toSec();
        /*//start tf transform
        double deltax = 0.1; //10cm
        double deltay = 0.0; //0cm
        double deltaz = 0.2; //20cm	
        tf_broad.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(deltax, deltay, deltaz)), ros::Time::now(),"base_link", "base_laser"));*/
      


      if ( (robotConfig1_.boardType == I90_Power) || (robotConfig1_.boardType == Sentinel3_Power)
          || (robotConfig1_.boardType == Hawk_H20_Power) )
      {
        if (drrobotPowerDriver_->portOpen())
        {
          drrobotPowerDriver_->readPowerSensorData(&powerSensorData_);
          drrobot_player_driver3::PowerInfo powerInfo;
          powerInfo.ref_vol = 1.5 * 4095 /(double)powerSensorData_.refVol;

          powerInfo.bat1_vol = (double)powerSensorData_.battery1Vol  * 8 / 4095 * powerInfo.ref_vol;
          powerInfo.bat2_vol = (double) powerSensorData_.battery2Vol * 8 / 4095 * powerInfo.ref_vol;

          powerInfo.bat1_temp = powerSensorData_.battery1Thermo;
          powerInfo.bat2_temp = powerSensorData_.battery2Thermo;

          powerInfo.dcin_vol = (double)powerSensorData_.dcINVol * 8 / 4095 * powerInfo.ref_vol;
          powerInfo.charge_path = powerSensorData_.powerChargePath;
          powerInfo.power_path = powerSensorData_.powerPath;
          powerInfo.power_status = powerSensorData_.powerStatus;

          powerInfo_pub.publish(powerInfo);
        }
      }
      if (drrobotMotionDriver_->portOpen())
      {
        drrobotMotionDriver_->readMotorSensorData(&motorSensorData_);
        drrobotMotionDriver_->readRangeSensorData(&rangeSensorData_);
        drrobotMotionDriver_->readStandardSensorData(&standardSensorData_);

        drrobotMotionDriver_->readCustomSensorData(&customSensorData_);
              // Translate from driver data to ROS data
            cntNum_++;
              drrobot_player_driver3::MotorInfoArray motorInfoArray;
              
              motorInfoArray.motorInfos.resize(MOTOR_NUM);
              for (uint32_t i = 0 ; i < MOTOR_NUM; ++i)
              {
                  motorInfoArray.motorInfos[i].header.stamp = ros::Time::now();
                  motorInfoArray.motorInfos[i].header.frame_id = string("drrobot_motor_");
                  motorInfoArray.motorInfos[i].header.frame_id += boost::lexical_cast<std::string>(i);
                  motorInfoArray.motorInfos[i].robot_type = robotConfig1_.boardType;
                  motorInfoArray.motorInfos[i].encoder_pos = motorSensorData_.motorSensorEncoderPos[i];
                  motorInfoArray.motorInfos[i].encoder_vel = motorSensorData_.motorSensorEncoderVel[i];
                  motorInfoArray.motorInfos[i].encoder_dir = motorSensorData_.motorSensorEncoderDir[i];
                  if (robotConfig1_.boardType == Hawk_H20_Motion)
                  {
                    motorInfoArray.motorInfos[i].motor_current = (float)motorSensorData_.motorSensorCurrent[i] * 3 /4096;;
                  }
                  else if(robotConfig1_.boardType != Jaguar)
                  {
                    motorInfoArray.motorInfos[i].motor_current = (float)motorSensorData_.motorSensorCurrent[i] / 728;
                  }
                  else
                  {
                    motorInfoArray.motorInfos[i].motor_current = 0.0;
                  }
                  motorInfoArray.motorInfos[i].motor_pwm = motorSensorData_.motorSensorPWM[i];
              }

              //ROS_INFO("publish motor info array");
              motorInfo_pub.publish(motorInfoArray);

	      //PUBLISH ODOM DATA HERE
	      //motors used are 3 & 4
              //3 is left motor
              //4 is right motor
              
              current_encoder_left = motorInfoArray.motorInfos[3].encoder_pos;
              current_encoder_right = motorInfoArray.motorInfos[4].encoder_pos;
              if (first_run < 10) {
                 first_run += 1;
                 ROS_INFO("FIRST RUN");
                 prev_encoder_left = motorInfoArray.motorInfos[3].encoder_pos;
                 prev_encoder_right = motorInfoArray.motorInfos[4].encoder_pos;
                 current_encoder_left = prev_encoder_left;
                 current_encoder_right = prev_encoder_right;
              }




              if (prev_encoder_left <= ENCODER_MAX && prev_encoder_left >= (ENCODER_MAX - ENCODER_THRESHOLD) && current_encoder_left <= (ENCODER_MIN + ENCODER_THRESHOLD)) {
                 //encoder has gone from high to low
                 delta_encoder_left = (ENCODER_MAX - prev_encoder_left) + current_encoder_left;
              } else if (prev_encoder_left >= ENCODER_MIN && prev_encoder_left <= (ENCODER_MIN + ENCODER_THRESHOLD) && current_encoder_left >= (ENCODER_MAX - ENCODER_THRESHOLD)) {
                 //encoder has gone from low to high
                 delta_encoder_left = (ENCODER_MAX - current_encoder_left) - prev_encoder_left;
              } else {
                 delta_encoder_left = current_encoder_left - prev_encoder_left;
              }

              if (prev_encoder_right <= ENCODER_MAX && prev_encoder_right >= (ENCODER_MAX - ENCODER_THRESHOLD) && current_encoder_right <= (ENCODER_MIN + ENCODER_THRESHOLD)) {
                 //encoder has gone from high to low
                 delta_encoder_right = (ENCODER_MAX - prev_encoder_right) + current_encoder_right;
              } else if (prev_encoder_right >= ENCODER_MIN && prev_encoder_right <= (ENCODER_MIN + ENCODER_THRESHOLD) && current_encoder_right >= (ENCODER_MAX - ENCODER_THRESHOLD)) {
                 //encoder has gone from low to high
                 delta_encoder_right = (ENCODER_MAX - current_encoder_right) - prev_encoder_right;
              } else {
                 delta_encoder_right = prev_encoder_right - current_encoder_right;
              }

              /*ROS_INFO("current_encoder_left: [%ld]", current_encoder_left);
              ROS_INFO("current_encoder_right: [%ld]", current_encoder_right);
              ROS_INFO("prev_encoder_left: [%ld]", prev_encoder_left);
              ROS_INFO("prev_encoder_right: [%ld]", prev_encoder_right);
              ROS_INFO("delta_encoder_left: [%ld]", delta_encoder_left);
              ROS_INFO("delta_encoder_right: [%ld]", delta_encoder_right);*/

              right_cm = delta_encoder_right/TICKS_PER_CM;
              left_cm = delta_encoder_left/TICKS_PER_CM;

              distance_cm = (left_cm + right_cm)/2.0;

              ROS_INFO("distance: %f", distance_cm);

              theta_pos -= ((left_cm - right_cm)/WHEEL_BASE_CM)/2.54;// * (180/PI))/2.54;

              x_pos += (distance_cm * cos(theta_pos));
              y_pos += (distance_cm * sin(theta_pos));
              
              ROS_INFO("x_pos: %f", x_pos);
              ROS_INFO("y_pos: %f", y_pos);
              ROS_INFO("theta_pos: %f", theta_pos);


              prev_encoder_left = current_encoder_left;
              prev_encoder_right = current_encoder_right;

              geometry_msgs::TransformStamped odom_trans;
              
              odom_trans.header.stamp = current_time;
              odom_trans.header.frame_id = "odom";
              odom_trans.child_frame_id = "base_link";
              
              odom_trans.transform.translation.x = x_pos;
              odom_trans.transform.translation.y = y_pos;
              odom_trans.transform.translation.z = 0.0;
              odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta_pos);

              odom_broadcaster.sendTransform(odom_trans);

              nav_msgs::Odometry odom;
              odom.header.stamp = current_time;
              odom.header.frame_id = "odom";
              odom.child_frame_id = "base_link";

              odom.pose.pose.position.x = x_pos;
              odom.pose.pose.position.y = y_pos;
              odom.pose.pose.position.z = 0.0;
              odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_pos);

              odom.twist.twist.linear.x = 0.0;
              odom.twist.twist.linear.y = 0.0;
              odom.twist.twist.angular.z = 0.0;

              odom_pub.publish(odom);


              drrobot_player_driver3::RangeArray rangerArray;
              rangerArray.ranges.resize(US_NUM);
	      if(enable_sonar_)
	      {
		      for (uint32_t i = 0 ; i < US_NUM; ++i)
		      {

		          rangerArray.ranges[i].header.stamp = ros::Time::now();
		          rangerArray.ranges[i].header.frame_id = string("drrobot_sonar_");
		          rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
		          rangerArray.ranges[i].range = (float)rangeSensorData_.usRangeSensor[i]/100;     //to meters

		          // around 30 degrees
		          rangerArray.ranges[i].field_of_view = 0.5236085;
		          rangerArray.ranges[i].max_range = 2.55;
		          rangerArray.ranges[i].min_range = 0;
                  rangerArray.ranges[i].radiation_type = drrobot_player_driver3::Range::ULTRASOUND;
		      }

		      sonar_pub.publish(rangerArray);
		}


	      if(enable_ir_)
	      {
		      rangerArray.ranges.resize(IR_NUM);
		      for (uint32_t i = 0 ; i < IR_NUM; ++i)
		      {
		          rangerArray.ranges[i].header.stamp = ros::Time::now();
		          rangerArray.ranges[i].header.frame_id = string("drrobot_ir_");
		          rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
		          rangerArray.ranges[i].range = ad2Dis(rangeSensorData_.irRangeSensor[i]);
                  rangerArray.ranges[i].radiation_type = drrobot_player_driver3::Range::INFRARED;
		      }

		      ir_pub.publish(rangerArray);
	     }

              drrobot_player_driver3::StandardSensor standardSensor;
              standardSensor.humanSensorData.resize(4);
              standardSensor.tiltingSensorData.resize(2);
              standardSensor.overHeatSensorData.resize(2);
              standardSensor.header.stamp = ros::Time::now();
              standardSensor.header.frame_id = string("drrobot_standardsensor");
              for (uint32_t i = 0; i < 4; i++)
                standardSensor.humanSensorData[i] = standardSensorData_.humanSensorData[i];
              for (uint32_t i = 0; i < 2; i++)
                standardSensor.tiltingSensorData[i] = standardSensorData_.tiltingSensorData[i];
              for (uint32_t i = 0; i < 2; i++)
                standardSensor.overHeatSensorData[i] = standardSensorData_.overHeatSensorData[i];

              standardSensor.thermoSensorData = standardSensorData_.thermoSensorData;

              standardSensor.boardPowerVol = (double)standardSensorData_.boardPowerVol * 9 /4095;
              standardSensor.servoPowerVol = (double)standardSensorData_.servoPowerVol * 9 /4095;

              if (robotConfig1_.boardType != Jaguar)
              {
                standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 24 /4095;
              }
              else
              {
                standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 34.498 /4095;
              }
              standardSensor.refVol = (double)standardSensorData_.refVol / 4095 * 6;
              standardSensor.potVol = (double)standardSensorData_.potVol / 4095 * 6;
              standardSensor_pub.publish(standardSensor);

              drrobot_player_driver3::CustomSensor customSensor;
              customSensor.customADData.resize(8);
              customSensor.header.stamp = ros::Time::now();
              customSensor.header.frame_id = string("drrobot_customsensor");

              for (uint32_t i = 0; i < 8; i ++)
              {
                customSensor.customADData[i] = customSensorData_.customADData[i];
              }
              customSensor.customIO = (uint8_t)(customSensorData_.customIO & 0xff);
              customSensor_pub.publish(customSensor);
              last_time = ros::Time::now();
      }
      
    }

private:

    DrRobotMotionSensorDriver* drrobotMotionDriver_;
    DrRobotMotionSensorDriver* drrobotPowerDriver_;
    struct DrRobotMotionConfig robotConfig1_;
    struct DrRobotMotionConfig robotConfig2_;

    std::string odom_frame_id_;
    struct MotorSensorData motorSensorData_;
    struct RangeSensorData rangeSensorData_;
    struct PowerSensorData powerSensorData_;
    struct StandardSensorData standardSensorData_;
    struct CustomSensorData customSensorData_;


    std::string robotType_;
    std::string robotID_;
    std::string robotIP_;
    std::string robotCommMethod_;
    std::string robotSerialPort_;
    bool enable_ir_;
    bool enable_sonar_;
    int  commPortNum_;
    int  encoderOneCircleCnt_;
    double wheelDis_;
    double wheelRadius_;
    int motorDir_;
    double minSpeed_;
    double maxSpeed_;

    int cntNum_;
    double ad2Dis(int adValue)
    {
      double temp = 0;
      double irad2Dis = 0;

      if (adValue <= 0)
        temp = -1;
      else
        temp = 21.6 /((double)adValue * 3 /4096 - 0.17);

      if ( (temp > 80) || (temp < 0))
      {
        irad2Dis = 0.81;
      }
      else if( (temp < 10) && (temp > 0))
      {
        irad2Dis = 0.09;
      }
      else
        irad2Dis = temp /100;
      return irad2Dis;
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "drrobot_player_driver3");

    DrRobotPlayerNode drrobotPlayer;
    ros::NodeHandle n;
    // Start up the robot
    if (drrobotPlayer.start() != 0)
    {
        exit(-1);
    }
    /////////////////////////////////////////////////////////////////

    ros::Rate loop_rate(5);      

    while (n.ok())
    {
     drrobotPlayer.doUpdate();
     ros::spinOnce();
     loop_rate.sleep();
    }
    /////////////////////////////////////////////////////////////////

    // Stop the robot
    drrobotPlayer.stop();

    return(0);
}

