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
 */

#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <drrobot_player_driver3/MotorInfo.h>
#include <drrobot_player_driver3/MotorInfoArray.h>

// these msgs probably do not work?
#include <drrobot_player_driver3/PowerInfo.h>
#include <drrobot_player_driver3/StandardSensor.h>
#include <drrobot_player_driver3/CustomSensor.h>

#include <DrRobotMotionSensorDriver.hpp>

#define MOTOR_NUM       6


using namespace std;
using namespace DrRobot_MotionSensorDriver;

class DrRobotPlayerNode
{

public:

    ros::NodeHandle node_;

    tf::TransformBroadcaster tf_;

    ros::Publisher motorInfo_pub_;
    ros::Publisher powerInfo_pub_;
    ros::Publisher standardSensor_pub_;
    ros::Publisher customSensor_pub_;

    ros::Subscriber cmd_vel_sub_;

    std::string robot_prefix_;

    DrRobotPlayerNode()
    {
        ros::NodeHandle private_nh("drrobot_player_driver3");

        // declaring default robot parameters values ( for Jaguar lite)
        const std::string ROBOT_TYPE = "Jaguar";
        const std::string ROBOT_ID = "DrRobot";
        const std::string ROBOT_IP = "192.168.0.60";
        const std::string ROBOT_COM_METHOD = "Network";
        const std::string ROBOT_SERIAL_PORT = "/dev/ttyS0";
        const int  COM_PORT_NUM = 10001;
        const int  ENCODER_ONE_CIRCLE_CNT = 380;
        const double WHEEL_DISTANCE = 0.45;
        const double WHEEL_RADIUS = 0.0835;
        const int MOTOR_DIRECTION = 1;
        const double MIN_SPEED = 0.1;
        const double MAX_SPEED = 1;


        // getting parameters from ROS param server ( if a param file has been loaded)
        private_nh.param("RobotID", robotID_ , ROBOT_ID);
        ROS_INFO("I get ROBOT_ID: [%s]", robotID_.c_str());

        private_nh.param("RobotType",robotType_, ROBOT_TYPE);
        ROS_INFO("I get ROBOT_Type: [%s]", robotType_.c_str());

        private_nh.param("RobotCommMethod",robotCommMethod_, ROBOT_COM_METHOD);
        ROS_INFO("I get ROBOT_CommMethod: [%s]", robotCommMethod_.c_str());

        private_nh.param("RobotBaseIP",robotIP_, ROBOT_IP);
        ROS_INFO("I get ROBOT_IP: [%s]", robotIP_.c_str());


        private_nh.param("RobotPortNum",commPortNum_, COM_PORT_NUM);
        ROS_INFO("I get ROBOT_PortNum: [%d]", commPortNum_);

        private_nh.param("RobotSerialPort",robotSerialPort_, ROBOT_SERIAL_PORT);
        ROS_INFO("I get ROBOT_SerialPort: [%s]", robotSerialPort_.c_str());

        private_nh.param("MotorDir", motorDir_, MOTOR_DIRECTION);
        ROS_INFO("I get MotorDir: [%d]", motorDir_);


        private_nh.param("WheelRadius", wheelRadius_, WHEEL_RADIUS);
        ROS_INFO("I get Wheel Radius: [%f]", wheelRadius_);

        private_nh.param("WheelDistance", wheelDis_, WHEEL_DISTANCE);
        ROS_INFO("I get Wheel Distance: [%f]", wheelDis_);

        private_nh.param("MinSpeed", minSpeed_, MIN_SPEED);
        ROS_INFO("I get Min Speed: [%f]", minSpeed_);

        private_nh.param("MaxSpeed", maxSpeed_, MAX_SPEED);
        ROS_INFO("I get Max Speed: [%f]", maxSpeed_);

        private_nh.param("EncoderCircleCnt", encoderOneCircleCnt_, ENCODER_ONE_CIRCLE_CNT);
        ROS_INFO("I get Encoder One Circle Count: [%d]", encoderOneCircleCnt_);


        if (robotCommMethod_ == "Network")
        {
            robotConfig1_.commMethod = Network;
        }
        else
        {
            robotConfig1_.commMethod = Serial;
        }

        if (robotType_ == "Jaguar")
        {
            robotConfig1_.boardType = Jaguar;
        }
        else {
            ROS_INFO("This is a Jaguar only driver, please check the type of robot you pass as a parameter");
        }


        robotConfig1_.portNum = commPortNum_;

        //  strcat(robotConfig1_.robotIP,robotIP_.c_str());
        strcpy(robotConfig1_.robotIP,robotIP_.c_str());


        //  strcat(robotConfig1_.serialPortName,robotSerialPort_.c_str());
        strcpy(robotConfig1_.serialPortName,robotSerialPort_.c_str());

        //create publishers for sensor data information
        motorInfo_pub_ = node_.advertise<drrobot_player_driver3::MotorInfoArray>("drrobot_motor", 1);
        powerInfo_pub_ = node_.advertise<drrobot_player_driver3::PowerInfo>("drrobot_powerinfo", 1);

        standardSensor_pub_ = node_.advertise<drrobot_player_driver3::StandardSensor>("drrobot_standardsensor", 1);
        customSensor_pub_ = node_.advertise<drrobot_player_driver3::CustomSensor>("drrobot_customsensor", 1);

        // Creates driver object and loads the parameters of the robot
        drrobotMotionDriver_ = new DrRobotMotionSensorDriver();
        drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);


    }

    ~DrRobotPlayerNode()
    {
    }

    int start()
    {
        // Connect to Robot or return error msg
        int res = -1;
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

        // Subscribes to motor velocity commands topic
        cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("drrobot_cmd_vel", 1, boost::bind(&DrRobotPlayerNode::cmdVelReceived, this, _1));

        return(0);
    }

    // stops the robot and return status = 0
    int stop()
    {
        int status = 0;
        drrobotMotionDriver_->close();
        usleep(1000000);
        return(status);
    }


    // callback function for when velocity commands are published
    void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
        double g_vel = cmd_vel->linear.x;
        double t_vel = cmd_vel->angular.z;


        int forwardPWM = -motorDir_ * g_vel * 16384 + 16384;
        int turnPWM = -motorDir_ * t_vel * 16384 + 16384;
        if (forwardPWM > 32767) forwardPWM = 32767;
        if (forwardPWM < 0) forwardPWM = 0;
        if (turnPWM > 32767) turnPWM = 32767;
        if (turnPWM < 0) turnPWM = 0;
        drrobotMotionDriver_->sendMotorCtrlAllCmd(PWM,NOCONTROL,NOCONTROL,NOCONTROL,forwardPWM,turnPWM, NOCONTROL);


    }

    void doUpdate()
    {

        // For other Robots
        //                drrobotPowerDriver_->readPowerSensorData(&powerSensorData_);
        //                drrobot_player_driver3::PowerInfo powerInfo;
        //                powerInfo.ref_vol = 1.5 * 4095 /(double)powerSensorData_.refVol;

        //                powerInfo.bat1_vol = (double)powerSensorData_.battery1Vol  * 8 / 4095 * powerInfo.ref_vol;
        //                powerInfo.bat2_vol = (double) powerSensorData_.battery2Vol * 8 / 4095 * powerInfo.ref_vol;

        //                powerInfo.bat1_temp = powerSensorData_.battery1Thermo;
        //                powerInfo.bat2_temp = powerSensorData_.battery2Thermo;

        //                powerInfo.dcin_vol = (double)powerSensorData_.dcINVol * 8 / 4095 * powerInfo.ref_vol;
        //                powerInfo.charge_path = powerSensorData_.powerChargePath;
        //                powerInfo.power_path = powerSensorData_.powerPath;
        //                powerInfo.power_status = powerSensorData_.powerStatus;

        //                powerInfo_pub_.publish(powerInfo);

        if (drrobotMotionDriver_->portOpen())
        {
            drrobotMotionDriver_->readMotorSensorData(&motorSensorData_);
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
                motorInfoArray.motorInfos[i].motor_current = 0.0;
                motorInfoArray.motorInfos[i].motor_pwm = motorSensorData_.motorSensorPWM[i];
            }

            //ROS_INFO("publish motor info array");
            motorInfo_pub_.publish(motorInfoArray);


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


            standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 34.498 /4095;
            standardSensor.refVol = (double)standardSensorData_.refVol / 4095 * 6;
            standardSensor.potVol = (double)standardSensorData_.potVol / 4095 * 6;
            standardSensor_pub_.publish(standardSensor);

            drrobot_player_driver3::CustomSensor customSensor;
            customSensor.customADData.resize(8);
            customSensor.header.stamp = ros::Time::now();
            customSensor.header.frame_id = string("drrobot_customsensor");

            for (uint32_t i = 0; i < 8; i ++)
            {
                customSensor.customADData[i] = customSensorData_.customADData[i];
            }
            customSensor.customIO = (uint8_t)(customSensorData_.customIO & 0xff);
            customSensor_pub_.publish(customSensor);
        }
    }

private:

    DrRobotMotionSensorDriver* drrobotMotionDriver_;

    struct DrRobotMotionConfig robotConfig1_;


    std::string odom_frame_id_;
    struct MotorSensorData motorSensorData_;
    struct PowerSensorData powerSensorData_;
    struct StandardSensorData standardSensorData_;
    struct CustomSensorData customSensorData_;

    std::string robotType_;
    std::string robotID_;
    std::string robotIP_;
    std::string robotCommMethod_;
    std::string robotSerialPort_;
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

    ros::Rate loop_rate(10);      //10Hz

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

