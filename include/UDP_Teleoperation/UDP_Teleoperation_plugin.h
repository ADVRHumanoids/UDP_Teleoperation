/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef UDP_Teleoperation_PLUGIN_H_
#define UDP_Teleoperation_PLUGIN_H_

#include <XCM/XBotControlPlugin.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


#include "angularsegmentkinematicsdatagram.h"
#include "parsermanager.h"
#include "quaterniondatagram.h"


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "rotation_matrix.h"

#define BUFSIZE 2048

namespace XBotPlugin {

/**
 * @brief UDP_Teleoperation XBot RT Plugin
 *
 **/
class UDP_Teleoperation : public XBot::XBotControlPlugin
{

public:

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);
    
    virtual ~UDP_Teleoperation();

protected:

    virtual void control_loop(double time, double period);

private:

    XBot::RobotInterface::Ptr _robot;

    
    XBot::SharedObject<Eigen::Quaternionf> pelvquat, upparmquat, forearmquat, handquat;
    
    double _start_time;

    Eigen::VectorXd _q0;

    XBot::MatLogger::Ptr _logger;
    
    bool readDatagram();
    
    struct sockaddr_in myaddr; /* our address */ 
    struct sockaddr_in remaddr; /* remote address */ 
    socklen_t addrlen = sizeof(remaddr); /* length of addresses */ 
    int recvlen; /* # bytes received */ 
    int fd; /* our socket */ 
    char buf[BUFSIZE]; /* receive buffer */ 
    QuaternionDatagram* quaternionDatagram_;
    std::unique_ptr<ParserManager> parserManager;
      
    struct qKinematics{
    quaternionKinematics Pelvis_, L5_, RightHipP_, RightKneeP_, RightAnkleP_, RightToeP_, LeftHipP_, LeftKneeP_, LeftAnkleP_, LeftToeP_, RightUpperArmP_, RightForeArmP_, LeftUpperArmP_, LeftForeArmP_, RightHandP_, LeftHandP_;
    };

    struct qKinematics qKinematics_;
    
    qKinematics ReadXsensQuaternion() const{

        struct qKinematics qKinematics;

        qKinematics.Pelvis_ = quaternionDatagram_->getItem(1);
        qKinematics.L5_ = quaternionDatagram_->getItem(2);
        qKinematics.RightHipP_ = quaternionDatagram_->getItem(16);
        qKinematics.RightKneeP_ = quaternionDatagram_->getItem(17);
        qKinematics.RightAnkleP_ = quaternionDatagram_->getItem(18);
        qKinematics.RightToeP_ = quaternionDatagram_->getItem(19);
        qKinematics.LeftHipP_ = quaternionDatagram_->getItem(20);
        qKinematics.LeftKneeP_ = quaternionDatagram_->getItem(21);
        qKinematics.LeftAnkleP_ = quaternionDatagram_->getItem(22);
        qKinematics.LeftToeP_ = quaternionDatagram_->getItem(23);
        qKinematics.RightUpperArmP_ = quaternionDatagram_->getItem(9);
        qKinematics.RightForeArmP_ = quaternionDatagram_->getItem(10);
        qKinematics.RightHandP_ = quaternionDatagram_->getItem(11);
        qKinematics.LeftUpperArmP_ = quaternionDatagram_->getItem(13);
        qKinematics.LeftForeArmP_ = quaternionDatagram_->getItem(14);
        qKinematics.LeftHandP_ = quaternionDatagram_->getItem(15);
    //    std::cout << "TestQuarternion: "  << std::endl;

      return qKinematics;
    }

    
};

}

#endif // UDP_Teleoperation_PLUGIN_H_
