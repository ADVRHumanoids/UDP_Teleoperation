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

#include <UDP_Teleoperation_plugin.h>

#define PORT 1153 
#include <tf_conversions/tf_eigen.h>

/* Specify that the class XBotPlugin::UDP_Teleoperation is a XBot RT plugin with name "UDP_Teleoperation" */
REGISTER_XBOT_PLUGIN_(XBotPlugin::UDP_Teleoperation)

namespace XBotPlugin {

bool UDP_Teleoperation::readDatagram(){
  
   recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
    //std::cout<<"received "<<recvlen<< "bytes"<<std::endl; 
    if (recvlen > 0) {
      //buf[recvlen] = 0; 
      //std::cout<<"received message: "<< buf <<std::endl;      
      parserManager->readDatagram(buf);        
    }
        
    std::memset(buf, 0, BUFSIZE);
    quaternionDatagram_ = parserManager->getQuaternionDatagram();    

    if (quaternionDatagram_ != NULL) {     
       qKinematics_ = ReadXsensQuaternion();
       return true;
    }
    return false;
}  
  
bool UDP_Teleoperation::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */

    handquat = handle->getSharedMemory()->getSharedObject<Eigen::Quaternionf>("hand");
    forearmquat = handle->getSharedMemory()->getSharedObject<Eigen::Quaternionf>("fore_arm");
    upparmquat = handle->getSharedMemory()->getSharedObject<Eigen::Quaternionf>("upper_arm");
    pelvquat = handle->getSharedMemory()->getSharedObject<Eigen::Quaternionf>("pelvis");

    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */

    _logger = XBot::MatLogger::getLogger("/tmp/UDP_Teleoperation_log");
    
            
    parserManager.reset(new ParserManager(false, false));
    
    /* create a UDP socket */ 
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { perror("cannot create socket\n"); return false; }
    /* bind the socket to any valid IP address and a specific port */ 
    memset((char *)&myaddr, 0, sizeof(myaddr)); 
    myaddr.sin_family = AF_INET; 
    myaddr.sin_addr.s_addr = inet_addr("10.255.32.109");// htonl(INADDR_ANY); 
    myaddr.sin_port = htons(PORT); 
    if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) { perror("bind failed"); return false; }

    std::cout<<"waiting on port"<< PORT << std::endl;
    return true;


}

void UDP_Teleoperation::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /UDP_Teleoperation_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the plugin starting time to a class member */
    _robot->getMotorPosition(_q0);    
    
    /* Save the robot starting config to a class member */
    _start_time = time;
}

void UDP_Teleoperation::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /UDP_Teleoperation_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void UDP_Teleoperation::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* The following code checks if any command was received from the plugin standard port
     * (e.g. from ROS you can send commands with
     *         rosservice call /UDP_Teleoperation_cmd "cmd: 'MY_COMMAND_1'"
     * If any command was received, the code inside the if statement is then executed. */
    
    if(!current_command.str().empty()){

        if(current_command.str() == "stop"){
            /* Handle command */
            return;
        }

        if(current_command.str() == "MY_COMMAND_2"){
            /* Handle command */
        }

    }
   
    if (readDatagram()){
      
        float* pelvis = qKinematics_.Pelvis_.quatRotation;
        float*  uparm = qKinematics_.RightUpperArmP_.quatRotation;
        float*  forearm = qKinematics_.RightForeArmP_.quatRotation;
        float*  hand = qKinematics_.RightHandP_.quatRotation;

        Matrix3f handmat = QuaternionRotation(hand)*rotx(-M_PI/2);
        Matrix3f forearmmat = QuaternionRotation(forearm);//*rotx(-M_PI/2);
        Matrix3f uparmmat = QuaternionRotation(uparm);//*rotx(-M_PI/2);
        Matrix3f pelvismmat = QuaternionRotation(uparm);//*rotx(-M_PI/2);

        handquat.set(RotationQuaternion(handmat));
        forearmquat.set(RotationQuaternion(forearmmat));
        upparmquat.set(RotationQuaternion(uparmmat));
        pelvquat.set(RotationQuaternion(pelvismmat));
       
      }    
 
}

bool UDP_Teleoperation::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}

UDP_Teleoperation::~UDP_Teleoperation()
{
  
}

}
