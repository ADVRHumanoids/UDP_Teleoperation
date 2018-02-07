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


    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */

    _logger = XBot::MatLogger::getLogger("/tmp/UDP_Teleoperation_log");
    
    
     // ROS init
    int argc = 1;
    const char *arg = "dummy_arg";
    char* argg = const_cast<char*>(arg);
    char** argv = &argg;

    ros::init(argc, argv, "xsenseteleop");    
    _nh = std::make_shared<ros::NodeHandle>();     
    _pub = _nh->advertise<geometry_msgs::PoseStamped>("/w_T_right_ee", 1);
    _publ = _nh->advertise<geometry_msgs::PoseStamped>("/w_T_left_ee", 1);
        
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
    
    KDL::Frame l_sole_T_Waist;
    _robot->model().getPose("Waist", "l_sole", l_sole_T_Waist);
    l_sole_T_Waist.p.x(0.0);
    l_sole_T_Waist.p.y(0.0);    
    _robot->model().setFloatingBasePose(l_sole_T_Waist);    
    _robot->model().update();
    
    
    Eigen::Affine3d _pose;
    _robot->model().getPose("LSoftHand", 
			 "world",_pose);   
    
    std::cout<<"POSition Lhand robot "<<_pose.translation()[0]<<" "<< _pose.translation()[1]<< " "<<_pose.translation()[2]<<std::endl;
        
    Eigen::Affine3d _rhpose;
    _robot->model().getPose("RSoftHand", 
			 "world",_rhpose);
    
    std::cout<<"POSition Rhand robot "<<_rhpose.translation()[0]<<" "<< _rhpose.translation()[1]<< " "<<_rhpose.translation()[2]<<std::endl;
    
    init_pos_lh_robot[0] = _pose.translation()[0];
    init_pos_lh_robot[1] = _pose.translation()[1];
    init_pos_lh_robot[2] = _pose.translation()[2];
    
    init_pos_rh_robot[0] = _rhpose.translation()[0];
    init_pos_rh_robot[1] = _rhpose.translation()[1];
    init_pos_rh_robot[2] = _rhpose.translation()[2];

    
    
    if (readDatagram()){
      
	float* pos_rh = qKinematics_.RightHandP_.sensorPos; 
	float* pos_lh = qKinematics_.LeftHandP_.sensorPos; 
	init_pos_rh_xsense[0] = pos_rh[0];
	init_pos_rh_xsense[1] = pos_rh[1];
	init_pos_rh_xsense[2] = pos_rh[2];
	
	init_pos_lh_xsense[0] =pos_lh[0];
	init_pos_lh_xsense[1] =pos_lh[1];
	init_pos_lh_xsense[2] =pos_lh[2];
    }
    
    /* Save the robot starting config to a class member */
    _start_time = time;
     std::cout<<"start"<<std::endl;
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
      
	float* pos_rh = qKinematics_.RightHandP_.sensorPos; 
	float* pos_lh = qKinematics_.LeftHandP_.sensorPos; 
	//std::cout<<"position hand global frame "<< "X:"<< pos_h[0] << " Y:"<< pos_h[1] << " Z:" << pos_h[2] <<std::endl;
	float* pos_p = qKinematics_.Pelvis_.sensorPos;  
	//std::cout<<"position pelvis global frame "<< "X:"<< pos_p[0] << " Y:"<< pos_p[1] << " Z:" << pos_p[2] <<std::endl;
	float deltarh[3], deltalh[3];
	
        deltarh[0] = pos_rh[0] - init_pos_rh_xsense[0];
        deltarh[1] = pos_rh[1] - init_pos_rh_xsense[1];
        deltarh[2] = pos_rh[2] - init_pos_rh_xsense[2];
	
	deltalh[0] = pos_lh[0] - init_pos_lh_xsense[0];
        deltalh[1] = pos_lh[1] - init_pos_lh_xsense[1];
        deltalh[2] = pos_lh[2] - init_pos_lh_xsense[2];
	
	//std::cout<<"position hand local frame "<< "X:"<< delta[0] << " Y:"<< delta[1] << " Z:" << delta[2] <<std::endl;    
	
	float* rorient = qKinematics_.RightHandP_.quatRotation;
	float* lorient = qKinematics_.LeftHandP_.quatRotation;
	  
	Matrix3f rmat = QuaternionRotation(rorient)*rotx(-M_PI/2);
	Matrix3f lmat = QuaternionRotation(lorient)*rotx(M_PI/2);
	
	Eigen::Quaternionf rq;
	rq = RotationQuaternion(rmat);
	
	Eigen::Quaternionf lq;
	lq = RotationQuaternion(lmat);
  //        q.x() = orient[1];
  //        q.y() = orient[2];
  //        q.z() = orient[3];
  //        q.w() = orient[0];
	  //q.normalize();
	
	geometry_msgs::PoseStamped msg;
	msg.pose.position.x = init_pos_rh_robot[0]+deltarh[0]*1.8;
	msg.pose.position.y = init_pos_rh_robot[1]+deltarh[1]*1.8;
	msg.pose.position.z = init_pos_rh_robot[2]+deltarh[2]*1.8;
	  
	msg.pose.orientation.x = rq.x();
	msg.pose.orientation.y = rq.y();
	msg.pose.orientation.z = rq.z();
	msg.pose.orientation.w = rq.w();  
	
	
	geometry_msgs::PoseStamped msgl;
	msgl.pose.position.x = init_pos_lh_robot[0]+deltalh[0]*1.8;
	msgl.pose.position.y = init_pos_lh_robot[1]+deltalh[1]*1.8;
	msgl.pose.position.z = init_pos_lh_robot[2]+deltalh[2]*1.8;
	  
	msgl.pose.orientation.x = lq.x();
	msgl.pose.orientation.y = lq.y();
	msgl.pose.orientation.z = lq.z();
	msgl.pose.orientation.w = lq.w();  
	
  
	_pub.publish(msg);    
	_publ.publish(msgl);
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
