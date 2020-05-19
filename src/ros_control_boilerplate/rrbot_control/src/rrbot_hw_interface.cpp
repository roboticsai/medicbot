/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <rrbot_control/rrbot_hw_interface.h>

namespace rrbot_control
{

RRBotHWInterface::RRBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
    ROS_INFO_NAMED("rrbot_hw_interface", "RRBotHWInterface Ready.");

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    struct termios SerialPortSettings;  // Create the structure
    tcgetattr(fd, &SerialPortSettings);
    cfsetispeed(&SerialPortSettings,B115200); // Set Read  Speed as 115200
    cfsetospeed(&SerialPortSettings,B115200);
    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) // Set the attributes to the termios structure
      printf("Error while setting attributes \n");
      printf("\nfunction name is  %s >>\n",__func__);
      printf("reading from Serial port== %c >>\n",red);
      printf("returned fd is :%d\n",fd );
      if (fd == -1)
      {
        perror("open_port: Unable to open /dev/ttyUSB0 - ");
      }

}

void RRBotHWInterface::read(ros::Duration &elapsed_time)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

int myMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

std::string to_format(const int number,int decPlace) {
    std::stringstream ss;
    ss << std::setw(decPlace) << std::setfill('0') << number;
    return ss.str();
}

void Write(int fd,std::string data,std::size_t del_time) {
  //std::cout<<"sending data serial="<<data<<std::endl;
  for(int i=0;i<data.length();i++) {
      int ret=write(fd,&data[i],1);
  }
  std::this_thread::sleep_for (std::chrono::microseconds (del_time));
}

void RRBotHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  //
  // DUMMY PASS-THROUGH CODE
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) { 
    joint_velocity_[joint_id] += joint_velocity_command_[joint_id];
  }

  int joint1_vel = myMap(joint_velocity_command_[0],-1,1,-255,255);
  int joint2_vel = myMap(joint_velocity_command_[1],-1,1,-255,255);
  
  std::string data = to_format(joint1_vel,4)+to_format(joint2_vel,4)+"\n";
  Write(fd,data,10000);
  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void RRBotHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace
