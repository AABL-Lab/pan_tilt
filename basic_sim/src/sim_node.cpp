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
   Desc:   Example ros_control main() entry point for controlling robots in ROS
   Original from: https://github.com/PickNikRobotics/ros_control_boilerplate
   Adapted by: Reuben Aronson, reuben.aronson@tufts.edu
*/

#include <pan_tilt/sim_hw_interface.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_hw_interface");
  ros::NodeHandle nh, pnh("~");

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  ros_control_boilerplate::SimHWInterface sim_hw_interface(pnh);
  sim_hw_interface.init();
  if (!ros::ok()) {
    return 1;
  }
  
  controller_manager::ControllerManager controller(&sim_hw_interface, nh);

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), [&sim_hw_interface, &controller] (ros::TimerEvent const & e) {
    ros::Duration elapsed_time = e.current_real - e.last_real;
    sim_hw_interface.read(elapsed_time);
    controller.update(e.current_real, elapsed_time);
    sim_hw_interface.write(elapsed_time);
  });

  ros::waitForShutdown();

  return 0;
}
