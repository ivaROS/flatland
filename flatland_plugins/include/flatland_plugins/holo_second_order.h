/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	holo_second_order.h
 * @brief   Second Order Holonomic plugin
 * @author  Max Asselmeier
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Avidbots Corp. nor the names of its
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
 */

#include <Box2D/Box2D.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <random>

#ifndef FLATLAND_PLUGINS_HOLO_SECOND_ORDER_H
#define FLATLAND_PLUGINS_HOLO_SECOND_ORDER_H

using namespace flatland_server;

namespace flatland_plugins {

class HoloSecondOrder : public flatland_server::ModelPlugin {
 public:
  ros::Subscriber twist_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher acc_pub_;
  ros::Publisher ground_truth_pub_;
  ros::Publisher twist_pub_;
  Body* body_;
  geometry_msgs::Twist twist_des_msg_;
  geometry_msgs::Twist twist_msg_;
  nav_msgs::Odometry odom_msg_;
  nav_msgs::Odometry ground_truth_msg_;
  UpdateTimer update_timer_;
  tf::TransformBroadcaster tf_broadcaster;  ///< For publish ROS TF
  bool enable_odom_pub_;   ///< YAML parameter to enable odom publishing
  bool enable_twist_pub_;  ///< YAML parameter to enable twist publishing
  bool enable_acc_pub_;  ///< YAML parameter to enable twist publishing

  float a_x = 0.0;
  float a_y = 0.0;
  float a_theta = 0.0;
  float error_x_tmin1 = 0.0;
  float error_y_tmin1 = 0.0;
  float error_theta_tmin1 = 0.0; 
  
  float K_p_x = 5.0; // 10.0;
  float K_p_y = 5.0; // 10.0;
  float K_p_z = 5.0; // 10.0;

  float K_d_x = 0.0; // 0.001;
  float K_d_y = 0.0; // 0.001;
  float K_d_z = 0.0; // 0.001;

  float linear_acc_lim = 5.0; // pulling from yaml, not sure if there's a good way to set it otherwise
  float angular_acc_lim = 5.0; // pulling from yaml, not sure if there's a good way to set it otherwise
  
  ros::Time t_prev;

  std::default_random_engine rng_;
  std::array<std::normal_distribution<double>, 6> noise_gen_;

  /**
   * @name          OnInitialize
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */
  void OnInitialize(const YAML::Node& config) override;
  /**
   * @name          BeforePhysicsStep
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */
  void BeforePhysicsStep(const Timekeeper& timekeeper) override;
  /**
   * @name        TwistCallback
   * @brief       callback to apply twist (velocity and omega)
   * @param[in]   timestep how much the physics time will increment
   */
  void TwistCallback(const geometry_msgs::Twist& msg);

   /**
   * @name          AfterPhysicsStep
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */

  void AfterPhysicsStep(const Timekeeper& timekeeper) override;
};
};

#endif
