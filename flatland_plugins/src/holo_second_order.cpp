/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	holo_second_order.cpp
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
#include <flatland_plugins/holo_second_order.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/model_plugin.h>
#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace flatland_plugins {

void HoloSecondOrder::TwistCallback(const geometry_msgs::Twist& msg) {
  twist_des_msg_ = msg;
}

void HoloSecondOrder::OnInitialize(const YAML::Node& config) {
  YamlReader reader(config);
  enable_odom_pub_ = reader.Get<bool>("enable_odom_pub", true);
  enable_twist_pub_ = reader.Get<bool>("enable_twist_pub", true);
  enable_acc_pub_ = true; // hardcoding for now

  std::string body_name = reader.Get<std::string>("body");
  std::string odom_frame_id = reader.Get<std::string>("odom_frame_id", "odom");

  std::string twist_topic = reader.Get<std::string>("twist_sub", "cmd_vel");
  std::string odom_topic =
      reader.Get<std::string>("odom_pub", "odometry/filtered");
  std::string ground_truth_topic =
      reader.Get<std::string>("ground_truth_pub", "odometry/ground_truth");
  std::string twist_pub_topic = reader.Get<std::string>("twist_pub", "twist");
  std::string acc_pub_topic = "acc"; // hardcoding for now

  // noise are in the form of linear x, linear y, angular variances
  std::vector<double> odom_twist_noise =
      reader.GetList<double>("odom_twist_noise", {0, 0, 0}, 3, 3);
  std::vector<double> odom_pose_noise =
      reader.GetList<double>("odom_pose_noise", {0, 0, 0}, 3, 3);

  double pub_rate =
      reader.Get<double>("pub_rate", std::numeric_limits<double>::infinity());
  update_timer_.SetRate(pub_rate);

  // by default the covariance diagonal is the variance of actual noise
  // generated, non-diagonal elements are zero assuming the noises are
  // independent, we also don't care about linear z, angular x, and angular y
  std::array<double, 36> odom_pose_covar_default = {0};
  odom_pose_covar_default[0] = odom_pose_noise[0];
  odom_pose_covar_default[7] = odom_pose_noise[1];
  odom_pose_covar_default[35] = odom_pose_noise[2];

  std::array<double, 36> odom_twist_covar_default = {0};
  odom_twist_covar_default[0] = odom_twist_noise[0];
  odom_twist_covar_default[7] = odom_twist_noise[1];
  odom_twist_covar_default[35] = odom_twist_noise[2];

  auto odom_twist_covar = reader.GetArray<double, 36>("odom_twist_covariance",
                                                      odom_twist_covar_default);
  auto odom_pose_covar = reader.GetArray<double, 36>("odom_pose_covariance",
                                                     odom_pose_covar_default);

  reader.EnsureAccessedAllKeys();

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name " + Q(body_name) + " does not exist");
  }

  // publish and subscribe to topics
  twist_sub_ = nh_.subscribe(twist_topic, 1, &HoloSecondOrder::TwistCallback, this);
  if (enable_odom_pub_) {
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 1);
    ground_truth_pub_ =
        nh_.advertise<nav_msgs::Odometry>(ground_truth_topic, 1);
  }

  if (enable_twist_pub_) {
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_pub_topic, 1);
  }

  if (enable_acc_pub_) {
    acc_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(acc_pub_topic, 1);
  }

  // init the values for the messages
  ground_truth_msg_.header.frame_id =
  tf::resolve("", GetModel()->NameSpaceTF(odom_frame_id));
  ground_truth_msg_.child_frame_id =
      tf::resolve("", GetModel()->NameSpaceTF(body_->name_));
  ground_truth_msg_.twist.covariance.fill(0);
  ground_truth_msg_.pose.covariance.fill(0);
  odom_msg_ = ground_truth_msg_;

  // copy from std::array to boost array
  for (unsigned int i = 0; i < 36; i++) {
    odom_msg_.twist.covariance[i] = odom_twist_covar[i];
    odom_msg_.pose.covariance[i] = odom_pose_covar[i];
  }

  // init the random number generators
  std::random_device rd;
  rng_ = std::default_random_engine(rd());
  for (unsigned int i = 0; i < 3; i++) {
    // variance is standard deviation squared
    noise_gen_[i] =
        std::normal_distribution<double>(0.0, sqrt(odom_pose_noise[i]));
  }

  for (unsigned int i = 0; i < 3; i++) {
    noise_gen_[i + 3] =
        std::normal_distribution<double>(0.0, sqrt(odom_twist_noise[i]));
  }

  ROS_DEBUG_NAMED("Holo",
                  "Initialized with params body(%p %s) odom_frame_id(%s) "
                  "twist_sub(%s) odom_pub(%s) ground_truth_pub(%s) "
                  "odom_pose_noise({%f,%f,%f}) odom_twist_noise({%f,%f,%f}) "
                  "pub_rate(%f)\n",
                  body_, body_->name_.c_str(), odom_frame_id.c_str(),
                  twist_topic.c_str(), odom_topic.c_str(),
                  ground_truth_topic.c_str(), odom_pose_noise[0],
                  odom_pose_noise[1], odom_pose_noise[2], odom_twist_noise[0],
                  odom_twist_noise[1], odom_twist_noise[2], pub_rate);

  error_x_tmin1 = 0.0;
  error_y_tmin1 = 0.0;
  error_theta_tmin1 = 0.0;
  t_prev = ros::Time::now();
}

void HoloSecondOrder::AfterPhysicsStep(const Timekeeper& timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);

  b2Body* b2body = body_->physics_body_;

  b2Vec2 position = b2body->GetPosition();
  float angle = b2body->GetAngle();

  if (publish) {
    // get the state of the body and publish the data
    b2Vec2 linear_vel_local =
        b2body->GetLinearVelocityFromLocalPoint(b2Vec2(0, 0));
    float angular_vel = b2body->GetAngularVelocity();

    ground_truth_msg_.header.stamp = timekeeper.GetSimTime();
    ground_truth_msg_.pose.pose.position.x = position.x;
    ground_truth_msg_.pose.pose.position.y = position.y;
    ground_truth_msg_.pose.pose.position.z = 0;
    ground_truth_msg_.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(angle);
    ground_truth_msg_.twist.twist.linear.x = linear_vel_local.x;
    ground_truth_msg_.twist.twist.linear.y = linear_vel_local.y;
    ground_truth_msg_.twist.twist.linear.z = 0;
    ground_truth_msg_.twist.twist.angular.x = 0;
    ground_truth_msg_.twist.twist.angular.y = 0;
    ground_truth_msg_.twist.twist.angular.z = angular_vel;

    // add the noise to odom messages
    odom_msg_.header.stamp = timekeeper.GetSimTime();
    odom_msg_.pose.pose = ground_truth_msg_.pose.pose;
    odom_msg_.twist.twist = ground_truth_msg_.twist.twist;
    odom_msg_.pose.pose.position.x += noise_gen_[0](rng_);
    odom_msg_.pose.pose.position.y += noise_gen_[1](rng_);
    odom_msg_.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(angle + noise_gen_[2](rng_));
    odom_msg_.twist.twist.linear.x += noise_gen_[3](rng_);
    odom_msg_.twist.twist.linear.y += noise_gen_[4](rng_);
    odom_msg_.twist.twist.angular.z += noise_gen_[5](rng_);

    if (enable_odom_pub_) {
      ground_truth_pub_.publish(ground_truth_msg_);
      odom_pub_.publish(odom_msg_);
    }

    if (enable_twist_pub_) {
      // Transform global frame velocity into local frame to simulate encoder
      // readings
      geometry_msgs::TwistStamped twist_pub_msg;
      twist_pub_msg.header.stamp = timekeeper.GetSimTime();
      twist_pub_msg.header.frame_id = odom_msg_.child_frame_id;

      // Forward velocity in twist.linear.x
      twist_pub_msg.twist.linear.x = cos(angle) * linear_vel_local.x +
                                     sin(angle) * linear_vel_local.y +
                                     noise_gen_[3](rng_);

      // Forward velocity in twist.linear.y
      twist_pub_msg.twist.linear.y = -sin(angle) * linear_vel_local.x +
                                     cos(angle) * linear_vel_local.y +
                                     noise_gen_[4](rng_);

      // Angular velocity in twist.angular.z
      twist_pub_msg.twist.angular.z = angular_vel + noise_gen_[5](rng_);
      twist_pub_.publish(twist_pub_msg);
    }

    if (enable_acc_pub_) {
      geometry_msgs::TwistStamped acc_pub_msg;
      acc_pub_msg.header.stamp = timekeeper.GetSimTime();
      acc_pub_msg.header.frame_id = odom_msg_.child_frame_id;

      acc_pub_msg.twist.linear.x = a_x;
      acc_pub_msg.twist.linear.y = a_y;
      acc_pub_msg.twist.angular.z = a_theta;

      acc_pub_.publish(acc_pub_msg);
    }

    // publish odom tf
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header = odom_msg_.header;
    odom_tf.child_frame_id = odom_msg_.child_frame_id;
    odom_tf.transform.translation.x = odom_msg_.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg_.pose.pose.position.y;
    odom_tf.transform.translation.z = 0;
    odom_tf.transform.rotation = odom_msg_.pose.pose.orientation;
    tf_broadcaster.sendTransform(odom_tf);
  }

}

void HoloSecondOrder::BeforePhysicsStep(const Timekeeper& timekeeper) {

  b2Body* b2body = body_->physics_body_;

  b2Vec2 position = b2body->GetPosition();
  float angle = b2body->GetAngle();
  b2Vec2 linear_vel_curr_local = b2body->GetLinearVelocityFromLocalPoint(b2Vec2(0, 0));
  float angular_vel_curr = b2body->GetAngularVelocity();

  b2Vec2 linear_vel_des_local(twist_des_msg_.linear.x, twist_des_msg_.linear.y);
  float angular_vel_des = twist_des_msg_.angular.z;  // angular is independent of frames

  float K_p_x = 10.0;
  float K_p_y = 10.0;
  float K_p_z = 10.0;

  float K_d_x = 0.0; // 0.001;
  float K_d_y = 0.0; // 0.001;
  float K_d_z = 0.0; // 0.001;

  // Proportional term
  float error_x = linear_vel_des_local.x - linear_vel_curr_local.x;
  float error_y = linear_vel_des_local.y - linear_vel_curr_local.y;
  float error_theta = angular_vel_des - angular_vel_curr;

  ros::Time t_now = ros::Time::now();
  float dt = (t_now - t_prev).toSec() + 1e-10;
  
  // Derivative control
  // float d_error_x_dt_k = (error_x - error_x_tmin1) / dt;
  // float d_error_y_dt_k = (error_y - error_y_tmin1) / dt;
  // float d_error_theta_dt_k = (error_theta - error_theta_tmin1) / dt;

  // Calculate acceleration
  a_x = (K_p_x * error_x); //  + (K_d_x * d_error_x_dt_k);
  a_y = (K_p_y * error_y); // + (K_d_y * d_error_y_dt_k);
  a_theta = (K_p_z * error_theta); // + (K_d_z * d_error_theta_dt_k);

  // Update velocity
  // Kind of weird, updating velocity before position 

  b2Vec2 linear_vel_local = linear_vel_curr_local;
  linear_vel_local.x += (a_x * dt);
  linear_vel_local.y += (a_y * dt);

  float angular_vel = angular_vel_curr;
  angular_vel += (a_theta * dt);

  // we apply the twist velocities, this must be done every physics step to make
  // sure Box2D solver applies the correct velocity through out. The velocity
  // given in the twist message should be in the local frame
  b2Vec2 linear_vel = b2body->GetWorldVector(linear_vel_local);

  // we want the velocity vector in the world frame at the center of mass

  // V_cm = V_o + W x r_cm/o
  // velocity at center of mass equals to the velocity at the body origin plus,
  // angular velocity cross product the displacement from the body origin to the
  // center of mass

  // r is the vector from body origin to the CM in world frame
  b2Vec2 r = b2body->GetWorldCenter() - position;
  b2Vec2 linear_vel_cm = linear_vel + angular_vel * b2Vec2(-r.y, r.x);

  // apply acceleration

  b2body->SetLinearVelocity(linear_vel_cm);
  b2body->SetAngularVelocity(angular_vel);

  t_prev = t_now;
  error_x_tmin1 = error_x;
  error_y_tmin1 = error_y;
  error_theta_tmin1 = error_theta;
}
}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::HoloSecondOrder,
                       flatland_server::ModelPlugin)