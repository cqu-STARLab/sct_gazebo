//
// Created by myx on 2025/3/28.
//

#pragma once

// reference: gazebo_ros_skid_steer_drive
/*
 * \file  gazebo_ros_skid_steer_drive.h
 *
 * \brief A skid steering drive plugin. Inspired by gazebo_ros_diff_drive and SkidSteerDrivePlugin
 *
 * \author  Zdenek Materna (imaterna@fit.vutbr.cz)
 *
 * $ Id: 06/25/2013 11:23:40 AM materna $
 */


#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace sct_gazebo {

class Joint;
class Entity;

class SctChassisSim : public gazebo::ModelPlugin {

public:
  SctChassisSim();
  ~SctChassisSim();
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  virtual void UpdateChild();
  virtual void FiniChild();

private:
  void calculateWheelSpeeds();

  gazebo::physics::WorldPtr world;
  gazebo::physics::ModelPtr parent;
  gazebo::event::ConnectionPtr update_connection_;

  std::string left_front_joint_name_;
  std::string right_front_joint_name_;
  std::string left_rear_joint_name_;
  std::string right_rear_joint_name_;

  double wheel_track_;
  double wheel_radius_;
  double torque;
  double wheel_speed_[4];
  double wheel_transmission_[4];

  gazebo::physics::JointPtr joints[4];

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Subscriber cmd_vel_subscriber_;


  boost::mutex lock;

  std::string robot_namespace_;
  std::string command_topic_;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  // DiffDrive stuff
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  double x_;
  double rot_;
  bool cmd_update_;
  bool alive_;

  // Update Rate
  double update_rate_;
  double update_period_;
  gazebo::common::Time last_update_time_;

  double covariance_x_;
  double covariance_y_;
  double covariance_yaw_;
};

}
