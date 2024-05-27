//
// Created by myx on 2024/5/27.
//

#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/Odometry.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/Node.hh>


namespace sct_gazebo
{
class SctOdomSim:public gazebo::ModelPlugin{
public:
  SctOdomSim() {}
  virtual ~SctOdomSim() {
    node_handle_->shutdown();
    delete node_handle_;
  }

  virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

  virtual void Update();

private:
  void QueueThread();

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr robot_link_;
  gazebo::event::ConnectionPtr update_connection_;
  ros::NodeHandle* node_handle_;

  ros::Publisher odom_pub_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string topic_name_;
  double publish_rate_;
  std::string robot_namespace_;
  ros::Time last_publish_time_;
};
}

