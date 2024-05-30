//
// Created by myx on 2024/5/27.
//

#include "sct_gazebo/sct_odom_sim.h"


namespace sct_gazebo
{
void SctOdomSim::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  model_ = _model;

  // Get parameters from SDF
  if (_sdf->HasElement("publish_rate"))
    publish_rate_ = _sdf->Get<double>("publish_rate");
  else{
    publish_rate_ = 50.0;
    gzerr << "No publish_rate specified, defaulting to set 50.0" << std::endl;
  }

  if (_sdf->HasElement("robot_namespace"))
    robot_namespace_ = _sdf->Get<std::string>("robot_namespace");
  else{
    robot_namespace_ = "/";
    gzerr << "No robot_namespace specified, defaulting to set \"/\"" << std::endl;
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load sct_odom_sim plugin.");
    return;
  }

  ROS_INFO("Starting sct_odom_sim Plugin (ns = %s)", robot_namespace_.c_str() );

  // Create our ROS node
  node_handle_ = new ros::NodeHandle(robot_namespace_);

  // Create a ROS publisher
  odom_pub_ = node_handle_->advertise<nav_msgs::Odometry>("odom", 1);

  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_link";
  odom_msg_.twist.covariance = {};

  odom2base_.header.frame_id = node_handle_->getNamespace() + "odom";
  odom2base_.header.stamp = ros::Time::now();
  odom2base_.child_frame_id = node_handle_->getNamespace() + "base_link";
  odom2base_.transform.rotation.w = 1;

  // Get the link
  robot_link_ = model_->GetLink("base_link");
  if (!robot_link_)
    ROS_INFO_STREAM("Link with name " << robot_link_ << " not found!" << std::endl);

  // Connect to the world update event
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&SctOdomSim::Update, this));

}

void SctOdomSim::Update()
{
  ros::Time current_time = ros::Time::now();
  if ((current_time - last_publish_time_).toSec() >= (1.0 / publish_rate_))
  {
    // Get the pose of the link
    ignition::math::Pose3d pose = robot_link_->WorldPose();
    ignition::math::Vector3d linear_vel = robot_link_->RelativeLinearVel();
    ignition::math::Vector3d angular_vel = robot_link_->RelativeAngularVel();

    rpy_to_quat_.setRPY(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());

    // Create a nav_msgs/Odometry message
    odom2base_.header.stamp = ros::Time::now();
    odom2base_.transform.translation.x = pose.Pos().X();
    odom2base_.transform.translation.y = pose.Pos().Y();
    odom2base_.transform.translation.z = pose.Pos().Z();
    odom2base_.transform.rotation.x = rpy_to_quat_.x();
    odom2base_.transform.rotation.y = rpy_to_quat_.y();
    odom2base_.transform.rotation.z = rpy_to_quat_.z();
    odom2base_.transform.rotation.w = rpy_to_quat_.w();
    tf_broadcaster_.sendTransform(odom2base_);

    odom_msg_.header.stamp = ros::Time::now();
    odom_msg_.pose.pose.position.x = pose.Pos().X();
    odom_msg_.pose.pose.position.y = pose.Pos().Y();
    odom_msg_.pose.pose.orientation.x = rpy_to_quat_.x();
    odom_msg_.pose.pose.orientation.y = rpy_to_quat_.y();
    odom_msg_.pose.pose.orientation.z = rpy_to_quat_.z();
    odom_msg_.pose.pose.orientation.w = rpy_to_quat_.w();
    odom_msg_.twist.twist.linear.x = linear_vel.X();
    odom_msg_.twist.twist.linear.y = linear_vel.Y();
    odom_msg_.twist.twist.angular.z = linear_vel.Z();
    odom_pub_.publish(odom_msg_);

    last_publish_time_ = current_time;
  }
}

GZ_REGISTER_MODEL_PLUGIN(SctOdomSim)
}
