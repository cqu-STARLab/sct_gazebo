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
  if (_sdf->HasElement("frame_id"))
    frame_id_ = _sdf->Get<std::string>("frame_id");
  else{
    frame_id_ = "base_link";
    gzerr << "No frame_id specified, defaulting to set base_link" << std::endl;
  }

  if (_sdf->HasElement("child_frame_id"))
    child_frame_id_ = _sdf->Get<std::string>("child_frame_id");
  else{
    child_frame_id_ = "base_link";
    gzerr << "No child_frame_id_ specified, defaulting to set base_link" << std::endl;
  }

  if (_sdf->HasElement("topic_name"))
    topic_name_ = _sdf->Get<std::string>("topic_name");
  else{
    topic_name_ = "odom";
    gzerr << "No topic_name specified, defaulting to set odom" << std::endl;
  }

  if (_sdf->HasElement("publish_rate"))
    publish_rate_ = _sdf->Get<double>("publish_rate");
  else{
    publish_rate_ = 10.0;
    gzerr << "No publish_rate specified, defaulting to set 10.0" << std::endl;
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
  odom_pub_ = node_handle_->advertise<nav_msgs::Odometry>(topic_name_, 1);

  // Get the link
  robot_link_ = model_->GetLink(child_frame_id_);
  if (!robot_link_)
    gzerr << "Link with name " << robot_link_ << " not found!" << std::endl;

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

    // Create a nav_msgs/Odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = frame_id_;
    odom_msg.child_frame_id = child_frame_id_;

    odom_msg.pose.pose.position.x = pose.Pos().X();
    odom_msg.pose.pose.position.y = pose.Pos().Y();
    odom_msg.pose.pose.position.z = pose.Pos().Z();
    odom_msg.pose.pose.orientation.x = pose.Rot().X();
    odom_msg.pose.pose.orientation.y = pose.Rot().Y();
    odom_msg.pose.pose.orientation.z = pose.Rot().Z();
    odom_msg.pose.pose.orientation.w = pose.Rot().W();

    // Publish the message
    odom_pub_.publish(odom_msg);

    last_publish_time_ = current_time;
  }
}

GZ_REGISTER_MODEL_PLUGIN(SctOdomSim)
}
