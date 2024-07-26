//
// Created by myx on 2024/7/26.
//

#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <tf2/LinearMath/Quaternion.h>
#include <ignition/math/Vector3.hh>
#include <sensor_msgs/Imu.h>
#include <string>


namespace sct_gazebo
{
class ImuSim : public gazebo::ModelPlugin
{
public:
  ImuSim()
  {
    seed_ = 0;
  }
  ~ImuSim()
  {
    node_handle_->shutdown();
    delete node_handle_;
  }

  virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

  virtual void Update();

private:
  // \brief Gaussian noise generator.
  // \param mu offset value.
  // \param sigma scaling value.
  double GuassianKernel(double mu, double sigma);
  // \brief Seed for the Gaussian noise generator.
  unsigned int seed_;
  // \brief Gaussian noise.
  double gaussian_noise_;

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr imu_link_;
  gazebo::event::ConnectionPtr update_connection_;
  ros::NodeHandle* node_handle_;

  sensor_msgs::Imu imu_msg_;
  ros::Publisher imu_pub_;
  ros::Time last_publish_time_;

  // parameters
  double publish_rate_;
  std::string robot_namespace_;
  std::string imu_topic_;
  std::string frame_id_;
  tf2::Quaternion rpy_to_quat_;
};
}


