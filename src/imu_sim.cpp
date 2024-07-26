//
// Created by myx on 2024/5/27.
//

#include "sct_gazebo/imu_sim.h"


namespace sct_gazebo
{
void ImuSim::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  model_ = _model;

  // Get parameters from SDF
  if (_sdf->HasElement("imu_update_rate_HZ"))
    publish_rate_ = _sdf->Get<double>("imu_update_rate_HZ");
  else{
    publish_rate_ = 50.0;
    ROS_WARN_STREAM("No imu_update_rate_HZ be specified, defaulting to set: " << robot_namespace_);
  }

  if (_sdf->HasElement("robotNamespace"))
    robot_namespace_ = _sdf->Get<std::string>("robotNamespace");
  else{
    robot_namespace_ = "/";
    ROS_WARN_STREAM("No robotNamespace be specified, defaulting to set: " << robot_namespace_);
  }

  if (_sdf->HasElement("imu_topic"))
    imu_topic_ = _sdf->Get<std::string>("imu_topic");
  else{
    imu_topic_ = "imu_raw";
    ROS_WARN_STREAM("No imu_topic be specified, defaulting to set: " << imu_topic_);
  }

  if (_sdf->HasElement("imu_frame"))
    frame_id_ = _sdf->Get<std::string>("imu_frame");
  else{
    frame_id_ = "imu";
    ROS_ERROR_STREAM("No imu_frame be specified, defaulting to set: " << frame_id_);
    ROS_FATAL("missing <frameName>, cannot proceed");
//    return;
  }

  if (_sdf->HasElement("imu_gaussian_noise"))
    gaussian_noise_ =  _sdf->Get<double>("imu_gaussian_noise");
  else{
    gaussian_noise_ = 0.0;
    ROS_WARN_STREAM("No imu_gaussian_noise be specified, defaulting to set: " << gaussian_noise_);
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load imu_sim plugin.");
    return;
  }

  // Create our ROS node
  node_handle_ = new ros::NodeHandle(robot_namespace_);

  // Create a ROS publisher
  imu_pub_ = node_handle_->advertise<sensor_msgs::Imu>(imu_topic_, 1);

  imu_msg_.header.frame_id = frame_id_;
  imu_msg_.header.seq = 0;

  // Get the link

  imu_link_ = model_->GetLink(frame_id_);
  if (!imu_link_){
    ROS_ERROR_STREAM("Link with name " << model_->GetName()<< ":" << frame_id_ << " not found!" << std::endl);
    return;
  }
  // Connect to the world update event
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ImuSim::Update, this));

  ROS_INFO("Starting imu_sim Plugin (ns = %s)", robot_namespace_.c_str() );
}

void ImuSim::Update()
{
  ros::Time current_time = ros::Time::now();
  if ((current_time - last_publish_time_).toSec() >= (1.0 / publish_rate_))
  {
    // Get the pose of the link
    ignition::math::Pose3d pose = imu_link_->RelativePose();
    ignition::math::Vector3d linear_vel = imu_link_->RelativeLinearAccel();
    ignition::math::Vector3d angular_vel = imu_link_->RelativeAngularVel();

    rpy_to_quat_.setRPY(pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw());

    // Create a sensor_msgs/Imu message
    imu_msg_.header.stamp.sec = current_time.sec;
    imu_msg_.header.stamp.nsec = current_time.nsec;
    imu_msg_.header.seq++;

    imu_msg_.orientation.x = rpy_to_quat_.x() + GuassianKernel(0,gaussian_noise_);
    imu_msg_.orientation.y = rpy_to_quat_.y() + GuassianKernel(0,gaussian_noise_);
    imu_msg_.orientation.z = rpy_to_quat_.z() + GuassianKernel(0,gaussian_noise_);
    imu_msg_.orientation.w = rpy_to_quat_.w() + GuassianKernel(0,gaussian_noise_);

    imu_msg_.linear_acceleration.x = linear_vel.X();
    imu_msg_.linear_acceleration.y = linear_vel.Y();
    imu_msg_.linear_acceleration.z = linear_vel.Z();

    imu_msg_.angular_velocity.x = angular_vel.X();
    imu_msg_.angular_velocity.y = angular_vel.Y();
    imu_msg_.angular_velocity.z = angular_vel.Z();

    //covariance is related to the Gaussian noise
    double gn2 = gaussian_noise_*gaussian_noise_;
    imu_msg_.orientation_covariance[0] = gn2;
    imu_msg_.orientation_covariance[4] = gn2;
    imu_msg_.orientation_covariance[8] = gn2;
    imu_msg_.linear_acceleration_covariance[0] = gn2;
    imu_msg_.linear_acceleration_covariance[4] = gn2;
    imu_msg_.linear_acceleration_covariance[8] = gn2;
    imu_msg_.angular_velocity_covariance[0] = gn2;
    imu_msg_.angular_velocity_covariance[4] = gn2;
    imu_msg_.angular_velocity_covariance[8] = gn2;

    imu_pub_.publish(imu_msg_);

    last_publish_time_ = current_time;
  }
}

double ImuSim::GuassianKernel(double mu, double sigma)
{
  // generation of two normalized uniform random variables
  double U1 = static_cast<double>(rand_r(&seed_)) / static_cast<double>(RAND_MAX);
  double U2 = static_cast<double>(rand_r(&seed_)) / static_cast<double>(RAND_MAX);

  // using Box-Muller transform to obtain a varaible with a standard normal distribution
  double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

  // scaling
  Z0 = sigma * Z0 + mu;
  return Z0;
}

GZ_REGISTER_MODEL_PLUGIN(ImuSim)
}
