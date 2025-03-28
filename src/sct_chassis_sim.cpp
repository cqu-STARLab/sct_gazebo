

/*
* \file  gazebo_ros_sct_chassis_sim.cpp
*
* \brief A skid steering drive plugin. Inspired by gazebo_ros_diff_drive and SkidSteerDrivePlugin
*
* \author  Zdenek Materna (imaterna@fit.vutbr.cz)
*
* $ Id: 06/25/2013 11:23:40 AM materna $
*/


#include <algorithm>
#include <assert.h>

#include <sct_gazebo/sct_chassis_sim.h>

#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace sct_gazebo {

enum {
 RIGHT_FRONT=0,
 LEFT_FRONT=1,
 RIGHT_REAR=2,
 LEFT_REAR=3,
};

SctChassisSim::SctChassisSim() {}

// Destructor
SctChassisSim::~SctChassisSim() {
 delete rosnode_;
}

// Load the controller
void SctChassisSim::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

 this->parent = _parent;
 this->world = _parent->GetWorld();

 this->robot_namespace_ = "";
 if (!_sdf->HasElement("robotNamespace")) {
   ROS_INFO_NAMED("sct_chassis_sim", "SctChassisSim Plugin missing <robotNamespace>, defaults to \"%s\"",
                  this->robot_namespace_.c_str());
 } else {
   this->robot_namespace_ =
       _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
 }


 // TODO write error if joint doesn't exist!
 this->left_front_joint_name_ = "left_front_joint";
 if (!_sdf->HasElement("leftFrontJoint")) {
   ROS_WARN_NAMED("sct_chassis_sim", "SctChassisSim Plugin (ns = %s) missing <leftFrontJoint>, defaults to \"%s\"",
                  this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
 } else {
   this->left_front_joint_name_ = _sdf->GetElement("leftFrontJoint")->Get<std::string>();
   ROS_INFO_STREAM("Get left front joint: " << left_front_joint_name_);
 }
 double left_front_transmission = 1.0;
 if (!_sdf->HasElement("leftFrontTransmission")) {
   ROS_WARN("sct_chassis_sim, SctChassisSim Plugin (ns = %s) missing <leftFrontTransmission>, defaults to \"%f\"",
            robot_namespace_.c_str(), left_front_transmission );
 } else {
   left_front_transmission = _sdf->GetElement("leftFrontTransmission")->Get<double>();
   ROS_INFO_STREAM("Get left front joint transmission: " << left_front_transmission);
 }

 this->right_front_joint_name_ = "right_front_joint";
 if (!_sdf->HasElement("rightFrontJoint")) {
   ROS_WARN_NAMED("sct_chassis_sim", "SctChassisSim Plugin (ns = %s) missing <rightFrontJoint>, defaults to \"%s\"",
                  this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
 } else {
   this->right_front_joint_name_ = _sdf->GetElement("rightFrontJoint")->Get<std::string>();
   ROS_INFO_STREAM("Get right front joint: " << right_front_joint_name_);
 }
 double right_front_transmission = 1.0;
 if (!_sdf->HasElement("rightFrontTransmission")) {
   ROS_WARN("sct_chassis_sim, SctChassisSim Plugin (ns = %s) missing <rightFrontTransmission>, defaults to \"%f\"",
            robot_namespace_.c_str(), right_front_transmission );
 } else {
   right_front_transmission = _sdf->GetElement("rightFrontTransmission")->Get<double>();
   ROS_INFO_STREAM("Get right front joint transmission: " << right_front_transmission);
 }

 this->left_rear_joint_name_ = "left_rear_joint";
 if (!_sdf->HasElement("leftRearJoint")) {
   ROS_WARN_NAMED("sct_chassis_sim", "SctChassisSim Plugin (ns = %s) missing <leftRearJoint>, defaults to \"%s\"",
                  this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
 } else {
   this->left_rear_joint_name_ = _sdf->GetElement("leftRearJoint")->Get<std::string>();
   ROS_INFO_STREAM("Get left rear joint: " << left_rear_joint_name_);
 }
 double left_rear_transmission = 1.0;
 if (!_sdf->HasElement("leftRearTransmission")) {
   ROS_WARN("sct_chassis_sim, SctChassisSim Plugin (ns = %s) missing <leftRearTransmission>, defaults to \"%f\"",
            robot_namespace_.c_str(), left_rear_transmission );
 } else {
   left_rear_transmission = _sdf->GetElement("leftRearTransmission")->Get<double>();
   ROS_INFO_STREAM("Get left rear joint transmission: " << left_rear_transmission);
 }

 this->right_rear_joint_name_ = "right_rear_joint";
 if (!_sdf->HasElement("rightRearJoint")) {
   ROS_WARN_NAMED("sct_chassis_sim", "SctChassisSim Plugin (ns = %s) missing <rightRearJoint>, defaults to \"%s\"",
                  this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
 } else {
   this->right_rear_joint_name_ = _sdf->GetElement("rightRearJoint")->Get<std::string>();
   ROS_INFO_STREAM("Get right rear joint: " << right_rear_joint_name_);
 }
 double right_rear_transmission = 1.0;
 if (!_sdf->HasElement("rightRearTransmission")) {
   ROS_WARN("sct_chassis_sim, SctChassisSim Plugin (ns = %s) missing <rightRearTransmission>, defaults to \"%f\"",
            robot_namespace_.c_str(), right_rear_transmission );
 } else {
   right_rear_transmission = _sdf->GetElement("rightRearTransmission")->Get<double>();
   ROS_INFO_STREAM("Get right rear joint transmission: " << right_rear_transmission);
 }

 this->wheel_track_ = 0.4;

 if (!_sdf->HasElement("wheelTrack")) {
   ROS_WARN_NAMED("sct_chassis_sim", "SctChassisSim Plugin (ns = %s) missing <wheelTrack>, defaults to value from robot_description: %f",
                  this->robot_namespace_.c_str(), this->wheel_track_);
 } else {
   this->wheel_track_ = _sdf->GetElement("wheelTrack")->Get<double>();
   ROS_INFO_STREAM("Get wheel track: " << wheel_track_);
 }

 // TODO get this from robot_description
 this->wheel_radius_ = 0.15;
 if (!_sdf->HasElement("wheelRadius")) {
   ROS_WARN_NAMED("sct_chassis_sim", "SctChassisSim Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
                  this->robot_namespace_.c_str(), this->wheel_radius_);
 } else {
   this->wheel_radius_ = _sdf->GetElement("wheelRadius")->Get<double>();
   ROS_INFO_STREAM("Get wheel radius: " << wheel_radius_);
 }

 this->torque = 5.0;
 if (!_sdf->HasElement("torque")) {
   ROS_WARN_NAMED("sct_chassis_sim", "SctChassisSim Plugin (ns = %s) missing <torque>, defaults to %f",
                  this->robot_namespace_.c_str(), this->torque);
 } else {
   this->torque = _sdf->GetElement("torque")->Get<double>();
 }

 this->command_topic_ = "cmd_vel";
 if (!_sdf->HasElement("commandTopic")) {
   ROS_WARN_NAMED("sct_chassis_sim", "SctChassisSim Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
                  this->robot_namespace_.c_str(), this->command_topic_.c_str());
 } else {
   this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
   ROS_INFO_STREAM("Get command topic: " << command_topic_);
 }

 this->update_rate_ = 100.0;
 if (!_sdf->HasElement("updateRate")) {
   ROS_WARN_NAMED("sct_chassis_sim", "SctChassisSim Plugin (ns = %s) missing <updateRate>, defaults to %f",
                  this->robot_namespace_.c_str(), this->update_rate_);
 } else {
   this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
   ROS_INFO_STREAM("Get update rate: " << update_rate_);
 }

 this->covariance_x_ = 0.0001;
 if (!_sdf->HasElement("covariance_x")) {
   ROS_WARN_NAMED("sct_chassis_sim", "SctChassisSim Plugin (ns = %s) missing <covariance_x>, defaults to %f",
                  this->robot_namespace_.c_str(), covariance_x_);
 } else {
   covariance_x_ = _sdf->GetElement("covariance_x")->Get<double>();
 }

 this->covariance_y_ = 0.0001;
 if (!_sdf->HasElement("covariance_y")) {
   ROS_WARN_NAMED("sct_chassis_sim", "SctChassisSim Plugin (ns = %s) missing <covariance_y>, defaults to %f",
                  this->robot_namespace_.c_str(), covariance_y_);
 } else {
   covariance_y_ = _sdf->GetElement("covariance_y")->Get<double>();
 }

 this->covariance_yaw_ = 0.01;
 if (!_sdf->HasElement("covariance_yaw")) {
   ROS_WARN_NAMED("sct_chassis_sim", "SctChassisSim Plugin (ns = %s) missing <covariance_yaw>, defaults to %f",
                  this->robot_namespace_.c_str(), covariance_yaw_);
 } else {
   covariance_yaw_ = _sdf->GetElement("covariance_yaw")->Get<double>();
 }


 // Initialize update rate stuff
 if (this->update_rate_ > 0.0) {
   this->update_period_ = 1.0 / this->update_rate_;
 } else {
   this->update_period_ = 0.0;
 }

 last_update_time_ = this->world->SimTime();

 // Initialize velocity stuff
 wheel_speed_[RIGHT_FRONT] = 0;
 wheel_speed_[LEFT_FRONT] = 0;
 wheel_speed_[RIGHT_REAR] = 0;
 wheel_speed_[LEFT_REAR] = 0;
 wheel_transmission_[RIGHT_FRONT] = 0.0;
 wheel_transmission_[LEFT_FRONT] = 0.0;
 wheel_transmission_[RIGHT_REAR] = 0.0;
 wheel_transmission_[LEFT_REAR] = 0.0;

 x_ = 0;
 rot_ = 0;
 cmd_update_ = false;
 alive_ = true;

 joints[LEFT_FRONT] = this->parent->GetJoint(left_front_joint_name_);
 joints[RIGHT_FRONT] = this->parent->GetJoint(right_front_joint_name_);
 joints[LEFT_REAR] = this->parent->GetJoint(left_rear_joint_name_);
 joints[RIGHT_REAR] = this->parent->GetJoint(right_rear_joint_name_);

 wheel_transmission_[RIGHT_FRONT] = right_front_transmission;
 wheel_transmission_[LEFT_FRONT] = left_front_transmission;
 wheel_transmission_[RIGHT_REAR] = right_rear_transmission;
 wheel_transmission_[LEFT_REAR] = left_rear_transmission;

 if (!joints[LEFT_FRONT]) {
   char error[200];
   snprintf(error, 200,
            "SctChassisSim Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
            this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
   gzthrow(error);
 }

 if (!joints[RIGHT_FRONT]) {
   char error[200];
   snprintf(error, 200,
            "SctChassisSim Plugin (ns = %s) couldn't get right front hinge joint named \"%s\"",
            this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
   gzthrow(error);
 }

 if (!joints[LEFT_REAR]) {
   char error[200];
   snprintf(error, 200,
            "SctChassisSim Plugin (ns = %s) couldn't get left rear hinge joint named \"%s\"",
            this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
   gzthrow(error);
 }

 if (!joints[RIGHT_REAR]) {
   char error[200];
   snprintf(error, 200,
            "SctChassisSim Plugin (ns = %s) couldn't get right rear hinge joint named \"%s\"",
            this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
   gzthrow(error);
 }

#if GAZEBO_MAJOR_VERSION > 2
 joints[LEFT_FRONT]->SetParam("fmax", 0, torque);
 joints[RIGHT_FRONT]->SetParam("fmax", 0, torque);
 joints[LEFT_REAR]->SetParam("fmax", 0, torque);
 joints[RIGHT_REAR]->SetParam("fmax", 0, torque);
#else
 joints[LEFT_FRONT]->SetMaxForce(0, torque);
 joints[RIGHT_FRONT]->SetMaxForce(0, torque);
 joints[LEFT_REAR]->SetMaxForce(0, torque);
 joints[RIGHT_REAR]->SetMaxForce(0, torque);
#endif

 // Make sure the ROS node for Gazebo has already been initialized
 if (!ros::isInitialized())
 {
   ROS_FATAL_STREAM_NAMED("sct_chassis_sim", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                           << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
   return;
 }

 rosnode_ = new ros::NodeHandle(this->robot_namespace_);

 ROS_INFO_NAMED("sct_chassis_sim", "Starting SctChassisSim Plugin (ns = %s)", this->robot_namespace_.c_str());

 // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
 ros::SubscribeOptions so =
     ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                                                         boost::bind(&SctChassisSim::cmdVelCallback, this, boost::placeholders::_1),
                                                         ros::VoidPtr(), &queue_);

 cmd_vel_subscriber_ = rosnode_->subscribe(so);

 // start custom queue for diff drive
 this->callback_queue_thread_ = boost::thread(boost::bind(&SctChassisSim::QueueThread, this));

 // listen to the update event (broadcast every simulation iteration)
 this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&SctChassisSim::UpdateChild, this));

}

// Update the controller
void SctChassisSim::UpdateChild()
{
 gazebo::common::Time current_time = this->world->SimTime();

 if (cmd_update_) {
   calculateWheelSpeeds();

   joints[LEFT_FRONT]->SetParam("vel", 0, wheel_speed_[LEFT_FRONT]);
   joints[RIGHT_FRONT]->SetParam("vel", 0, wheel_speed_[RIGHT_FRONT]);
   joints[LEFT_REAR]->SetParam("vel", 0, wheel_speed_[LEFT_REAR]);
   joints[RIGHT_REAR]->SetParam("vel", 0, wheel_speed_[RIGHT_REAR]);
   cmd_update_ = false;
 } else{
   joints[LEFT_FRONT]->SetParam("vel", 0, 0);
   joints[RIGHT_FRONT]->SetParam("vel", 0, 0);
   joints[LEFT_REAR]->SetParam("vel", 0, 0);
   joints[RIGHT_REAR]->SetParam("vel", 0, 0);
 }
 last_update_time_ = current_time;
}

// Finalize the controller
void SctChassisSim::FiniChild() {
 alive_ = false;
 queue_.clear();
 queue_.disable();
 rosnode_->shutdown();
 callback_queue_thread_.join();
}

void SctChassisSim::cmdVelCallback(
   const geometry_msgs::Twist::ConstPtr& cmd_msg) {

 boost::mutex::scoped_lock scoped_lock(lock);
 x_ = cmd_msg->linear.x;
 rot_ = cmd_msg->angular.z;
 cmd_update_ = true;
}

void SctChassisSim::calculateWheelSpeeds() {
  boost::mutex::scoped_lock scoped_lock(lock);

  double v_x = x_;
  double w_rot_ = rot_;

  double v_right = v_x + (w_rot_ * wheel_track_ / 2);
  double v_left = v_x - (w_rot_ * wheel_track_ / 2);

  wheel_speed_[RIGHT_FRONT] = wheel_transmission_[RIGHT_FRONT] * (v_right / wheel_radius_);
  wheel_speed_[RIGHT_REAR] = wheel_transmission_[RIGHT_REAR] * (v_right / wheel_radius_);

  wheel_speed_[LEFT_FRONT] = wheel_transmission_[LEFT_FRONT] * (v_left / wheel_radius_);
  wheel_speed_[LEFT_REAR] = wheel_transmission_[LEFT_REAR] * (v_left / wheel_radius_);
}

void SctChassisSim::QueueThread() {
 static const double timeout = 0.01;

 while (alive_ && rosnode_->ok()) {
   queue_.callAvailable(ros::WallDuration(timeout));
   if(!cmd_update_){
     joints[LEFT_FRONT]->SetParam("vel", 0, 0);
     joints[RIGHT_FRONT]->SetParam("vel", 0, 0);
     joints[LEFT_REAR]->SetParam("vel", 0, 0);
     joints[RIGHT_REAR]->SetParam("vel", 0, 0);
   }
 }
}

GZ_REGISTER_MODEL_PLUGIN(SctChassisSim)
}