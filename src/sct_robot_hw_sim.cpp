//
// Created by myx on 2023/10/8.
//

#include "sct_gazebo/sct_robot_hw_sim.h"

#include <string>
#include <tinyxml.h>
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <control_toolbox/pid.h>

namespace sct_gazebo
{
bool SctRobotHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
                            gazebo::physics::ModelPtr parent_model, const urdf::Model* urdf_model,
                            std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);

  if (!model_nh.hasParam("angular_control"))
    ROS_WARN_STREAM("No " << model_nh.getNamespace() <<" angular_control specified");
  if (!angle_pid_controller_.init(ros::NodeHandle(model_nh, "angular_control/pid")))
    return false;

  base_link_ = parent_model->GetLink("base_link");
  if (!base_link_){
    ROS_ERROR_STREAM("Gazebo physics link with name: " << robot_namespace << " base_link" << " not found!" << std::endl);
    return false;
  }

  XmlRpc::XmlRpcValue wheels;

  ROS_INFO_STREAM("Get robot: " << model_nh.getNamespace() << " successfully");
  if (!model_nh.getParam("scout_wheels", wheels)){
    ROS_ERROR_STREAM("No sct_wheels specified");
    return false;
  }
  else{
    chassis2joints_.resize(wheels.size(), 2);
    size_t i = 0;
    for (const auto& wheel : wheels)
    {
      ROS_ASSERT(wheel.second.hasMember("pose"));
      ROS_ASSERT(wheel.second["pose"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(wheel.second["pose"].size() == 3);
      ROS_ASSERT(wheel.second.hasMember("wheel_radius"));

      for (const auto& transmission : transmissions){
        for (size_t i = 0; i < transmission.joints_.size(); ++i){
          std::string wheel_joint = wheel.second["joint"];
          if(!std::strcmp(transmission.joints_[i].name_.c_str(), wheel_joint.c_str())){
            TiXmlDocument actuator_xml;
            actuator_xml.Parse(transmission.actuators_[i].xml_element_.c_str());
            joint2trans_.push_back(std::stod(actuator_xml.FirstChildElement("actuator")->FirstChildElement("mechanicalReduction")->GetText()));
          }
        }
      }

      Eigen::MatrixXd chassis2joint(1, 2);
      chassis2joint << 1.0 / (double)wheel.second["wheel_radius"],
          -(double)wheel.second["pose"][1] / (double)wheel.second["wheel_radius"];

      chassis2joints_.block<1, 2>(i, 0) = chassis2joint;

      ros::NodeHandle nh_wheel = ros::NodeHandle(model_nh, "scout_wheels/" + wheel.first);

      joints_.push_back(std::make_shared<SimpleJointVelocityController>());

      if (!joints_.back()->init(&ej_interface_, nh_wheel))
        return false;
      i++;
    }
  }

  sct_command_data_.stamp = ros::Time::now();
  sct_command_data_.motion_ctl_cmd.linear_vel = 0;
  sct_command_data_.motion_ctl_cmd.angle_vel = 0;
  sct_command_data_.motion_ctl_cmd.update_cmd = false;
  sct_command_data_.ctl_mode_set_cmd.control_mode = 0;
  sct_command_data_.ctl_mode_set_cmd.update_cmd = false;
  sct_command_data_.state_set_cmd.state_set = 0;
  sct_command_data_.state_set_cmd.update_cmd = false;
  sct_command_data_.light_set_cmd.light_ctl_enable = 0;
  sct_command_data_.light_set_cmd.light_mode = sct_common::OFF;
  sct_command_data_.light_set_cmd.user_define_light = 0;
  sct_command_data_.light_set_cmd.count_check = 0;
  sct_command_data_.light_set_cmd.update_cmd = false;

  sct_common::ScoutHandle scout_handle(model_nh.getNamespace(), &sct_command_data_);
  scout_interface_.registerHandle(scout_handle);
  registerInterface(&scout_interface_);

  if(ret)
    ROS_INFO_STREAM("sct_robot_hw_sim Init successful!");
  return ret;
}

void SctRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);

  // Set cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  for (auto& cmd : joint_effort_command_)
    cmd = 0;
  for (auto& cmd : joint_velocity_command_)
    cmd = 0;
}

void SctRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  double error = sct_command_data_.motion_ctl_cmd.angle_vel - base_link_->RelativeAngularVel().Z();
  sct_command_data_.motion_ctl_cmd.angle_vel = angle_pid_controller_.computeCommand(error, period);
  if (sct_command_data_.motion_ctl_cmd.update_cmd)
  {
    Eigen::Matrix<double, 2, 1>  vel_chassis;
    vel_chassis << sct_command_data_.motion_ctl_cmd.linear_vel, sct_command_data_.motion_ctl_cmd.angle_vel;
    Eigen::VectorXd vel_joints = chassis2joints_ * vel_chassis;

    for (size_t i = 0; i < joints_.size(); i++)
    {
      joints_[i]->setCommand(vel_joints(i)*joint2trans_[i]);
      joints_[i]->update(time, period);
    }
    sct_command_data_.motion_ctl_cmd.update_cmd = false;
  }
  gazebo_ros_control::DefaultRobotHWSim::writeSim(time, period);
}

SimpleJointVelocityController::SimpleJointVelocityController() : command_(0)
{
}

bool SimpleJointVelocityController::init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Load PID Controller using gains set on parameter server
  if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;

  return true;
}

// Set the joint velocity command
void SimpleJointVelocityController::setCommand(double cmd)
{
  command_ = cmd;
}

void SimpleJointVelocityController::update(const ros::Time& time, const ros::Duration& period)
{
  double error = command_ - joint_.getVelocity();

  // Set the PID error and compute the PID command with nonuniform time
  // step size. The derivative error is computed from the change in the error
  // and the timestep dt.
  double commanded_effort = pid_controller_.computeCommand(error, period);
  joint_.setCommand(commanded_effort);
}

std::string SimpleJointVelocityController::getJointName()
{
  return joint_.getName();
}

double SimpleJointVelocityController::getVelocity()
{
  return joint_.getVelocity();
}
}  // namespace sct_gazebo

PLUGINLIB_EXPORT_CLASS(sct_gazebo::SctRobotHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)  // Default plugin
