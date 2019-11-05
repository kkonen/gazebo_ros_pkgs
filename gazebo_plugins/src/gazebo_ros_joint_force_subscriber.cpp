// Copyright 2013 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_plugins/gazebo_ros_joint_force_subscriber.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosJointForceSubscriberPrivate
{
public:
  /// A pointer to the joint, where force is applied
  gazebo::physics::JointPtr joint_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Wrench subscriber
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr float_sub_;

  /// Container for the joint force that this plugin exerts on the joint.
  std_msgs::msg::Float32 force_;
  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;
};

GazeboRosJointForceSubscriber::GazeboRosJointForceSubscriber()
: impl_(std::make_unique<GazeboRosJointForceSubscriberPrivate>())
{
}

GazeboRosJointForceSubscriber::~GazeboRosJointForceSubscriber()
{
}

void GazeboRosJointForceSubscriber::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{

  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  auto logger = rclcpp::get_logger("gazebo_ros_joint_force_subscriber");

  // Target joint
  if (!sdf->HasElement("joint_name")) {
    RCLCPP_ERROR(logger, "Joint Force Subscriber plugin missing <joint_name>, cannot proceed");
    return;
  }

  auto joint_name = sdf->GetElement("joint_name")->Get<std::string>();

  impl_->joint_ = model->GetJoint(joint_name);
  if (!impl_->joint_) {
    RCLCPP_ERROR(logger, "Joint named: %s does not exist\n", joint_name.c_str());
    return;
  }

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Going to create force subscriber for joint [%s]",
    joint_name.c_str() );
  // Subscribe to float messages
  impl_->force_.data = 0.0;
  impl_->float_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(
    "gazebo_ros_joint_force_subscriber", std::bind(&GazeboRosJointForceSubscriber::OnRosFloat32Msg, this,
    std::placeholders::_1));

//impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
//    "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)),
//    std::bind(&GazeboRosPlanarMovePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosJointForceSubscriber::OnUpdate, this));
}

void GazeboRosJointForceSubscriber::OnRosFloat32Msg(const std_msgs::msg::Float32::SharedPtr msg)
{
  impl_->force_ = *msg;
}

void GazeboRosJointForceSubscriber::OnUpdate()
{
  impl_->joint_->SetForce(0, impl_->force_.data);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosJointForceSubscriber)
}  // namespace gazebo_plugins

