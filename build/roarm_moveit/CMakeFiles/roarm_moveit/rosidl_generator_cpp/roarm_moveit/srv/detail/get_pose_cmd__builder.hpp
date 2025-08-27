// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from roarm_moveit:srv/GetPoseCmd.idl
// generated code does not contain a copyright notice

#ifndef ROARM_MOVEIT__SRV__DETAIL__GET_POSE_CMD__BUILDER_HPP_
#define ROARM_MOVEIT__SRV__DETAIL__GET_POSE_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "roarm_moveit/srv/detail/get_pose_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace roarm_moveit
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_moveit::srv::GetPoseCmd_Request>()
{
  return ::roarm_moveit::srv::GetPoseCmd_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace roarm_moveit


namespace roarm_moveit
{

namespace srv
{

namespace builder
{

class Init_GetPoseCmd_Response_z
{
public:
  explicit Init_GetPoseCmd_Response_z(::roarm_moveit::srv::GetPoseCmd_Response & msg)
  : msg_(msg)
  {}
  ::roarm_moveit::srv::GetPoseCmd_Response z(::roarm_moveit::srv::GetPoseCmd_Response::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_moveit::srv::GetPoseCmd_Response msg_;
};

class Init_GetPoseCmd_Response_y
{
public:
  explicit Init_GetPoseCmd_Response_y(::roarm_moveit::srv::GetPoseCmd_Response & msg)
  : msg_(msg)
  {}
  Init_GetPoseCmd_Response_z y(::roarm_moveit::srv::GetPoseCmd_Response::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_GetPoseCmd_Response_z(msg_);
  }

private:
  ::roarm_moveit::srv::GetPoseCmd_Response msg_;
};

class Init_GetPoseCmd_Response_x
{
public:
  Init_GetPoseCmd_Response_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPoseCmd_Response_y x(::roarm_moveit::srv::GetPoseCmd_Response::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GetPoseCmd_Response_y(msg_);
  }

private:
  ::roarm_moveit::srv::GetPoseCmd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_moveit::srv::GetPoseCmd_Response>()
{
  return roarm_moveit::srv::builder::Init_GetPoseCmd_Response_x();
}

}  // namespace roarm_moveit

#endif  // ROARM_MOVEIT__SRV__DETAIL__GET_POSE_CMD__BUILDER_HPP_
