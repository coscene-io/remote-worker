// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from roarm_moveit:srv/MoveCircleCmd.idl
// generated code does not contain a copyright notice

#ifndef ROARM_MOVEIT__SRV__DETAIL__MOVE_CIRCLE_CMD__BUILDER_HPP_
#define ROARM_MOVEIT__SRV__DETAIL__MOVE_CIRCLE_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "roarm_moveit/srv/detail/move_circle_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace roarm_moveit
{

namespace srv
{

namespace builder
{

class Init_MoveCircleCmd_Request_radius
{
public:
  explicit Init_MoveCircleCmd_Request_radius(::roarm_moveit::srv::MoveCircleCmd_Request & msg)
  : msg_(msg)
  {}
  ::roarm_moveit::srv::MoveCircleCmd_Request radius(::roarm_moveit::srv::MoveCircleCmd_Request::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_moveit::srv::MoveCircleCmd_Request msg_;
};

class Init_MoveCircleCmd_Request_z
{
public:
  explicit Init_MoveCircleCmd_Request_z(::roarm_moveit::srv::MoveCircleCmd_Request & msg)
  : msg_(msg)
  {}
  Init_MoveCircleCmd_Request_radius z(::roarm_moveit::srv::MoveCircleCmd_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_MoveCircleCmd_Request_radius(msg_);
  }

private:
  ::roarm_moveit::srv::MoveCircleCmd_Request msg_;
};

class Init_MoveCircleCmd_Request_y
{
public:
  explicit Init_MoveCircleCmd_Request_y(::roarm_moveit::srv::MoveCircleCmd_Request & msg)
  : msg_(msg)
  {}
  Init_MoveCircleCmd_Request_z y(::roarm_moveit::srv::MoveCircleCmd_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_MoveCircleCmd_Request_z(msg_);
  }

private:
  ::roarm_moveit::srv::MoveCircleCmd_Request msg_;
};

class Init_MoveCircleCmd_Request_x
{
public:
  Init_MoveCircleCmd_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveCircleCmd_Request_y x(::roarm_moveit::srv::MoveCircleCmd_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MoveCircleCmd_Request_y(msg_);
  }

private:
  ::roarm_moveit::srv::MoveCircleCmd_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_moveit::srv::MoveCircleCmd_Request>()
{
  return roarm_moveit::srv::builder::Init_MoveCircleCmd_Request_x();
}

}  // namespace roarm_moveit


namespace roarm_moveit
{

namespace srv
{

namespace builder
{

class Init_MoveCircleCmd_Response_message
{
public:
  explicit Init_MoveCircleCmd_Response_message(::roarm_moveit::srv::MoveCircleCmd_Response & msg)
  : msg_(msg)
  {}
  ::roarm_moveit::srv::MoveCircleCmd_Response message(::roarm_moveit::srv::MoveCircleCmd_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_moveit::srv::MoveCircleCmd_Response msg_;
};

class Init_MoveCircleCmd_Response_success
{
public:
  Init_MoveCircleCmd_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveCircleCmd_Response_message success(::roarm_moveit::srv::MoveCircleCmd_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MoveCircleCmd_Response_message(msg_);
  }

private:
  ::roarm_moveit::srv::MoveCircleCmd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_moveit::srv::MoveCircleCmd_Response>()
{
  return roarm_moveit::srv::builder::Init_MoveCircleCmd_Response_success();
}

}  // namespace roarm_moveit

#endif  // ROARM_MOVEIT__SRV__DETAIL__MOVE_CIRCLE_CMD__BUILDER_HPP_
