// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from roarm_moveit:srv/MovePointCmd.idl
// generated code does not contain a copyright notice

#ifndef ROARM_MOVEIT__SRV__DETAIL__MOVE_POINT_CMD__BUILDER_HPP_
#define ROARM_MOVEIT__SRV__DETAIL__MOVE_POINT_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "roarm_moveit/srv/detail/move_point_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace roarm_moveit
{

namespace srv
{

namespace builder
{

class Init_MovePointCmd_Request_z
{
public:
  explicit Init_MovePointCmd_Request_z(::roarm_moveit::srv::MovePointCmd_Request & msg)
  : msg_(msg)
  {}
  ::roarm_moveit::srv::MovePointCmd_Request z(::roarm_moveit::srv::MovePointCmd_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_moveit::srv::MovePointCmd_Request msg_;
};

class Init_MovePointCmd_Request_y
{
public:
  explicit Init_MovePointCmd_Request_y(::roarm_moveit::srv::MovePointCmd_Request & msg)
  : msg_(msg)
  {}
  Init_MovePointCmd_Request_z y(::roarm_moveit::srv::MovePointCmd_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_MovePointCmd_Request_z(msg_);
  }

private:
  ::roarm_moveit::srv::MovePointCmd_Request msg_;
};

class Init_MovePointCmd_Request_x
{
public:
  Init_MovePointCmd_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovePointCmd_Request_y x(::roarm_moveit::srv::MovePointCmd_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MovePointCmd_Request_y(msg_);
  }

private:
  ::roarm_moveit::srv::MovePointCmd_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_moveit::srv::MovePointCmd_Request>()
{
  return roarm_moveit::srv::builder::Init_MovePointCmd_Request_x();
}

}  // namespace roarm_moveit


namespace roarm_moveit
{

namespace srv
{

namespace builder
{

class Init_MovePointCmd_Response_message
{
public:
  explicit Init_MovePointCmd_Response_message(::roarm_moveit::srv::MovePointCmd_Response & msg)
  : msg_(msg)
  {}
  ::roarm_moveit::srv::MovePointCmd_Response message(::roarm_moveit::srv::MovePointCmd_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_moveit::srv::MovePointCmd_Response msg_;
};

class Init_MovePointCmd_Response_success
{
public:
  Init_MovePointCmd_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovePointCmd_Response_message success(::roarm_moveit::srv::MovePointCmd_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MovePointCmd_Response_message(msg_);
  }

private:
  ::roarm_moveit::srv::MovePointCmd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_moveit::srv::MovePointCmd_Response>()
{
  return roarm_moveit::srv::builder::Init_MovePointCmd_Response_success();
}

}  // namespace roarm_moveit

#endif  // ROARM_MOVEIT__SRV__DETAIL__MOVE_POINT_CMD__BUILDER_HPP_
