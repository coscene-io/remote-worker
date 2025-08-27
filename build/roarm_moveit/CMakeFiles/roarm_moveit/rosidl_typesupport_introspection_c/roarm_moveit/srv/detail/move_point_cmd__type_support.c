// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from roarm_moveit:srv/MovePointCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "roarm_moveit/srv/detail/move_point_cmd__rosidl_typesupport_introspection_c.h"
#include "roarm_moveit/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "roarm_moveit/srv/detail/move_point_cmd__functions.h"
#include "roarm_moveit/srv/detail/move_point_cmd__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void roarm_moveit__srv__MovePointCmd_Request__rosidl_typesupport_introspection_c__MovePointCmd_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  roarm_moveit__srv__MovePointCmd_Request__init(message_memory);
}

void roarm_moveit__srv__MovePointCmd_Request__rosidl_typesupport_introspection_c__MovePointCmd_Request_fini_function(void * message_memory)
{
  roarm_moveit__srv__MovePointCmd_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember roarm_moveit__srv__MovePointCmd_Request__rosidl_typesupport_introspection_c__MovePointCmd_Request_message_member_array[3] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_moveit__srv__MovePointCmd_Request, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_moveit__srv__MovePointCmd_Request, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_moveit__srv__MovePointCmd_Request, z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers roarm_moveit__srv__MovePointCmd_Request__rosidl_typesupport_introspection_c__MovePointCmd_Request_message_members = {
  "roarm_moveit__srv",  // message namespace
  "MovePointCmd_Request",  // message name
  3,  // number of fields
  sizeof(roarm_moveit__srv__MovePointCmd_Request),
  roarm_moveit__srv__MovePointCmd_Request__rosidl_typesupport_introspection_c__MovePointCmd_Request_message_member_array,  // message members
  roarm_moveit__srv__MovePointCmd_Request__rosidl_typesupport_introspection_c__MovePointCmd_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  roarm_moveit__srv__MovePointCmd_Request__rosidl_typesupport_introspection_c__MovePointCmd_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t roarm_moveit__srv__MovePointCmd_Request__rosidl_typesupport_introspection_c__MovePointCmd_Request_message_type_support_handle = {
  0,
  &roarm_moveit__srv__MovePointCmd_Request__rosidl_typesupport_introspection_c__MovePointCmd_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roarm_moveit
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_moveit, srv, MovePointCmd_Request)() {
  if (!roarm_moveit__srv__MovePointCmd_Request__rosidl_typesupport_introspection_c__MovePointCmd_Request_message_type_support_handle.typesupport_identifier) {
    roarm_moveit__srv__MovePointCmd_Request__rosidl_typesupport_introspection_c__MovePointCmd_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &roarm_moveit__srv__MovePointCmd_Request__rosidl_typesupport_introspection_c__MovePointCmd_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "roarm_moveit/srv/detail/move_point_cmd__rosidl_typesupport_introspection_c.h"
// already included above
// #include "roarm_moveit/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "roarm_moveit/srv/detail/move_point_cmd__functions.h"
// already included above
// #include "roarm_moveit/srv/detail/move_point_cmd__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void roarm_moveit__srv__MovePointCmd_Response__rosidl_typesupport_introspection_c__MovePointCmd_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  roarm_moveit__srv__MovePointCmd_Response__init(message_memory);
}

void roarm_moveit__srv__MovePointCmd_Response__rosidl_typesupport_introspection_c__MovePointCmd_Response_fini_function(void * message_memory)
{
  roarm_moveit__srv__MovePointCmd_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember roarm_moveit__srv__MovePointCmd_Response__rosidl_typesupport_introspection_c__MovePointCmd_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_moveit__srv__MovePointCmd_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_moveit__srv__MovePointCmd_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers roarm_moveit__srv__MovePointCmd_Response__rosidl_typesupport_introspection_c__MovePointCmd_Response_message_members = {
  "roarm_moveit__srv",  // message namespace
  "MovePointCmd_Response",  // message name
  2,  // number of fields
  sizeof(roarm_moveit__srv__MovePointCmd_Response),
  roarm_moveit__srv__MovePointCmd_Response__rosidl_typesupport_introspection_c__MovePointCmd_Response_message_member_array,  // message members
  roarm_moveit__srv__MovePointCmd_Response__rosidl_typesupport_introspection_c__MovePointCmd_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  roarm_moveit__srv__MovePointCmd_Response__rosidl_typesupport_introspection_c__MovePointCmd_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t roarm_moveit__srv__MovePointCmd_Response__rosidl_typesupport_introspection_c__MovePointCmd_Response_message_type_support_handle = {
  0,
  &roarm_moveit__srv__MovePointCmd_Response__rosidl_typesupport_introspection_c__MovePointCmd_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roarm_moveit
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_moveit, srv, MovePointCmd_Response)() {
  if (!roarm_moveit__srv__MovePointCmd_Response__rosidl_typesupport_introspection_c__MovePointCmd_Response_message_type_support_handle.typesupport_identifier) {
    roarm_moveit__srv__MovePointCmd_Response__rosidl_typesupport_introspection_c__MovePointCmd_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &roarm_moveit__srv__MovePointCmd_Response__rosidl_typesupport_introspection_c__MovePointCmd_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "roarm_moveit/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "roarm_moveit/srv/detail/move_point_cmd__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers roarm_moveit__srv__detail__move_point_cmd__rosidl_typesupport_introspection_c__MovePointCmd_service_members = {
  "roarm_moveit__srv",  // service namespace
  "MovePointCmd",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // roarm_moveit__srv__detail__move_point_cmd__rosidl_typesupport_introspection_c__MovePointCmd_Request_message_type_support_handle,
  NULL  // response message
  // roarm_moveit__srv__detail__move_point_cmd__rosidl_typesupport_introspection_c__MovePointCmd_Response_message_type_support_handle
};

static rosidl_service_type_support_t roarm_moveit__srv__detail__move_point_cmd__rosidl_typesupport_introspection_c__MovePointCmd_service_type_support_handle = {
  0,
  &roarm_moveit__srv__detail__move_point_cmd__rosidl_typesupport_introspection_c__MovePointCmd_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_moveit, srv, MovePointCmd_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_moveit, srv, MovePointCmd_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roarm_moveit
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_moveit, srv, MovePointCmd)() {
  if (!roarm_moveit__srv__detail__move_point_cmd__rosidl_typesupport_introspection_c__MovePointCmd_service_type_support_handle.typesupport_identifier) {
    roarm_moveit__srv__detail__move_point_cmd__rosidl_typesupport_introspection_c__MovePointCmd_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)roarm_moveit__srv__detail__move_point_cmd__rosidl_typesupport_introspection_c__MovePointCmd_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_moveit, srv, MovePointCmd_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_moveit, srv, MovePointCmd_Response)()->data;
  }

  return &roarm_moveit__srv__detail__move_point_cmd__rosidl_typesupport_introspection_c__MovePointCmd_service_type_support_handle;
}
