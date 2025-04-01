// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from nav2_sms_behavior:action/SendSms.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "nav2_sms_behavior/action/detail/send_sms__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace nav2_sms_behavior
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _SendSms_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SendSms_Goal_type_support_ids_t;

static const _SendSms_Goal_type_support_ids_t _SendSms_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SendSms_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SendSms_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SendSms_Goal_type_support_symbol_names_t _SendSms_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nav2_sms_behavior, action, SendSms_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, nav2_sms_behavior, action, SendSms_Goal)),
  }
};

typedef struct _SendSms_Goal_type_support_data_t
{
  void * data[2];
} _SendSms_Goal_type_support_data_t;

static _SendSms_Goal_type_support_data_t _SendSms_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SendSms_Goal_message_typesupport_map = {
  2,
  "nav2_sms_behavior",
  &_SendSms_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_SendSms_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_SendSms_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SendSms_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SendSms_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace nav2_sms_behavior

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<nav2_sms_behavior::action::SendSms_Goal>()
{
  return &::nav2_sms_behavior::action::rosidl_typesupport_cpp::SendSms_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, nav2_sms_behavior, action, SendSms_Goal)() {
  return get_message_type_support_handle<nav2_sms_behavior::action::SendSms_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "nav2_sms_behavior/action/detail/send_sms__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace nav2_sms_behavior
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _SendSms_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SendSms_Result_type_support_ids_t;

static const _SendSms_Result_type_support_ids_t _SendSms_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SendSms_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SendSms_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SendSms_Result_type_support_symbol_names_t _SendSms_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nav2_sms_behavior, action, SendSms_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, nav2_sms_behavior, action, SendSms_Result)),
  }
};

typedef struct _SendSms_Result_type_support_data_t
{
  void * data[2];
} _SendSms_Result_type_support_data_t;

static _SendSms_Result_type_support_data_t _SendSms_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SendSms_Result_message_typesupport_map = {
  2,
  "nav2_sms_behavior",
  &_SendSms_Result_message_typesupport_ids.typesupport_identifier[0],
  &_SendSms_Result_message_typesupport_symbol_names.symbol_name[0],
  &_SendSms_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SendSms_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SendSms_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace nav2_sms_behavior

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<nav2_sms_behavior::action::SendSms_Result>()
{
  return &::nav2_sms_behavior::action::rosidl_typesupport_cpp::SendSms_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, nav2_sms_behavior, action, SendSms_Result)() {
  return get_message_type_support_handle<nav2_sms_behavior::action::SendSms_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "nav2_sms_behavior/action/detail/send_sms__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace nav2_sms_behavior
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _SendSms_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SendSms_Feedback_type_support_ids_t;

static const _SendSms_Feedback_type_support_ids_t _SendSms_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SendSms_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SendSms_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SendSms_Feedback_type_support_symbol_names_t _SendSms_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nav2_sms_behavior, action, SendSms_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, nav2_sms_behavior, action, SendSms_Feedback)),
  }
};

typedef struct _SendSms_Feedback_type_support_data_t
{
  void * data[2];
} _SendSms_Feedback_type_support_data_t;

static _SendSms_Feedback_type_support_data_t _SendSms_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SendSms_Feedback_message_typesupport_map = {
  2,
  "nav2_sms_behavior",
  &_SendSms_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_SendSms_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_SendSms_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SendSms_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SendSms_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace nav2_sms_behavior

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<nav2_sms_behavior::action::SendSms_Feedback>()
{
  return &::nav2_sms_behavior::action::rosidl_typesupport_cpp::SendSms_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, nav2_sms_behavior, action, SendSms_Feedback)() {
  return get_message_type_support_handle<nav2_sms_behavior::action::SendSms_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "nav2_sms_behavior/action/detail/send_sms__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace nav2_sms_behavior
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _SendSms_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SendSms_SendGoal_Request_type_support_ids_t;

static const _SendSms_SendGoal_Request_type_support_ids_t _SendSms_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SendSms_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SendSms_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SendSms_SendGoal_Request_type_support_symbol_names_t _SendSms_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nav2_sms_behavior, action, SendSms_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, nav2_sms_behavior, action, SendSms_SendGoal_Request)),
  }
};

typedef struct _SendSms_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _SendSms_SendGoal_Request_type_support_data_t;

static _SendSms_SendGoal_Request_type_support_data_t _SendSms_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SendSms_SendGoal_Request_message_typesupport_map = {
  2,
  "nav2_sms_behavior",
  &_SendSms_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SendSms_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SendSms_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SendSms_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SendSms_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace nav2_sms_behavior

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<nav2_sms_behavior::action::SendSms_SendGoal_Request>()
{
  return &::nav2_sms_behavior::action::rosidl_typesupport_cpp::SendSms_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, nav2_sms_behavior, action, SendSms_SendGoal_Request)() {
  return get_message_type_support_handle<nav2_sms_behavior::action::SendSms_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "nav2_sms_behavior/action/detail/send_sms__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace nav2_sms_behavior
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _SendSms_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SendSms_SendGoal_Response_type_support_ids_t;

static const _SendSms_SendGoal_Response_type_support_ids_t _SendSms_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SendSms_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SendSms_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SendSms_SendGoal_Response_type_support_symbol_names_t _SendSms_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nav2_sms_behavior, action, SendSms_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, nav2_sms_behavior, action, SendSms_SendGoal_Response)),
  }
};

typedef struct _SendSms_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _SendSms_SendGoal_Response_type_support_data_t;

static _SendSms_SendGoal_Response_type_support_data_t _SendSms_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SendSms_SendGoal_Response_message_typesupport_map = {
  2,
  "nav2_sms_behavior",
  &_SendSms_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SendSms_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SendSms_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SendSms_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SendSms_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace nav2_sms_behavior

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<nav2_sms_behavior::action::SendSms_SendGoal_Response>()
{
  return &::nav2_sms_behavior::action::rosidl_typesupport_cpp::SendSms_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, nav2_sms_behavior, action, SendSms_SendGoal_Response)() {
  return get_message_type_support_handle<nav2_sms_behavior::action::SendSms_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "nav2_sms_behavior/action/detail/send_sms__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace nav2_sms_behavior
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _SendSms_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SendSms_SendGoal_type_support_ids_t;

static const _SendSms_SendGoal_type_support_ids_t _SendSms_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SendSms_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SendSms_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SendSms_SendGoal_type_support_symbol_names_t _SendSms_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nav2_sms_behavior, action, SendSms_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, nav2_sms_behavior, action, SendSms_SendGoal)),
  }
};

typedef struct _SendSms_SendGoal_type_support_data_t
{
  void * data[2];
} _SendSms_SendGoal_type_support_data_t;

static _SendSms_SendGoal_type_support_data_t _SendSms_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SendSms_SendGoal_service_typesupport_map = {
  2,
  "nav2_sms_behavior",
  &_SendSms_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_SendSms_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_SendSms_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SendSms_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SendSms_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace nav2_sms_behavior

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<nav2_sms_behavior::action::SendSms_SendGoal>()
{
  return &::nav2_sms_behavior::action::rosidl_typesupport_cpp::SendSms_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, nav2_sms_behavior, action, SendSms_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<nav2_sms_behavior::action::SendSms_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "nav2_sms_behavior/action/detail/send_sms__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace nav2_sms_behavior
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _SendSms_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SendSms_GetResult_Request_type_support_ids_t;

static const _SendSms_GetResult_Request_type_support_ids_t _SendSms_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SendSms_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SendSms_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SendSms_GetResult_Request_type_support_symbol_names_t _SendSms_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nav2_sms_behavior, action, SendSms_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, nav2_sms_behavior, action, SendSms_GetResult_Request)),
  }
};

typedef struct _SendSms_GetResult_Request_type_support_data_t
{
  void * data[2];
} _SendSms_GetResult_Request_type_support_data_t;

static _SendSms_GetResult_Request_type_support_data_t _SendSms_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SendSms_GetResult_Request_message_typesupport_map = {
  2,
  "nav2_sms_behavior",
  &_SendSms_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SendSms_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SendSms_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SendSms_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SendSms_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace nav2_sms_behavior

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<nav2_sms_behavior::action::SendSms_GetResult_Request>()
{
  return &::nav2_sms_behavior::action::rosidl_typesupport_cpp::SendSms_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, nav2_sms_behavior, action, SendSms_GetResult_Request)() {
  return get_message_type_support_handle<nav2_sms_behavior::action::SendSms_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "nav2_sms_behavior/action/detail/send_sms__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace nav2_sms_behavior
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _SendSms_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SendSms_GetResult_Response_type_support_ids_t;

static const _SendSms_GetResult_Response_type_support_ids_t _SendSms_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SendSms_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SendSms_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SendSms_GetResult_Response_type_support_symbol_names_t _SendSms_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nav2_sms_behavior, action, SendSms_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, nav2_sms_behavior, action, SendSms_GetResult_Response)),
  }
};

typedef struct _SendSms_GetResult_Response_type_support_data_t
{
  void * data[2];
} _SendSms_GetResult_Response_type_support_data_t;

static _SendSms_GetResult_Response_type_support_data_t _SendSms_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SendSms_GetResult_Response_message_typesupport_map = {
  2,
  "nav2_sms_behavior",
  &_SendSms_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SendSms_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SendSms_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SendSms_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SendSms_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace nav2_sms_behavior

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<nav2_sms_behavior::action::SendSms_GetResult_Response>()
{
  return &::nav2_sms_behavior::action::rosidl_typesupport_cpp::SendSms_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, nav2_sms_behavior, action, SendSms_GetResult_Response)() {
  return get_message_type_support_handle<nav2_sms_behavior::action::SendSms_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "nav2_sms_behavior/action/detail/send_sms__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace nav2_sms_behavior
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _SendSms_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SendSms_GetResult_type_support_ids_t;

static const _SendSms_GetResult_type_support_ids_t _SendSms_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SendSms_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SendSms_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SendSms_GetResult_type_support_symbol_names_t _SendSms_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nav2_sms_behavior, action, SendSms_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, nav2_sms_behavior, action, SendSms_GetResult)),
  }
};

typedef struct _SendSms_GetResult_type_support_data_t
{
  void * data[2];
} _SendSms_GetResult_type_support_data_t;

static _SendSms_GetResult_type_support_data_t _SendSms_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SendSms_GetResult_service_typesupport_map = {
  2,
  "nav2_sms_behavior",
  &_SendSms_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_SendSms_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_SendSms_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SendSms_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SendSms_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace nav2_sms_behavior

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<nav2_sms_behavior::action::SendSms_GetResult>()
{
  return &::nav2_sms_behavior::action::rosidl_typesupport_cpp::SendSms_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, nav2_sms_behavior, action, SendSms_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<nav2_sms_behavior::action::SendSms_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "nav2_sms_behavior/action/detail/send_sms__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace nav2_sms_behavior
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _SendSms_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SendSms_FeedbackMessage_type_support_ids_t;

static const _SendSms_FeedbackMessage_type_support_ids_t _SendSms_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SendSms_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SendSms_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SendSms_FeedbackMessage_type_support_symbol_names_t _SendSms_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nav2_sms_behavior, action, SendSms_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, nav2_sms_behavior, action, SendSms_FeedbackMessage)),
  }
};

typedef struct _SendSms_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _SendSms_FeedbackMessage_type_support_data_t;

static _SendSms_FeedbackMessage_type_support_data_t _SendSms_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SendSms_FeedbackMessage_message_typesupport_map = {
  2,
  "nav2_sms_behavior",
  &_SendSms_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_SendSms_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_SendSms_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SendSms_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SendSms_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace nav2_sms_behavior

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<nav2_sms_behavior::action::SendSms_FeedbackMessage>()
{
  return &::nav2_sms_behavior::action::rosidl_typesupport_cpp::SendSms_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, nav2_sms_behavior, action, SendSms_FeedbackMessage)() {
  return get_message_type_support_handle<nav2_sms_behavior::action::SendSms_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "nav2_sms_behavior/action/detail/send_sms__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace nav2_sms_behavior
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t SendSms_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace nav2_sms_behavior

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<nav2_sms_behavior::action::SendSms>()
{
  using ::nav2_sms_behavior::action::rosidl_typesupport_cpp::SendSms_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  SendSms_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::nav2_sms_behavior::action::SendSms::Impl::SendGoalService>();
  SendSms_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::nav2_sms_behavior::action::SendSms::Impl::GetResultService>();
  SendSms_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::nav2_sms_behavior::action::SendSms::Impl::CancelGoalService>();
  SendSms_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::nav2_sms_behavior::action::SendSms::Impl::FeedbackMessage>();
  SendSms_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::nav2_sms_behavior::action::SendSms::Impl::GoalStatusMessage>();
  return &SendSms_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, nav2_sms_behavior, action, SendSms)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<nav2_sms_behavior::action::SendSms>();
}

#ifdef __cplusplus
}
#endif
