// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from move_pos_interfaces:action/GoToPose.idl
// generated code does not contain a copyright notice

#ifndef MOVE_POS_INTERFACES__ACTION__DETAIL__GO_TO_POSE__STRUCT_H_
#define MOVE_POS_INTERFACES__ACTION__DETAIL__GO_TO_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/GoToPose in the package move_pos_interfaces.
typedef struct move_pos_interfaces__action__GoToPose_Goal
{
  float target_x;
  float target_y;
} move_pos_interfaces__action__GoToPose_Goal;

// Struct for a sequence of move_pos_interfaces__action__GoToPose_Goal.
typedef struct move_pos_interfaces__action__GoToPose_Goal__Sequence
{
  move_pos_interfaces__action__GoToPose_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} move_pos_interfaces__action__GoToPose_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/GoToPose in the package move_pos_interfaces.
typedef struct move_pos_interfaces__action__GoToPose_Result
{
  bool success;
  rosidl_runtime_c__String message;
} move_pos_interfaces__action__GoToPose_Result;

// Struct for a sequence of move_pos_interfaces__action__GoToPose_Result.
typedef struct move_pos_interfaces__action__GoToPose_Result__Sequence
{
  move_pos_interfaces__action__GoToPose_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} move_pos_interfaces__action__GoToPose_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/GoToPose in the package move_pos_interfaces.
typedef struct move_pos_interfaces__action__GoToPose_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} move_pos_interfaces__action__GoToPose_Feedback;

// Struct for a sequence of move_pos_interfaces__action__GoToPose_Feedback.
typedef struct move_pos_interfaces__action__GoToPose_Feedback__Sequence
{
  move_pos_interfaces__action__GoToPose_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} move_pos_interfaces__action__GoToPose_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "move_pos_interfaces/action/detail/go_to_pose__struct.h"

/// Struct defined in action/GoToPose in the package move_pos_interfaces.
typedef struct move_pos_interfaces__action__GoToPose_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  move_pos_interfaces__action__GoToPose_Goal goal;
} move_pos_interfaces__action__GoToPose_SendGoal_Request;

// Struct for a sequence of move_pos_interfaces__action__GoToPose_SendGoal_Request.
typedef struct move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence
{
  move_pos_interfaces__action__GoToPose_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/GoToPose in the package move_pos_interfaces.
typedef struct move_pos_interfaces__action__GoToPose_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} move_pos_interfaces__action__GoToPose_SendGoal_Response;

// Struct for a sequence of move_pos_interfaces__action__GoToPose_SendGoal_Response.
typedef struct move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence
{
  move_pos_interfaces__action__GoToPose_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/GoToPose in the package move_pos_interfaces.
typedef struct move_pos_interfaces__action__GoToPose_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} move_pos_interfaces__action__GoToPose_GetResult_Request;

// Struct for a sequence of move_pos_interfaces__action__GoToPose_GetResult_Request.
typedef struct move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence
{
  move_pos_interfaces__action__GoToPose_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "move_pos_interfaces/action/detail/go_to_pose__struct.h"

/// Struct defined in action/GoToPose in the package move_pos_interfaces.
typedef struct move_pos_interfaces__action__GoToPose_GetResult_Response
{
  int8_t status;
  move_pos_interfaces__action__GoToPose_Result result;
} move_pos_interfaces__action__GoToPose_GetResult_Response;

// Struct for a sequence of move_pos_interfaces__action__GoToPose_GetResult_Response.
typedef struct move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence
{
  move_pos_interfaces__action__GoToPose_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "move_pos_interfaces/action/detail/go_to_pose__struct.h"

/// Struct defined in action/GoToPose in the package move_pos_interfaces.
typedef struct move_pos_interfaces__action__GoToPose_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  move_pos_interfaces__action__GoToPose_Feedback feedback;
} move_pos_interfaces__action__GoToPose_FeedbackMessage;

// Struct for a sequence of move_pos_interfaces__action__GoToPose_FeedbackMessage.
typedef struct move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence
{
  move_pos_interfaces__action__GoToPose_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOVE_POS_INTERFACES__ACTION__DETAIL__GO_TO_POSE__STRUCT_H_
