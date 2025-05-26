// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from move_pos_interfaces:action/GoToPose.idl
// generated code does not contain a copyright notice

#ifndef MOVE_POS_INTERFACES__ACTION__DETAIL__GO_TO_POSE__BUILDER_HPP_
#define MOVE_POS_INTERFACES__ACTION__DETAIL__GO_TO_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "move_pos_interfaces/action/detail/go_to_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace move_pos_interfaces
{

namespace action
{

namespace builder
{

class Init_GoToPose_Goal_target_y
{
public:
  explicit Init_GoToPose_Goal_target_y(::move_pos_interfaces::action::GoToPose_Goal & msg)
  : msg_(msg)
  {}
  ::move_pos_interfaces::action::GoToPose_Goal target_y(::move_pos_interfaces::action::GoToPose_Goal::_target_y_type arg)
  {
    msg_.target_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_Goal msg_;
};

class Init_GoToPose_Goal_target_x
{
public:
  Init_GoToPose_Goal_target_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToPose_Goal_target_y target_x(::move_pos_interfaces::action::GoToPose_Goal::_target_x_type arg)
  {
    msg_.target_x = std::move(arg);
    return Init_GoToPose_Goal_target_y(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::move_pos_interfaces::action::GoToPose_Goal>()
{
  return move_pos_interfaces::action::builder::Init_GoToPose_Goal_target_x();
}

}  // namespace move_pos_interfaces


namespace move_pos_interfaces
{

namespace action
{

namespace builder
{

class Init_GoToPose_Result_message
{
public:
  explicit Init_GoToPose_Result_message(::move_pos_interfaces::action::GoToPose_Result & msg)
  : msg_(msg)
  {}
  ::move_pos_interfaces::action::GoToPose_Result message(::move_pos_interfaces::action::GoToPose_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_Result msg_;
};

class Init_GoToPose_Result_success
{
public:
  Init_GoToPose_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToPose_Result_message success(::move_pos_interfaces::action::GoToPose_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GoToPose_Result_message(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::move_pos_interfaces::action::GoToPose_Result>()
{
  return move_pos_interfaces::action::builder::Init_GoToPose_Result_success();
}

}  // namespace move_pos_interfaces


namespace move_pos_interfaces
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::move_pos_interfaces::action::GoToPose_Feedback>()
{
  return ::move_pos_interfaces::action::GoToPose_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace move_pos_interfaces


namespace move_pos_interfaces
{

namespace action
{

namespace builder
{

class Init_GoToPose_SendGoal_Request_goal
{
public:
  explicit Init_GoToPose_SendGoal_Request_goal(::move_pos_interfaces::action::GoToPose_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::move_pos_interfaces::action::GoToPose_SendGoal_Request goal(::move_pos_interfaces::action::GoToPose_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_SendGoal_Request msg_;
};

class Init_GoToPose_SendGoal_Request_goal_id
{
public:
  Init_GoToPose_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToPose_SendGoal_Request_goal goal_id(::move_pos_interfaces::action::GoToPose_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GoToPose_SendGoal_Request_goal(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::move_pos_interfaces::action::GoToPose_SendGoal_Request>()
{
  return move_pos_interfaces::action::builder::Init_GoToPose_SendGoal_Request_goal_id();
}

}  // namespace move_pos_interfaces


namespace move_pos_interfaces
{

namespace action
{

namespace builder
{

class Init_GoToPose_SendGoal_Response_stamp
{
public:
  explicit Init_GoToPose_SendGoal_Response_stamp(::move_pos_interfaces::action::GoToPose_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::move_pos_interfaces::action::GoToPose_SendGoal_Response stamp(::move_pos_interfaces::action::GoToPose_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_SendGoal_Response msg_;
};

class Init_GoToPose_SendGoal_Response_accepted
{
public:
  Init_GoToPose_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToPose_SendGoal_Response_stamp accepted(::move_pos_interfaces::action::GoToPose_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_GoToPose_SendGoal_Response_stamp(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::move_pos_interfaces::action::GoToPose_SendGoal_Response>()
{
  return move_pos_interfaces::action::builder::Init_GoToPose_SendGoal_Response_accepted();
}

}  // namespace move_pos_interfaces


namespace move_pos_interfaces
{

namespace action
{

namespace builder
{

class Init_GoToPose_GetResult_Request_goal_id
{
public:
  Init_GoToPose_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::move_pos_interfaces::action::GoToPose_GetResult_Request goal_id(::move_pos_interfaces::action::GoToPose_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::move_pos_interfaces::action::GoToPose_GetResult_Request>()
{
  return move_pos_interfaces::action::builder::Init_GoToPose_GetResult_Request_goal_id();
}

}  // namespace move_pos_interfaces


namespace move_pos_interfaces
{

namespace action
{

namespace builder
{

class Init_GoToPose_GetResult_Response_result
{
public:
  explicit Init_GoToPose_GetResult_Response_result(::move_pos_interfaces::action::GoToPose_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::move_pos_interfaces::action::GoToPose_GetResult_Response result(::move_pos_interfaces::action::GoToPose_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_GetResult_Response msg_;
};

class Init_GoToPose_GetResult_Response_status
{
public:
  Init_GoToPose_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToPose_GetResult_Response_result status(::move_pos_interfaces::action::GoToPose_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_GoToPose_GetResult_Response_result(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::move_pos_interfaces::action::GoToPose_GetResult_Response>()
{
  return move_pos_interfaces::action::builder::Init_GoToPose_GetResult_Response_status();
}

}  // namespace move_pos_interfaces


namespace move_pos_interfaces
{

namespace action
{

namespace builder
{

class Init_GoToPose_FeedbackMessage_feedback
{
public:
  explicit Init_GoToPose_FeedbackMessage_feedback(::move_pos_interfaces::action::GoToPose_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::move_pos_interfaces::action::GoToPose_FeedbackMessage feedback(::move_pos_interfaces::action::GoToPose_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_FeedbackMessage msg_;
};

class Init_GoToPose_FeedbackMessage_goal_id
{
public:
  Init_GoToPose_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToPose_FeedbackMessage_feedback goal_id(::move_pos_interfaces::action::GoToPose_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GoToPose_FeedbackMessage_feedback(msg_);
  }

private:
  ::move_pos_interfaces::action::GoToPose_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::move_pos_interfaces::action::GoToPose_FeedbackMessage>()
{
  return move_pos_interfaces::action::builder::Init_GoToPose_FeedbackMessage_goal_id();
}

}  // namespace move_pos_interfaces

#endif  // MOVE_POS_INTERFACES__ACTION__DETAIL__GO_TO_POSE__BUILDER_HPP_
