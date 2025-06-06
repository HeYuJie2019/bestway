// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from move_pos_interfaces:action/GoToPose.idl
// generated code does not contain a copyright notice
#include "move_pos_interfaces/action/detail/go_to_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
move_pos_interfaces__action__GoToPose_Goal__init(move_pos_interfaces__action__GoToPose_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // target_x
  // target_y
  return true;
}

void
move_pos_interfaces__action__GoToPose_Goal__fini(move_pos_interfaces__action__GoToPose_Goal * msg)
{
  if (!msg) {
    return;
  }
  // target_x
  // target_y
}

bool
move_pos_interfaces__action__GoToPose_Goal__are_equal(const move_pos_interfaces__action__GoToPose_Goal * lhs, const move_pos_interfaces__action__GoToPose_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_x
  if (lhs->target_x != rhs->target_x) {
    return false;
  }
  // target_y
  if (lhs->target_y != rhs->target_y) {
    return false;
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_Goal__copy(
  const move_pos_interfaces__action__GoToPose_Goal * input,
  move_pos_interfaces__action__GoToPose_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // target_x
  output->target_x = input->target_x;
  // target_y
  output->target_y = input->target_y;
  return true;
}

move_pos_interfaces__action__GoToPose_Goal *
move_pos_interfaces__action__GoToPose_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_Goal * msg = (move_pos_interfaces__action__GoToPose_Goal *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(move_pos_interfaces__action__GoToPose_Goal));
  bool success = move_pos_interfaces__action__GoToPose_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
move_pos_interfaces__action__GoToPose_Goal__destroy(move_pos_interfaces__action__GoToPose_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    move_pos_interfaces__action__GoToPose_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
move_pos_interfaces__action__GoToPose_Goal__Sequence__init(move_pos_interfaces__action__GoToPose_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_Goal * data = NULL;

  if (size) {
    data = (move_pos_interfaces__action__GoToPose_Goal *)allocator.zero_allocate(size, sizeof(move_pos_interfaces__action__GoToPose_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = move_pos_interfaces__action__GoToPose_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        move_pos_interfaces__action__GoToPose_Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
move_pos_interfaces__action__GoToPose_Goal__Sequence__fini(move_pos_interfaces__action__GoToPose_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      move_pos_interfaces__action__GoToPose_Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

move_pos_interfaces__action__GoToPose_Goal__Sequence *
move_pos_interfaces__action__GoToPose_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_Goal__Sequence * array = (move_pos_interfaces__action__GoToPose_Goal__Sequence *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = move_pos_interfaces__action__GoToPose_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
move_pos_interfaces__action__GoToPose_Goal__Sequence__destroy(move_pos_interfaces__action__GoToPose_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    move_pos_interfaces__action__GoToPose_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
move_pos_interfaces__action__GoToPose_Goal__Sequence__are_equal(const move_pos_interfaces__action__GoToPose_Goal__Sequence * lhs, const move_pos_interfaces__action__GoToPose_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_Goal__Sequence__copy(
  const move_pos_interfaces__action__GoToPose_Goal__Sequence * input,
  move_pos_interfaces__action__GoToPose_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(move_pos_interfaces__action__GoToPose_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    move_pos_interfaces__action__GoToPose_Goal * data =
      (move_pos_interfaces__action__GoToPose_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!move_pos_interfaces__action__GoToPose_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          move_pos_interfaces__action__GoToPose_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
move_pos_interfaces__action__GoToPose_Result__init(move_pos_interfaces__action__GoToPose_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    move_pos_interfaces__action__GoToPose_Result__fini(msg);
    return false;
  }
  return true;
}

void
move_pos_interfaces__action__GoToPose_Result__fini(move_pos_interfaces__action__GoToPose_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
move_pos_interfaces__action__GoToPose_Result__are_equal(const move_pos_interfaces__action__GoToPose_Result * lhs, const move_pos_interfaces__action__GoToPose_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_Result__copy(
  const move_pos_interfaces__action__GoToPose_Result * input,
  move_pos_interfaces__action__GoToPose_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

move_pos_interfaces__action__GoToPose_Result *
move_pos_interfaces__action__GoToPose_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_Result * msg = (move_pos_interfaces__action__GoToPose_Result *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(move_pos_interfaces__action__GoToPose_Result));
  bool success = move_pos_interfaces__action__GoToPose_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
move_pos_interfaces__action__GoToPose_Result__destroy(move_pos_interfaces__action__GoToPose_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    move_pos_interfaces__action__GoToPose_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
move_pos_interfaces__action__GoToPose_Result__Sequence__init(move_pos_interfaces__action__GoToPose_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_Result * data = NULL;

  if (size) {
    data = (move_pos_interfaces__action__GoToPose_Result *)allocator.zero_allocate(size, sizeof(move_pos_interfaces__action__GoToPose_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = move_pos_interfaces__action__GoToPose_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        move_pos_interfaces__action__GoToPose_Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
move_pos_interfaces__action__GoToPose_Result__Sequence__fini(move_pos_interfaces__action__GoToPose_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      move_pos_interfaces__action__GoToPose_Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

move_pos_interfaces__action__GoToPose_Result__Sequence *
move_pos_interfaces__action__GoToPose_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_Result__Sequence * array = (move_pos_interfaces__action__GoToPose_Result__Sequence *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = move_pos_interfaces__action__GoToPose_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
move_pos_interfaces__action__GoToPose_Result__Sequence__destroy(move_pos_interfaces__action__GoToPose_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    move_pos_interfaces__action__GoToPose_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
move_pos_interfaces__action__GoToPose_Result__Sequence__are_equal(const move_pos_interfaces__action__GoToPose_Result__Sequence * lhs, const move_pos_interfaces__action__GoToPose_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_Result__Sequence__copy(
  const move_pos_interfaces__action__GoToPose_Result__Sequence * input,
  move_pos_interfaces__action__GoToPose_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(move_pos_interfaces__action__GoToPose_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    move_pos_interfaces__action__GoToPose_Result * data =
      (move_pos_interfaces__action__GoToPose_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!move_pos_interfaces__action__GoToPose_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          move_pos_interfaces__action__GoToPose_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
move_pos_interfaces__action__GoToPose_Feedback__init(move_pos_interfaces__action__GoToPose_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
move_pos_interfaces__action__GoToPose_Feedback__fini(move_pos_interfaces__action__GoToPose_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
move_pos_interfaces__action__GoToPose_Feedback__are_equal(const move_pos_interfaces__action__GoToPose_Feedback * lhs, const move_pos_interfaces__action__GoToPose_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_Feedback__copy(
  const move_pos_interfaces__action__GoToPose_Feedback * input,
  move_pos_interfaces__action__GoToPose_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

move_pos_interfaces__action__GoToPose_Feedback *
move_pos_interfaces__action__GoToPose_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_Feedback * msg = (move_pos_interfaces__action__GoToPose_Feedback *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(move_pos_interfaces__action__GoToPose_Feedback));
  bool success = move_pos_interfaces__action__GoToPose_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
move_pos_interfaces__action__GoToPose_Feedback__destroy(move_pos_interfaces__action__GoToPose_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    move_pos_interfaces__action__GoToPose_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
move_pos_interfaces__action__GoToPose_Feedback__Sequence__init(move_pos_interfaces__action__GoToPose_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_Feedback * data = NULL;

  if (size) {
    data = (move_pos_interfaces__action__GoToPose_Feedback *)allocator.zero_allocate(size, sizeof(move_pos_interfaces__action__GoToPose_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = move_pos_interfaces__action__GoToPose_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        move_pos_interfaces__action__GoToPose_Feedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
move_pos_interfaces__action__GoToPose_Feedback__Sequence__fini(move_pos_interfaces__action__GoToPose_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      move_pos_interfaces__action__GoToPose_Feedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

move_pos_interfaces__action__GoToPose_Feedback__Sequence *
move_pos_interfaces__action__GoToPose_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_Feedback__Sequence * array = (move_pos_interfaces__action__GoToPose_Feedback__Sequence *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = move_pos_interfaces__action__GoToPose_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
move_pos_interfaces__action__GoToPose_Feedback__Sequence__destroy(move_pos_interfaces__action__GoToPose_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    move_pos_interfaces__action__GoToPose_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
move_pos_interfaces__action__GoToPose_Feedback__Sequence__are_equal(const move_pos_interfaces__action__GoToPose_Feedback__Sequence * lhs, const move_pos_interfaces__action__GoToPose_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_Feedback__Sequence__copy(
  const move_pos_interfaces__action__GoToPose_Feedback__Sequence * input,
  move_pos_interfaces__action__GoToPose_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(move_pos_interfaces__action__GoToPose_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    move_pos_interfaces__action__GoToPose_Feedback * data =
      (move_pos_interfaces__action__GoToPose_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!move_pos_interfaces__action__GoToPose_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          move_pos_interfaces__action__GoToPose_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "move_pos_interfaces/action/detail/go_to_pose__functions.h"

bool
move_pos_interfaces__action__GoToPose_SendGoal_Request__init(move_pos_interfaces__action__GoToPose_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    move_pos_interfaces__action__GoToPose_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!move_pos_interfaces__action__GoToPose_Goal__init(&msg->goal)) {
    move_pos_interfaces__action__GoToPose_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
move_pos_interfaces__action__GoToPose_SendGoal_Request__fini(move_pos_interfaces__action__GoToPose_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  move_pos_interfaces__action__GoToPose_Goal__fini(&msg->goal);
}

bool
move_pos_interfaces__action__GoToPose_SendGoal_Request__are_equal(const move_pos_interfaces__action__GoToPose_SendGoal_Request * lhs, const move_pos_interfaces__action__GoToPose_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!move_pos_interfaces__action__GoToPose_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_SendGoal_Request__copy(
  const move_pos_interfaces__action__GoToPose_SendGoal_Request * input,
  move_pos_interfaces__action__GoToPose_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!move_pos_interfaces__action__GoToPose_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

move_pos_interfaces__action__GoToPose_SendGoal_Request *
move_pos_interfaces__action__GoToPose_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_SendGoal_Request * msg = (move_pos_interfaces__action__GoToPose_SendGoal_Request *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(move_pos_interfaces__action__GoToPose_SendGoal_Request));
  bool success = move_pos_interfaces__action__GoToPose_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
move_pos_interfaces__action__GoToPose_SendGoal_Request__destroy(move_pos_interfaces__action__GoToPose_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    move_pos_interfaces__action__GoToPose_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence__init(move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_SendGoal_Request * data = NULL;

  if (size) {
    data = (move_pos_interfaces__action__GoToPose_SendGoal_Request *)allocator.zero_allocate(size, sizeof(move_pos_interfaces__action__GoToPose_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = move_pos_interfaces__action__GoToPose_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        move_pos_interfaces__action__GoToPose_SendGoal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence__fini(move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      move_pos_interfaces__action__GoToPose_SendGoal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence *
move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence * array = (move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence__destroy(move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence__are_equal(const move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence * lhs, const move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence__copy(
  const move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence * input,
  move_pos_interfaces__action__GoToPose_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(move_pos_interfaces__action__GoToPose_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    move_pos_interfaces__action__GoToPose_SendGoal_Request * data =
      (move_pos_interfaces__action__GoToPose_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!move_pos_interfaces__action__GoToPose_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          move_pos_interfaces__action__GoToPose_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
move_pos_interfaces__action__GoToPose_SendGoal_Response__init(move_pos_interfaces__action__GoToPose_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    move_pos_interfaces__action__GoToPose_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
move_pos_interfaces__action__GoToPose_SendGoal_Response__fini(move_pos_interfaces__action__GoToPose_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
move_pos_interfaces__action__GoToPose_SendGoal_Response__are_equal(const move_pos_interfaces__action__GoToPose_SendGoal_Response * lhs, const move_pos_interfaces__action__GoToPose_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_SendGoal_Response__copy(
  const move_pos_interfaces__action__GoToPose_SendGoal_Response * input,
  move_pos_interfaces__action__GoToPose_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

move_pos_interfaces__action__GoToPose_SendGoal_Response *
move_pos_interfaces__action__GoToPose_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_SendGoal_Response * msg = (move_pos_interfaces__action__GoToPose_SendGoal_Response *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(move_pos_interfaces__action__GoToPose_SendGoal_Response));
  bool success = move_pos_interfaces__action__GoToPose_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
move_pos_interfaces__action__GoToPose_SendGoal_Response__destroy(move_pos_interfaces__action__GoToPose_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    move_pos_interfaces__action__GoToPose_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence__init(move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_SendGoal_Response * data = NULL;

  if (size) {
    data = (move_pos_interfaces__action__GoToPose_SendGoal_Response *)allocator.zero_allocate(size, sizeof(move_pos_interfaces__action__GoToPose_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = move_pos_interfaces__action__GoToPose_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        move_pos_interfaces__action__GoToPose_SendGoal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence__fini(move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      move_pos_interfaces__action__GoToPose_SendGoal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence *
move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence * array = (move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence__destroy(move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence__are_equal(const move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence * lhs, const move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence__copy(
  const move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence * input,
  move_pos_interfaces__action__GoToPose_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(move_pos_interfaces__action__GoToPose_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    move_pos_interfaces__action__GoToPose_SendGoal_Response * data =
      (move_pos_interfaces__action__GoToPose_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!move_pos_interfaces__action__GoToPose_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          move_pos_interfaces__action__GoToPose_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
move_pos_interfaces__action__GoToPose_GetResult_Request__init(move_pos_interfaces__action__GoToPose_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    move_pos_interfaces__action__GoToPose_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
move_pos_interfaces__action__GoToPose_GetResult_Request__fini(move_pos_interfaces__action__GoToPose_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
move_pos_interfaces__action__GoToPose_GetResult_Request__are_equal(const move_pos_interfaces__action__GoToPose_GetResult_Request * lhs, const move_pos_interfaces__action__GoToPose_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_GetResult_Request__copy(
  const move_pos_interfaces__action__GoToPose_GetResult_Request * input,
  move_pos_interfaces__action__GoToPose_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

move_pos_interfaces__action__GoToPose_GetResult_Request *
move_pos_interfaces__action__GoToPose_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_GetResult_Request * msg = (move_pos_interfaces__action__GoToPose_GetResult_Request *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(move_pos_interfaces__action__GoToPose_GetResult_Request));
  bool success = move_pos_interfaces__action__GoToPose_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
move_pos_interfaces__action__GoToPose_GetResult_Request__destroy(move_pos_interfaces__action__GoToPose_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    move_pos_interfaces__action__GoToPose_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence__init(move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_GetResult_Request * data = NULL;

  if (size) {
    data = (move_pos_interfaces__action__GoToPose_GetResult_Request *)allocator.zero_allocate(size, sizeof(move_pos_interfaces__action__GoToPose_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = move_pos_interfaces__action__GoToPose_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        move_pos_interfaces__action__GoToPose_GetResult_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence__fini(move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      move_pos_interfaces__action__GoToPose_GetResult_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence *
move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence * array = (move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence__destroy(move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence__are_equal(const move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence * lhs, const move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence__copy(
  const move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence * input,
  move_pos_interfaces__action__GoToPose_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(move_pos_interfaces__action__GoToPose_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    move_pos_interfaces__action__GoToPose_GetResult_Request * data =
      (move_pos_interfaces__action__GoToPose_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!move_pos_interfaces__action__GoToPose_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          move_pos_interfaces__action__GoToPose_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "move_pos_interfaces/action/detail/go_to_pose__functions.h"

bool
move_pos_interfaces__action__GoToPose_GetResult_Response__init(move_pos_interfaces__action__GoToPose_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!move_pos_interfaces__action__GoToPose_Result__init(&msg->result)) {
    move_pos_interfaces__action__GoToPose_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
move_pos_interfaces__action__GoToPose_GetResult_Response__fini(move_pos_interfaces__action__GoToPose_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  move_pos_interfaces__action__GoToPose_Result__fini(&msg->result);
}

bool
move_pos_interfaces__action__GoToPose_GetResult_Response__are_equal(const move_pos_interfaces__action__GoToPose_GetResult_Response * lhs, const move_pos_interfaces__action__GoToPose_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!move_pos_interfaces__action__GoToPose_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_GetResult_Response__copy(
  const move_pos_interfaces__action__GoToPose_GetResult_Response * input,
  move_pos_interfaces__action__GoToPose_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!move_pos_interfaces__action__GoToPose_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

move_pos_interfaces__action__GoToPose_GetResult_Response *
move_pos_interfaces__action__GoToPose_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_GetResult_Response * msg = (move_pos_interfaces__action__GoToPose_GetResult_Response *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(move_pos_interfaces__action__GoToPose_GetResult_Response));
  bool success = move_pos_interfaces__action__GoToPose_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
move_pos_interfaces__action__GoToPose_GetResult_Response__destroy(move_pos_interfaces__action__GoToPose_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    move_pos_interfaces__action__GoToPose_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence__init(move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_GetResult_Response * data = NULL;

  if (size) {
    data = (move_pos_interfaces__action__GoToPose_GetResult_Response *)allocator.zero_allocate(size, sizeof(move_pos_interfaces__action__GoToPose_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = move_pos_interfaces__action__GoToPose_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        move_pos_interfaces__action__GoToPose_GetResult_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence__fini(move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      move_pos_interfaces__action__GoToPose_GetResult_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence *
move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence * array = (move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence__destroy(move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence__are_equal(const move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence * lhs, const move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence__copy(
  const move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence * input,
  move_pos_interfaces__action__GoToPose_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(move_pos_interfaces__action__GoToPose_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    move_pos_interfaces__action__GoToPose_GetResult_Response * data =
      (move_pos_interfaces__action__GoToPose_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!move_pos_interfaces__action__GoToPose_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          move_pos_interfaces__action__GoToPose_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "move_pos_interfaces/action/detail/go_to_pose__functions.h"

bool
move_pos_interfaces__action__GoToPose_FeedbackMessage__init(move_pos_interfaces__action__GoToPose_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    move_pos_interfaces__action__GoToPose_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!move_pos_interfaces__action__GoToPose_Feedback__init(&msg->feedback)) {
    move_pos_interfaces__action__GoToPose_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
move_pos_interfaces__action__GoToPose_FeedbackMessage__fini(move_pos_interfaces__action__GoToPose_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  move_pos_interfaces__action__GoToPose_Feedback__fini(&msg->feedback);
}

bool
move_pos_interfaces__action__GoToPose_FeedbackMessage__are_equal(const move_pos_interfaces__action__GoToPose_FeedbackMessage * lhs, const move_pos_interfaces__action__GoToPose_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!move_pos_interfaces__action__GoToPose_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_FeedbackMessage__copy(
  const move_pos_interfaces__action__GoToPose_FeedbackMessage * input,
  move_pos_interfaces__action__GoToPose_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!move_pos_interfaces__action__GoToPose_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

move_pos_interfaces__action__GoToPose_FeedbackMessage *
move_pos_interfaces__action__GoToPose_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_FeedbackMessage * msg = (move_pos_interfaces__action__GoToPose_FeedbackMessage *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(move_pos_interfaces__action__GoToPose_FeedbackMessage));
  bool success = move_pos_interfaces__action__GoToPose_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
move_pos_interfaces__action__GoToPose_FeedbackMessage__destroy(move_pos_interfaces__action__GoToPose_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    move_pos_interfaces__action__GoToPose_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence__init(move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_FeedbackMessage * data = NULL;

  if (size) {
    data = (move_pos_interfaces__action__GoToPose_FeedbackMessage *)allocator.zero_allocate(size, sizeof(move_pos_interfaces__action__GoToPose_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = move_pos_interfaces__action__GoToPose_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        move_pos_interfaces__action__GoToPose_FeedbackMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence__fini(move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      move_pos_interfaces__action__GoToPose_FeedbackMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence *
move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence * array = (move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence *)allocator.allocate(sizeof(move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence__destroy(move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence__are_equal(const move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence * lhs, const move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence__copy(
  const move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence * input,
  move_pos_interfaces__action__GoToPose_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(move_pos_interfaces__action__GoToPose_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    move_pos_interfaces__action__GoToPose_FeedbackMessage * data =
      (move_pos_interfaces__action__GoToPose_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!move_pos_interfaces__action__GoToPose_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          move_pos_interfaces__action__GoToPose_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!move_pos_interfaces__action__GoToPose_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
