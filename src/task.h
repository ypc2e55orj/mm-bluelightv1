#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/message_buffer.h>
#include <freertos/event_groups.h>

const BaseType_t CORE_SENSE = 0;
const BaseType_t CORE_CTRL = 0;
const BaseType_t CORE_LOG = 1;
const BaseType_t CORE_MAIN = 1;

const UBaseType_t PRIORITY_HIGH = 5;
const UBaseType_t PRIORITY_NORMAL = 10;
const UBaseType_t PRIORITY_LOW = 15;

struct task_param
{
  TaskHandle_t task;
  MessageBufferHandle_t message;
  EventGroupHandle_t event;
};
