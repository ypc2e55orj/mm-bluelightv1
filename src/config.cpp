#include "config.hpp"

// C++
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>

// ESP-IDF
#include <cJSON.h>
#include <esp_log.h>

// Project

namespace config {
static constexpr auto TAG = "config::Config";

#define JSON_DEBUG
#if defined(JSON_DEBUG)
#define JSON_LOGE(tag, format, ...) ESP_LOGE(tag, format, __VA_ARGS__);
#else
#define JSON_LOGE(tag, format, ...)
#endif
#define JSON_READ_NUMBER(root, field)                                   \
  do {                                                                  \
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, #field); \
    if (!cJSON_IsNumber(item)) {                                        \
      JSON_LOGE(TAG, "Error: Config::" #field " is not number.");       \
    } else {                                                            \
      field = static_cast<decltype(field)>(item->valuedouble);          \
    }                                                                   \
  } while (0)

#define JSON_READ_NUMBER_ARRAY(root, field, len)                             \
  do {                                                                       \
    const cJSON *array = cJSON_GetObjectItemCaseSensitive(root, #field);     \
    if (!cJSON_IsArray(array)) {                                             \
      ESP_LOGE(TAG, "Error: Config::" #field " is not array.");              \
    } else {                                                                 \
      size_t i = 0;                                                          \
      for (cJSON *item = array->child; i < (len) && item != nullptr;         \
           i++, item = item->next) {                                         \
        if (!cJSON_IsNumber(item)) {                                         \
          ESP_LOGE(TAG, "Error: Config::" #field "[%d] is not number", i);   \
        } else {                                                             \
          field[i] = static_cast<std::remove_reference_t<decltype(*field)>>( \
              item->valuedouble);                                            \
        }                                                                    \
      }                                                                      \
    }                                                                        \
  } while (0)

bool Config::to_struct(std::string_view str) {
  cJSON *json = cJSON_ParseWithLength(str.data(), str.length());
  if (!json) {
    auto error_ptr = cJSON_GetErrorPtr();
    if (error_ptr) {
      ESP_LOGE(TAG, "Error before: %s", error_ptr);
      return false;
    }
  }

  JSON_READ_NUMBER(json, tire_diameter);
  JSON_READ_NUMBER(json, spur_gear_teeth);
  JSON_READ_NUMBER(json, pinion_gear_teeth);
  JSON_READ_NUMBER_ARRAY(json, photo_wall_threshold, 4);
  JSON_READ_NUMBER_ARRAY(json, photo_wall_reference, 4);
  JSON_READ_NUMBER_ARRAY(json, straight_pid, 3);
  JSON_READ_NUMBER(json, straight_velocity);
  JSON_READ_NUMBER(json, straight_accel);
  JSON_READ_NUMBER(json, straight_jerk);
  JSON_READ_NUMBER_ARRAY(json, turn_pid, 3);
  JSON_READ_NUMBER(json, turn_velocity);
  JSON_READ_NUMBER(json, turn_accel);
  JSON_READ_NUMBER(json, turn_jerk);
  JSON_READ_NUMBER(json, slalom_turn_velocity);
  JSON_READ_NUMBER(json, slalom_turn_accel);
  JSON_READ_NUMBER(json, slalom_turn_jerk);
  JSON_READ_NUMBER(json, slalom_turn_offset_pre);
  JSON_READ_NUMBER(json, slalom_turn_offset_post);
  JSON_READ_NUMBER_ARRAY(json, maze_goal, 2);
  JSON_READ_NUMBER_ARRAY(json, size, 2);

  return true;
}
[[maybe_unused]] bool Config::read_file(const char *const path) {
  std::ifstream file(path);
  std::stringstream ss;
  if (!file) {
    return false;
  }
  ss << file.rdbuf();
  return to_struct(ss.str());
}
[[maybe_unused]] bool Config::read_stdin() {
  std::string buf, str;
  while (std::cin >> buf) {
    str += buf;
  }

  return to_struct(str);
}
#undef JSON_READ_NUMBER
#undef JSON_READ_NUMBER_ARRAY

#define JSON_WRITE_NUMBER(root, field)                                        \
  do {                                                                        \
    if (!cJSON_AddNumberToObject(root, #field, static_cast<double>(field))) { \
      JSON_LOGE(TAG, "Error: Failed to write" #field ".");                    \
    }                                                                         \
  } while (0)

#define JSON_WRITE_NUMBER_ARRAY(root, field, len)                            \
  do {                                                                       \
    cJSON *array = cJSON_AddArrayToObject(root, #field);                     \
    if (!array) {                                                            \
      JSON_LOGE(TAG, "Error: Failed to add array.");                         \
    } else {                                                                 \
      for (size_t i = 0; i < len; i++) {                                     \
        if (!cJSON_AddItemToArray(                                           \
                array, cJSON_CreateNumber(static_cast<double>(field[i])))) { \
          ESP_LOGE(TAG, "Error: Failed to add Config" #field "[%d]", i);     \
        }                                                                    \
      }                                                                      \
    }                                                                        \
  } while (0)

std::string Config::to_str() {
  cJSON *json = cJSON_CreateObject();
  if (!json) {
    return "";
  }

  JSON_WRITE_NUMBER(json, tire_diameter);
  JSON_WRITE_NUMBER(json, spur_gear_teeth);
  JSON_WRITE_NUMBER(json, pinion_gear_teeth);
  JSON_WRITE_NUMBER_ARRAY(json, photo_wall_threshold, 4);
  JSON_WRITE_NUMBER_ARRAY(json, photo_wall_reference, 4);
  JSON_WRITE_NUMBER_ARRAY(json, straight_pid, 3);
  JSON_WRITE_NUMBER(json, straight_velocity);
  JSON_WRITE_NUMBER(json, straight_accel);
  JSON_WRITE_NUMBER(json, straight_jerk);
  JSON_WRITE_NUMBER_ARRAY(json, turn_pid, 3);
  JSON_WRITE_NUMBER(json, turn_velocity);
  JSON_WRITE_NUMBER(json, turn_accel);
  JSON_WRITE_NUMBER(json, turn_jerk);
  JSON_WRITE_NUMBER(json, slalom_turn_velocity);
  JSON_WRITE_NUMBER(json, slalom_turn_accel);
  JSON_WRITE_NUMBER(json, slalom_turn_jerk);
  JSON_WRITE_NUMBER(json, slalom_turn_offset_pre);
  JSON_WRITE_NUMBER(json, slalom_turn_offset_post);
  JSON_WRITE_NUMBER_ARRAY(json, maze_goal, 2);
  JSON_WRITE_NUMBER_ARRAY(json, size, 2);

  char *c_str = cJSON_Print(json);
  std::string str(c_str);
  cJSON_Delete(json);
  free(c_str);

  return str;
}
[[maybe_unused]] bool Config::write_file(const char *path) {
  std::ofstream file(path);
  if (!file) {
    return false;
  }

  file << to_str();
  return true;
}
[[maybe_unused]] bool Config::write_stdout() {
  std::cout << to_str() << std::endl;
  return true;
}
#undef JSON_WRITE_NUMBER
#undef JSON_WRITE_NUMBER_ARRAY
}  // namespace config

config::Config conf = {
    .tire_diameter = 12.80f,
    .spur_gear_teeth = 38.0f,
    .pinion_gear_teeth = 9.0f,
    .photo_wall_threshold = {0, 0, 0, 0},
    .photo_wall_reference = {0, 0, 0, 0},
    .straight_pid = {0.0f, 0.0f, 0.0f},
    .straight_velocity = 0.0f,
    .straight_accel = 0.0f,
    .straight_jerk = 0.0f,
    .turn_pid = {0.0f, 0.0f, 0.0f},
    .turn_velocity = 0.0f,
    .turn_accel = 0.0f,
    .turn_jerk = 0.0f,
    .slalom_turn_velocity = 0.0f,
    .slalom_turn_accel = 0.0f,
    .slalom_turn_jerk = 0.0f,
    .slalom_turn_offset_pre = 0.0f,
    .slalom_turn_offset_post = 0.0f,
};
