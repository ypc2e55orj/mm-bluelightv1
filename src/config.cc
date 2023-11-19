#include "config.h"

// C++
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>

// ESP-IDF
#include <cJSON.h>
#include <esp_log.h>

namespace config {
static constexpr auto TAG = "config::Config";

#define JSON_READ_NUMBER(root, field)                                   \
  do {                                                                  \
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, #field); \
    if (!cJSON_IsNumber(item)) {                                        \
      ESP_LOGE(TAG, "Error: Config::" #field " is not number.");        \
    } else {                                                            \
      field = static_cast<decltype(field)>(item->valuedouble);          \
    }                                                                   \
  } while (0)

#define JSON_READ_NUMBER_ARRAY(root, field)                                 \
  do {                                                                      \
    const cJSON *array = cJSON_GetObjectItemCaseSensitive(root, #field);    \
    if (!cJSON_IsArray(array)) {                                            \
      ESP_LOGE(TAG, "Error: Config::" #field " is not array.");             \
    } else {                                                                \
      size_t i = 0;                                                         \
      for (cJSON *item = array->child; i < field.size() && item != nullptr; \
           i++, item = item->next) {                                        \
        if (!cJSON_IsNumber(item)) {                                        \
          ESP_LOGE(TAG, "Error: Config::" #field "[%d] is not number.", i); \
        } else {                                                            \
          field[i] =                                                        \
              static_cast<decltype(field)::value_type>(item->valuedouble);  \
        }                                                                   \
      }                                                                     \
    }                                                                       \
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

  JSON_READ_NUMBER(json, low_voltage);
  JSON_READ_NUMBER(json, tire_diameter);
  JSON_READ_NUMBER(json, spur_gear_teeth);
  JSON_READ_NUMBER(json, pinion_gear_teeth);
  JSON_READ_NUMBER_ARRAY(json, photo_wall_threshold);
  JSON_READ_NUMBER_ARRAY(json, photo_wall_reference);
  JSON_READ_NUMBER_ARRAY(json, straight_pid);
  JSON_READ_NUMBER(json, straight_velocity);
  JSON_READ_NUMBER(json, straight_accel);
  JSON_READ_NUMBER(json, straight_jerk);
  JSON_READ_NUMBER_ARRAY(json, turn_pid);
  JSON_READ_NUMBER(json, turn_velocity);
  JSON_READ_NUMBER(json, turn_accel);
  JSON_READ_NUMBER(json, turn_jerk);
  JSON_READ_NUMBER(json, slalom_turn_velocity);
  JSON_READ_NUMBER(json, slalom_turn_accel);
  JSON_READ_NUMBER(json, slalom_turn_jerk);
  JSON_READ_NUMBER(json, slalom_turn_offset_pre);
  JSON_READ_NUMBER(json, slalom_turn_offset_post);
  JSON_READ_NUMBER_ARRAY(json, maze_goal);
  JSON_READ_NUMBER_ARRAY(json, maze_size);

  return true;
}
[[maybe_unused]] bool Config::read_file(std::string_view path) {
  std::ifstream file((std::string(path)));
  std::stringstream ss;
  if (!file) {
    return false;
  }
  ss << file.rdbuf();
  return to_struct(ss.str());
}
[[maybe_unused]] bool Config::read_stdin() {
  std::string buf, str;
  while (std::getline(std::cin, buf)) {
    if (buf[0] == '$') {
      break;
    }
    std::cout << buf << std::endl;
    str += buf;
  }

  return to_struct(str);
}
#undef JSON_READ_NUMBER
#undef JSON_READ_NUMBER_ARRAY

#define JSON_WRITE_NUMBER(root, field)                                        \
  do {                                                                        \
    if (!cJSON_AddNumberToObject(root, #field, static_cast<double>(field))) { \
      ESP_LOGE(TAG, "Error: Failed to write Config::" #field ".");            \
    }                                                                         \
  } while (0)

#define JSON_WRITE_NUMBER_ARRAY(root, field)                                 \
  do {                                                                       \
    cJSON *array = cJSON_AddArrayToObject(root, #field);                     \
    if (!array) {                                                            \
      ESP_LOGE(TAG, "Error: Failed to add array.");                          \
    } else {                                                                 \
      for (size_t i = 0; i < field.size(); i++) {                            \
        if (!cJSON_AddItemToArray(                                           \
                array, cJSON_CreateNumber(static_cast<double>(field[i])))) { \
          ESP_LOGE(TAG, "Error: Failed to add Config::" #field "[%d].", i);  \
        }                                                                    \
      }                                                                      \
    }                                                                        \
  } while (0)

std::string Config::to_str() {
  cJSON *json = cJSON_CreateObject();
  if (!json) {
    return "";
  }

  JSON_WRITE_NUMBER(json, low_voltage);
  JSON_WRITE_NUMBER(json, tire_diameter);
  JSON_WRITE_NUMBER(json, spur_gear_teeth);
  JSON_WRITE_NUMBER(json, pinion_gear_teeth);
  JSON_WRITE_NUMBER_ARRAY(json, photo_wall_threshold);
  JSON_WRITE_NUMBER_ARRAY(json, photo_wall_reference);
  JSON_WRITE_NUMBER_ARRAY(json, straight_pid);
  JSON_WRITE_NUMBER(json, straight_velocity);
  JSON_WRITE_NUMBER(json, straight_accel);
  JSON_WRITE_NUMBER(json, straight_jerk);
  JSON_WRITE_NUMBER_ARRAY(json, turn_pid);
  JSON_WRITE_NUMBER(json, turn_velocity);
  JSON_WRITE_NUMBER(json, turn_accel);
  JSON_WRITE_NUMBER(json, turn_jerk);
  JSON_WRITE_NUMBER(json, slalom_turn_velocity);
  JSON_WRITE_NUMBER(json, slalom_turn_accel);
  JSON_WRITE_NUMBER(json, slalom_turn_jerk);
  JSON_WRITE_NUMBER(json, slalom_turn_offset_pre);
  JSON_WRITE_NUMBER(json, slalom_turn_offset_post);
  JSON_WRITE_NUMBER_ARRAY(json, maze_goal);
  JSON_WRITE_NUMBER_ARRAY(json, maze_size);

  char *c_str = cJSON_Print(json);
  std::string str(c_str);
  cJSON_Delete(json);
  free(c_str);

  return str;
}
[[maybe_unused]] bool Config::write_file(std::string_view path) {
  std::ofstream file((std::string(path)));
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
