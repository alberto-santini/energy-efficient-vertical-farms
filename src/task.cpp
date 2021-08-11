#include "task.h"
#include "instance.h"
#include <json.hpp>

namespace elevator {
  Task::Task(const nlohmann::json& obj) {
    start = obj["start"];
    end = obj["end"];
    duration = obj["duration"];
    
    if(obj["type"] == "planting") {
      type = TaskType::PLANTING;
    } else if(obj["type"] == "harvest") {
      type = TaskType::HARVEST;
    } else {
      type = TaskType::NORMAL;
    }
  }

  Task Task::get_dummy_start_task() {
    Task t;

    t.start = 0;
    t.end = 0;
    t.duration = 1;
    t.type = TaskType::PLANTING;

    return t;
  }

  Task Task::get_dummy_end_task(size_t omega) {
    Task t;

    t.start = omega;
    t.end = omega;
    t.duration = 1;
    t.type = TaskType::HARVEST;

    return t;
  }
}