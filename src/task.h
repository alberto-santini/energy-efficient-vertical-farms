#ifndef _TASK_H
#define _TASK_H

#include "task_type.h"
#include <json.hpp>
#include <cstdint>

namespace elevator {
  struct Instance;

  struct Task {
    size_t start;
    size_t end;
    size_t duration;
    TaskType type;
    
    explicit Task(const nlohmann::json& obj);

  private:
    Task() {}

  public:
    static Task get_dummy_start_task();
    static Task get_dummy_end_task(size_t omega);
  };
}

#endif