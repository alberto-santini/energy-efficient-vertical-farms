#ifndef _TRAY_H
#define _TRAY_H

#include "task.h"
#include <json.hpp>
#include <cstdint>
#include <vector>

namespace elevator {
  struct Instance;

  struct Tray {
    size_t shelf;

    std::vector<Task> tasks;

    // Modelling utilities:
    std::vector<size_t> task_idx; // Indices of all tasks (starting from 0)
    std::vector<size_t> task_idx_ns; // Indices of all tasks except the start task (which is at index 0)

    explicit Tray(const nlohmann::json& obj);

    [[nodiscard]] size_t start_task_id() const { return 0u; }
    [[nodiscard]] size_t end_task_id() const { return tasks.size() - 1u; }
    [[nodiscard]] const Task& start_task() const { return tasks.at(0u); }
    [[nodiscard]] const Task& end_task() const { return tasks.at(tasks.size() - 1u); }
    [[nodiscard]] const Task& task(size_t task_id) const { return tasks.at(task_id); }
    [[nodiscard]] size_t target_shelf(size_t task_id) const;

  private:
    Tray() {}

  public:
    static Tray get_dummy_tray(size_t omega);
  };
}

#endif