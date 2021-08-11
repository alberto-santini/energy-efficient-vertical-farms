#include "tray.h"
#include <numeric>
#include <optional>
#include <iostream>
#include <stdexcept>
#include <json.hpp>
#include <termcolor.hpp>

namespace elevator {
  Tray::Tray(const nlohmann::json& obj) {
    namespace tc = termcolor;

    std::optional<size_t> start_task_i, end_task_i;
    shelf = obj["shelf"];

    for(const auto& task : obj["tasks"]) {
      tasks.emplace_back(task);

      if(tasks.back().type == TaskType::PLANTING) {
        start_task_i = tasks.size() - 1u;
      } else if(tasks.back().type == TaskType::HARVEST) {
        end_task_i = tasks.size() - 1u;
      }
    }

    // Put the start task first, if it isn't already
    if(!start_task_i) {
      throw std::logic_error("No start task!");
    } else {
      if(*start_task_i != start_task_id()) {
        std::iter_swap(tasks.begin() + *start_task_i, tasks.begin() + start_task_id());
      }
    }

    // Put the end task last, if it isn't already
    if(!end_task_i) {
      throw std::logic_error("No end task!");
    } else {
      if(*end_task_i != end_task_id()) {
        std::iter_swap(tasks.begin() + *end_task_i, tasks.begin() + end_task_id());
      }
    }

    // Sort intermediate tasks by start tw:
    if(tasks.size() > 2u) {
      std::sort(std::next(tasks.begin()), std::prev(tasks.end()), [] (const Task& t1, const Task& t2) -> bool { return t1.start < t2.start; });

      // If the start tw of the end task is < than the start tw of the last intermediate task, trim it:
      if(tasks[*end_task_i].start < tasks[*end_task_i - 1u].start) {
        tasks[*end_task_i].start = tasks[*end_task_i - 1u].start;
      }

      // Check that everything is now correct:
      for(auto j = 2u; j < tasks.size(); ++j) {
        if(task(j - 1u).start > task(j).start) {
          std::cerr << tc::red << tc::bold << "Warning: " << tc::reset << tc::red
            << "there is a tray on shelf " << shelf << " for which task "
            << j << " has start time " << task(j).start << ", while task "
            << j - 1u << " has start time " << task(j - 1u).start << "!"
            << tc::reset << "\n";
        }
      }
    }

    task_idx = std::vector<size_t>(tasks.size());
    std::iota(task_idx.begin(), task_idx.end(), 0u);
    task_idx_ns = std::vector<size_t>(tasks.size() - 1u);
    std::iota(task_idx_ns.begin(), task_idx_ns.end(), 1u);
  }

  Tray Tray::get_dummy_tray(size_t omega) {
    Tray t;

    t.shelf = 0u;
    t.tasks = {
        Task::get_dummy_start_task(),
        Task::get_dummy_end_task(omega)
    };
    t.task_idx = {0u, 1u};
    t.task_idx_ns = {1u};

    return t;
  }

  size_t Tray::target_shelf(size_t task_id) const {
    if(task_id == end_task_id()) {
      return 0u;
    } else {
      return shelf;
    }
  }
}