#ifndef _SOLUTION_H
#define _SOLUTION_H

#include "instance.h"
#include <optional>
#include <string>
#include <vector>
#include <filesystem>

namespace elevator {
  struct ElevatorStatus {
    size_t time;
    size_t shelf;
    std::optional<size_t> tray;
    std::optional<size_t> task;
    bool idle;
    std::string comment;
  };

  using ElevatorHistory = std::vector<ElevatorStatus>;
  std::string to_json(const ElevatorHistory& eh);
  ElevatorHistory from_json(const std::filesystem::path& json_file);

  struct ElevatorOperation {
    size_t tray;
    size_t task;
    size_t time;
  };

  bool check_within_tw(const ElevatorHistory& eh, const Instance& i);
  bool check_no_overlaps(const ElevatorHistory& eh, const Instance& i);
  bool check_feasible(const ElevatorHistory& eh, const Instance& i);
  size_t compute_cost(const ElevatorHistory& eh);

  [[maybe_unused]] std::ostream& operator<<(std::ostream& out, const ElevatorStatus& es);
  [[maybe_unused]] std::ostream& operator<<(std::ostream& out, const ElevatorHistory& eh);
}

#endif
