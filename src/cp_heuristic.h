//
// Created by alberto on 22/06/2021.
//

#ifndef ELEVATOR_CP_HEURISTIC_H
#define ELEVATOR_CP_HEURISTIC_H

#include "solution.h"
#include <optional>

namespace elevator {
  std::optional<ElevatorHistory> generate_cp_solution(const Instance& i);
};

#endif //ELEVATOR_CP_HEURISTIC_H
