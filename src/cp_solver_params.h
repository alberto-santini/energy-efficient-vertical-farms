//
// Created by alberto on 19/07/2021.
//

#ifndef ELEVATOR_CP_SOLVER_PARAMS_H
#define ELEVATOR_CP_SOLVER_PARAMS_H

#include <string>
#include <iostream>

namespace elevator {
  struct CpSolverParams {
    std::string output_folder = "../solutions/raw";
    float timeout_s = 3600.0f;
    bool output_to_file = false;

    static const std::string header;
  };

  std::ostream& operator<<(std::ostream& out, const CpSolverParams& params);
}

#endif //ELEVATOR_CP_SOLVER_PARAMS_H
