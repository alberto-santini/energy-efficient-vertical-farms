//
// Created by alberto on 19/07/2021.
//

#include "cp_solver_params.h"

namespace elevator {
  const std::string CpSolverParams::header = "timeout_s";

  std::ostream& operator<<(std::ostream& out, const CpSolverParams& params) {
    out << params.timeout_s;
    return out;
  }
}