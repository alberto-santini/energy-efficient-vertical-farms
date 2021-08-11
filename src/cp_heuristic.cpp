//
// Created by alberto on 22/06/2021.
//

// Required to make CPLEX compile on Mac OS:
#define IL_STD

#include "cp_heuristic.h"
#include "cp_solver.h"
#include "cp_solver_params.h"
#include "solution.h"
#include <termcolor.hpp>
#include <iostream>

namespace elevator {
  std::optional<ElevatorHistory> generate_cp_solution(const Instance &i) {
    namespace tc = termcolor;

    std::cout << tc::blue << "Launching constraint programming heuristic...\n" << tc::reset;

    CpSolverParams params = { .timeout_s = 300.0f, .output_to_file = false };
    CpSolver solver{i, params};

    return solver.solve();
  }
}
