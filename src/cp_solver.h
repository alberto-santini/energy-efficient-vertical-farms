//
// Created by alberto on 19/07/2021.
//

#ifndef ELEVATOR_CP_SOLVER_H
#define ELEVATOR_CP_SOLVER_H

// Required by cplex:
#define IL_STD

#include "solution.h"
#include "cp_solver_params.h"
#include "model_stats.h"
#include <optional>
#include <ilcplex/ilocplex.h>
#include <ilcp/cp.h>
#include <boost/bimap.hpp>

namespace elevator {
  struct CpSolver {
    using TrayTaskId = std::tuple<size_t, size_t>;
    using IndexMap = boost::bimap<TrayTaskId, IloInt>;

    const Instance& i;
    CpSolverParams params;
    ModelStats stats;

    IloEnv env;
    IloModel model;
    IloCP cp;
    IloInt n_tasks;
    IloIntArray task_index;
    IloIntArray2 cost;
    IloIntervalVarArray task_interval;
    IloIntervalSequenceVar task_seq;
    IndexMap tmap;

    CpSolver(const Instance& i, CpSolverParams params)
      : i{i}, params{std::move(params)}, env{}, model{env}
      { build_model(); }

    std::optional<ElevatorHistory> solve();

  private:
    void build_model();
    void deal_with_infeasible();
    ElevatorHistory deal_with_feasible();
    void save_solution_to_file(const ElevatorHistory& h);
    void save_stats_to_file(bool feasible);
  };
}

#endif //ELEVATOR_CP_SOLVER_H
