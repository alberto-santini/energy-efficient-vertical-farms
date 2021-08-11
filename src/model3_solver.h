#ifndef _MODEL3_SOLVER_H
#define _MODEL3_SOLVER_H

#include "instance.h"
#include "solution.h"
#include "model3_params.h"
#include "model_stats.h"
#include <gurobi_c++.h>
#include <boost/bimap.hpp>
#include <vector>
#include <string>
#include <tuple>
#include <utility>
#include <optional>

namespace elevator {
  struct Model3Solver {
    using y_index = std::tuple<size_t, size_t, size_t, size_t>;
    using x_index = std::tuple<size_t, size_t, size_t>;
    using y_map = boost::bimap<y_index, size_t>;
    using x_map = boost::bimap<x_index, size_t>;

    const Instance& i;
    Model3Params params;
    ModelStats stats;

    GRBEnv env;
    GRBModel model;
    static int model_start_n;

    GRBVar* y;
    GRBVar* x;
    y_map yi;
    x_map xi;

    std::vector<double> y_costs;
    std::vector<std::string> y_names;
    std::vector<std::string> x_names;

    Model3Solver(const Instance& instance, Model3Params params)
        : i{instance}, params{std::move(params)}, env{}, model{env}
        { build_model(); }

    ~Model3Solver() { delete[] y; delete[] x; }

    void solve();
    void load_initial_solution(const ElevatorHistory& initial);

  private:
    bool are_tasks_compatible(size_t t1, size_t j1, size_t t2, size_t j2);
    int get_cost(size_t t1, size_t j1, size_t t2, size_t j2);
    void build_index_maps();
    void build_costs();
    void build_model();
    void build_names();
    void deal_with_feasible(bool primal_available);
    void deal_with_infeasible();
    void save_stats_to_file(bool feasible) const;
    void save_solution_to_file() const;

    std::vector<ElevatorOperation> extract_tasks() const;
    std::optional<std::tuple<size_t, size_t>> next_task(const ElevatorOperation& op) const;
    ElevatorHistory get_history() const;
  };
}

#endif
