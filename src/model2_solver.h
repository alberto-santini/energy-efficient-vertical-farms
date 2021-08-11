#ifndef _MODEL2_SOLVER_H
#define _MODEL2_SOLVER_H

#include "instance.h"
#include "solution.h"
#include "model2_params.h"
#include "model_stats.h"
#include <gurobi_c++.h>
#include <boost/bimap.hpp>
#include <vector>
#include <string>
#include <tuple>
#include <optional>
#include <utility>

namespace elevator {
  struct Model2Solver {
    using x_index = std::tuple<size_t, size_t, size_t>;
    using z_index = size_t;
    using p_index = std::tuple<size_t, size_t>;
    using x_map = boost::bimap<x_index, size_t>;
    using z_map = boost::bimap<z_index, size_t>;
    using p_map = boost::bimap<p_index, size_t>;

    const Instance& i;
    Model2Params params;
    ModelStats stats;

    GRBEnv env;
    GRBModel model;
    static int model_start_n;

    GRBVar* x;
    GRBVar* z;
    GRBVar* p;
    x_map xi;
    z_map zi;
    p_map pi;

    std::vector<std::string> x_names;
    std::vector<std::string> z_names;
    std::vector<std::string> p_names;

    Model2Solver(const Instance& instance, Model2Params params)
        : i{instance}, params{std::move(params)}, env{}, model{env}
    { build_model(); }

    ~Model2Solver() { delete[] x; delete[] z; delete[] p; }

    void solve();
    void load_initial_solution(const ElevatorHistory& initial);

  private:
    void build_index_maps();
    void build_model();
    void build_names();
    void deal_with_feasible(bool primal_available);
    void deal_with_infeasible();
    void save_stats_to_file(bool feasible) const;
    void save_solution_to_file() const;

    ElevatorHistory get_history() const;
  };
}

#endif
