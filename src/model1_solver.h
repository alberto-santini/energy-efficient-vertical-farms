#ifndef _MODEL1_SOLVER_H
#define _MODEL1_SOLVER_H

#include "instance.h"
#include "solution.h"
#include "model1_params.h"
#include "model_stats.h"
#include <gurobi_c++.h>
#include <boost/bimap.hpp>
#include <vector>
#include <string>
#include <tuple>
#include <utility>
#include <optional>

namespace elevator {
  struct Model1Solver {
    using y_index = std::tuple<size_t, size_t, size_t, size_t>;
    using s_index = std::tuple<size_t, size_t>;
    using d_index = std::tuple<size_t, size_t>;
    using y_map = boost::bimap<y_index, size_t>;
    using s_map = boost::bimap<s_index, size_t>;
    using d_map = boost::bimap<d_index, size_t>;

    const Instance& i;
    Model1Params params;
    ModelStats stats;

    GRBEnv env;
    GRBModel model;
    static int model_start_n;

    GRBVar* y;
    GRBVar* s;
    GRBVar* d;
    y_map yi;
    s_map si;
    d_map di;

    std::vector<double> y_costs;
    std::vector<double> s_lb;
    std::vector<double> s_ub;
    std::vector<std::string> y_names;
    std::vector<std::string> s_names;
    std::vector<std::string> d_names;

    Model1Solver(const Instance& instance, Model1Params params)
        : i{instance}, params{std::move(params)}, env{}, model{env}
        { build_model(); }

    ~Model1Solver() { delete[] y; delete[] s; delete[] d; }

    void solve();
    void load_initial_solution(const ElevatorHistory& initial);

  private:
    bool are_tasks_compatible(size_t t1, size_t j1, size_t t2, size_t j2);
    int get_cost(size_t t1, size_t j1, size_t t2, size_t j2);
    void build_index_maps();
    void build_costs();
    void build_bounds();
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
