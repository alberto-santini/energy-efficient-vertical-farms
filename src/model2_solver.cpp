#include "model2_solver.h"
#include "gurobi_root_node_cb.h"
#include "model2_solution_checker_cb.h"
#include <fstream>
#include <gurobi_c++.h>
#include <termcolor.hpp>

namespace elevator {
  int Model2Solver::model_start_n = 0;

  void Model2Solver::load_initial_solution(const ElevatorHistory& initial) {
    size_t current_tray = 0u;
    size_t current_task = 0u;

    model.set(GRB_IntAttr_NumStart, model_start_n + 1);
    model.set(GRB_IntParam_StartNumber, model_start_n);
    ++model_start_n;

    for(const auto& es : initial) {
      if(es.tray && es.task) {
        if(current_tray != *es.tray || current_task != *es.task) {
          const auto idx = std::make_tuple(*es.tray, *es.task, es.time);
          x[xi.left.at(idx)].set(GRB_DoubleAttr_Start, 1.0);
          current_tray = *es.tray;
          current_task = *es.task;
        }
      }
    }
  }

  void Model2Solver::solve() {
    namespace tc = termcolor;

    if(params.initial_sol_path) {
      std::cout << tc::blue << "Loading initial solution from " << *params.initial_sol_path << tc::reset << "\n";
      const auto initial = from_json(*params.initial_sol_path);
      std::cout << tc::blue << "Setting the solution as the start solution for the model..." << tc::reset << "\n";
      load_initial_solution(initial);
    }

    model.set(GRB_DoubleParam_TimeLimit, params.timeout_s);

    if(!std::isnan(params.max_bb_nodes)) {
      model.set(GRB_DoubleParam_NodeLimit, params.max_bb_nodes);
    }

    if(params.disable_gurobi_presolve) {
      model.set(GRB_IntParam_Cuts, 0);
      model.set(GRB_IntParam_Presolve, 0);
    }

    auto root_node_cb = RootNodeInfoCB{stats};
    model.setCallback(&root_node_cb);

    // sol_checker_cb must be defined outside the if, or its lifetime would expire
    // before the callback is called.
    auto sol_checker_cb = Model2SolutionCheckerCB{xi, x, i};
    if(params.check_all_incumbents) {
      model.setCallback(&sol_checker_cb);
    }

    if(params.save_model) {
      std::string fname = "model-M2-" + i.name + ".lp";
      std::cout << tc::blue << "Exporting model to file..." << tc::reset << "\n";
      model.write(fname);
    }

    model.optimize();

    const auto status = model.get(GRB_IntAttr_Status);

    if(status == GRB_INFEASIBLE) {
      std::cout << tc::red << tc::bold << "Model is infeasible!" << tc::reset << "\n";
      deal_with_infeasible();
    } else {
      if(status == GRB_OPTIMAL) {
        std::cout << tc::color<2> << tc::bold << "Model solved to optimality!" << tc::reset << "\n";
      } else if(status == GRB_TIME_LIMIT) {
        std::cout << tc::yellow << tc::bold << "Time limit occurred before optimality!" << tc::reset << "\n";
      } else if(status == GRB_NODE_LIMIT) {
        std::cout << tc::yellow << tc::bold << "Node limit occurred before optimaility!" << tc::reset << "\n";
      } else {
        std::cout << tc::yellow << "Not solved to optimality. Status = " << status << tc::reset << "\n";
      }

      deal_with_feasible(status == GRB_OPTIMAL || status == GRB_SUBOPTIMAL);
    }
  }

  void Model2Solver::deal_with_feasible(bool primal_available) {
    namespace tc = termcolor;

    stats.time_s = model.get(GRB_DoubleAttr_Runtime);
    stats.visited_nodes = model.get(GRB_DoubleAttr_NodeCount);
    stats.primal = model.get(GRB_DoubleAttr_ObjVal);
    stats.dual = model.get(GRB_DoubleAttr_ObjBound);
    stats.gap = std::min(model.get(GRB_DoubleAttr_MIPGap), 1.0);

    std::cout << tc::blue << "\t" << "Elapsed time: " << stats.time_s << "\n";
    std::cout << "\t" << "Visited nodes: " << stats.visited_nodes << "\n";
    std::cout << "\t" << "Best primal bound: " << stats.primal << "\n";
    std::cout << "\t" << "Best dual bound: " << stats.dual << "\n";
    std::cout << "\t" << "Gap: " << stats.gap * 100 << "%" << "\n";

    if(std::isnan(stats.root_gap)) {
      std::cout << "\t" << "Model solved by presolve!\n";
      stats.root_primal = stats.primal;
      stats.root_dual = stats.dual;
      stats.root_gap = stats.gap;
      stats.root_time_s = stats.time_s;
    } else {
      std::cout << "\t" << "Root elapsed time: " << stats.root_time_s << "\n";
      std::cout << "\t" << "Root primal bound: " << stats.root_primal << "\n";
      std::cout << "\t" << "Root dual bound: " << stats.root_dual << "\n";
      std::cout << "\t" << "Root gap: " << stats.root_gap * 100 << "%" << tc::reset << "\n";
    }

    save_stats_to_file(true);

    if(primal_available) {
      save_solution_to_file();
    }
  }

  void Model2Solver::deal_with_infeasible() {
    namespace tc = termcolor;

    stats.time_s = model.get(GRB_DoubleAttr_Runtime);
    stats.visited_nodes = model.get(GRB_DoubleAttr_NodeCount);

    std::cout << tc::blue << "\t" << "Elapsed time: " << stats.time_s << "\n";
    std::cout << "\t" << "Visited nodes: " << stats.visited_nodes << "\n";

    save_stats_to_file(false);

    std::cout << tc::blue << "\nComputing the Irreducible Inconsistent Subsystem..." << tc::reset << "\n";
    model.computeIIS();

    std::string fname = "infeasible-M2-" + i.name + ".ilp";
    std::cout << tc::blue << "Writing the IIS to " << fname << tc::reset << "\n";
    model.write(fname);
  }

  void Model2Solver::save_stats_to_file(bool feasible) const {
    const auto fname = "stats-M2-" + i.name + ".csv";
    const auto fpath = std::filesystem::path(params.output_folder) / fname;
    std::ofstream ofs{fpath, std::ofstream::out};

    ofs << "model,feasible," << Instance::header << "," << ModelStats::header << "," << Model2Params::header << "\n";
    ofs << "M2," << std::boolalpha << feasible << "," << i << "," << stats << "," << params << "\n";
  }

  void Model2Solver::save_solution_to_file() const {
    namespace tc = termcolor;

    const auto history = get_history();

    check_feasible(history, i);

    const auto cost = compute_cost(history);

    if(cost != (size_t)stats.primal) {
      std::cerr << tc::red << tc::bold << "/!\\ Warning: different costs: "
        << "Gurobi returned " << stats.primal << " but recomputing "
        << "from the solution gives " << cost << tc::reset << "\n";
    }

    const auto fname = "sol-M2-" + i.name + ".json";
    const auto fpath = std::filesystem::path(params.output_folder) / fname;
    std::ofstream ofs{fpath, std::ofstream::out};

    ofs << to_json(history) << "\n";
  }

  ElevatorHistory Model2Solver::get_history() const {
    using std::make_tuple;

    ElevatorHistory hist;
    size_t current_time = 1u;
    size_t current_shelf = 0u;

    while(current_time <= i.n_time_instants) {
      bool starting_task = false;

      for(auto t : i.tray_idx_nd) {
        for(auto j : i.tray(t).task_idx) {
          const auto tjk = make_tuple(t, j, current_time);

          if(xi.left.find(tjk) == xi.left.end()) { continue; }

          const auto x_tjk = xi.left.at(tjk);

          if(x[x_tjk].get(GRB_DoubleAttr_X) > 0.5) {
            const auto& tray = i.tray(t);
            const auto& task = tray.task(j);
            starting_task = true;

            if(task.type == TaskType::PLANTING) {
              for(auto kk = current_time; kk <= current_time + task.duration; ++kk) {
                hist.push_back({kk, Instance::depot_id, t, j, false, "DEPOT_PICKUP_PLANTING"});
              }
              hist.push_back({current_time + task.duration, tray.shelf, t, j, false, "SHELF_DELIVERY_PLANTING"});
              current_time += task.duration;
              current_shelf = tray.shelf;
              break;
            } else if(task.type == TaskType::HARVEST) {
              hist.push_back({current_time, tray.shelf, t, j, false, "SHELF_PICKUP_HARVEST"});
              for(auto kk = current_time; kk <= current_time + task.duration; ++kk) {
                hist.push_back({kk, Instance::depot_id, t, j, false, "DEPOT_DELIVERY_HARVEST"});
              }
              current_time += task.duration;
              current_shelf = Instance::depot_id;
              break;
            } else if(task.type == TaskType::NORMAL) {
              for(auto kk = current_time; kk <= current_time + task.duration; ++kk) {
                hist.push_back({kk, tray.shelf, t, j, false, "REGULAR"});
              }
              current_time += task.duration;
              current_shelf = tray.shelf;
            }
          }
        }

        if(starting_task) { break; }
      }

      if(!starting_task) {
        hist.push_back({current_time, current_shelf, std::nullopt, std::nullopt, true, "IDLE"});
        current_time += 1;
      }
    }

    return hist;
  }

  void Model2Solver::build_model() {
    using std::to_string;
    using std::make_tuple;

    build_index_maps();
    build_names();

    std::vector<char> x_types(xi.size(), GRB_BINARY);
    x = model.addVars(NULL /* LB */, NULL /* UB */, NULL /* OBJ */, x_types.data(), x_names.data(), (int) xi.size());

    std::vector<char> z_types(zi.size(), GRB_INTEGER);
    std::vector<double> z_lb(zi.size(), 0.0);
    std::vector<double> z_ub(zi.size(), 2.0 * i.n_shelves);
    std::vector<double> z_costs(zi.size(), 1.0);
    z = model.addVars(z_lb.data(), z_ub.data(), z_costs.data(), z_types.data(), z_names.data(), (int) zi.size());

    std::vector<char> p_types(pi.size(), GRB_BINARY);
    p = model.addVars(NULL /* LB */, NULL /* UB */, NULL /* OBJ */, p_types.data(), p_names.data(), (int) pi.size());

    for(auto t : i.tray_idx_nd) {
      for(auto j : i.tray(t).task_idx) {
        GRBLinExpr lhs;

        for(auto k = i.alpha(t, j); k <= i.beta(t, j); ++k) {
          const auto x_tjk = xi.left.at(make_tuple(t, j, k));
          lhs += x[x_tjk];
        }

        model.addConstr(lhs == 1, "dotask[" + to_string(t) + "][" + to_string(j) + "]");
      }
    }

    for(auto k : i.time_instant_idx) {
      GRBLinExpr lhs;

      for(auto ii : i.shelf_idx) {
        const auto p_ik = pi.left.at(make_tuple(ii, k));
        lhs += p[p_ik];
      }

      model.addConstr(lhs == 1, "uniquepos[" + to_string(k) + "]");
    }

    for(auto k : i.time_instant_idx) {
      GRBLinExpr lhs;

      for(auto t : i.tray_idx_nd) {
        for(auto j : i.tray(t).task_idx) {
          for(auto k1 : i.big_omega.at(make_tuple(t, j, k))) {
            const auto x_tjk1 = xi.left.at(make_tuple(t, j, k1));
            lhs += x[x_tjk1];
          }
        }
      }

      model.addConstr(lhs <= 1, "onetask[" + to_string(k) + "]");
    }

    for(auto ii : i.shelf_idx) {
      for(auto k : i.time_instant_idx) {
        GRBLinExpr lhs;

        for(auto t : i.tray_idx_nd) {
          for(auto j : i.tray(t).task_idx) {
            if(i.tray(t).target_shelf(j) != ii) { continue; }
            if(k < i.alpha(t, j) || k > i.beta(t, j)) { continue; }

            const auto x_tjk = xi.left.at(make_tuple(t, j, k));
            lhs += x[x_tjk];
          }
        }

        const auto p_ik = pi.left.at(make_tuple(ii, k));
        model.addConstr(lhs <= p[p_ik], "elevpos[" + to_string(ii) + "][" + to_string(k) + "]");
      }
    }

    for(auto t : i.tray_idx_nd) {
      for(auto j : i.tray(t).task_idx_ns) {
        for(auto k = i.alpha(t, j); k <= i.beta(t, j); ++k) {
          GRBLinExpr rhs;
          const auto& tray = i.tray(t);
          const auto& task = tray.task(j);

          for(auto k1 = tray.start_task().start; k1 <= tray.start_task().end; ++k1) {
            if(k >= k1 && task.start <= k - k1 && k - k1 <= task.end) {
              const auto x_t0k1 = xi.left.at(make_tuple(t, tray.start_task_id(), k1));
              rhs += x[x_t0k1];
            }
          }

          const auto x_tjk = xi.left.at(make_tuple(t, j, k));
          model.addConstr(x[x_tjk] <= rhs, "tw[" + to_string(t) + "][" + to_string(j) + "][" + to_string(k) + "]");
        }
      }
    }

    for(auto t : i.tray_idx_nd) {
      for(auto j : i.tray(t).task_idx_ns) {
        for(auto k = i.alpha(t, j); k <= i.beta(t, j); ++k) {
          GRBLinExpr rhs;

          for(auto k1 = i.alpha(t, j - 1u); k1 <= i.beta(t, j - 1u); ++k1) {
            if(k1 > k) { continue; }

            if(i.tray(t).task(j - 1u).duration <= k - k1) {
              const auto x_tj1k1 = xi.left.at(make_tuple(t, j - 1u, k1));
              rhs += x[x_tj1k1];
            }
          }

          const auto x_tjk = xi.left.at(make_tuple(t, j, k));
          model.addConstr(x[x_tjk] <= rhs, "seq[" + to_string(t) + "][" + to_string(j) + "][" + to_string(k) + "]");
        }
      }
    }

    for(auto t : i.tray_idx_nd) {
      const auto& tray = i.tray(t);
      for(auto k = i.alpha(t, tray.start_task_id()); k <= i.beta(t, tray.start_task_id()); ++k) {
        GRBLinExpr rhs_sum;

        for(auto t1 : i.tray_idx_nd) {
          if(t == t1) { continue; }

          const auto& tray1 = i.tray(t1);

          if(tray.shelf != tray1.shelf) { continue; }

          for(auto k1 = i.alpha(t1, tray1.start_task_id()); k1 <= std::min(k, i.beta(t1, tray1.start_task_id())); ++k1) {
            const auto x_t10k1 = xi.left.at(make_tuple(t1, tray1.start_task_id(), k1));
            rhs_sum += x[x_t10k1];
          }
          for(auto k1 = i.alpha(t1, tray1.end_task_id()); k1 <= std::min(k, i.beta(t1, tray1.end_task_id())); ++k1) {
            const auto x_t1lt1k1 = xi.left.at(make_tuple(t1, tray1.end_task_id(), k1));
            rhs_sum -= x[x_t1lt1k1];
          }
        }

        const auto x_t0k = xi.left.at(make_tuple(t, tray.start_task_id(), k));
        model.addConstr(x[x_t0k] <= 1 - rhs_sum, "nonoverlap[" + to_string(t) + "][" + to_string(k) + "]");
      }
    }

    for(auto k : i.time_instant_idx) {
      GRBLinExpr sum1;
      if(k > 1u) {
        for(auto ii : i.shelf_idx) {
          const auto p_ik1 = pi.left.at(make_tuple(ii, k - 1u));
          sum1 += ii * p[p_ik1];
        }
      }

      GRBLinExpr sum2;
      for(auto ii : i.shelf_idx) {
        const auto p_ik = pi.left.at(make_tuple(ii, k));
        sum2 += ii * p[p_ik];
      }

      GRBLinExpr sum3;
      for(auto t : i.tray_idx_nd) {
        if(k < i.alpha(t, i.tray(t).start_task_id()) || k > i.beta(t, i.tray(t).start_task_id())) { continue; }
        const auto x_t0k = xi.left.at(make_tuple(t, i.tray(t).start_task_id(), k));
        sum3 += 2 * i.tray(t).shelf * x[x_t0k];
      }

      const auto z_k = zi.left.at(k);
      model.addConstr(z[z_k] >= sum1 - sum2 + sum3, "compz1[" + to_string(k) + "]");
    }

    for(auto k : i.time_instant_idx) {
      GRBLinExpr sum1;
      for(auto ii : i.shelf_idx) {
        const auto p_ik = pi.left.at(make_tuple(ii, k));
        sum1 += ii * p[p_ik];
      }

      GRBLinExpr sum2;
      if(k > 1u) {
        for(auto ii : i.shelf_idx) {
          const auto p_ik1 = pi.left.at(make_tuple(ii, k - 1u));
          sum2 += ii * p[p_ik1];
        }
      }

      GRBLinExpr sum3;
      for(auto t : i.tray_idx_nd) {
        if(k < i.alpha(t, i.tray(t).end_task_id()) || k > i.beta(t, i.tray(t).end_task_id())) { continue; }
        const auto x_tlk = xi.left.at(make_tuple(t, i.tray(t).end_task_id(), k));
        sum3 += 2 * i.tray(t).shelf * x[x_tlk];
      }

      const auto z_k = zi.left.at(k);
      model.addConstr(z[z_k] >= sum1 - sum2 + sum3, "compz2[" + to_string(k) + "]");
    }

    if(params.use_valid_inequality4) {
      stats.valid_ineq4_added = 0;

      // A call to ::update is required to access LB/UB of
      // variables before the model is optimised!
      model.update();

      for(auto [tjk, idx] : xi.left) {
        const auto [t, j, k] = tjk;

        if(k < i.L_earliest.at(make_tuple(t, j))) {
          x[idx].set(GRB_DoubleAttr_UB, 0);
          ++stats.valid_ineq4_added;
        }

        if(k > i.L_latest.at(make_tuple(t, j))) {
          x[idx].set(GRB_DoubleAttr_UB, 0);
          ++stats.valid_ineq4_added;
        }
      }
    }

    if(params.use_valid_inequality5) {
      stats.valid_ineq5_addded = 0;

      for(auto [tjk1, idx1] : xi.left) {
        const auto [t1, j1, k1] = tjk1;

        for(auto t2 : i.tray_idx_nd) {
          for(auto j2 : i.tray(t2).task_idx) {
            if(t1 == t2 && j1 == j2) { continue; }

            if(i.alpha(t1, j1) + i.tray(t1).task(j1).duration > i.beta(t2, j2)) {
              GRBLinExpr rhs;

              for(auto k = i.alpha(t2, j2); k <= i.beta(t2, j2); ++k) {
                if(k1 >= k && i.tray(t2).task(j2).duration <= k1 - k) {
                  const auto idx = xi.left.at(make_tuple(t2, j2, k));
                  rhs += x[idx];
                }
              }

              model.addConstr(x[idx1] <= rhs, "valid_ineq5[" + to_string(t1) + "][" + to_string(j1) + "][" + to_string(k1) + "][" + to_string(t2) + "][" + to_string(j2) + "]");
              ++stats.valid_ineq5_addded;
            }
          }
        }
      }
    }

    model.update();
    stats.n_rows = (double) model.get(GRB_IntAttr_NumConstrs);
    stats.n_cols = (double) model.get(GRB_IntAttr_NumVars);
    stats.n_nz = (double) model.get(GRB_IntAttr_NumNZs);
  }

  void Model2Solver::build_index_maps() {
    size_t index = 0u;
    for(auto t : i.tray_idx_nd) {
      for(auto j : i.tray(t).task_idx) {
        for(auto k = i.alpha(t, j); k <= i.beta(t, j); ++k) {
          xi.insert({{t, j, k}, index++});
        }
      }
    }

    index = 0u;
    for(auto k : i.time_instant_idx) {
      zi.insert({k, index++});
    }

    index = 0u;
    for(auto ii : i.shelf_idx) {
      for(auto k : i.time_instant_idx) {
        pi.insert({{ii, k}, index++});
      }
    }
  }

  void Model2Solver::build_names() {
    using std::to_string;

    x_names = std::vector<std::string>(xi.size());
    for(auto [tjk, idx] : xi) {
      const auto [t, j, k] = tjk;
      x_names[idx] = "x[" + to_string(t) + "][" + to_string(j) + "][" + to_string(k) + "]";
    }

    z_names = std::vector<std::string>(zi.size());
    for(auto [k, idx] : zi) {
      z_names[idx] = "z[" + to_string(k) + "]";
    }

    p_names = std::vector<std::string>(pi.size());
    for(auto [ik, idx] : pi) {
      const auto [ii, k] = ik;
      p_names[idx] = "p[" + to_string(ii) + "][" + to_string(k) + "]";
    }
  }
}