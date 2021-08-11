#include "model1_solver.h"
#include "gurobi_root_node_cb.h"
#include "model1_save_cont_rel_cb.h"
#include <gurobi_c++.h>
#include <termcolor.hpp>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <chrono>
#include <filesystem>

namespace elevator {
  int Model1Solver::model_start_n = 0;

  void Model1Solver::build_model() {
    using std::to_string;
    using std::make_tuple;
    using namespace std::chrono;
    namespace tc = termcolor;

    const auto time_1 = steady_clock::now();
    build_index_maps();
    const auto time_2 = steady_clock::now();
    std::cout << tc::italic << "Index maps build (" << duration_cast<seconds>(time_2 - time_1).count() << "s)\n";
    build_costs();
    const auto time_3 = steady_clock::now();
    std::cout << "Costs calculated (" << duration_cast<seconds>(time_3 - time_2).count() << "s)\n";
    build_bounds();
    const auto time_4 = steady_clock::now();
    std::cout << "Bounds computed (" << duration_cast<seconds>(time_4 - time_3).count() << "s)\n";
    build_names();
    const auto time_5 = steady_clock::now();
    std::cout << "Generated variable names (" << duration_cast<seconds>(time_5 - time_4).count() << "s)\n" << tc::reset;

    std::vector<char> y_types(yi.size(), GRB_BINARY);
    y = model.addVars(NULL /* LB */, NULL /* UB */, y_costs.data() /* OBJ */, y_types.data(), y_names.data(), (int) yi.size() /* COUNT */);

    std::vector<char> s_types(si.size(), GRB_INTEGER);
    s = model.addVars(s_lb.data(), s_ub.data(), NULL /* OBJ */, s_types.data(), s_names.data(), (int) si.size() /* COUNT */);

    std::vector<char> d_types(di.size(), GRB_BINARY);
    d = model.addVars(NULL /* LB */, NULL /* UB */, NULL /* OBJ */, d_types.data(), d_names.data(), (int) di.size() /* COUNT */);

    for(auto t : i.tray_idx) {
      for(auto j : i.tray(t).task_idx) {
        if(t == Instance::dummy_id && j == i.tray(t).start_task_id()) { continue; }

        GRBLinExpr lhs;

        for(auto [tjs, idx] : yi.left) {
          const auto [t1, j1, t2, j2] = tjs;

          if(t2 == t && j2 == j) {
            lhs += y[idx];
          }
        }

        model.addConstr(lhs == 1, "processall[" + to_string(t) + "][" + to_string(j) + "]");
      }
    }

    {
      GRBLinExpr lhs;

      for(auto [tjs, idx] : yi.left) {
        const auto [t1, j1, t2, j2] = tjs;

        if(t1 == Instance::dummy_id && j1 == i.tray(t1).start_task_id()) {
          lhs += y[idx];
        }
      }

      model.addConstr(lhs == 1, "servedummy");
    }

    for(auto t : i.tray_idx_nd) {
      if(t == Instance::dummy_id) { continue; }

      for(auto j : i.tray(t).task_idx) {
        GRBLinExpr lhs;

        for(auto [tjs, idx] : yi.left ) {
          const auto [t1, j1, t2, j2] = tjs;

          if(t2 == t && j2 == j) {
            lhs += y[idx];
          }
        }

        GRBLinExpr rhs;

        for(auto [tjs, idx] : yi.left) {
          const auto [t1, j1, t2, j2] = tjs;

          if(t1 == t && j1 == j) {
            rhs += y[idx];
          }
        }

        model.addConstr(lhs == rhs, "predsucc[" + to_string(t) + "][" + to_string(j) + "]");
      }
    }

    for(auto t : i.tray_idx) {
      for(auto j : i.tray(t).task_idx_ns) {
        const auto s_tj = si.left.at(make_tuple(t, j));
        const auto s_t0 = si.left.at(make_tuple(t, i.tray(t).start_task_id()));
        const auto s_tj1 = si.left.at(make_tuple(t, j - 1u));

        model.addConstr(s[s_tj] >= s[s_t0] + i.tray(t).task(j).start, "twstart[" + to_string(t) + "][" + to_string(j) + "]");
        model.addConstr(s[s_tj] <= s[s_t0] + i.tray(t).task(j).end, "twend[" + to_string(t) + "][" + to_string(j) + "]");
        model.addConstr(s[s_tj] >= s[s_tj1] + i.tray(t).task(j - 1u).duration, "seq[" + to_string(t) + "][" + to_string(j) + "]");
      }
    }

    for(auto [tjs, idx] : yi.left) {
      const auto [t1, j1, t2, j2] = tjs;
      const auto s_t1j1 = si.left.at(make_tuple(t1, j1));
      const auto s_t2j2 = si.left.at(make_tuple(t2, j2));
      auto M = 999u;

      if(i.beta(t1, j1) + i.tray(t1).task(j1).duration >= i.alpha(t2, j2)) {
        M = i.beta(t1, j1) + i.tray(t1).task(j1).duration - i.alpha(t2, j2);
      }

      model.addConstr(s[s_t2j2] >= s[s_t1j1] + i.tray(t1).task(j1).duration - M * (1 - y[idx]),
                      "disj1[" + to_string(t1) + "][" + to_string(j1) + "][" + to_string(t2) + "][" + to_string(j2) + "]");
    }

    for(auto t1 : i.tray_idx) {
      for(auto t2 : i.tray_idx) {
        if(t1 == t2 || i.tray(t1).shelf != i.tray(t2).shelf) { continue; }

        const auto s_t10 = si.left.at(make_tuple(t1, i.tray(t1).start_task_id()));
        const auto s_t2l = si.left.at(make_tuple(t2, i.tray(t2).end_task_id()));
        const auto d_t1t2 = di.left.at(make_tuple(t1, t2));
        const auto d_t2t1 = di.left.at(make_tuple(t2, t1));
        const auto dur = i.tray(t2).end_task().duration;
        auto M = 999u;

        if(i.beta(t2, i.tray(t2).end_task_id()) + dur >= i.alpha(t1, i.tray(t1).start_task_id())) {
          M = i.beta(t2, i.tray(t2).end_task_id()) + dur - i.alpha(t1, i.tray(t1).start_task_id());
        }

        model.addConstr(s[s_t10] >= s[s_t2l] + dur - M * d[d_t1t2], "disj2[" + to_string(t1) + "][" + to_string(t2) + "]");

        if(t1 < t2) {
          model.addConstr(d[d_t1t2] + d[d_t2t1] <= 1, "order[" + to_string(t1) + "][" + to_string(t2) + "]");
        }
      }
    }

    const auto time_6 = steady_clock::now();
    std::cout << tc::italic << "Added variables and constraints (" << duration_cast<seconds>(time_6 - time_5).count() << "s)\n" << tc::reset;

    if(params.use_valid_inequality1) {
      stats.valid_ineq1_added = 0;
      for(auto [tjs1, idx1] : yi.left) {
        const auto [t1, j1, t2, j2] = tjs1;
        for(auto [tjs2, idx2] : yi.left) {
          const auto [tt, jj, t3, j3] = tjs2;

          if(tt != t2 || jj != j2) {
            continue;
          }

          if(i.beta(t3, j3) >= i.alpha(t1, j1) + i.tray(t1).task(j1).duration + i.tray(t2).task(j2).duration) {
            continue;
          }

          model.addConstr(y[idx1] + y[idx2] <= 1,
            "valid1[" + to_string(t1) +
                 "][" + to_string(j1) +
                 "][" + to_string(t2) +
                 "][" + to_string(j2) +
                 "][" + to_string(t3) +
                 "][" + to_string(j3) + "]");
          ++stats.valid_ineq1_added;
        }
      }
    }

    if(params.use_valid_inequality2) {
      stats.valid_ineq2_added = 0;

      for(auto [tjs, idx] : yi.left) {
        const auto [t1, j1, t2, j2] = tjs;
        if(t1 == t2 && j1 == j2) { continue; }

        const bool known_order_12 = (i.alpha(t2, j2) + i.tray(t2).task(j2).duration > i.beta(t1, j1));

        for(auto t3 : i.tray_idx_nd) {
          for(auto j3 : i.tray(t3).task_idx) {
            if(t3 == t1 && j3 == j1) { continue; }
            if(t3 == t2 && j3 == j2) { continue; }

            for(auto k = i.alpha(t3, j3); k <= i.beta(t3, j3); ++k) {
              if(k >= i.alpha(t1, j1) + i.tray(t1).task(j1).duration && k + i.tray(t3).task(j3).duration <= i.beta(t2, j2)) {
                goto abort;
              }
            }
          }
        }

        if(known_order_12) {
          model.addConstr(y[idx] == 1, "valid2[" + to_string(t1) + "][" + to_string(j1) + "][" + to_string(t2) + "][" + to_string(j2) + "]");
        } else if(const auto tjs2 = make_tuple(t2, j2, t1, j1); t1 < t2 && yi.left.find(tjs2) != yi.left.end()) {
          for(auto t3 : i.tray_idx_nd) {
            for(auto j3 : i.tray(t3).task_idx) {
              if(t3 == t1 && j3 == j1) { continue; }
              if(t3 == t2 && j3 == j2) { continue; }

              for(auto k = i.alpha(t3, j3); k <= i.beta(t3, j3); ++k) {
                if(k >= i.alpha(t2, j2) + i.tray(t2).task(j2).duration && k + i.tray(t3).task(j3).duration <= i.beta(t1, j1)) {
                  goto abort;
                }
              }
            }
          }

          const auto idx2 = yi.left.at(tjs2);
          model.addConstr(y[idx] + y[idx2] == 1, "valid2bis[" + to_string(t1) + "][" + to_string(j1) + "][" + to_string(t2) + "][" + to_string(j2) + "]");
        }
        ++stats.valid_ineq2_added;
        
        abort: ;
      }
    }

    if(params.use_valid_inequality3) {
      stats.valid_ineq3_added = 0;

      for(auto [tjs, idx] : yi.left) {
        const auto [t1, j1, t2, j2] = tjs;
        const auto inv = make_tuple(t2, j2, t1, j1);

        if(yi.left.find(inv) == yi.left.end()) { continue; }

        const auto idx2 = yi.left.at(inv);

        model.addConstr(y[idx] + y[idx2] <= 1, "valid3cycle2[" + to_string(t1) + "][" + to_string(j1) + "][" + to_string(t2) + "][" + to_string(j2) + "]");
        ++stats.valid_ineq3_added;
      }

      for(auto [tjs1, idx1] : yi.left) {
        const auto [t1, j1, t2, j2] = tjs1;
        for(auto [tjs2, idx2] : yi.left) {
          const auto [tt, jj, t3, j3] = tjs2;

          if(tt != t2 || jj != j2) {
            continue;
          }

          const auto inv = make_tuple(t3, j3, t1, j1);

          if(yi.left.find(inv) == yi.left.end()) { continue; }

          const auto idx3 = yi.left.at(inv);

          model.addConstr(y[idx1] + y[idx2] + y[idx3] <= 2, "valid3cycle3[" + to_string(t1) + "][" + to_string(j1) + "][" + to_string(t2) + "][" + to_string(j2) + "][" + to_string(t3) + "][" + to_string(j3) + "]");
          ++stats.valid_ineq3_added;
        }
      }
    }

    if(params.use_valid_inequality4) {
      stats.valid_ineq4_added = 0;
      
      // A call to ::update is required to access LB/UB of
      // variables before the model is optimised!
      model.update();

      for(auto [tj, idx] : si.left) {
        if(s[idx].get(GRB_DoubleAttr_LB) < i.L_earliest.at(tj)) {
          s[idx].set(GRB_DoubleAttr_LB, i.L_earliest.at(tj));
          ++stats.valid_ineq4_added;
        }

        if(s[idx].get(GRB_DoubleAttr_UB) > i.L_latest.at(tj)) {
          s[idx].set(GRB_DoubleAttr_UB, i.L_latest.at(tj));
          ++stats.valid_ineq4_added;
        }
      }
    }

    if(params.use_valid_inequality5) {
      stats.valid_ineq5_addded = 0;

      for(auto [tj1, idx1] : si.left) {
        const auto [t1, j1] = tj1;
        for(auto [tj2, idx2] : si.left) {
          const auto [t2, j2] = tj2;

          if(t1 == t2 && j1 == j2) { continue; }

          if(i.alpha(t2, j2) + i.tray(t2).task(j2).duration > i.beta(t1, j1)) {
            model.addConstr(s[idx2] >= s[idx1] + i.tray(t1).task(j1).duration,
              "valid5[" + to_string(t1) + "][" + to_string(j1) + "][" + to_string(t2) + "][" + to_string(j2) + "]");
            ++stats.valid_ineq5_addded;
          }
        }
      }
    }

    const auto time_7 = steady_clock::now();
    std::cout << tc::italic << "Added valid inequalities (" << duration_cast<seconds>(time_7 - time_6).count() << "s)\n" << tc::reset;

    // A call to ::update is required to access the number of
    // vars/constraints/non-zero.
    model.update();
    stats.n_rows = (double) model.get(GRB_IntAttr_NumConstrs);
    stats.n_cols = (double) model.get(GRB_IntAttr_NumVars);
    stats.n_nz = (double) model.get(GRB_IntAttr_NumNZs);
  }

  int Model1Solver::get_cost(size_t t1, size_t j1, size_t t2, size_t j2) {
    const auto& tray1 = i.tray(t1);
    const auto& tray2 = i.tray(t2);
    const auto& task1 = tray1.task(j1);
    const auto& task2 = tray2.task(j2);
    const auto shelf1 = static_cast<int>(tray1.shelf);
    const auto shelf2 = static_cast<int>(tray2.shelf);

    if(task1.type == TaskType::PLANTING) {
      if(task2.type == TaskType::PLANTING) {
        return shelf1 + shelf2;
      } else if(task2.type == TaskType::NORMAL) {
        return std::abs(shelf1 - shelf2);
      } else if(task2.type == TaskType::HARVEST) {
        return std::abs(shelf1 - shelf2) + shelf2;
      }
    } else if(task1.type == TaskType::NORMAL) {
      if(task2.type == TaskType::PLANTING) {
        return shelf1 + shelf2;
      } else if(task2.type == TaskType::NORMAL) {
        return std::abs(shelf1 - shelf2);
      } else if(task2.type == TaskType::HARVEST) {
        return std::abs(shelf1 - shelf2) + shelf2;
      }
    } else if(task1.type == TaskType::HARVEST) {
      if(task2.type == TaskType::PLANTING) {
        return shelf2;
      } else if(task2.type == TaskType::NORMAL) {
        return shelf2;
      } else if(task2.type == TaskType::HARVEST) {
        return 2 * shelf2;
      }
    }

    throw std::logic_error("Calculating cost of tasks with unrecognised types");
  }

  void Model1Solver::build_costs() {
    y_costs = std::vector<double>(yi.size());

    for(auto [tjs, idx] : yi.left) {
      const auto [t1, j1, t2, j2] = tjs;
      y_costs[idx] = (double) get_cost(t1, j1, t2, j2);
    }
  }

  void Model1Solver::build_bounds() {
    s_lb = std::vector<double>(si.size());
    s_ub = std::vector<double>(si.size());

    for(auto [tj, idx] : si.left) {
      const auto [t, j] = tj;
      s_lb[idx] = (double) i.alpha(t, j);
      s_ub[idx] = (double) i.beta(t, j);
    }
  }

  void Model1Solver::build_names() {
    using std::to_string;

    y_names = std::vector<std::string>(yi.size());
    for(auto [tjs, idx] : yi.left) {
      const auto [t1, j1, t2, j2] = tjs;
      y_names[idx] = "y[" + to_string(t1) +
                     "][" + to_string(j1) +
                     "][" + to_string(t2) +
                     "][" + to_string(j2) + "]";
    }

    s_names = std::vector<std::string>(si.size());
    for(auto [tj, idx] : si.left) {
      const auto [t, j] = tj;
      s_names[idx] = "s[" + to_string(t) + "][" + to_string(j) + "]";
    }

    d_names = std::vector<std::string>(di.size());
    for(auto [tt, idx] : di.left) {
      const auto [t1, t2] = tt;
      d_names[idx] = "d[" + to_string(t1) + "][" + to_string(t2) + "]";
    }
  }

  void Model1Solver::build_index_maps() {
    size_t index = 0u;
    for(auto t1 : i.tray_idx) {
      for(auto j1 : i.tray(t1).task_idx) {
        for(auto t2 : i.tray_idx) {
          for(auto j2 : i.tray(t2).task_idx) {
            if(are_tasks_compatible(t1, j1, t2, j2)) {
              yi.insert({{t1, j1, t2, j2}, index++});
            }
          }
        }
      }
    }

    index = 0u;
    for(auto t : i.tray_idx) {
      for(auto j : i.tray(t).task_idx) {
        si.insert({{t, j}, index++});
      }
    }

    index = 0u;
    for(auto t1 : i.tray_idx) {
      for(auto t2 : i.tray_idx) {
        if(t1 != t2 && i.tray(t1).shelf == i.tray(t2).shelf) {
          di.insert({{t1, t2}, index++});
        }
      }
    }
  }

  bool Model1Solver::are_tasks_compatible(size_t t1, size_t j1, size_t t2, size_t j2) {
    if(t1 == Instance::dummy_id && t2 == Instance::dummy_id) {
      return false; // Dummy -> dummy would mean... an empty solution?
    }

    if(t2 == Instance::dummy_id && j2 == i.tray(t2).start_task_id()) {
      return false; // Start task of the dummy tray can have no inbound.
    }

    if(t1 == Instance::dummy_id && j1 == i.tray(t1).end_task_id()) {
      return false; // End task of the dummy tray can have on outbound.
    }

    if(t1 == Instance::dummy_id && j1 == i.tray(t1).start_task_id() && j2 != i.tray(t2).start_task_id()) {
      return false; // From the starting dummy you can only go to the start task of j2.
    }

    if(t2 == Instance::dummy_id && j2 == i.tray(t2).end_task_id() && j1 != i.tray(t1).end_task_id()) {
      return false; // You can only get to the ending dummy from the end task of j1.
    }

    if(t1 == t2 && j2 != j1 + 1u) {
      return false; // Tasks for a given tray must be performed in the given order.
    }

    const auto& task1 = i.tray(t1).task(j1);
    const auto& task2 = i.tray(t2).task(j2);

    auto latest_start_time_2 = i.tray(t2).start_task().end;
    if(task2.type != TaskType::PLANTING) {
      latest_start_time_2 += task2.end;
    }
    latest_start_time_2 = std::min(i.n_time_instants - 1u - task2.duration, latest_start_time_2);

    auto earliest_start_time_1 = i.tray(t1).start_task().start;
    if(task1.type != TaskType::PLANTING) {
      earliest_start_time_1 += task1.start;
    }
    earliest_start_time_1 = std::min(i.n_time_instants - 1u - task1.duration, earliest_start_time_1);

    if(latest_start_time_2 < earliest_start_time_1 + task1.duration) {
      return false;
    }

    if(t1 != Instance::dummy_id && t2 != Instance::dummy_id) {
      if(i.alpha(t2, j2) > i.beta(t1, j1) && i.alpha(t2, j2) - i.beta(t1, j1) - task1.duration + i.total_task_durations > i.n_time_instants) {
        return false;
      }

      for(auto t3 : i.tray_idx_nd) {
        if(t3 == t1 || t3 == t2) { continue; }

        for(auto j3 : i.tray(t3).task_idx) {
          const auto& task3 = i.tray(t3).task(j3);

          if(i.alpha(t3, j3) + task3.duration > i.beta(t1, j1) && i.beta(t3, j3) < i.alpha(t2, j2) + task2.duration) {
            // Task j3 must happen between j1 and j2, therefore j1 and j2 cannot be consecutive.
            return false;
          }
        }
      }
    }

    return true;
  }

  void Model1Solver::load_initial_solution(const ElevatorHistory& initial) {
    namespace tc = termcolor;

    size_t current_tray = 0u;
    size_t current_task = 0u;

    model.set(GRB_IntAttr_NumStart, model_start_n + 1);
    model.set(GRB_IntParam_StartNumber, model_start_n);
    ++model_start_n;

    for(const auto& es : initial) {
      if(es.tray && es.task) {
        if(current_tray != *es.tray || current_task != *es.task) {
          const auto y_idx = std::make_tuple(current_tray, current_task, *es.tray, *es.task);
          
          if(yi.left.find(y_idx) != yi.left.end()) {
            y[yi.left.at(y_idx)].set(GRB_DoubleAttr_Start, 1.0);
          } else {
            std::cout << tc::yellow << "Warning, no index (" << current_tray << ", " << current_task
              << ", " << *es.tray << ", " << *es.task << ") for y variables when adding an initial solution."
              << " Elevator status: " << es << tc::reset << "\n";
          }
          
          const auto s_idx = std::make_tuple(*es.tray, *es.task);

          if(si.left.find(s_idx) != si.left.end()) {
            s[si.left.at(s_idx)].set(GRB_DoubleAttr_Start, es.time);
          } else {
            std::cout << tc::yellow << "Warning, no index (" << *es.tray << ", " << *es.task
              << ") for s variables when adding an initial solution."
              << " Elevator status: " << es << tc::reset << "\n";
          }

          current_tray = *es.tray;
          current_task = *es.task;
        }
      }
    }

    const auto get_start_t = [&initial] (size_t t) -> std::optional<std::size_t> {
      for(const auto& es : initial) {
        if(es.tray && *es.tray == t && es.comment == "DEPOT_PICKUP_PLANTING") {
          return es.time;
        }
      }
      return std::nullopt;
    };

    for(auto [tt, d_idx] : di.left) {
      const auto [t1, t2] = tt;
      const auto start1 = get_start_t(t1);
      const auto start2 = get_start_t(t2);

      assert(start1 && start2);

      if(*start1 < *start2) {
        d[d_idx].set(GRB_DoubleAttr_Start, 1.0);
      }
    }
  }

  void Model1Solver::solve() {
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

    // Object save_cont_rel_cv must stay outside the "if" or its lifetime would end before the callback is called.
    auto save_cont_rel_cv = Model1SaveContRelCB{i, yi, si, di, y, s, d};
    if(params.save_contrelax) {
      model.setCallback(&save_cont_rel_cv);
    }

    if(params.save_model) {
      std::string fname = "model-M1-" + i.name + ".lp";
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

  void Model1Solver::deal_with_feasible(bool primal_available) {
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

  void Model1Solver::save_solution_to_file() const {
    namespace tc = termcolor;

    const auto history = get_history();

    check_feasible(history, i);

    const auto cost = compute_cost(history);

    if(cost != (size_t)stats.primal) {
      std::cerr << tc::red << tc::bold << "/!\\ Warning: different costs: "
        << "Gurobi returned " << stats.primal << " but recomputing "
        << "from the solution gives " << cost << tc::reset << "\n";
    }

    const auto fname = "sol-M1-" + i.name + ".json";
    const auto fpath = std::filesystem::path(params.output_folder) / fname;
    std::ofstream ofs{fpath, std::ofstream::out};

    ofs << to_json(history) << "\n";
  }

  void Model1Solver::deal_with_infeasible() {
    namespace tc = termcolor;

    stats.time_s = model.get(GRB_DoubleAttr_Runtime);
    stats.visited_nodes = model.get(GRB_DoubleAttr_NodeCount);

    std::cout << tc::blue << "\t" << "Elapsed time: " << stats.time_s << "\n";
    std::cout << "\t" << "Visited nodes: " << stats.visited_nodes << "\n";

    save_stats_to_file(false);

    std::cout << tc::blue << "\nComputing the Irreducible Inconsistent Subsystem..." << tc::reset << "\n";
    model.computeIIS();

    std::string fname = "infeasible-M1-" + i.name + ".ilp";
    std::cout << tc::blue << "Writing the IIS to " << fname << tc::reset << "\n";
    model.write(fname);
  }

  void Model1Solver::save_stats_to_file(bool feasible) const {
    const auto fname = "stats-M1-" + i.name + ".csv";
    const auto fpath = std::filesystem::path(params.output_folder) / fname;
    std::ofstream ofs{fpath, std::ofstream::out};

    ofs << "model,feasible," << Instance::header << "," << ModelStats::header << "," << Model1Params::header << "\n";
    ofs << "M1," << std::boolalpha << feasible << "," << i << "," << stats << "," << params << "\n";
  }

  ElevatorHistory Model1Solver::get_history() const {
    const auto tasks = extract_tasks();
    ElevatorHistory eh;

    for(auto id = 0u; id < tasks.size(); ++id) {
      const auto [t, j, start_time] = tasks[id];
      if(t == Instance::dummy_id) { continue; }

      const auto& tray = i.tray(t);
      const auto& task = tray.task(j);
      const auto duration = task.duration;
      auto end_shelf = tray.shelf;

      if(task.type == TaskType::PLANTING) {
        for(auto ti = start_time; ti <= start_time + duration; ++ti) {
          eh.push_back({ti, Instance::depot_id, t, j, false, "DEPOT_PICKUP_PLANTING"});
        }
        eh.push_back({start_time + duration, tray.shelf, t, j, false, "SHELF_DELIVERY_PLANTING"});
      } else if(task.type == TaskType::HARVEST) {
        eh.push_back({start_time, tray.shelf, t, j, false, "SHELF_PICKUP_HARVEST"});
        for(auto ti = start_time; ti <= start_time + duration; ++ti) {
          eh.push_back({ti, Instance::depot_id, t, j, false, "DEPOT_DELIVERY_HARVEST"});
        }
        end_shelf = Instance::depot_id;
      } else if(task.type == TaskType::NORMAL) {
        for(auto ti = start_time; ti <= start_time + duration; ++ti) {
          eh.push_back({ti, tray.shelf, t, j, false, "REGULAR"});
        }
      }

      if(id < tasks.size() - 1u) {
        const auto next_start_time = tasks[id + 1u].time;
        for(auto ti = start_time + duration + 1u; ti < next_start_time; ++ti) {
          eh.push_back({ti, end_shelf, std::nullopt, std::nullopt, true, "IDLE"});
        }
      }
    }

    return eh;
  }

  std::vector<ElevatorOperation> Model1Solver::extract_tasks() const {
    std::vector<ElevatorOperation> tasks;

    // Start from the dummy tray:
    tasks.push_back({Instance::dummy_id, i.tray(Instance::dummy_id).start_task_id(), 0u /* TIME */});

    while(true) {
      const auto nt = next_task(tasks.back());
      if(!nt) { break; }

      const auto [tray_id, task_id] = *nt;
      const auto start_time = (size_t) s[si.left.at(std::make_tuple(tray_id, task_id))].get(GRB_DoubleAttr_X);
      tasks.push_back({tray_id, task_id, start_time});
    }

    return tasks;
  }

  std::optional<std::tuple<size_t, size_t>> Model1Solver::next_task(const ElevatorOperation& op) const {
    for(auto t : i.tray_idx) {
      for(auto j : i.tray(t).task_idx) {
        const auto tjs = std::make_tuple(op.tray, op.task, t, j);
        if(yi.left.find(tjs) != yi.left.end()) {
          if(y[yi.left.at(tjs)].get(GRB_DoubleAttr_X) > 0.5) {
            return std::make_tuple(t, j);
          }
        }
      }
    }
    return std::nullopt;
  }
}