//
// Created by alberto on 19/07/2021.
//

#include "cp_solver.h"
#include <termcolor.hpp>
#include <tuple>
#include <cassert>

namespace elevator {
  namespace {
    int get_cost(size_t t1, size_t j1, size_t t2, size_t j2, const Instance& i) {
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
  }

  void CpSolver::deal_with_infeasible() {
    namespace tc = termcolor;

    stats.time_s = cp.getInfo(IloCP::NumInfo::SolveTime);
    stats.n_cols = cp.getInfo(IloCP::IntInfo::NumberOfVariables);
    stats.n_rows = cp.getInfo(IloCP::IntInfo::NumberOfConstraints);

    std::cout << tc::blue << "\t" << "Elapsed time: " << stats.time_s << "\n";

    if(params.output_to_file) {
      save_stats_to_file(false);
    }
  }

  void CpSolver::save_stats_to_file(bool feasible) {
    const auto fname = "stats-CP-" + i.name + ".csv";
    const auto fpath = std::filesystem::path(params.output_folder) / fname;
    std::ofstream ofs{fpath, std::ofstream::out};

    ofs << "model,feasible," << Instance::header << "," << ModelStats::header << "," << CpSolverParams::header << "\n";
    ofs << "CP," << std::boolalpha << feasible << "," << i << "," << stats << "," << params << "\n";
  }

  ElevatorHistory CpSolver::deal_with_feasible() {
    namespace tc = termcolor;

    stats.time_s = cp.getInfo(IloCP::NumInfo::SolveTime);
    stats.primal = cp.getObjValue();
    stats.dual = cp.getObjBound();
    stats.gap = std::min(cp.getObjGap(), 1.0);
    stats.n_cols = cp.getInfo(IloCP::IntInfo::NumberOfVariables);
    stats.n_rows = cp.getInfo(IloCP::IntInfo::NumberOfConstraints);

    using StartTrayTask = std::tuple<size_t, size_t, size_t>;
    std::vector<StartTrayTask> tts;

    ElevatorHistory h;

    for(auto i1 = 0; i1 < n_tasks; ++i1) {
      const auto[t, j] = tmap.right.at(i1);
      const size_t s_start_time = cp.getStart(task_interval[i1]);

      assert(cp.getEnd(task_interval[i1]) == (IloInt) (s_start_time + i.tray(t).task(j).duration));
      tts.emplace_back(s_start_time, t, j);
    }

    std::sort(tts.begin(), tts.end());

    for(auto [s_start_time, t, j] : tts) {
      const auto& tray = i.tray(t);
      const auto& task = tray.task(j);
      const auto shelf = (task.type == TaskType::NORMAL) ? tray.shelf : Instance::depot_id;
      const std::string comment = task.type == TaskType::NORMAL ?
                                  "REGULAR" : task.type == TaskType::PLANTING ?
                                              "DEPOT_PICKUP_PLANTING" : "DEPOT_DELIVERY_HARVEST";

      if(task.type == TaskType::HARVEST) {
        h.push_back({s_start_time, tray.shelf, t, j, false, "SHELF_PICKUP_HARVEST"});
      }

      for(auto k = s_start_time; k <= s_start_time + task.duration; ++k) {
        h.push_back({k, shelf, t, j, false, comment});
      }

      if(task.type == TaskType::PLANTING) {
        h.push_back({s_start_time + task.duration, tray.shelf, t, j, false, "SHELF_DELIVERY_PLANTING"});
      }
    }

    if(!check_feasible(h, i)) {
      std::cout << tc::red << "Constraint Programming solution not feasible!" << tc::reset << "\n";
      std::cout << h << "\n";
    }

    std::cout << tc::blue << "\tElapsed time: " << stats.time_s << "\n";
    std::cout << "\tSolution cost: " << compute_cost(h) << "\n" << tc::reset;

    if(params.output_to_file) {
      save_stats_to_file(true);
      save_solution_to_file(h);
    }

    return h;
  }

  void CpSolver::save_solution_to_file(const ElevatorHistory& h) {
    const auto fname = "sol-CP-" + i.name + ".json";
    const auto fpath = std::filesystem::path(params.output_folder) / fname;
    std::ofstream ofs{fpath, std::ofstream::out};

    ofs << to_json(h) << "\n";
  }

  void CpSolver::build_model() {
    using std::make_tuple;

    const auto tot_n_tasks = std::accumulate(i.trays.begin(), i.trays.end(), 0u,
      [] (auto accum, const auto& tray) { return accum + tray.tasks.size(); });

    n_tasks = (IloInt) tot_n_tasks;
    task_index = IloIntArray{env, n_tasks};
    cost = IloIntArray2{env, n_tasks};

    IloInt task_id = 0;
    for(auto t : i.tray_idx) {
      for(auto j : i.tray(t).task_idx) {
        task_index[task_id] = task_id;
        tmap.insert({make_tuple(t, j), task_id});
        ++task_id;
      }
    }

    for(auto i1 = 0; i1 < n_tasks; ++i1) {
      cost[i1] = IloIntArray{env, n_tasks};
      const auto [t1, j1] = tmap.right.at(i1);
      for(auto i2 = 0; i2 < n_tasks; ++i2) {
        const auto [t2, j2] = tmap.right.at(i2);
        cost[i1][i2] = get_cost(t1, j1, t2, j2, i);
      }
    }

    task_interval = IloIntervalVarArray{env, n_tasks};

    for(auto i1 = 0; i1 < n_tasks; ++i1) {
      const auto [t1, j1] = tmap.right.at(i1);
      const auto& tray1 = i.tray(t1);
      const auto& task1 = tray1.task(j1);

      task_interval[i1] = IloIntervalVar{env, (IloInt) task1.duration};
      task_interval[i1].setStartMin((IloInt) i.alpha(t1, j1));
      task_interval[i1].setStartMax((IloInt) i.beta(t1, j1));
      task_interval[i1].setLengthMin((IloInt) task1.duration);
      task_interval[i1].setLengthMax((IloInt) task1.duration);
      task_interval[i1].setEndMin((IloInt) i.alpha(t1, j1) + (IloInt) task1.duration);
      task_interval[i1].setEndMax((IloInt) i.beta(t1, j1) + (IloInt) task1.duration);
    }

    for(auto t : i.tray_idx) {
      const auto& tray = i.tray(t);
      const IloInt idx_start = tmap.left.at(make_tuple(t, tray.start_task_id()));

      for(auto j : tray.task_idx_ns) {
        const auto& task = tray.task(j);
        const IloInt idx_cur = tmap.left.at(make_tuple(t, j));
        const IloInt idx_prev = tmap.left.at(make_tuple(t, j - 1u));

        model.add(IloStartBeforeStart(env, task_interval[idx_start], task_interval[idx_cur], (IloInt) task.start));
        model.add(IloStartOf(task_interval[idx_cur]) - IloStartOf(task_interval[idx_start]) <= (IloInt) task.end);
        model.add(IloEndBeforeStart(env, task_interval[idx_prev], task_interval[idx_cur]));
      }
    }

    for(auto t1 : i.tray_idx) {
      const auto& tray1 = i.tray(t1);
      for(auto t2 : i.tray_idx) {
        const auto& tray2 = i.tray(t2);

        if(t1 == t2) { continue; }
        if(tray1.shelf != tray2.shelf) { continue; }

        auto idx1start = tmap.left.at(make_tuple(t1, tray1.start_task_id()));
        auto idx1end = tmap.left.at(make_tuple(t1, tray1.end_task_id()));
        auto idx2start = tmap.left.at(make_tuple(t2, tray2.start_task_id()));
        auto idx2end = tmap.left.at(make_tuple(t2, tray2.end_task_id()));

        model.add(IloStartOf(task_interval[idx1start]) >= IloEndOf(task_interval[idx2end]) ||
                  IloStartOf(task_interval[idx2start]) >= IloEndOf(task_interval[idx1end]));
      }
    }

    task_seq = IloIntervalSequenceVar{env, task_interval, task_index};
    model.add(IloNoOverlap(env, task_seq));

    IloIntExpr obj{env};
    for(auto i1 = 0; i1 < n_tasks; ++i1) {
      obj += cost[i1][IloTypeOfNext(task_seq, task_interval[i1], 0)];
    }
    model.add(IloMinimize(env, obj));
  }

  std::optional<ElevatorHistory> CpSolver::solve() {
    namespace tc = termcolor;

    cp = IloCP{model};
    cp.setParameter(IloCP::SearchType, IloCP::MultiPoint);
    cp.setParameter(IloCP::TimeLimit, params.timeout_s);

    try {
      if(!cp.solve()) {
        std::cout << tc::yellow << "CP did not find any feasible solution within the time limit!\n" << tc::reset;
        deal_with_infeasible();
        return std::nullopt;
      }

      return deal_with_feasible();
    } catch(const IloException& e) {
      std::cout << tc::red << "Cplex exception: " << e.getMessage() << "\n" << tc::reset;
      deal_with_infeasible();
      return std::nullopt;
    }
  }
}