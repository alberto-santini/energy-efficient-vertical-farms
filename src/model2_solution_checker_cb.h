//
// Created by Alberto Santini on 01/06/2021.
//

#ifndef ELEVATOR_MODEL2_SOLUTION_CHECKER_CB_H
#define ELEVATOR_MODEL2_SOLUTION_CHECKER_CB_H

#include "model2_solver.h"
#include "instance.h"
#include "solution.h"
#include <gurobi_c++.h>
#include <tuple>

namespace elevator {
  struct Model2SolutionCheckerCB : public GRBCallback {
    const Model2Solver::x_map& xi;
    const GRBVar *const x;
    const Instance& i;

    Model2SolutionCheckerCB(const Model2Solver::x_map& xi, const GRBVar *const x, const Instance& i) : xi{xi}, x{x}, i{i} {}

    void callback() override {
      if(where == GRB_CB_MIPSOL) {
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

              if(getSolution(x[x_tjk]) > 0.5) {
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

        if(!check_feasible(hist, i)) {
          abort();
        }
      }
    }
  };
}

#endif //ELEVATOR_MODEL2_SOLUTION_CHECKER_CB_H
