//
// Created by Alberto Santini on 18/06/2021.
//

#include "greedy_heuristic.h"
#include "termcolor/termcolor.hpp"
#include <algorithm>
#include <string>
#include <chrono>

namespace elevator {
  namespace {
    bool task_stable_order(const ElevatorStatus& s1, const ElevatorStatus& s2) {
      if(s1.time < s2.time) { return true; }
      if(s1.time > s2.time) { return false; }

      // All next cases assume time is the same

      if(s1.tray == s2.tray && s1.task == s2.task) {
        // Case 1: end of planting
        if(s1.comment == "DEPOT_PICKUP_PLANTING" && s2.comment == "SHELF_DELIVERY_PLANTING") { return true; }
        else if(s1.comment == "SHELF_DELIVERY_PLANTING" && s2.comment == "DEPOT_PICKUP_PLANTING") { return false; }

        // Case 2: start of harvesting
        if(s1.comment == "SHELF_PICKUP_HARVEST" && s2.comment == "DEPOT_DELIVERY_HARVEST") { return true; }
        else if(s1.comment == "DEPOT_DELIVERY_HARVEST" && s2.comment == "SHELF_PICKUP_HARVEST") { return false; }

        // No other case possible!
        assert(false);
      }

      // No other case possible!
      assert(false);
      return false; // <- avoids "control reaches end of non-void function"
    }

    struct PartialSolution {
      ElevatorHistory h;
      size_t unassigned;
      size_t cost;

      bool operator<(const PartialSolution& other) const {
        return std::tie(unassigned, cost) < std::tie(other.unassigned, other.cost);
      }
    };

    PartialSolution greedy_by_tray(const elevator::Instance& i) {
      namespace tc = termcolor;

      std::cout << tc::blue << "Running the greedy-tray heuristic..." << tc::reset << "\n";

      ElevatorHistory h;

      // busy[k] == true if the elevator is busy at time k
      std::vector<bool> busy(i.n_time_instants + 1, false);

      // planting_time[t] == the start time of the planting task for tray t,
      // or nullopt if we could not find a time to plant t
      std::vector<std::optional<size_t>> planting_time(i.n_trays, std::nullopt);

      // give_up = list of (tray, task) we did not schedule
      std::vector<std::tuple<size_t, size_t>> give_up;

      auto sorted_tray_idx = i.tray_idx;
      std::sort(sorted_tray_idx.begin(), sorted_tray_idx.end(), [&] (size_t t1, size_t t2) -> bool {
        return i.tray(t1).start_task().start < i.tray(t2).start_task().start;
      });

      for(auto t : sorted_tray_idx) {
        const auto& tray = i.tray(t);

        for(auto j : tray.task_idx) {
          const auto& task = tray.task(j);
          bool could_schedule = false;

          size_t start_time, end_time;
          if(task.type == TaskType::PLANTING) {
            start_time = task.start;
            end_time = task.end;
          } else if(planting_time[t]) { // We managed to find a time to plant this tray
            start_time = *planting_time[t] + task.start;
            end_time = *planting_time[t] + task.end;
          } else {
            start_time = i.alpha(t, j);
            end_time = i.beta(t, j);
          }

          for(auto k = start_time; k <= end_time; ++k) {
            bool start_at_k = true;

            for(auto kk = k; kk <= k + task.duration; ++kk) {
              if(busy[kk]) {
                start_at_k = false;
                break;
              }
            }

            if(!start_at_k) { continue; }

            if(task.type == TaskType::HARVEST) {
              h.push_back({k, tray.shelf, t, j, false, "SHELF_PICKUP_HARVEST"});
            }
            for(auto kk = k; kk <= k + task.duration; ++kk) {
              busy[kk] = true;
              const size_t shelf = (task.type == TaskType::NORMAL ? tray.shelf : Instance::depot_id);
              const std::string comment = task.type == TaskType::NORMAL ?
                "REGULAR" : task.type == TaskType::PLANTING ?
                "DEPOT_PICKUP_PLANTING" : "DEPOT_DELIVERY_HARVEST";

              h.push_back({kk, shelf, t, j, false, comment});
            }
            if(task.type == TaskType::PLANTING) {
              planting_time[t] = k;
              h.push_back({k + task.duration, tray.shelf, t, j, false, "SHELF_DELIVERY_PLANTING"});
            }

            could_schedule = true;
            break;
          }

          if(!could_schedule) {
            give_up.emplace_back(t, j);
          }
        }
      }

      std::stable_sort(h.begin(), h.end(), task_stable_order);

      if(give_up.empty()) {
        std::cout << tc::green << "Added all tasks to the greedy-trays solution!" << tc::reset << "\n";
      } else {
        const auto n_tasks = std::accumulate(i.trays.begin(), i.trays.end(), 0u,
          [] (size_t sum, const Tray& tray) -> size_t { return sum + tray.tasks.size(); });
        std::cout << tc::yellow << "Added " << (n_tasks - give_up.size()) << " tasks out of " << n_tasks << tc::reset << "\n";
      }

      if(!check_feasible(h, i)) {
        std::cout << tc::red << "Greedy-trays solution not feasible!" << tc::reset << "\n";
        std::cout << h << "\n";
      }

      return {h, give_up.size(), compute_cost(h)};
    }

    PartialSolution greedy_by_time(const elevator::Instance& i) {
      namespace tc = termcolor;

      std::cout << tc::blue << "Running the greedy-time heuristic..." << tc::reset << "\n";

      ElevatorHistory h;

      // busy[k] == true if the elevator is busy at time k
      std::vector<bool> busy(i.n_time_instants + 1, false);

      // planting_time[t] == the start time of the planting task for tray t,
      // or nullopt if we could not find a time to plant t
      std::vector<std::optional<size_t>> planting_time(i.n_trays, std::nullopt);

      // give_up = list of (tray, task) we did not schedule
      std::vector<std::tuple<size_t, size_t>> give_up;

      auto sorted_tray_idx = i.tray_idx;
      std::sort(sorted_tray_idx.begin(), sorted_tray_idx.end(), [&] (size_t t1, size_t t2) -> bool {
        return i.tray(t1).start_task().start < i.tray(t2).start_task().start;
      });

      for(auto t : sorted_tray_idx) {
        const auto& tray = i.tray(t);
        const auto& plant = tray.start_task();
        planting_time[t] = plant.start;
        bool could_plant = false;

        for(auto k = plant.start; k <= plant.end; ++k) {
          bool elev_busy = false;

          for(auto kk = k; kk <= k + plant.duration; ++kk) {
            if(busy[kk]) {
              elev_busy = true;
              break;
            }
          }

          if(elev_busy) { continue; }

          planting_time[t] = k;
          for(auto kk = k; kk <= k + plant.duration; ++kk) {
            busy[kk] = true;

            h.push_back({
              kk, Instance::depot_id, t, tray.start_task_id(), false, "DEPOT_PICKUP_PLANTING"
            });
          }
          h.push_back({k + plant.duration, tray.shelf, t, tray.start_task_id(), false, "SHELF_DELIVERY_PLANTING"});

          could_plant = true;

          break;
        }

        if(!could_plant) {
          give_up.emplace_back(t, tray.start_task_id());
        }
      }

      using StartTrayTask = std::tuple<size_t, size_t, size_t>;
      std::vector<StartTrayTask> to_assign;

      for(auto t : i.tray_idx) {
        for(auto j : i.tray(t).task_idx_ns) {
          auto abs_start = i.tray(t).task(j).start;

          if(planting_time[t]) {
            abs_start += *planting_time[t];
          } else {
            abs_start += i.tray(t).start_task().start;
          }

          to_assign.emplace_back(abs_start, t, j);
        }
      }

      std::sort(to_assign.begin(), to_assign.end());

      for(const auto& ta : to_assign) {
        const auto [start_time, t, j] = ta;
        const auto& tray = i.tray(t);
        const auto& task = tray.task(j);
        bool could_assign = false;

        for(auto k = start_time; k <= start_time - task.start + task.end; ++k) {
          if(k + task.duration > i.n_time_instants) { break; }

          bool elev_busy = false;

          for(auto kk = k; kk <= k + task.duration; ++kk) {
            if(busy[kk]) {
              elev_busy = true;
              break;
            }
          }

          if(elev_busy) { continue; }
          
          if(task.type == TaskType::HARVEST) {
            h.push_back({k, tray.shelf, t, j, false, "SHELF_PICKUP_HARVEST"});
          }

          for(auto kk = k; kk <= k + task.duration; ++kk) {
            const std::string comment = (task.type == TaskType::NORMAL ? "REGULAR" : "DEPOT_DELIVERY_HARVEST");
            const size_t shelf = (task.type == TaskType::NORMAL ? tray.shelf : Instance::depot_id);

            busy[kk] = true;
            h.push_back({kk, shelf, t, j, false, comment});
          }
          could_assign = true;

          break;
        }

        if(!could_assign) {
          give_up.emplace_back(t, j);
        }
      }

      std::stable_sort(h.begin(), h.end(), task_stable_order);

      if(give_up.empty()) {
        std::cout << tc::green << "Added all tasks to the greedy-time solution!" << tc::reset << "\n";
      } else {
        const auto n_tasks = to_assign.size() + i.n_trays;
        std::cout << tc::yellow << "Added " << (n_tasks - give_up.size()) << " tasks out of " << n_tasks << tc::reset << "\n";
      }

      if(!check_feasible(h, i)) {
        std::cout << tc::red << "Greedy-time solution not feasible!" << tc::reset << "\n";
        std::cout << h << "\n";
      }

      return {h, give_up.size(), compute_cost(h)};
    }
  }

  ElevatorHistory generate_greedy_solution(const elevator::Instance& i) {
    namespace tc = termcolor;
    using namespace std::chrono;

    const auto time_1 = steady_clock::now();
    const auto sol1 = greedy_by_time(i);
    const auto time_2 = steady_clock::now();

    std::cout << tc::italic << "Greedy by time completed in " << duration_cast<seconds>(time_2 - time_1).count() << " seconds\n" << tc::reset;

    const auto sol2 = greedy_by_tray(i);
    const auto time_3 = steady_clock::now();

    std::cout << tc::italic << "Greedy by tray completed in " << duration_cast<seconds>(time_3 - time_2).count() << " seconds\n" << tc::reset;
    
    return sol1 < sol2 ? sol1.h : sol2.h;
  }
}