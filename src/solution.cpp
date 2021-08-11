#include "solution.h"
#include <json.hpp>
#include <termcolor.hpp>
#include <optional>
#include <iostream>
#include <fstream>
#include <cassert>
#include <exception>

namespace {
  struct ShelfLife {
    std::optional<size_t> start_depot;
    std::optional<size_t> start_shelf;
    std::optional<size_t> end_shelf;
    std::optional<size_t> end_depot;

    ShelfLife() : start_depot{std::nullopt}, start_shelf{std::nullopt}, end_shelf{std::nullopt}, end_depot{std::nullopt} {}
    [[nodiscard]] bool any_undefined() const { return !(start_depot && start_shelf && end_shelf && end_depot); }
  };

  [[maybe_unused]] std::ostream& operator<<(std::ostream& out, const ShelfLife& sl) {
    if(sl.start_depot) { out << *sl.start_depot; } else { out << "?"; } out << " ";
    if(sl.start_shelf) { out << *sl.start_shelf; } else { out << "?"; } out << " ";
    if(sl.end_shelf) { out << *sl.end_shelf; } else { out << "?"; } out << " ";
    if(sl.end_depot) { out << *sl.end_depot; } else { out << "?"; }
    return out;
  }
}

namespace elevator {
  std::string to_json(const ElevatorHistory& eh) {
    using json = nlohmann::json;
    json sol;

   for(const auto& es : eh) {
     json obj;
     obj["time"] = es.time;
     obj["shelf"] = es.shelf;

     if(es.tray) {
       obj["tray"] = *es.tray;
     }

     if(es.task) {
       obj["task"] = *es.task;
     }

     obj["idle"] = es.idle;
     obj["comment"] = es.comment;

     sol.push_back(obj);
   }

    return sol.dump(2);
  }

  bool check_within_tw(const ElevatorHistory& eh, const Instance& i) {
    namespace tc = termcolor;
    bool feasible = true;

    std::vector<std::vector<std::optional<size_t>>> start_time(i.n_trays);
    for(const auto t : i.tray_idx) {
      start_time[t] = std::vector<std::optional<size_t>>(i.tray(t).tasks.size(), std::nullopt);
    }

    for(const auto& es : eh) {
      if(es.tray && es.task && !start_time[*es.tray][*es.task]) {
        const auto t = *es.tray;
        const auto j = *es.task;
        const auto k = es.time;

        start_time[t][j] = k;
        if((j == i.tray(t).start_task_id())) {
          if(k < i.alpha(t, j) || k > i.beta(t, j)) {
            std::cerr << tc::red << tc::bold << "/!\\ Error:" <<
              " start task " << j << " of tray " << t << " starts at" <<
              " time " << k << ", but its TW is [" << i.alpha(t, j) << ", " <<
              i.beta(t, j) << tc::reset << "\n";
            feasible = false;
          }
        } else if(start_time[t][i.tray(t).start_task_id()]) {
          const auto tray_start = *start_time[t][i.tray(t).start_task_id()];
          assert(k >= tray_start);

          if(k - tray_start < i.tray(t).task(j).start ||
             k - tray_start > i.tray(t).task(j).end) {
            std::cerr << tc::red << tc::bold << "/!\\ Error:" <<
                      " task " << j << " of tray " << t << " starts at" <<
                      " time " << k << "\nThe start task of the tray" <<
                      " started at " << tray_start << ", giving a relative" <<
                      " start time for task " << j << " of " << k - tray_start <<
                      "\nThis is outside time window [" << i.tray(t).task(j).start <<
                      ", " << i.tray(t).task(j).end << "]" << tc::reset << "\n";
            feasible = false;
          } // else, no start time for this tray (which is possible if the solution is partial).
        }
      }
    }

    return feasible;
  }

  size_t compute_cost(const ElevatorHistory& eh) {
    size_t cost = 0u;

    for(auto k = 0u; k < eh.size() - 1u; ++k) {
      cost += std::abs((long)eh[k].shelf - (long)eh[k + 1u].shelf);
    }

    return cost;
  }

  ElevatorHistory from_json(const std::filesystem::path& json_file) {
    namespace tc = termcolor;
    using nlohmann::json;

    ElevatorHistory eh;
    std::ifstream ifs{json_file};

    if(ifs.fail()) {
      std::cerr << tc::red << "Cannot read from file " << json_file << tc::reset << "\n";
      throw std::runtime_error("Cannot read elevator history from file");
    }

    json j;
    ifs >> j;

    for(const auto& obj : j) {
      ElevatorStatus es{
        obj["time"].get<size_t>(),
        obj["shelf"].get<size_t>(),
        std::nullopt,
        std::nullopt,
        obj["idle"].get<bool>(),
        obj["comment"].get<std::string>()
      };
      if(obj.count("tray")) {
        es.tray = obj["tray"].get<size_t>();
      }
      if(obj.count("task")) {
        es.task = obj["task"].get<size_t>();
      }
      eh.push_back(std::move(es));
    }

    return eh;
  }

  bool check_no_overlaps(const ElevatorHistory& eh, const Instance& i) {
    namespace tc = termcolor;
    std::vector<ShelfLife> lives(i.n_trays, ShelfLife{});

    for(const auto& es : eh) {
      if(es.comment == "DEPOT_PICKUP_PLANTING") {
        assert(es.tray);
        assert(es.shelf == Instance::depot_id);

        const auto t = *es.tray;

        // Earliest moment with DEPOT_PICKUP_PLANTING is the start_depot
        if(!lives[t].start_depot || *lives[t].start_depot > es.time) {
          lives[t].start_depot = es.time;
        }
      } else if(es.comment == "SHELF_DELIVERY_PLANTING") {
        assert(es.tray);

        const auto t = *es.tray;

        assert(es.shelf == i.tray(t).shelf);

        // Unique moment with SHELF_DELIVERY_PLANTING is the start_shelf
        assert(!lives[t].start_shelf);
        lives[t].start_shelf = es.time;
      } else if(es.comment == "SHELF_PICKUP_HARVEST") {
        assert(es.tray);

        const auto t = *es.tray;

        assert(es.shelf == i.tray(t).shelf);

        // Unique moment with SHELF_PICKUP_HARVEST is end_shelf
        assert(!lives[t].end_shelf);
        lives[t].end_shelf = es.time;
      } else if(es.comment == "DEPOT_DELIVERY_HARVEST") {
        assert(es.tray);
        assert(es.shelf == Instance::depot_id);

        const auto t = *es.tray;

        // Latest moment with DEPOT_DELIVERY_HARVEST is end_depot
        if(!lives[t].end_depot || *lives[t].end_depot < es.time) {
          lives[t].end_depot = es.time;
        }
      }
    }

    const auto overlap = [] (size_t a1, size_t b1, size_t a2, size_t b2) -> bool {
      return (a1 <= b2) && (a2 <= b1);
    };

    for(const auto t1 : i.tray_idx_nd) {
      if(lives[t1].any_undefined()) {
        if(!lives[t1].start_depot) {
          std::cerr << tc::red << tc::bold << "/!\\ Error:" <<
            " Tray " << t1 << " does not have a start depot time\n" << tc::reset;
          return false;
        }

        if(!lives[t1].start_shelf) {
          std::cerr << tc::red << tc::bold << "/!\\ Error:" <<
            " Tray " << t1 << " does not have a start shelf time\n" << tc::reset;
          return false;
        }

        if(!lives[t1].end_shelf) {
          std::cerr << tc::red << tc::bold << "/!\\ Error:" <<
            " Tray " << t1 << " does not have an end shelf time\n" << tc::reset;
          return false;
        }

        if(!lives[t1].end_depot) {
          std::cerr << tc::red << tc::bold << "/!\\ Error:" <<
            " Tray " << t1 << " does not have an end depot time\n" << tc::reset;
          return false;
        }
      }

      for(const auto t2 : i.tray_idx_nd) {
        if(t2 <= t1) { continue; }

        // First depot TW of t1 vs first depot TW of t2
        if(overlap(*lives[t1].start_depot, *lives[t1].start_shelf - 1u, *lives[t2].start_depot, *lives[t2].start_shelf - 1u)) {
          std::cerr << tc::red << tc::bold << "/!\\ Warning:" <<
            " Trays " << t1 << " (start: " << *lives[t1].start_depot << ", " <<
            *lives[t1].start_shelf - 1u << ") and " << t2 << " (start: " <<
            *lives[t2].start_depot << ", " << *lives[t2].start_shelf - 1u <<
            ") are both on the depot shelf at the same time!\n" << tc::reset;
          return false;
        }

        // First depot TW of t1 vs second depot TW of t2
        if(overlap(*lives[t1].start_depot, *lives[t1].start_shelf - 1u, *lives[t2].end_shelf, *lives[t2].end_depot - 1u)) {
          std::cerr << tc::red << tc::bold << "/!\\ Warning:" <<
            " Trays " << t1 << " (start: " << *lives[t1].start_depot << ", " <<
            *lives[t1].start_shelf - 1u << ") and " << t2 << " (end: " <<
            *lives[t2].end_shelf << ", " << *lives[t2].end_depot - 1u <<
            ") are both on the depot shelf at the same time!\n" << tc::reset;
          return false;
        }

        // Second depot TW of t1 vs first depot TW of t2
        if(overlap(*lives[t1].end_shelf, *lives[t1].end_depot - 1u, *lives[t2].start_depot, *lives[t2].start_shelf - 1u)) {
          std::cerr << tc::red << tc::bold << "/!\\ Warning:" <<
            " Trays " << t1 << " (end: " << *lives[t1].end_shelf << ", " <<
            *lives[t1].end_depot - 1u << ") and " << t2 << " (start: " <<
            *lives[t2].start_depot << ", " << *lives[t2].start_shelf - 1u <<
            ") are both on the depot shelf at the same time!\n" << tc::reset;
          return false;
        }

        // Second depot TW of t1 vs second depot TW of t2
        if(overlap(*lives[t1].end_shelf, *lives[t1].end_depot - 1u, *lives[t2].end_shelf, *lives[t2].end_depot - 1u)) {
          std::cerr << tc::red << tc::bold << "/!\\ Warning:" <<
            " Trays " << t1 << " (end: " << *lives[t1].end_shelf << ", " <<
            *lives[t1].end_depot - 1u << ") and " << t2 << " (end: " <<
            *lives[t2].end_shelf << ", " << *lives[t2].end_depot - 1u <<
            ") are both on the depot shelf at the same time!\n" << tc::reset;
          return false;
        }

        // Shelf TW of t1 overlaps with shelf TW of t2
        if(i.tray(t1).shelf == i.tray(t2).shelf && overlap(*lives[t1].start_shelf, *lives[t1].end_shelf, *lives[t2].start_shelf, *lives[t2].end_shelf)) {
          std::cerr << tc::red << tc::bold << "/!\\ Error:" <<
            " Trays " << t1 << " (" << *lives[t1].start_shelf << ", " <<
            *lives[t1].end_shelf - 1u << ") and " << t2 << " (" <<
            *lives[t2].start_shelf << ", " << *lives[t2].end_shelf - 1u <<
            ") are both on shelf " << i.tray(t1).shelf << " at the same time!\n" << tc::reset;
          return false;
        }
      }
    }

    return true;
  }

  bool check_feasible(const ElevatorHistory& eh, const Instance& i) {
    return check_within_tw(eh, i) && check_no_overlaps(eh, i);
  }

  std::ostream& operator<<(std::ostream& out, const ElevatorStatus& es) {
    out << "time: " << es.time << ", shelf: " << es.shelf << ", tray: ";
    if(es.tray) { out << *es.tray; } else { out << "none"; }
    out << ", task: ";
    if(es.task) { out << *es.task; } else { out << "none"; }
    out << ", idle: " << std::boolalpha << es.idle << ", comment: " << es.comment;
    return out;
  }

  std::ostream& operator<<(std::ostream& out, const ElevatorHistory& eh) {
    for(const auto& es : eh) {
      out << es << "\n";
    }
    return out;
  }
}