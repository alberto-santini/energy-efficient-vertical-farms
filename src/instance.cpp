#include "instance.h"
#include <fstream>
#include <numeric>
#include <cassert>
#include <optional>
#include <chrono>
#include <filesystem>
#include <json.hpp>
#include <termcolor.hpp>

namespace elevator {
  const std::string Instance::header = "name,n_shelves,n_trays,n_time_instants";

  Instance::Instance(const std::filesystem::path& inst_path, bool needs_big_omega) {
    namespace tc = termcolor;
    using namespace std::chrono;

    std::ifstream ifs{inst_path};
    nlohmann::json obj;

    const auto time_1 = steady_clock::now();

    ifs >> obj;

    name = inst_path.stem();
    n_shelves = obj["n_shelves"];
    n_trays = obj["n_trays"];
    n_time_instants = obj["time_horizon_len"];

    n_shelves += 1u; // To account for the depot
    n_trays += 1u; // To account for the dummy tray (0)

    trays.emplace_back(Tray::get_dummy_tray(n_time_instants));
    for(const auto& tray : obj["trays"]) {
      trays.emplace_back(tray);
    }

    tray_idx = std::vector<size_t>(trays.size());
    std::iota(tray_idx.begin(), tray_idx.end(), 0u);

    tray_idx_nd = std::vector<size_t>(trays.size() - 1u);
    std::iota(tray_idx_nd.begin(), tray_idx_nd.end(), 1u);

    shelf_idx = std::vector<size_t>(n_shelves);
    std::iota(shelf_idx.begin(), shelf_idx.end(), 0u);

    shelf_idx_nd = std::vector<size_t>(n_shelves - 1u);
    std::iota(shelf_idx_nd.begin(), shelf_idx_nd.end(), 1u);

    time_instant_idx = std::vector<size_t>(n_time_instants);
    std::iota(time_instant_idx.begin(), time_instant_idx.end(), 1u);

    const auto time_2 = steady_clock::now();
    std::cout << tc::italic << "Read model from file (" << duration_cast<seconds>(time_2 - time_1).count() << "s)\n";

    total_task_durations = 0u;
    for(auto t : tray_idx) {
      for(auto j : tray(t).task_idx) {
        total_task_durations += tray(t).task(j).duration;

        if(needs_big_omega) {
          for(auto k : time_instant_idx) {
            big_omega[std::make_tuple(t, j, k)] = compute_big_omega(t, j, k);
          }
        }

        L_earliest[std::make_tuple(t, j)] = compute_L_earliest(t, j);
        L_latest[std::make_tuple(t, j)] = compute_L_latest(t, j);
      }
    }

    const auto time_3 = steady_clock::now();
    std::cout << "Build auxiliary data structures Omega and L (" << duration_cast<seconds>(time_3 - time_2).count() << "s)\n" << tc::reset;
  }

  size_t Instance::compute_L_earliest(size_t tray_id, size_t task_id) const {
    std::optional<size_t> earliest;
    const auto a = alpha(tray_id, task_id);
    const auto d = tray(tray_id).task(task_id).duration;

    for(auto t : tray_idx) {
      for(auto j : tray(t).task_idx) {
        if(t == tray_id && j == task_id) { continue; }
        if(a + d > beta(t, j)) {
          if(!earliest || *earliest < alpha(t, j) + tray(t).task(j).duration) {
            earliest = alpha(t, j) + tray(t).task(j).duration;
          }
        }
      }
    }

    if(earliest) {
      assert(*earliest <= beta(tray_id, task_id));
      return std::max(a, *earliest);
    } else {
      return a;
    }
  }

  size_t Instance::compute_L_latest(size_t tray_id, size_t task_id) const {
    std::optional<size_t> latest;
    const auto b = beta(tray_id, task_id);

    for(auto t : tray_idx) {
      for(auto j : tray(t).task_idx) {
        if(t == tray_id && j == task_id) { continue; }
        if(alpha(t, j) + tray(t).task(j).duration > b) {
          if(!latest || *latest > beta(t, j)) {
            latest = beta(t, j);
          }
        }
      }
    }

    if(latest) {
      const auto d = tray(tray_id).task(task_id).duration;
      assert(*latest >= d);
      assert(*latest - d >= alpha(tray_id, task_id));
      return std::min(b, *latest - d);
    } else {
      return b;
    }
  }

  size_t Instance::alpha(size_t tray_id, size_t task_id) const {
    const Tray& tray = trays.at(tray_id);
    const Task& task = tray.tasks.at(task_id);

    if(task.type == TaskType::PLANTING) {
      return std::min(n_time_instants, task.start);
    } else {
      return std::min(n_time_instants, tray.start_task().start + task.start);
    }
  }

  size_t Instance::beta(size_t tray_id, size_t task_id) const {
    const Tray& tray = trays.at(tray_id);
    const Task& task = tray.tasks.at(task_id);

    if(task.type == TaskType::PLANTING) {
      return std::min(n_time_instants, task.end);
    } else {
      return std::min(n_time_instants, tray.start_task().end + task.end);
    }
  }

  std::vector<size_t> Instance::compute_big_omega(size_t tray_id, size_t task_id, size_t time) const {
    std::vector<size_t> omega;

    for(auto k = alpha(tray_id, task_id); k <= beta(tray_id, task_id); ++k) {
      if(k <= time && k + tray(tray_id).task(task_id).duration > time) {
        omega.push_back(k);
      }
    }

    return omega;
  }

  std::ostream& operator<<(std::ostream& out, const Instance& i) {
    out << i.name << "," << i.n_shelves - 1u << ","
        << i.n_trays - 1u << "," << i.n_time_instants;
    return out;
  }
}