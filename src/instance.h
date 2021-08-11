#ifndef _INSTANCE_H
#define _INSTANCE_H

#include "tray.h"
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <filesystem>
#include <iostream>

namespace elevator {
  struct Instance {
    std::string name;
    size_t n_shelves;
    size_t n_trays;
    size_t n_time_instants;
    size_t total_task_durations;

    std::vector<Tray> trays;
    [[nodiscard]] const Tray& tray(size_t tray_id) const { return trays.at(tray_id); }

    // For modelling purposes
    std::vector<size_t> tray_idx; // Indices of all trays, starting from 0
    std::vector<size_t> tray_idx_nd; // Indices of all trays, excluding the dummy tray (which is tray 0)
    std::vector<size_t> shelf_idx; // Indices of all shelves, starting from 0
    std::vector<size_t> shelf_idx_nd; // Indices of all shelves, excluding the depot (which is shelf 0)
    std::vector<size_t> time_instant_idx; // Indices of time instants, starting from 1
    [[nodiscard]] size_t alpha(size_t tray_id, size_t task_id) const;
    [[nodiscard]] size_t beta(size_t tray_id, size_t task_id) const;

    using tj_index = std::tuple<size_t, size_t>;
    std::map<tj_index, size_t> L_earliest;
    std::map<tj_index, size_t> L_latest;

    using tjk_index = std::tuple<size_t, size_t, size_t>;
    std::map<tjk_index, std::vector<size_t>> big_omega;

    explicit Instance(const std::filesystem::path& inst_path, bool needs_big_omega = true);

    static const std::string header;
    static const size_t depot_id = 0u;
    static const size_t dummy_id = 0u;

  private:
    std::vector<size_t> compute_big_omega(size_t tray_id, size_t task_id, size_t time) const;
    size_t compute_L_earliest(size_t tray_id, size_t task_id) const;
    size_t compute_L_latest(size_t tray_id, size_t task_id) const;
  };

  std::ostream& operator<<(std::ostream& out, const Instance& i);
}

#endif