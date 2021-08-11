#ifndef _MODEL2_PARAMS_H
#define _MODEL2_PARAMS_H

#include <iostream>
#include <string>
#include <optional>
#include <filesystem>
#include <cmath>

namespace elevator {
  struct Model2Params {
    std::string output_folder = "../solutions/raw";
    std::optional<std::filesystem::path> initial_sol_path = std::nullopt;
    double max_bb_nodes = std::nan("");
    bool disable_gurobi_presolve = false;
    bool use_valid_inequality4 = true;
    bool use_valid_inequality5 = true;
    bool save_model = false;
    bool check_all_incumbents = false;
    float timeout_s = 3600.0f;

    static const std::string header;
  };

  std::ostream& operator<<(std::ostream& out, const Model2Params& params);
}


#endif
