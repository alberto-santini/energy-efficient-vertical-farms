#ifndef _MODEL3_PARAMS_H
#define _MODEL3_PARAMS_H

#include <iostream>
#include <string>
#include <optional>
#include <filesystem>
#include <cmath>

namespace elevator {
  struct Model3Params {
    std::string output_folder = "../solutions/raw";
    std::optional<std::filesystem::path> initial_sol_path = std::nullopt;
    double max_bb_nodes = std::nan("");
    bool use_valid_inequality1 = true;
    bool use_valid_inequality2 = true;
    bool use_valid_inequality3 = true;
    bool use_valid_inequality4 = true;
    bool use_valid_inequality5 = true;
    bool disable_gurobi_presolve = false;
    bool save_model = false;
    float timeout_s = 3600.0f;

    static const std::string header;
  };

  std::ostream& operator<<(std::ostream& out, const Model3Params& params);
}

#endif
