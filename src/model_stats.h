#ifndef _MODEL1_STATS_H
#define _MODEL1_STATS_H

#include <iostream>
#include <cmath>

namespace elevator {
  struct ModelStats {
    double n_rows = std::nan("");
    double n_cols = std::nan("");
    double n_nz = std::nan("");
    double valid_ineq1_added = std::nan("");
    double valid_ineq2_added = std::nan("");
    double valid_ineq3_added = std::nan("");
    double valid_ineq4_added = std::nan("");
    double valid_ineq5_addded = std::nan("");
    double root_primal = std::nan("");
    double root_dual = std::nan("");
    double root_gap = std::nan("");
    double root_time_s = std::nan("");
    double presolve_removed_rows = std::nan("");
    double presolve_removed_cols = std::nan("");
    double primal = std::nan("");
    double dual = std::nan("");
    double gap = std::nan("");
    double time_s = std::nan("");
    double visited_nodes = std::nan("");

    static const std::string header;
  };

  std::ostream& operator<<(std::ostream& out, const ModelStats& stats);
}

#endif
