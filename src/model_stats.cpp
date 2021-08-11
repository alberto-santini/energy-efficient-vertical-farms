#include "model_stats.h"
#include <iostream>
#include <string>

namespace elevator {
  const std::string ModelStats::header =
      "n_rows,n_cols,n_nz,valid_ineq1_added,valid_ineq2_added,"
      "valid_ineq3_added,valid_ineq4_added,valid_ineq5_added,"
      "root_primal,root_dual,root_gap,root_time_s,"
      "presolve_removed_rows,presolve_removed_cols,"
      "primal,dual,gap,time_s,visited_nodes";

  std::ostream& operator<<(std::ostream& out, const ModelStats& stats) {
    out << stats.n_rows << ","
        << stats.n_cols << ","
        << stats.n_nz << ","
        << stats.valid_ineq1_added << ","
        << stats.valid_ineq2_added << ","
        << stats.valid_ineq3_added << ","
        << stats.valid_ineq4_added << ","
        << stats.valid_ineq5_addded << ","
        << stats.root_primal << ","
        << stats.root_dual << ","
        << stats.root_gap << ","
        << stats.root_time_s << ","
        << stats.presolve_removed_rows << ","
        << stats.presolve_removed_cols << ","
        << stats.primal << ","
        << stats.dual << ","
        << stats.gap << ","
        << stats.time_s << ","
        << stats.visited_nodes;
    return out;
  }
}