#include "model2_params.h"
#include <iostream>

namespace elevator {
  const std::string Model2Params::header = "valid_ineq4,valid_ineq5,"
    "disable_gurobi_presolve,max_bb_nodes,timeout_s";

  std::ostream& operator<<(std::ostream& out, const Model2Params& params) {
    out << std::boolalpha << params.use_valid_inequality4 << ","
        << std::boolalpha << params.use_valid_inequality5 << ","
        << std::boolalpha << params.disable_gurobi_presolve << ","
        << params.max_bb_nodes << "," << params.timeout_s;
    return out;
  }
}