#ifndef _GUROBI_ROOT_NODE_CB_H
#define _GUROBI_ROOT_NODE_CB_H

#include "model_stats.h"
#include <gurobi_c++.h>

namespace elevator {
  struct RootNodeInfoCB : public GRBCallback {
    ModelStats& stats;

    RootNodeInfoCB(ModelStats& stats) : stats{stats} {}

    void callback() override {
      if(where == GRB_CB_MIPNODE) {
        const auto node = (size_t) getDoubleInfo(GRB_CB_MIPNODE_NODCNT);
        if(node == 0u) {
          stats.root_primal = getDoubleInfo(GRB_CB_MIPNODE_OBJBST);
          stats.root_dual = getDoubleInfo(GRB_CB_MIPNODE_OBJBND);
          stats.root_gap = std::abs(stats.root_primal - stats.root_dual) / stats.root_primal;
          stats.root_time_s = getDoubleInfo(GRB_CB_RUNTIME);
        }
      } else if(where == GRB_CB_PRESOLVE) {
        stats.presolve_removed_rows = (double) getIntInfo(GRB_CB_PRE_ROWDEL);
        stats.presolve_removed_cols = (double) getIntInfo(GRB_CB_PRE_COLDEL);
      }
    }
  };
}

#endif
