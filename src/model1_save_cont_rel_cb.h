//
// Created by Alberto Santini on 30/05/2021.
//

#ifndef ELEVATOR_MODEL1_SAVE_CONT_REL_CB_H
#define ELEVATOR_MODEL1_SAVE_CONT_REL_CB_H

#include "model1_solver.h"
#include "instance.h"
#include <gurobi_c++.h>
#include <fstream>
#include <tuple>

namespace elevator {
  struct Model1SaveContRelCB : public GRBCallback {
    const Instance& instance;

    const Model1Solver::y_map& yi;
    const Model1Solver::s_map& si;
    const Model1Solver::d_map& di;

    const GRBVar *const y;
    const GRBVar *const s;
    const GRBVar *const d;

    Model1SaveContRelCB(const Instance& i,
                        const Model1Solver::y_map& yi,
                        const Model1Solver::s_map& si,
                        const Model1Solver::d_map& di,
                        const GRBVar *const y,
                        const GRBVar *const s,
                        const GRBVar *const d) :
                        instance{i}, yi{yi}, si{si}, di{di}, y{y}, s{s}, d{d} {}

    void callback() override {
      if(where == GRB_CB_MIPNODE) {
        const auto node = (size_t) getDoubleInfo(GRB_CB_MIPNODE_NODCNT);
        if(node == 0u) {
          if(getIntInfo(GRB_CB_MIPNODE_STATUS) != GRB_OPTIMAL) {
            std::cerr << "Cannot retrieve root node continuous relaxation (status not optimal)!\n";
            std::exit(1);
          }

          double* yv = getNodeRel(y, yi.size());
          double* sv = getNodeRel(s, si.size());
          double* dv = getNodeRel(d, di.size());

          std::ofstream ofs{"contrel-M1-" + instance.name + ".txt", std::ofstream::out};

          for(auto i = 0u; i < yi.size(); ++i) {
            if(yv[i] > 0.0) {
              const auto [t1, j1, t2, j2] = yi.right.at(i);
              ofs << "y[" << t1 << "][" << j1 << "][" << t2 << "][" << j2 << "] = " << yv[i] << "\n";

              const auto sidx1 = si.left.at(std::make_tuple(t1, j1));
              const auto sidx2 = si.left.at(std::make_tuple(t2, j2));

              ofs << "\ts[" << t1 << "][" << j1 << "] = " << sv[sidx1] << " (alpha: " << instance.alpha(t1, j1) << ", beta: " << instance.beta(t1, j1) << ", duration: " << instance.tray(t1).task(j1).duration << ")\n";
              if(sv[sidx1] < instance.L_earliest.at(std::make_tuple(t1, j1)) || sv[sidx1] > instance.L_latest.at(std::make_tuple(t1, j1))) {
                ofs << "\t\tL would have caught it! (earliest: " << instance.L_earliest.at(std::make_tuple(t1, j1)) << ", latest: " << instance.L_latest.at(std::make_tuple(t1, j1)) << ")\n";
              }
              ofs << "\ts[" << t2 << "][" << j2 << "] = " << sv[sidx2] << " (alpha: " << instance.alpha(t2, j2) << ", beta: " << instance.beta(t2, j2) << ", duration: " << instance.tray(t2).task(j2).duration << ")\n";
              if(sv[sidx2] < instance.L_earliest.at(std::make_tuple(t2, j2)) || sv[sidx2] > instance.L_latest.at(std::make_tuple(t2, j2))) {
                ofs << "\t\tL would have caught it! (earliest: " << instance.L_earliest.at(std::make_tuple(t2, j2)) << ", latest: " << instance.L_latest.at(std::make_tuple(t2, j2)) << ")\n";
              }
            }
          }

          for(auto i = 0u; i < di.size(); ++i) {
            if(dv[i] > 0) {
              const auto [t1, t2] = di.right.at(i);
              ofs << "delta[" << t1 << "][" << t2 << "] = " << dv[i] << "\n";
            }
          }

          delete yv;
          delete sv;
          delete dv;
        }
      }
    }
  };
}

#endif //ELEVATOR_MODEL1_SAVE_CONT_REL_CB_H
