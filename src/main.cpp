#include "instance.h"
#include "model1_params.h"
#include "model1_solver.h"
#include "model2_params.h"
#include "model2_solver.h"
#include "model3_params.h"
#include "model3_solver.h"
#include "cp_solver_params.h"
#include "cp_solver.h"
#include "greedy_heuristic.h"
#include "cp_heuristic.h"
#include <cxxopts.hpp>
#include <termcolor.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <filesystem>

int main(int argc, char** argv) {
  using namespace cxxopts;
  using namespace elevator;
  namespace fs = std::filesystem;
  namespace tc = termcolor;

  Options opts("elevator", "Solves the Vertical Farming Elevator Energy Minimisation Problem");
  opts.add_options()
      ("i,instance", "Path of the instance file", value<std::string>())
      ("w,warmstart", "Path to the initial solution (MIP models only)", value<std::string>())
      ("g,greedyheur", "Try to generate an initial solution with a simple greedy heuristic (MIP models only)", value<bool>()->default_value("false"))
      ("p,cpheur", "Try to generate an initial solution with a Constraint Programming heuristic (MIP models only)", value<bool>()->default_value("false"))
      ("m,model", "MIP/CP model to use", value<std::string>())
      ("s,savemodel", "Save the model to file? (MIP models only)", value<bool>()->default_value("false"))
      ("c,contrelax", "Saves the variable values of the continuous reaxation (MIP models only)", value<bool>()->default_value("false"))
      ("d,disablepresolve", "Disables Gurobi's cuts and presolve (MIP models only)", value<bool>()->default_value("false"))
      ("v,valid", "Comma-separated list of valid inequalities to use (MIP models only)", value<std::vector<std::string>>())
      ("n,bbnodes", "Maximum number of BB nodes to explore (MIP models only)", value<double>())
      ("t,timeout", "Solver timeout in seconds", value<float>()->default_value("3600"))
      ("o,outfolder", "Folder in which to save output files", value<std::string>()->default_value("../solutions/raw"))
      ("k,checkincumbent", "Check all incumbent primal solutions, to ensure they are valid (MIP models only)", value<bool>()->default_value("false"))
      ("h, help", "Print usage");

  const auto res = opts.parse(argc, argv);

  if(res.count("help")) {
    std::cout << opts.help() << "\n";
    return EXIT_SUCCESS;
  }

  const auto instance_path = fs::path(res["instance"].as<std::string>());
  if(!fs::exists(instance_path)) {
    std::cerr << "File not found: " << instance_path << "\n";
    return EXIT_FAILURE;
  }

  const auto model_name = res["model"].as<std::string>();
  const bool needs_big_omega = (model_name == "M2") || (model_name == "M3");
  const auto instance = Instance(instance_path, needs_big_omega);
  std::cout << tc::color<2> << "Read instance " << tc::bold << instance.name << tc::reset << "\n";
  std::cout << tc::blue << "\tShelves: " << instance.n_shelves << " (including the depot)\n";
  std::cout << "\tTrays: " << instance.n_trays - 1u << "\n";
  std::cout << "\tTime horizon: " << instance.n_time_instants << tc::reset << "\n";

  const auto timeout_s = res["timeout"].as<float>();
  const auto output_folder = res["outfolder"].as<std::string>();
  const auto save_model = res["savemodel"].count() > 0u;
  const auto disable_gurobi_presolve = res["disablepresolve"].count() > 0u;
  const auto cont_relax = res["contrelax"].count() > 0;
  const auto check_incumbent = res["checkincumbent"].count() > 0;
  double max_bb_nodes = std::nan("");

  if(res["bbnodes"].count() > 0u) {
    max_bb_nodes = res["bbnodes"].as<double>();
  }

  std::filesystem::create_directories(output_folder);

  if(model_name == "M1") {
    Model1Params params;
    params.timeout_s = timeout_s;
    params.max_bb_nodes = max_bb_nodes;
    params.output_folder = output_folder;
    params.disable_gurobi_presolve = disable_gurobi_presolve;
    params.save_model = save_model;
    params.save_contrelax = cont_relax;
    params.use_valid_inequality1 = false;
    params.use_valid_inequality2 = false;
    params.use_valid_inequality3 = false;
    params.use_valid_inequality4 = false;
    params.use_valid_inequality5 = false;

    if(check_incumbent) {
      std::cerr << tc::yellow << "Warning: incumbent check not implemented for model M1: ignoring." << tc::reset << "\n";
    }

    if(res.count("warmstart")) {
      params.initial_sol_path = res["warmstart"].as<std::string>();
    }

    if(res.count("valid")) {
      const auto valid_inequalities = res["valid"].as<std::vector<std::string>>();

      for(const auto &vi : valid_inequalities) {
        if(vi == "1") {
          params.use_valid_inequality1 = true;
        } else if(vi == "2") {
          params.use_valid_inequality2 = true;
        } else if(vi == "3") {
          params.use_valid_inequality3 = true;
        } else if(vi == "4") {
          params.use_valid_inequality4 = true;
        } else if(vi == "5") {
          params.use_valid_inequality5 = true;
        } else {
          std::cerr << "Valid inequality not recognised for model M1: " << vi << "\n";
          std::cerr << "It must be one of: 1, 2, 3, 4, 5\n";
          return EXIT_FAILURE;
        }
      }
    }

    std::cout << tc::color<2> << "Solving using model " << tc::bold << "M1" << tc::reset << "\n";
    std::cout << tc::blue << "\tTimeout: " << params.timeout_s << " s\n";
    std::cout << "\tMax BB nodes: " << params.max_bb_nodes << "\n";
    std::cout << "\tValid ineq 1: " << std::boolalpha << params.use_valid_inequality1 << "\n";
    std::cout << "\tValid ineq 2: " << std::boolalpha << params.use_valid_inequality2 << "\n";
    std::cout << "\tValid ineq 3: " << std::boolalpha << params.use_valid_inequality3 << "\n";
    std::cout << "\tValid ineq 4: " << std::boolalpha << params.use_valid_inequality4 << "\n";
    std::cout << "\tValid ineq 5: " << std::boolalpha << params.use_valid_inequality5 << tc::reset << "\n";

    Model1Solver solver{instance, params};

    if(res.count("greedyheur") > 0u) {
      solver.load_initial_solution(generate_greedy_solution(instance));
    }

    if(res.count("cpheur") > 0u) {
      const auto solution = generate_cp_solution(instance);

      if(solution) {
        solver.load_initial_solution(*solution);
      }
    }

    solver.solve();
  } else if(model_name == "M2") {
    Model2Params params;
    params.timeout_s = timeout_s;
    params.max_bb_nodes = max_bb_nodes;
    params.output_folder = output_folder;
    params.disable_gurobi_presolve = disable_gurobi_presolve;
    params.save_model = save_model;
    params.check_all_incumbents = check_incumbent;
    params.use_valid_inequality4 = false;
    params.use_valid_inequality5 = false;

    if(res.count("warmstart")) {
      params.initial_sol_path = res["warmstart"].as<std::string>();
    }

    if(res.count("valid")) {
      const auto valid_inequalities = res["valid"].as<std::vector<std::string>>();

      for(const auto &vi : valid_inequalities) {
        if(vi == "4") {
          params.use_valid_inequality4 = true;
        } else if(vi == "5") {
          params.use_valid_inequality5 = true;
        } else {
          std::cerr << "Valid inequality not recognised for model M2: " << vi << "\n";
          std::cerr << "It must be one of: 4, 5\n";
          return EXIT_FAILURE;
        }
      }
    }

    std::cout << tc::color<2> << "Solving using model " << tc::bold << "M2" << tc::reset << "\n";
    std::cout << tc::blue << "\tTimeout: " << params.timeout_s << " s\n";
    std::cout << "\tMax BB nodes: " << params.max_bb_nodes << "\n";
    std::cout << "\tValid ineq 4: " << std::boolalpha << params.use_valid_inequality4 << "\n";
    std::cout << "\tValid ineq 5: " << std::boolalpha << params.use_valid_inequality5 << tc::reset << "\n";

    Model2Solver solver{instance, params};

    if(res.count("greedyheur") > 0u) {
      solver.load_initial_solution(generate_greedy_solution(instance));
    }

    if(res.count("cpheur") > 0u) {
      const auto solution = generate_cp_solution(instance);

      if(solution) {
        solver.load_initial_solution(*solution);
      }
    }

    solver.solve();
  } else if(model_name == "M3") {
    Model3Params params;
    params.timeout_s = timeout_s;
    params.max_bb_nodes = max_bb_nodes;
    params.output_folder = output_folder;
    params.disable_gurobi_presolve = disable_gurobi_presolve;
    params.save_model = save_model;
    params.use_valid_inequality1 = false;
    params.use_valid_inequality2 = false;
    params.use_valid_inequality3 = false;
    params.use_valid_inequality4 = false;
    params.use_valid_inequality5 = false;

    if(check_incumbent) {
      std::cerr << tc::yellow << "Warning: incumbent check not implemented for model M1: ignoring." << tc::reset << "\n";
    }

    if(res.count("warmstart")) {
      params.initial_sol_path = res["warmstart"].as<std::string>();
    }

    if(res.count("valid")) {
      const auto valid_inequalities = res["valid"].as<std::vector<std::string>>();

      for(const auto &vi : valid_inequalities) {
        if(vi == "1") {
          params.use_valid_inequality1 = true;
        } else if(vi == "2") {
          params.use_valid_inequality2 = true;
        } else if(vi == "3") {
          params.use_valid_inequality3 = true;
        } else if(vi == "4") {
          params.use_valid_inequality4 = true;
        } else if(vi == "5") {
          params.use_valid_inequality5 = true;
        } else {
          std::cerr << "Valid inequality not recognised for model M3: " << vi << "\n";
          std::cerr << "It must be one of: 1, 2, 3, 4, 5\n";
          return EXIT_FAILURE;
        }
      }
    }

    std::cout << tc::color<2> << "Solving using model " << tc::bold << "M3" << tc::reset << "\n";
    std::cout << tc::blue << "\tTimeout: " << params.timeout_s << " s\n";
    std::cout << "\tMax BB nodes: " << params.max_bb_nodes << "\n";
    std::cout << "\tValid ineq 1: " << std::boolalpha << params.use_valid_inequality1 << "\n";
    std::cout << "\tValid ineq 2: " << std::boolalpha << params.use_valid_inequality2 << "\n";
    std::cout << "\tValid ineq 3: " << std::boolalpha << params.use_valid_inequality3 << "\n";
    std::cout << "\tValid ineq 4: " << std::boolalpha << params.use_valid_inequality4 << "\n";
    std::cout << "\tValid ineq 5: " << std::boolalpha << params.use_valid_inequality5 << tc::reset << "\n";

    Model3Solver solver{instance, params};

    if(res.count("greedyheur") > 0u) {
      solver.load_initial_solution(generate_greedy_solution(instance));
    }

    if(res.count("cpheur") > 0u) {
      const auto solution = generate_cp_solution(instance);

      if(solution) {
        solver.load_initial_solution(*solution);
      }
    }

    solver.solve();
  } else if(model_name == "CP") {
    CpSolverParams params;
    params.timeout_s = timeout_s;
    params.output_folder = output_folder;
    params.output_to_file = true;

    std::cout << tc::color<2> << "Solving using model " << tc::bold << "CP" << tc::reset << "\n";
    std::cout << tc::blue << "\tTimeout: " << params.timeout_s << " s" << tc::reset << "\n";

    CpSolver solver{instance, params};

    solver.solve();
  } else {
    std::cerr << "Model name not recognised: " << model_name << "\n";
    std::cerr << "It must be one of: M1, M2, M3, CP\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}