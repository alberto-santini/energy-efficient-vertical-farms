# Energy efficient automatic vertical farms

This repository contains the companion code and instances for the paper "[Energy efficient automated vertical farms](https://santini.in/files/papers/delorme-santini-2022.pdf)" by [Maxence Delorme](https://www.tilburguniversity.edu/staff/m-delorme) and [Alberto Santini](https://santini.in/).

## Citation

If you use this software, please cite the following paper:

```bib
@article{delorme_santini_vf_2021,
  title={Energy-efficient automated vertical farms},
  author={Delorme, Maxence and Santini, Alberto},
  journal={Omega},
  volume={109},
  doi={10.1016/j.omega.2022.102611},
  year={2022}
}
```

You can also cite this repository via Zenodo.

[![DOI](https://zenodo.org/badge/395009250.svg)](https://zenodo.org/badge/latestdoi/395009250)

```bib
@misc{vf_github,
    title={Energy efficient automatic vertical farms},
    author={Santini, Alberto},
    date=2021-08-12,
    year=2021,
    doi={10.5281/zenodo.5185902},
    url={https://github.com/alberto-santini/energy-efficient-vertical-farms/},
    howpublished={Github repository},
}
```

<p align="center">
    <img src="https://github.com/alberto-santini/energy-efficient-automatic-vertical-farms/blob/master/vf.jpg?raw=true" alt="Vertical Farm"/><br>
    Photo: ifarm.fi, CC BY-SA 4.0, via Wikimedia Commons
</p>

## Instructions

You can use CMake to build the project.
Relevant variables are `GUROBI_DIR` (to specify the home directory of solver Gurobi) and `CPLEX_DIR` (to specify the home directory of solver Cplex).

```
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DGUROBI_DIR=/opt/gurobi -DCPLEX_DIR=/opt/cplex ..
make -j4
```

Running `./elevator --help` describes the possible options:

```
$ ./elevator --help

Solves the Vertical Farming Elevator Energy Minimisation Problem
Usage:
  elevator [OPTION...]

  -i, --instance arg     Path of the instance file
  -w, --warmstart arg    Path to the initial solution (MIP models only)
  -g, --greedyheur       Try to generate an initial solution with a simple 
                         greedy heuristic (MIP models only)
  -p, --cpheur           Try to generate an initial solution with a 
                         Constraint Programming heuristic (MIP models only)
  -m, --model arg        MIP/CP model to use
  -s, --savemodel        Save the model to file? (MIP models only)
  -c, --contrelax        Saves the variable values of the continuous 
                         reaxation (MIP models only)
  -d, --disablepresolve  Disables Gurobi's cuts and presolve (MIP models 
                         only)
  -v, --valid arg        Comma-separated list of valid inequalities to use 
                         (MIP models only)
  -n, --bbnodes arg      Maximum number of BB nodes to explore (MIP models 
                         only)
  -t, --timeout arg      Solver timeout in seconds (default: 3600)
  -o, --outfolder arg    Folder in which to save output files (default: 
                         ../solutions/raw)
  -k, --checkincumbent   Check all incumbent primal solutions, to ensure 
                         they are valid (MIP models only)
  -h, --help             Print usage

```

## License

The code is distributed under the GNU General Public License v 3.0 (see the `LICENSE` file).
The code also ships header files of libraries:

* `nlohmann/json`, distributed under the MIT license.
* `jarro2783/cxxopts`, distributed under the MIT license.
* `ikalnytskyi/termcolor`, distributed under the BSD license.
