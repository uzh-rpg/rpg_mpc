# Model Predictive Control for Quadrotors with extension to Perception-Aware MPC
Model Predictive Control for Quadrotors by "Robotics and Perception Group" at the Dep. of Informatics, "University of Zurich", and Dep. of Neuroinformatics, ETH and University of Zurich.

This MPC is intended to be used with https://github.com/uzh-rpg/rpg_quadrotor_control.
It is available with the extension to be used as a "Perception Aware Model Predictive Controller" (**PAMPC**).

[**Check out our YouTube-Video, showing PAMPC in Action**](https://www.youtube.com/watch?v=9vaj829vE18)
[![PAMPC: Perception-Aware Model Predictive Control for Quadrotors](http://rpg.ifi.uzh.ch/img/quad_control/mpc_thumb_button_small.png)](https://www.youtube.com/watch?v=9vaj829vE18)

## Publication
If you use this code in an academic context, please cite the following [IROS 2018 paper](http://rpg.ifi.uzh.ch/docs/IROS18_Falanga.pdf).

Davide Falanga, Philipp Foehn, Peng Lu, Davide Scaramuzza: **PAMPC: Perception-Aware Model Predictive Control for Quadrotors**, IEEE/RSJ Int. Conf. Intell. Robot. Syst. (IROS), 2018.

```
@InProceedings{Falanga2018
  author = {Falanga, Davide and Foehn, Philipp and Lu, Peng and Scaramuzza, Davide},
  title = {{PAMPC}: Perception-Aware Model Predictive Control for Quadrotors},
  booktitle = {IEEE/RSJ Int. Conf. Intell. Robot. Syst. (IROS)},
  year = {2018}
}
```


## License

Copyright (C) 2017-2018 Philipp Foehn, Robotics and Perception Group, University of Zurich

The RPG MPC repository provides packages that are intended to be used with [RPG Quadrotor Control](https://github.com/uzh-rpg/rpg_quadrotor_control) and [ROS](http://www.ros.org/). 
This code has been tested with ROS kinetic on Ubuntu 16.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.
For a commercial license, please contact [Davide Scaramuzza](http://rpg.ifi.uzh.ch/people_scaramuzza.html).

```
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
```

This work depends on the ACADO Toolkit, developed by the Optimization in Engineering Center (OPTEC) under supervision of Moritz Diehl. Licensing detail can be found on the [ACADO licensing page](http://acado.github.io/licensing.html). It is released under GNU Lesser General Public License as published by the Free Software Foundation, version 3.
ACADO uses qpOASES by Hans Joachim Ferreau et al., released under GPL v2.1.

## Installation, Usage, and Documentation
For detailed instructions on installation, usage, and documentation, please head to our [Wiki](../../wiki).

Quick Links:
- [Installation](../../wiki/Installation)
- [Basic Usage](../../wiki/Basic-Usage)
- [Code Structure](../../wiki/Code-Structure)
- [License](../../wiki/License)

## Code Structure Overview
The whole MPC is based on ACADO (http://acado.github.io/).
ACADO's C++ interface is used to describe the quadrotor model and parameters for transcription into a quadratic program, which is then solved with qpOASES (https://projects.coin-or.org/qpOASES). To compile and run, none of these dependencies are needed, since the generated code is already included in this repository. To modify the model and solver options, please install ACADO from http://acado.github.io/install_linux.html.

The code is organized as follows:

### Solver `mpc_solver`

The auto-generated model, transcription, and solver is built as a library called `mpc_solver`.
This library consist of purely auto-generated code with nomenclature, code-style and structure as used in ACADO.

### Wrapper `mpc_wrapper`

To wrap this into a standard interface, the library `mpc_wrapper` is used.
ACADO uses arrays with column-major format to store matrices, since this is rather inconvenient, `mpc_wrapper` provides  interfaces using Eigen objects by mapping the arrays.
* It is written to be compatible even with changing model descriptions in the `mpc_solver`.
* It should prevent the most common runtime errors caused by the user by doing some initialization and checks with hardcoded parameters, which are overwritten in normal usage.

### Controller `mpc_controller`

To provide not only a solver, but a full controller, `mpc_controller` is a library based on the previous `mpc_solver` and `mpc_wrapper`, providing all funcitonality to implement a controller with minimal effort. It provides two main execution modes:

* **Embedded**: The mpc_controller can be included in any oder controller class or copilot by generating an object with the default constructor "MPC::MpcController<T> controller();". It still registers node-handles to publish the predicted trajectory after each control cycle, but does nothing more. It only provides the interfaces of `ControllerBase` as in the `LargeAngleController` but without a specific class for parameters.

* **Standalone (not yet provided)**: The `mpc_controller` object can be passed node-handles or creates its own, registers multiple subscribers and publishers to get a state estimate as well as the control goals (pose or trajectory) and registers a timer to run a control loop. It works as a full standalone controller which can be used with the oneliner: `MPC::MpcController<double> controller(ros::NodeHandle(), ros::NodeHandle("~"));` as in `test/control_node.cpp` and `launch/mpc_controller.launch`.
