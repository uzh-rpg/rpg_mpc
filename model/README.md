# How to compile the MPC code generation and analysis

1. Install ACADO as described in http://acado.github.io/install_linux.html

2. Source the file ACADO_ROOT/build/acdo_env.sh

3. Go into rpg_mpc/model

4. Prepare the make with "cmake ."

5. Compile it with "make"

6. Run the executable to generate all code "./quadrotor_model_codegen"