# Caravel User Project

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![UPRJ_CI](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml) [![Caravel Build](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml)

This project integrates a GCD unit generated using [this commercial flow](https://code.stanford.edu/ee272/skywater-digital-flow/-/tree/372/GcdUnitCaravel/design) into caravel's user project wrapper. Compared to the example project, the following files have changes (please see the commit history to look at the changes in detail):

1. `Makefile`: Fixes the `PRECHECK_ROOT` variable.
2. `copy_user_files.sh`: Copies the GCD unit macro files: GDS, LEF, DEF, gate level netlist and RTL netlist. TODO: Copy spice netlist as well.
3. `lef/user_proj_example.lef`: GCD unit's LEF file copied using the above script.
4. `verilog/gl/user_proj_example.v`: GCD unit's gate level netlist copied using the above script.
5. `verilog/rtl/user_proj_example.v`: GCD unit's RTL netlist copied using the above script.
6. `verilog/rtl/user_project_wrapper.v`: We make a few changes to the wrapper RTL to change the name of IRQ signal on the user design from `irq` to `user_irq` to make it consistent with the wrapper. We also connect `analog_io` and `user_clock2` ports that were left unconnected between the wrapper and the user design.
7. `openlane/user_project_wrapper/macro.cfg`: Sets the floorplan coordinates for the GCD unit.

```

Refer to [README](docs/source/quickstart.rst) for a quick start of how to use caravel_user_project.

Refer to [README](docs/source/index.rst) for this sample project documentation. 
