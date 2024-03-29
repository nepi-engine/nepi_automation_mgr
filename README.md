<!--
Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.

This file is part of nepi-engine
(see https://github.com/nepi-engine).

License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
-->

# nepi_automation_mgr #
This repository provides the _nepi_automation_mgr_ ROS node and support files. The _nepi_automation_mgr_ allows users to upload executables in the form of scripts (Python, Bash, etc.) or target-compiled binaries that can be started and stopped automatically on boot-up or at will via the NEPI ROS API. These executables are generally referred to as "automation scripts," though as noted they need not necessarily be in script form.

Scripts are uploaded to the NEPI user partition automation_scripts subdirectory, typically at `/mnt/nepi_storage/automation_scripts`. A collection of sample scripts is provided in the separate _nepi_sample_auto_scripts_ repository.

Console output of running scripts is logged to the NEPI user partition, typically at `/mnt/nepi_storage/logs/automation_script_logs`
under files named according to the script filename. The log file resets each time that particular script runs.

Static and runtime statistics for each automation script are collected by _nepi_automation_mgr_ and can be queried through the NEPI ROS API, viewed in the NEPI RUI, etc.

### Build and Install ###
This repository is typically built as part of the [nepi_engine_ws](https://github.com/nepi-engine/nepi_engine_ws) catkin workspace. Refer to that project's top-level README for build and install instructions.

The repository may be included in other custom catkin workspaces or built using standard CMake commands (with appropriate variable definitions provided), though it should be noted that the executables here work in concert with a complete NEPI Engine installation, so operability and functionality may be limited when building outside of the [nepi_engine_ws](https://github.com/nepi-engine/nepi_engine_ws) source tree.

### Branching and Tagging Strategy ###
In general branching, merging, tagging are all limited to the [nepi_engine_ws](https://github.com/nepi-engine/nepi_engine_ws) container repository, with the present submodule repository kept as simple and linear as possible.

### Contribution guidelines ###
Bug reports, feature requests, and source code contributions (in the form of pull requests) are gladly accepted!

### Who do I talk to? ###
At present, all user contributions and requests are vetted by Numurus technical staff, so you can use any convenient mechanism to contact us.