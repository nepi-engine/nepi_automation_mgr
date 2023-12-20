<!--
NEPI Dual-Use License
Project: nepi_automation_mgr

This license applies to any user of NEPI Engine software

Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
see https://github.com/numurus-nepi/nepi_automation_mgr

This software is dual-licensed under the terms of either a NEPI software developer license
or a NEPI software commercial license.

The terms of both the NEPI software developer and commercial licenses
can be found at: www.numurus.com/licensing-nepi-engine

Redistributions in source code must retain this top-level comment block.
Plagiarizing this software to sidestep the license obligations is illegal.

Contact Information:
====================
- https://www.numurus.com/licensing-nepi-engine
- mailto:nepi@numurus.com

-->

# nepi_automation_mgr #
This repository provides the _nepi_automation_mgr_ ROS node and support files. The _nepi_automation_mgr_ allows users to upload executables in the form of scripts (Python, Bash, etc.) or target-compiled binaries that can be started and stopped automatically on boot-up or at will via the NEPI ROS API. These executables are generally referred to as "automation scripts," though as noted they need not necessarily be in script form.

Scripts are uploaded to the NEPI user partition automation_scripts subdirectory, typically at `/mnt/nepi_storage/automation_scripts`. A collection of sample scripts is provided in the separate _nepi_sample_auto_scripts_ repository.

Console output of running scripts is logged to the NEPI user partition, typically at `/mnt/nepi_storage/logs/automation_script_logs`
under files named according to the script filename. The log file resets each time that particular script runs.

Static and runtime statistics for each automation script are collected by _nepi_automation_mgr_ and can be queried through the NEPI ROS API, viewed in the NEPI RUI, etc.

### Build and Install ###
This repository is typically built as part of the _nepi_base_ws_ catkin workspace. Refer to that project's top-level README for build and install instructions.

The repository may be included in other custom catkin workspaces or built using standard CMake commands (with appropriate variable definitions provided), though it should be noted that the executables here work in concert with a complete NEPI Engine installation, so operability and functionality may be limited when building outside of the _nepi_base_ws_ source tree.

### Branching and Tagging Strategy ###
In general branching, merging, tagging are all limited to the _nepi_base_ws_ container repository, with the present submodule repository kept as simple and linear as possible.

### Contribution guidelines ###
Bug reports, feature requests, and source code contributions (in the form of pull requests) are gladly accepted!

### Who do I talk to? ###
At present, all user contributions and requests are vetted by Numurus technical staff, so you can use any convenient mechanism to contact us.