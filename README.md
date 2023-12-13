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
The AutomationManager class, which handles the detection, launching, stopping, and monitoring of automation scripts. 
It uses ROS services and publishers to provide an API for interacting with the automation scripts, as well as logging the status of each script.

The get_scripts method, which detects and reports executable automation scripts that exist in a particular directory in the filesystem.

The load_config method, which loads the persistent configuration (ROS YAML config file) for enabling/disabling automation scripts to run at system start-up.

The save_config method, which saves the current configuration to the YAML file.

The handle_get_scripts method, which handles a request to get the list of available automation scripts.

The handle_set_script_enabled method, which handles a request to enable or disable an automation script in the configuration file.

The handle_launch_script method, which handles a request to launch an automation script.

The handle_stop_script method, which handles a request to stop an automation script.

The monitor_scripts method, which monitors the status of all automation scripts.

The main function, which initializes the ROS node and creates an instance of the AutomationManager class.
