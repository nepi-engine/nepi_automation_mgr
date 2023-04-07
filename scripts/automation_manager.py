#!/usr/bin/env python

import os
import subprocess
import threading
import traceback
import yaml
import resource

import pdb


import rospy
from std_msgs.msg import String
from num_sdk_msgs.srv import (
    GetScriptsQuery,
    GetScriptsQueryResponse,
    SetScriptEnabled,
    SetScriptEnabledResponse,
    LaunchScript,
    LaunchScriptResponse,
    StopScript,
    GetScriptStatusQuery,
    GetScriptStatusQueryResponse,
    GetSystemStatsQuery,
    GetSystemStatsQueryResponse,
)

AUTOMATION_DIR = "/home/numurus/nepi_scripts"
CONFIG_FILE = "/home/numurus/nepi_config/config.yaml"

#pdb.set_trace()

class AutomationManager:
    def __init__(self):

        self.scripts = self.get_scripts()
        self.config = self.load_config()

        self.processes = {}

        self.get_scripts_service = rospy.Service("get_scripts", GetScriptsQuery, self.handle_get_scripts)
        self.set_script_enabled_service = rospy.Service("set_script_enabled", SetScriptEnabled, self.handle_set_script_enabled)
        self.launch_script_service = rospy.Service("launch_script", LaunchScript, self.handle_launch_script)
        self.stop_script_service = rospy.Service("stop_script", StopScript, self.handle_stop_script)
        self.get_script_status_service = rospy.Service("get_script_status", GetScriptStatusQuery, self.handle_get_script_status)
        self.get_system_stats_service = rospy.Service("get_system_stats", GetSystemStatsQuery, self.handle_get_system_stats)


        self.monitor_thread = threading.Thread(target=self.monitor_scripts)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def get_scripts(self):
        """
        Detect and report executable automation scripts that exist in a particular directory in the filesystem.
        """
        scripts = []
        for filename in os.listdir(AUTOMATION_DIR):
            filepath = os.path.join(AUTOMATION_DIR, filename)
            if os.access(filepath, os.X_OK) and not os.path.isdir(filepath):
                scripts.append(filename)
        return scripts

    def load_config(self):
        """
        Load the persistent configuration (ROS yaml config file) for enabling/disabling automation scripts to run at
        system start-up.
        """
        if not os.path.isfile(CONFIG_FILE):
            return {}
        with open(CONFIG_FILE, "r") as f:
            config = yaml.safe_load(f)
            if config is None:
                config = {}
            return config

    def save_config(self):
        """
        Save the persistent configuration (ROS yaml config file).
        """
        with open(CONFIG_FILE, "w") as f:
            yaml.dump(self.config, f)

    def handle_get_scripts(self, req):
        """
        Handle a request to get the list of available automation scripts.
        """
        rospy.loginfo(self.scripts)

        return GetScriptsQueryResponse(self.scripts)

    def handle_set_script_enabled(self, req):
        """
        Handle a request to enable or disable an automation script.
        """
        if req.script in self.config:
            self.config[req.script] = req.enabled
            self.save_config()
            return SetScriptEnabledResponse(True)
        else:
            return SetScriptEnabledResponse(False)

    def handle_launch_script(self, req):
        """
        Handle a request to launch an automation script.
        """
        if req.script in self.scripts:
            if req.script not in self.config or self.config[req.script]:
                try:
                    process = subprocess.Popen([os.path.join(AUTOMATION_DIR, req.script)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    self.processes[req.script] = {'process': process, 'pid': process.pid}
                    rospy.loginfo("%s: running" % req.script)
                    return LaunchScriptResponse(True)

                except Exception as e:
                    rospy.logwarn("%s: error (%s)" % (req.script, str(e)))
                    return LaunchScriptResponse(False)
            else:
                rospy.loginfo("%s: disabled in configuration" % req.script)
                return LaunchScriptResponse(False)
        else:
            rospy.loginfo("%s: not found" % req.script)
            return LaunchScriptResponse(False)

    def handle_stop_script(self, req):
        """
        Handle a request to stop an automation script.
        """
        if req.script in self.processes:
            try:
                process = self.processes[req.script]['process']
                process.terminate()
                process.wait()
                del self.processes[req.script]
                rospy.loginfo("%s: stopped" % req.script)
                return True
            except Exception as e:
                rospy.logwarn("%s: error stopping (%s)" % (req.script, str(e)))
                return False
        else:
            rospy.logwarn("%s: not running" % req.script)
            return False
        
    def handle_get_script_status(self, req):
        """
        Handle a request to get the status of a script.
        """
        if req.script in self.scripts:
            if req.script in self.processes:
                return GetScriptStatusQueryResponse("running")
            else:
                return GetScriptStatusQueryResponse("ready to run")
        else:
            return GetScriptStatusQueryResponse("not found")

    def monitor_scripts(self):
        """
        Monitor the status of all automation scripts.
        """
        while not rospy.is_shutdown():
            for script in self.scripts:
                if script in self.processes:
                    process = self.processes[script]
                    if process['process'].poll() is not None:
                        del self.processes[script]
                        if process['process'].returncode == 0:
                            rospy.loginfo("%s: completed" % script)
                        else:
                            traceback_str = "".join(traceback.format_exception_only(type(process.returncode), process.returncode))
                            rospy.logwarn("%s: error (%s)" % (script, traceback_str.strip()))
                    else:
                        try:
                            stdout, stderr = process['process'].communicate()
                            if stdout:
                                rospy.loginfo("%s stdout: %s" % (script, stdout.strip().decode()))
                            if stderr:
                                rospy.logwarn("%s stderr: %s" % (script, stderr.strip().decode()))
                        except subprocess.CalledProcessError as e:
                            print("Error:", e.returncode, e.output)
                            pass
                else:
                    #rospy.loginfo("%s: ready to run" % script)
                    pass
            rospy.sleep(1)

    def handle_get_system_stats(self, req):
        """
        Handle a request to get system stats (CPU usage, memory usage, swap info, and disk usage).
        """
        # Get CPU usage
        self.cpu_percent = (resource.getrusage(resource.RUSAGE_SELF).ru_utime + resource.getrusage(resource.RUSAGE_SELF).ru_stime) / os.sysconf("SC_CLK_TCK")
        
        # Get memory usage
        self.memory_usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        
        # Get swap info
        self.swap_info = resource.getrusage(resource.RUSAGE_SELF).ru_nswap
        
        # Get disk usage
        self.disk_usage = resource.getrusage(resource.RUSAGE_SELF).ru_oublock
        
        rospy.loginfo("CPU Percent: %.2f%%, Memory Usage: %d bytes, Swap Info: %d, Disk Usage: %d" % (self.cpu_percent, self.memory_usage, self.swap_info, self.disk_usage))

        # Return the system stats as a GetSystemStatsQuery response object
        return GetSystemStatsQueryResponse(self.cpu_percent, self.memory_usage, self.swap_info, self.disk_usage)

def main():
    rospy.init_node("automation_manager")
    manager = AutomationManager()
    rospy.spin()

if __name__ == '__main__':
    main()