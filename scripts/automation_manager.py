#!/usr/bin/env python

import os
import subprocess
import threading
import traceback
import yaml
import resource
import time

import rospy
from std_msgs.msg import String
from num_sdk_msgs.srv import (
    GetScriptsQuery,
    GetScriptsQueryResponse,
    GetRunningScriptsQuery,
    GetRunningScriptsQueryResponse,
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

class AutomationManager:
    def __init__(self):

        self.scripts = self.get_scripts()
        self.config = self.load_config()
        self.file_sizes = self.get_file_sizes()

        self.processes = {}

        self.script_counters = {}
        for script in self.scripts:
            self.script_counters[script] = {'completed': 0, 'stopped_manually': 0}

        self.running_scripts = set()

        self.get_scripts_service = rospy.Service("get_scripts", GetScriptsQuery, self.handle_get_scripts)
        self.get_running_scripts_service = rospy.Service("get_running_scripts", GetRunningScriptsQuery, self.handle_get_running_scripts)
        self.set_script_enabled_service = rospy.Service("set_script_enabled", SetScriptEnabled, self.handle_set_script_enabled)
        self.launch_script_service = rospy.Service("launch_script", LaunchScript, self.handle_launch_script)
        self.stop_script_service = rospy.Service("stop_script", StopScript, self.handle_stop_script)
        self.get_script_status_service = rospy.Service("get_script_status", GetScriptStatusQuery, self.handle_get_script_status)
        self.get_system_stats_service = rospy.Service("get_system_stats", GetSystemStatsQuery, self.handle_get_system_stats)

        self.monitor_thread = threading.Thread(target=self.monitor_scripts)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

        self.watch_thread = threading.Thread(target=self.watch_directory, args=(AUTOMATION_DIR, self.on_new_file))
        self.watch_thread.daemon = True
        self.watch_thread.start()

        
    def get_scripts(self):
        """
        Detect and report executable automation scripts that exist in a particular directory in the filesystem.
        """
        #self.scripts = self.get_scripts()
        scripts = []
        
        for filename in os.listdir(AUTOMATION_DIR):
            filepath = os.path.join(AUTOMATION_DIR, filename)
            if os.access(filepath, os.X_OK) and not os.path.isdir(filepath):
                scripts.append(filename)
        return scripts

    def watch_directory(self, directory, callback):
        files_mtime = {}

        while True:
            for file in os.listdir(directory):
                file_path = os.path.join(directory, file)
                if os.path.isfile(file_path):
                    current_mtime = os.path.getmtime(file_path)
                    if file not in files_mtime or files_mtime[file] != current_mtime:
                        files_mtime[file] = current_mtime
                        callback(file_path)
            time.sleep(1)

    def on_new_file(self, file_path):
        rospy.loginfo("New file detected: %s", file_path)
        self.scripts = self.get_scripts()

    def get_file_sizes(self):
        """
        Get the file sizes of the automation scripts in the specified directory.
        """
        file_sizes = {}
        for filename in self.scripts:
            filepath = os.path.join(AUTOMATION_DIR, filename)
            file_size = os.path.getsize(filepath)
            file_sizes[filename] = file_size
        return file_sizes

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
        rospy.loginfo("Scripts: %s" % self.scripts)
        rospy.loginfo("File sizes: %s" % self.file_sizes)

        return GetScriptsQueryResponse(self.scripts)
    
    def handle_get_running_scripts(self, req):
        """
        Handle a request to get a list of currently running scripts.
        """
        running_scripts = list(self.running_scripts)
        rospy.loginfo("Running scripts: %s" % running_scripts)

        return GetRunningScriptsQueryResponse(running_scripts)

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
                    self.running_scripts.add(req.script)  # Update the running_scripts set
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
                self.running_scripts.remove(req.script)  # Update the running_scripts set
                rospy.loginfo("%s: stopped" % req.script)
                self.script_counters[req.script]['stopped_manually'] += 1  # update the counter
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
                        self.running_scripts.remove(script)  # Update the running_scripts set
                        if process['process'].returncode == 0:
                            self.script_counters[script]['completed'] += 1
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
                    # rospy.loginfo("%s: ready to run" % script)
                    pass

            # Output script counters
            rospy.loginfo("Script counters:")
            for script, counter in self.script_counters.items():
                rospy.loginfo("%s - completed: %d, stopped manually: %d" % (script, counter['completed'], counter['stopped_manually']))

            rospy.sleep(1)

    def handle_get_system_stats(self, req):
        script_name = req.script
        if not script_name or script_name not in self.processes:
            rospy.logwarn("Script not found or not running: %s" % script_name)
            return GetSystemStatsQueryResponse(None, None, None, None)
            
        # Ensure the script_name has a 'pid' key in the dictionary
        if 'pid' not in self.processes[script_name]:
            rospy.logwarn("PID not found for script: %s" % script_name)
            return GetSystemStatsQueryResponse(None, None, None, None)

        pid = self.processes[script_name]['pid']
        rospy.loginfo("PID for script %s: %d" % (script_name, pid))

        try:
            # Get resource usage for the specific PID
            usage = resource.getrusage(resource.RUSAGE_CHILDREN)

            # Get CPU usage
            self.cpu_percent = (usage.ru_utime + usage.ru_stime) / os.sysconf("SC_CLK_TCK")

            # Get memory usage
            self.memory_usage = usage.ru_maxrss

            # Get swap info
            self.swap_info = usage.ru_nswap

            # Get disk usage
            self.disk_usage = usage.ru_oublock

            rospy.loginfo("CPU Percent: %.2f%%, Memory Usage: %d bytes, Swap Info: %d, Disk Usage: %d" % (self.cpu_percent, self.memory_usage, self.swap_info, self.disk_usage))

            # Return the system stats as a GetSystemStatsQuery response object
            return GetSystemStatsQueryResponse(self.cpu_percent, self.memory_usage, self.swap_info, self.disk_usage)

        except OSError as e:
            rospy.logwarn("Error processing request: %s" % str(e))
            return GetSystemStatsQueryResponse(None, None, None, None)

def main():
    rospy.init_node("automation_manager")
    manager = AutomationManager()
    rospy.spin()

if __name__ == '__main__':
    main()