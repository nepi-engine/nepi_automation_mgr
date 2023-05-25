#!/usr/bin/env python

import os
import subprocess
import threading
import traceback
import yaml
#import resource
import time
import psutil

import rospy

from std_msgs.msg import String
from nepi_ros_interfaces.srv import (
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
    SystemStorageFolderQuery
)

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF
from nepi_ros_interfaces.msg import AutoStartEnabled

DEFAULT_AUTOSTART_SCRIPT = None
DEFAULT_AUTOSTART_ENABLED = False

CONFIG_FILE = "/opt/nepi/ros/etc/automation_mgr/automation_mgr.yaml"

class AutomationManager:
    AUTOMATION_DIR = "/mnt/nepi_storage/automation_scripts"

    def __init__(self):

        # Try to obtain the path to automation_scripts from the system_mgr
        try:
            rospy.wait_for_service('system_storage_folder_query', 10.0)
            system_storage_folder_query = rospy.ServiceProxy('system_storage_folder_query', SystemStorageFolderQuery)
            self.AUTOMATION_DIR = system_storage_folder_query('automation_scripts').folder_path
        except Exception as e:
            rospy.logwarn("Failed to obtain system automation_scripts folder... falling back to " + self.AUTOMATION_DIR)

        self.autostart_msgs = {}
        
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentSettingsAsDefault, paramsModifiedCallback=self.updateFromParamServer)

        self.scripts = self.get_scripts()
        self.config = self.load_config()
        self.file_sizes = self.get_file_sizes()

        self.processes = {}

        self.script_counters = {}
        for script in self.scripts:
            #TODO: These should be gathered from a stats file on disk to remain cumulative for all time (clearable on ROS command)
            self.script_counters[script] = {'started': 0, 'completed': 0, 'stopped_manually': 0, 'errored_out': 0, 'cumulative_run_time': 0.0}

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

        self.watch_thread = threading.Thread(target=self.watch_directory, args=(self.AUTOMATION_DIR, self.on_new_file))
        self.watch_thread.daemon = True
        self.watch_thread.start()

        # Subscribe to topics
        rospy.Subscriber('enable_script_autostart', AutoStartEnabled, self.AutoStartEnabled_cb)

    def get_scripts_parameters(self):
        rospy.loginfo("ready to run get_scripts_parameters!!!")
        try:
            scripts_params = rospy.get_param('~scripts')
        except KeyError:
            rospy.logwarn("Parameter ~scripts does not exist")
            scripts_params = {}

        rospy.loginfo("Scripts parameters: %s" % scripts_params)
        return scripts_params

    def AutoStartEnabled_cb(self, msg):
        rospy.loginfo("ready to run AUTOSTART!!!")
        rospy.loginfo("AUTOSTART : %s" % msg.script)
        rospy.loginfo("AUTOSTART : %s" % msg.enabled)

        # Insert or update msg into autostart_msgs dictionary
        self.autostart_msgs[msg.script] = msg.enabled

        # Log the current state of autostart_msgs dictionary
        rospy.loginfo("Current autostart_msgs dictionary: %s" % self.autostart_msgs)

        # Set the parameter on the ROS parameter server
        rospy.set_param('~scripts', self.autostart_msgs)
        self.updateFromParamServer()

    def updateFromParamServer(self):
        # Read the parameter from the ROS parameter server
        self.get_scripts_parameters()

    def setCurrentSettingsAsDefault(self):
        pass
        
    def get_scripts(self):
        """
        Detect and report automation scripts that exist in a particular directory in the filesystem.
        """
        #self.scripts = self.get_scripts()
        scripts = []
        for filename in os.listdir(self.AUTOMATION_DIR):
            filepath = os.path.join(self.AUTOMATION_DIR, filename)
            if not os.path.isdir(filepath):
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
        rospy.loginfo("New file detected: %s", os.path.basename(file_path))
        self.scripts = self.get_scripts()

    def get_file_sizes(self):
        """
        Get the file sizes of the automation scripts in the specified directory.
        """
        file_sizes = {}
        for filename in self.scripts:
            filepath = os.path.join(self.AUTOMATION_DIR, filename)
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
        rospy.logdebug("Scripts: %s" % self.scripts)
        rospy.logdebug("File sizes: %s" % self.file_sizes)

        return GetScriptsQueryResponse(self.scripts)
    
    def handle_get_running_scripts(self, req):
        """
        Handle a request to get a list of currently running scripts.
        """
        running_scripts = list(self.running_scripts)
        #rospy.loginfo("Running scripts: %s" % running_scripts)

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
                    process = subprocess.Popen([os.path.join(self.AUTOMATION_DIR, req.script)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    self.processes[req.script] = {'process': process, 'pid': process.pid, 'start_time': psutil.Process(process.pid).create_time()}
                    self.running_scripts.add(req.script)  # Update the running_scripts set
                    rospy.loginfo("%s: running" % req.script)
                    self.script_counters[req.script]['started'] += 1  # update the counter
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
                self.script_counters[req.script]['cumulative_run_time'] += (rospy.Time.now() - rospy.Time.from_sec(self.processes[req.script]['start_time'])).to_sec()
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
                            self.script_counters[script]['errored_out'] += 1
                            rospy.logwarn("%s: error (%s)" % (script, traceback_str.strip()))
                                               
                        # Update the cumulative run time whether exited on success or error
                        self.script_counters[script]['cumulative_run_time'] += (rospy.Time.now() - rospy.Time.from_sec(process['start_time'])).to_sec()
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
            rospy.logdebug("Script counters:")
            for script, counter in self.script_counters.items():
                rospy.logdebug("%s - completed: %d, stopped manually: %d" % (script, counter['completed'], counter['stopped_manually']))

            rospy.sleep(1)

    def handle_get_system_stats(self, req):
        script_name = req.script
        response = GetSystemStatsQueryResponse(cpu_percent=None, memory_percent=None, run_time_s=None,
                                               cumulative_run_time_s=None, file_size_bytes=None, started_runs=None,
                                               completed_runs=None, error_runs=None, stopped_manually=None)

        if not script_name:
            rospy.logwarn_throttle(10, "Requested script not found: %s" % script_name)
            return response # Blank response

        # Get file size for the script_name
        response.file_size_bytes = self.file_sizes[script_name]

        # Get the counter values for the script_name
        response.started_runs = self.script_counters[script_name]['started']
        response.completed_runs = self.script_counters[script_name]['completed']
        response.error_runs = self.script_counters[script_name]['errored_out']
        response.stopped_manually = self.script_counters[script_name]['stopped_manually']
        response.cumulative_run_time_s = self.script_counters[script_name]['cumulative_run_time']

        # Check if the script_name has a 'pid' key in the dictionary to determine whether or not to gather running-script stats
        if (script_name not in self.processes) or ('pid' not in self.processes[script_name]):
            return response  # Only includes the 'static' info

        pid = self.processes[script_name]['pid']
        #rospy.loginfo("PID for script %s: %d" % (script_name, pid))

        try:
            # Get resource usage for the specific PID
            #usage = resource.getrusage(resource.RUSAGE_CHILDREN)
            process = psutil.Process(pid)

            # Get CPU usage
            #self.cpu_percent = (usage.ru_utime + usage.ru_stime) / os.sysconf("SC_CLK_TCK")
            response.cpu_percent = process.cpu_percent(0.1)

            # Get memory usage
            #self.memory_usage = usage.ru_maxrss
            response.memory_percent = 100.0 * float(process.memory_full_info().uss) / float(psutil.virtual_memory().total)
                        
            # Get creation/start-up time
            response.run_time_s = (rospy.Time.now() - rospy.Time.from_sec(process.create_time())).to_sec()
            # The script_counters cumulative run time only gets updated on script termination, so to keep this value moving in the response,
            # increment it here.
            response.cumulative_run_time_s += response.run_time_s
            #rospy.loginfo("CPU Percent: %.5f%%, Memory Usage: %.5f%%, Run Time: %.2f" % (response.cpu_percent, response.memory_percent, response.run_time_s))

        except Exception as e:
            rospy.logwarn("Error gathering running stats: %s" % str(e))
            return response  # Add new None values for the counters
        
        # Return the system stats as a GetSystemStatsQuery response object
        return response

def main():
    rospy.init_node("automation_manager")
    manager = AutomationManager()
    rospy.spin()

if __name__ == '__main__':
    main()