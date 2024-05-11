#!/usr/bin/env python3
import os
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory
import shutil

class enabled_logs:
    logs:dict[str, bool] = {
        'saltas':             False, 
        'acquisition_left':   False,
        'acquisition_right':  False,
        'inference_left':     True,
        'inference_right':    True,
        # C++ 'slam_perception':    False,
        # C++ 'slam_odometry':      False,
        # C++ 'slam_optim':         True,
        # C++ 'velocity':           False,
        # C++ 'pid_pp_waypoints':   False,
        # C++ 'pid_pp_pose'  :      False,
        # C++ 'path_planning':      False,
        'vn200':              True,
        'vn300':              False,
        'canbus_sensor':      True,
        'canbus_velocity':    True,
        'canbus_wheel':       False,
        'canbus_steering':    True,
        'canbus_controls':    True,
        'autoexposure_left':  False,
        'autoexposure_right': False,
        'cone_data':          False
    }
    def __init__(self) -> None:
        self.logs = {
        'saltas':             False, 
        'acquisition_left':   False,
        'acquisition_right':  False,
        'inference_left':     True,
        'inference_right':    True,
        # C++ 'slam_perception':    False,
        # C++ 'slam_odometry':      False,
        # C++ 'slam_optim':         True,
        # C++ 'velocity':           False,
        # C++ 'pid_pp_waypoints':   False,
        # C++ 'pid_pp_pose'  :      False,
        # C++ 'path_planning':      False,
        'vn200':              True,
        'vn300':              False,
        'canbus_sensor':      True,
        'canbus_velocity':    True,
        'canbus_wheel':       False,
        'canbus_steering':    True,
        'canbus_controls':    True,
        'autoexposure_left':  False,
        'autoexposure_right': False,
        'cone_data':          False
        }
    def __call__(self, log: str) -> bool:
        try:
            return self.logs[log]
        except KeyError:
            return False
        

def create_new_run_log() -> str:
    try:
        path = get_package_share_directory("node_logger")
        base_path = os.path.join(path, "..", "..", "..", "..", "timestamp_logs")
        current_runs = len(os.listdir(base_path))
    except Exception as e:
        return str(repr(e))
    
    if(current_runs!=0):
        lsd = os.listdir(os.path.join(base_path, "run_{:d}".format(current_runs-1)))
        if(len(lsd)==0 or (len(lsd)==1 and 'config' in lsd)):
            return "New dir exists."
        
    try:
        new_path = os.path.join(base_path, "run_{:d}".format(current_runs))
        os.mkdir(new_path)
    except FileExistsError:
        return "New dir exists."
    except Exception as e:
        return "Error while creating new run dir:{:s}".format(repr(e))
    else:
        try:
            config_path = os.path.join(base_path, '..', 'src', '0.Master', 'lifecycle_manager', 'config')
            shutil.copytree(config_path, os.path.join(new_path, 'config'))
        except Exception as e:
            pass
        return "New run dir created successfully."
    
    
    
@dataclass
class Logger:
    ok: bool
    enabled: bool
    name: str
    run_idx: int
    file: any
    error: Exception
    run_path: str
    def __init__(self, name) -> None:
        ENABLED_LOGS = enabled_logs()
        if not ENABLED_LOGS(name):
            self.enabled = False
            self.ok = False
            self.name = name
            return
        self.enabled = True
        self.ok = True
        self.name = name

        try:
            path = get_package_share_directory("node_logger")
            base_path = os.path.join(path, "..", "..", "..", "..", "timestamp_logs")
            self.run_idx = len(os.listdir(base_path)) - 1
            self.file = open(os.path.join(base_path, "run_{:d}/{:s}_log.txt".format(self.run_idx, name)), "w")
            self.run_path = os.path.join(base_path, f"run_{self.run_idx}")
        except Exception as e:
            self.ok = False
            self.error = e
        else:
            self.error = None

    def __del__(self) -> None:
        if self.ok:
            self.file.close()

    def check(self) -> str:
        if not self.enabled:
            return "Logger {:s} has been disabled".format(self.name)
        if self.ok:
            return "Logger {:s} opened successfully".format(self.name)
        else:
            return "Couldn't open logger {:s}: {:s}".format(self.name, repr(self.error))

    def __call__(self, timestamp, type, index) -> None:
        if not self.ok or not self.enabled:
            return

        self.file.write("{:0.8f}\t{:d}\t{:d}\n".format(timestamp, type, index))

    def __call__(self, timestamp, io, index, data = None) -> None:
        if not self.ok or not self.enabled:
            return
        try:
            string = ""
            if data is not None:
                for i in data:
                    if type(i) is float:
                        string = "{:s}\t{:.3f}".format(string, i)
                    elif type(i) is int or type(i) is bool:
                        string = "{:s}\t{:d}".format(string, i)
                    elif type(i) is str:
                        string = "{:s}\t{:s}".format(string, i)
                    else:
                        print('Requested data type unaccounted for: {}'.format(type(i)))
            self.file.write("{:0.8f}\t{:d}\t{:d}{:s}\n".format(timestamp, io, index, string))
        except Exception as e:
            self.file.write("Error during writing: {:s}".format(repr(e)))
            self.file.write("Aborting writing. Plz fix!")
            print("Error during writing: {:s}".format(repr(e)))
            print("Aborting writing. Plz fix!")
            self.ok = False

    def get_run_path(self) -> str | None:
        if self.ok:
            return self.run_path
        else:
            return None

@dataclass
class old_Logger:
    ok: bool
    name: str
    run_idx: int
    file: any
    error: Exception
    def __init__(self, name):
        self.ok = True
        self.name = name
        
        try:
            path = get_package_share_directory("node_logger")
            base_path = os.path.join(path, "..", "..", "..", "..", "timestamp_logs")
            self.run_idx = len(os.listdir(base_path)) - 1
            self.filename = os.path.join(base_path, "run_{:d}/{:s}_log.txt".format(self.run_idx, name))
            self.file = open(self.filename , "w")
        except Exception as e:
            self.ok = False
            self.error = e
        else:
            self.error = None

    def __del__(self):
        if self.ok:
            self.file.close()

    def check(self):
        if self.ok:
            return "Logger {:s} opened successfully".format(self.name)
        else:
            return "Couldn't open logger {:s}, {:s}: {:s}".format(self.name,self.filename, repr(self.error))

    def __call__(self, data):
        if not self.ok:
            return
        self.file.write(data)