#!/usr/bin/env python3
import os
from dataclasses import dataclass

def create_new_run_log() -> str:
    try:
        current_runs = len(os.listdir("/home/prom/P23-DV/ROS_Workspace/timestamp_logs"))
    except Exception as e:
        return str(repr(e))
    
    if(current_runs!=0):
        if(len(os.listdir("/home/prom/P23-DV/ROS_Workspace/timestamp_logs/run_{:d}".format(current_runs-1)))==0):
            return "New dir exists."
        
    try:
        os.mkdir("/home/prom/P23-DV/ROS_Workspace/timestamp_logs/run_{:d}".format(current_runs))
    except FileExistsError:
        return "New dir exists."
    except Exception as e:
        return "Error while creating new run dir:{:s}".format(repr(e))
    else:
        return "New run dir created successfully."
    
@dataclass
class Logger:
    ok: bool
    name: str
    run_idx: int
    file: any
    error: Exception
    def __init__(self, name):
        self.ok = True
        self.name = name
        # if name[0:6]!='canbus':
        #   self.ok = False
        #   self.error = Exception()
        try:
            self.run_idx = len(os.listdir("/home/prom/P23-DV/ROS_Workspace/timestamp_logs")) - 1
            self.file = open("/home/prom/P23-DV/ROS_Workspace/timestamp_logs/run_{:d}/{:s}_log.txt".format(self.run_idx, name), "w")
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
            return "Couldn't open logger {:s}: {:s}".format(self.name, repr(self.error))

    def __call__(self, timestamp, type, index):
        if not self.ok:
            return

        self.file.write("{:0.8f}\t{:d}\t{:d}\n".format(timestamp, type, index))

    def __call__(self, timestamp, type, index, data = None):
        if not self.ok:
            return
        try:
            string = ""
            if data is not None:
                for i in data:
                    string = "{:s}\t{:.3f}".format(string, i)
            self.file.write("{:0.8f}\t{:d}\t{:d}{:s}\n".format(timestamp, type, index, string))
        except Exception as e:
            self.file.write("Error during writing: {:s}".format(repr(e)))
            self.file.write("Aborting writing. Plz fix!")
            print("Error during writing: {:s}".format(repr(e)))
            print("Aborting writing. Plz fix!")
            self.ok = False
