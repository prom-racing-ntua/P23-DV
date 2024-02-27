#!/usr/bin/env python3
import os
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory

def create_new_run_log() -> str:
    try:
        path = get_package_share_directory("node_logger")
        base_path = os.path.join(path, "..", "..", "..", "..", "timestamp_logs")
        current_runs = len(os.listdir(base_path))
    except Exception as e:
        return str(repr(e))
    
    if(current_runs!=0):
        if(len(os.listdir(os.path.join(base_path, "run_{:d}".format(current_runs-1))))==0):
            return "New dir exists."
        
    try:
        os.mkdir(os.path.join(base_path, "run_{:d}".format(current_runs)))
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
            path = get_package_share_directory("node_logger")
            base_path = os.path.join(path, "..", "..", "..", "..", "timestamp_logs")
            self.run_idx = len(os.listdir(base_path)) - 1
            self.file = open(os.path.join(base_path, "run_{:d}/{:s}_log.txt".format(self.run_idx, name)), "w")
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

    def __call__(self, timestamp, io, index, data = None):
        if not self.ok:
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