import os
import numpy as np
from matplotlib import pyplot as plt
import seaborn
from dataclasses import dataclass

@dataclass
class Entry:
    timestamp: float
    role: int
    index: int
    data: list[any]
    def __init__(self, timestamp, role, index, data):
        self.timestamp = timestamp
        self.role = role
        self.index = index
        self.data = data

@dataclass
class Log:
    lines: list[str]
    start_entries: list[float] = []
    pub_entries: list[float] = []
    time_response: list[float] = None
    t0: float = None
    times: list[float] = []
    run: int
    name: str

    def __init__(self, run: int, name: str, base: str):
        # run: 0, 1, ...
        # name: <<name>>_log.txt
        # base: <<base>>/timestamp_logs/run_...
        self.run = run
        self.name = name

        try:
            self.file = open("{:s}/timestamp_logs/run_{:d}/{:s}_log.txt".format(base, run, name), "r")
        except Exception as e:
            print("Couldn't open {:s} log: {:s}".format(name, e))
        else:
            print("Log {:s} open successfully.".format(name))

        self.fill_entries()

    def fill_entries(self):
        for line in self.lines:
            line = line.split()
            if self.t0 is None:
                self.t0 = line[0]
            self.times.append(line[0] - self.t0)
            if line[1] == 0:
                self.start_entries.append(Entry(line[0], line[1], line[2], line[3:]))
            elif line[1] == 1:
                self.pub_entries.append(Entry(line[0], line[1], line[2], line[3:]))
            else:
                print("Invalid log type encountered (!=0 or 1")

    def find_index(self, arr, a, b, x) -> int:
        i = int((a + b) / 2)
        ent = arr[i]
        if ent.index == x:
            return i
        elif ent.index > x:
            return self.find_index(arr, a, i, x)
        else:
            return self.find_index(arr, i, b, x)

    def match_index(self, requested_index, requested_type) -> Entry:
        if requested_type == 0:
            return self.start_entries[self.find_index(self.start_entries, 0, len(self.start_entries), requested_index)]
        elif requested_type == 1:
            return self.pub_entries[self.find_index(self.pub_entries, 0, len(self.pub_entries), requested_index)]
        else:
            return Entry(-1, -1, -1, [])
        
    def analyze_time_response(self, analysis_type: str):
        if len(self.start_entries)==0 or len(self.pub_entries)==0: return
        if self.time_response is None:
            for couple in [self.start_entries, self.pub_entries]:
                if couple[0].index == couple[1].index:
                    self.time_response.append(couple[1].timestamp - couple[0].timestamp)
        
        if self.time_response is not None:
            if(analysis_type == "plot"):
                plt.plot(self.times, self.time_response)
                plt.grid()
                plt.show()
            elif(analysis_type == 'histogram'):
                seaborn.histplot(self.time_response)
                plt.grid()
                plt.show()
            elif(analysis_type == 'data'):
                return [self.time, self.time_response]
            elif(analysis_type == 'mean'):
                return np.mean(self.time_response)
            else:
                print('invalid request: {:s}'.format(analysis_type))


@dataclass
class Chain:
    logs: list[Log]
    log_final: bool
    n: int
    delays_n: int
    mx: int = 0
    comm_delays: list[float] = []
    time_response: list[float] = None
    times: list[float] = []
    t0: float = None
    def __init__(self, logs: list[Log], log_final:bool = 0):
        self.logs = logs
        self.log_final = log_final
        self.n = len(logs)
        self.delays_n = 2 * self.n - 3 + log_final
        lengths = np.empty([self.n])

        for log in logs:
            lengths.append(max(len(log.start_entries), len(log.pub_entries)))
            self.mx = max(self.mx, lengths[-1])

        self.delays = np.zeros([self.mx, self.delays_n])

    def fill_delays(self):
        for i in range(1, self.n):
            if self.t0 in None:
                self.t0 = self.logs[i].t0
            else:
                self.t0 = min(self.logs[i].t0, self.t0)

            for j in range(0, len(self.logs[i].start_entries)):
                current_start_entry = self.logs[i].start_entries[j]
                current_pub_entry = self.logs[i].pub_entries[j]
                prev_pub_entry = self.logs[i-1].match_index(current_start_entry.index, 1)

                self.delays[j, 2*i - 1] = current_pub_entry.timestamp - current_start_entry.timestamp
                if i != self.n-1 or self.log_final == 1:
                    self.delays[j, 2*i - 2] = current_start_entry.timestamp - prev_pub_entry.timestamp
                    self.comm_delays.append(self.delays[j, 2*i - 2])

    def analyze_time_response(self, analysis_type: str):
        if len(self.start_entries)==0 or len(self.pub_entries)==0: return
        if self.time_response is None:
            for couple in [self.log[0].start_entries, self.log[-1].pub_entries]:
                if couple[0].index == couple[1].index:
                    self.time_response.append(couple[1].timestamp - couple[0].timestamp)
        
        if self.time_response is not None:
            if(analysis_type == "plot"):
                plt.plot(self.times, self.time_response)
                plt.grid()
                plt.show()
            elif(analysis_type == 'histogram'):
                seaborn.histplot(self.time_response)
                plt.grid()
                plt.show()
            elif(analysis_type == 'data'):
                return [self.time, self.time_response]
            elif(analysis_type == 'mean'):
                return np.mean(self.time_response)
            else:
                print('invalid request: {:s}'.format(analysis_type))

    def get_all_comm_delays(self):
        if self.comm_delays is not None:
            return self.comm_delays
        
    
base = '../ROS_Workspace'
run: int = 0
run = input('Enter desired run index: ')
callbacks = [
    'saltas', 
    'acquisition_left', 'acquisition_right' 
    'inference_left', 'inference_right',
    'slam_perception', 'slam_odometry', 'slam_optim',
    'velocity',
    'path_planning',
    'pid_pp_waypoints', 'pid_pp_pose',
    'canbus_sensor', 'canbus_wheel', 'canbus_steering', 'canbus_controls', 'canbus_velocity'
]

logs:list[Log] = []
logs_dict:dict[str, Log] = {}
for item in callbacks:
    logs.append(Log(run, item, base))
    logs_dict[item] = logs[-1]

chains: list[list[Log]] = [
    [logs_dict['saltas'], logs_dict['acquisition_left'], logs_dict['inference_left'], logs_dict['slam_perception']],
    [logs_dict['saltas'], logs_dict['acquisition_right'], logs_dict['inference_right'], logs_dict['slam_perception']],
    [logs_dict['saltas'], logs_dict['velocity'], logs_dict['canbub_velocity']],
    [logs_dict['saltas'], logs_dict['velocity'], logs_dict['slam_odometry'], logs_dict['pid_pp_pose'], logs_dict['canbus_controls'],],
    [logs_dict['slam_optim'], logs_dict['path_planning'], logs_dict['pid_pp_waypoints']]
]


chains_logs: list[Chain] = []
for chain in chains:
    chains_logs.append(Chain(chain, 'canbus' in chain[-1].name))

operation: int = None
corr_name: int = None

operation = input("""Enter the desired operation:
                  0: histogram of callback
                  1: plot of time response of callback
                  2: mean of time response of callback
                  4: histogram of chain
                  5: plot of time response of chain
                  6: mean of time response of chain
                  7: histogram of communication delays of callback
                  7: histogram of communication delays of chain
                  7: histogram of complete communication delays
                  >>> """)

corr_name = input("""Enter the desired item
                  0:  saltas
                  1:  acquisition left
                  2:  acquisition right
                  3:  inference left
                  4:  inference right
                  5:  SLAM perception
                  6:  SLAM odometry
                  7:  SLAM optimization
                  8:  Velocity Estimation
                  9:  Path Planning
                  10: PID - PP Pose Callback
                  11: PID - PP Waypoints Callback
                  12: CAN2USB Sensor Data
                  13: CAN2USB Wheel Speed
                  14: CAN2USB Steering Angle
                  15: CAN2USB Control Coommands
                  16: CAN2USB Velociy Estimation
                  ---------
                  17: saltas-acq-inf-slam (left)
                  18: saltas-acq-inf-slam (right)
                  19: saltas-velocity-can2usb
                  20: saltas-velocity-slam-controls
                  21: salm-path-controls
                  >>> """)

match operation:
    case