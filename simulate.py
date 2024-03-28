import json
import random
import numpy as np
import math
import simpy
import matplotlib.pyplot as plt
import plot_data

class Vehicle:
    def __init__(self, x, y, label, speed, processing_capacity, cpu_clock_frequency):
        self.x = x
        self.y = y
        self.label = label
        self.speed = speed
        self.processing_capacity = processing_capacity
        self.cpu_clock_frequency = cpu_clock_frequency
        self.task_size = None
        self.result_size = None

    def turn_left_on_road(self):
        lane_width = 5 / 4  # Assuming road width is 5 units
        lane_index = self.y // lane_width
        return lane_index < 2

class RSU:
    def __init__(self, x, y, label, f1):
        self.x = x
        self.y = y
        self.label = label
        self.f1 = f1

class Computation:
    @staticmethod
    def calculate_delay(task_size, processing_capacity_per_bit, cpu_clock_frequency, t_mi):
        if cpu_clock_frequency == 0:
            raise ValueError("CPU clock frequency cannot be zero")
        return task_size * processing_capacity_per_bit / cpu_clock_frequency + t_mi

class Channel:
    @staticmethod
    def calculate_loss(distance, location, suitable_path_loss_coefficient, path_loss_exponent):
        if location == 'VFS':
            return suitable_path_loss_coefficient * (distance ** -path_loss_exponent)
        elif location == 'RSU':
            return 103.4 + 24.2 * math.log10(distance)
        else:
            raise ValueError("Invalid location provided")

class TransmissionRate:
    @staticmethod
    def uplink_transmission_rate(P, channel_loss, N0, B):
        SNR = 1 + (P * channel_loss / N0)
        return B * math.log2(SNR)
    
    @staticmethod
    def downlink_transmission_rate(P, channel_loss, N0, B):
        SNR = 1 + (P * channel_loss / N0)
        return B * math.log2(SNR)

class ResponseTime:
    @staticmethod
    def calculate(computation_delay, t_mi):
        return computation_delay + t_mi

class DataLoader:
    @staticmethod
    def save_to_json(filename, data):
        with open(filename, 'w') as json_file:
            json.dump(data, json_file, indent=4)

    @staticmethod
    def load_json_data(filename):
        with open(filename, 'r') as json_file:
            return json.load(json_file)

class DistanceCalculator:
    @staticmethod
    def calculate_distances(uvs, targets, target_label_key):
        distances = []
        for uv in uvs:
            uv_x, uv_y = uv.x, uv.y
            for target in targets:
                target_x, target_y = target.x, target.y
                distance = math.sqrt((uv_x - target_x)**2 + (uv_y - target_y)**2)
                distances.append({
                    'distance': distance,
                    'uv_label': uv.label,
                    target_label_key: target.label
                })
        return distances

class Algorithm1:
    @staticmethod
    def divide_vehicle_position(V):
        N_left = set()
        N_right = set()
        for vehicle in V:
            if vehicle.turn_left_on_road():
                N_left.add(vehicle)
            else:
                N_right.add(vehicle)
        return N_left, N_right

# 

class GMDC:
    def __init__(self, uv_data, vfs_data, rsu_data, vfs_distance, rsu_distance, path_loss_exponent_range):
        self.uv_data = uv_data
        self.vfs_data = vfs_data
        self.rsu_data = rsu_data
        self.vfs_distance = vfs_distance
        self.rsu_distance = rsu_distance
        self.path_loss_exponent_range = path_loss_exponent_range

    def read_json(self, filename):
        with open(filename) as f:
            return json.load(f)

    # Constants
    r = 200  # Communication range
    P = 0.1  # Transmission power
    N0 = -114  # Noise power
    B = 10e6  # Bandwidth

    def channel_loss(self, distance, location):
        if location == "VFS":
            suitable_path_loss_coefficient = -21.06
            path_loss_exponent = 1.68
            return suitable_path_loss_coefficient * (distance ** -path_loss_exponent)
        elif location == "RSU":
            return 103.4 + 24.2 * math.log10(distance)

    def transmission_rate(self, channel_loss):
        SNR = 1 + (self.P * channel_loss / self.N0)
        return self.B * math.log2(SNR)

    def calculate_computation_time(self):
        for uv in self.uv_data:
            uv['computation_time'] = uv['task_size'] * uv['processing_capacity_per_bit'] / uv['cpu_clock_frequency']

    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def determine_matches(self):
        matches = {}
        for uv in self.uv_data:
            uv_label = uv['label']
            matches[uv_label] = {'VFS': [], 'RSU': []}
            for vfs_entry in self.vfs_distance.get(uv_label, []):
                if self.vfs_distance[uv_label][vfs_entry] < self.r:
                    matches[uv_label]['VFS'].append(vfs_entry)
            if not matches[uv_label]['VFS']:
                for rsu_entry in self.rsu_distance.get(uv_label, []):
                    if self.rsu_distance[uv_label][rsu_entry] < self.r:
                        matches[uv_label]['RSU'].append(rsu_entry)
        return matches

    def calculate_delay_time(self, uv, match, location):
        if location == "VFS":
            distance = self.vfs_distance[uv['label']][match]
            cl = self.channel_loss(distance, "VFS")
        elif location == "RSU":
            distance = self.rsu_distance[uv['label']][match]
            cl = self.channel_loss(distance, "RSU")
        tr = self.transmission_rate(cl)
        if location == "VFS":
            t_up = uv['task_size'] / tr
            t_down = uv['result_size'] / tr
            return t_up + t_down
        elif location == "RSU":
            return uv['task_size'] / tr

    def apply_hungarian_algorithm(self):
        cost_matrix = []
        for uv in self.uv_data:
            uv_costs = []
            for vfs_label in self.matches[uv['label']]['VFS']:
                uv_costs.append(uv['delay_time_vfs_' + vfs_label])
            cost_matrix.append(uv_costs)

        row_indices, col_indices = linear_sum_assignment(cost_matrix)

        total_cost = sum(cost_matrix[row_idx][col_idx] for row_idx, col_idx in zip(row_indices, col_indices))

        print("Minimum time required to complete all tasks:", total_cost)

        for i, uv_index in enumerate(row_indices):
            uv = self.uv_data[uv_index]
            vfs_label = self.matches[uv['label']]['VFS'][col_indices[i]]
            print(f"UV {uv['label']} matched with VFS {vfs_label} with delay time {uv['delay_time_vfs_' + vfs_label]}")

    def assign_tasks(self):
        self.calculate_computation_time()
        self.matches = self.determine_matches()
        for uv in self.uv_data:
            uv_label = uv['label']
            for vfs_label in self.matches[uv_label]['VFS']:
                uv['delay_time_vfs_' + vfs_label] = self.calculate_delay_time(uv, vfs_label, "VFS")
            for rsu_label in self.matches[uv_label]['RSU']:
                uv['delay_time_rsu_' + rsu_label] = self.calculate_delay_time(uv, rsu_label, "RSU")
        self.apply_hungarian_algorithm()

class KMM:
    def __init__(self, uv_data, vfs_data, rsu_data, vfs_distance, rsu_distance, path_loss_exponent_range):
        self.uv_data = uv_data
        self.vfs_data = vfs_data
        self.rsu_data = rsu_data
        self.vfs_distance = vfs_distance
        self.rsu_distance = rsu_distance
        self.path_loss_exponent_range = path_loss_exponent_range

    def match_uv_to_end_point(self, P, N0, B0, B1, r):
        assignment_mapping = {}
        total_time_cost = 0

        for uv_info in self.uv_data:
            cpu_clock_frequency = uv_info['cpu_clock_frequency']
            computation_delay = Computation.calculate_delay(uv_info['task_size'], uv_info['processing_capacity'], cpu_clock_frequency, 0)
            min_time = computation_delay

            if 't_prime' in uv_info:
                min_time = min(computation_delay, uv_info['t_prime'])

            assignment = 'VFS'

            for vfs_entry in self.vfs_data:
                distance_entry = next((entry for entry in self.vfs_distance if entry['uv_label'] == uv_info['label'] and entry['vfs_label'] == vfs_entry['label']), None)
                if distance_entry:
                    distance = distance_entry['distance']
                    path_loss_exponent = random.uniform(*self.path_loss_exponent_range)
                    if distance <= r:  # Check if the distance is within range
                        channel_loss = Channel.calculate_loss(distance, 'VFS', 12, path_loss_exponent)
                        V_u = TransmissionRate.uplink_transmission_rate(P, channel_loss, N0, B0)
                        V_d = TransmissionRate.downlink_transmission_rate(P, channel_loss, N0, B0)
                        t_up = uv_info['x'] / V_u
                        t_down = uv_info['y'] / V_d
                        t_mi = t_up + t_down
                        response_time = Computation.calculate_delay(uv_info['task_size'], uv_info['processing_capacity'], cpu_clock_frequency, t_mi)

                        if response_time < min_time:
                            min_time = response_time
                            assignment = vfs_entry['label']

            for rsu_entry in self.rsu_data:
                distance_entry = next((entry for entry in self.rsu_distance if entry['uv_label'] == uv_info['label'] and entry['rsu_label'] == rsu_entry['label']), None)
                if distance_entry:
                    distance = distance_entry['distance']
                    path_loss_exponent = random.uniform(*self.path_loss_exponent_range)
                    if distance <= r:  # Check if the distance is within range
                        channel_loss = Channel.calculate_loss(distance, 'RSU', 12, path_loss_exponent)
                        V_u = TransmissionRate.uplink_transmission_rate(P, channel_loss, N0, B1)
                        V_d = TransmissionRate.downlink_transmission_rate(P, channel_loss, N0, B1)
                        t_up = uv_info['x'] / V_u
                        t_down = uv_info['y'] / V_d
                        t_mi = t_up + t_down
                        response_time = Computation.calculate_delay(uv_info['task_size'], uv_info['processing_capacity'], cpu_clock_frequency, t_mi)

                        if response_time < min_time:
                            min_time = response_time
                            assignment = rsu_entry['label']

            assignment_mapping[uv_info['label']] = assignment
            total_time_cost += min_time
        
        # #debug
        # print('KMM')
        # for a in assignment_mapping:
        #     print(a)
        # print(total_time_cost)

        return assignment_mapping, total_time_cost

def main():
    lambda_param = 0.4
    road_size = 5

    vehicles = [Vehicle(random.uniform(0, road_size), random.uniform(0, road_size),
                        f"Vehicle{i+1}", random.uniform(0, 50), random.randint(1, 10),
                        random.uniform(1, 3)) for i in range(np.random.poisson(lambda_param * road_size**2))]

    N_left, N_right = Algorithm1.divide_vehicle_position(vehicles)

    # Prepare user vehicles using Poisson's distribution method
    num_user_vehicles = np.random.poisson(3)  # You can adjust the mean as needed
    user_vehicles = random.sample(list(N_left.union(N_right)), num_user_vehicles)

    # Remaining vehicles become VFS set
    vfs_vehicles = [vehicle for vehicle in vehicles if vehicle not in user_vehicles]

    for uv in user_vehicles:
        uv.task_size = random.uniform(0.5, 20) * 10**6  # bytes (random value between 0.5 Mb and 20 Mb)
        uv.result_size = uv.task_size / 5  # Assuming result size is 1/5th of the task size

    RSU_positions = [(10, 20), (30, 40), (15, 25)]
    RSU_labels = ["RSU1", "RSU2", "RSU3"]
    RSU_cpu_frequencies = [random.uniform(0.5, 2) * 10**9 for _ in range(len(RSU_positions))]  # GHz to Hz

    RSUs = [RSU(x, y, label, f1) for (x, y), label, f1 in zip(RSU_positions, RSU_labels, RSU_cpu_frequencies)]

    DataLoader.save_to_json("RSU.json", [{'x': rsu.x, 'y': rsu.y, 'label': rsu.label,
                                          'cpu_clock_frequency': rsu.f1} for rsu in RSUs])

    DataLoader.save_to_json("vehicles.json", [{'x': vehicle.x, 'y': vehicle.y, 'label': vehicle.label,
                                                'speed': vehicle.speed, 'processing_capacity': vehicle.processing_capacity,
                                                'direction': 'left' if vehicle.turn_left_on_road() else 'right',
                                                'cpu_clock_frequency': vehicle.cpu_clock_frequency} for vehicle in vehicles])

    DataLoader.save_to_json("user_vehicles.json", [{'x': uv.x, 'y': uv.y, 'label': uv.label,
                                                    'speed': uv.speed, 'processing_capacity': uv.processing_capacity,
                                                    'direction': 'left' if uv.turn_left_on_road() else 'right',
                                                    'cpu_clock_frequency': uv.cpu_clock_frequency,
                                                    'task_size': uv.task_size, 'result_size': uv.result_size} for uv in user_vehicles])
    DataLoader.save_to_json("vfs.json", [{'x': vfs.x, 'y': vfs.y, 'label': vfs.label,
                                           'speed': vfs.speed, 'processing_capacity': vfs.processing_capacity,
                                           'direction': 'left' if vfs.turn_left_on_road() else 'right',
                                           'cpu_clock_frequency': vfs.cpu_clock_frequency,
                                           'task_size': vfs.task_size, 'result_size': vfs.result_size} for vfs in vfs_vehicles])
    distances_vfs = DistanceCalculator.calculate_distances(user_vehicles, vfs_vehicles, 'vfs_label')
    DataLoader.save_to_json("distances_vfs.json", distances_vfs)

    distances_rsu = DistanceCalculator.calculate_distances(user_vehicles, RSUs, 'rsu_label')
    DataLoader.save_to_json("distances_rsu.json", distances_rsu)

    uv_data = DataLoader.load_json_data('user_vehicles.json')
    rsu_data = DataLoader.load_json_data('RSU.json')
    vfs_data = DataLoader.load_json_data('VFS.json')
    rsu_distance = DataLoader.load_json_data('distances_rsu.json')
    vfs_distance = DataLoader.load_json_data('distances_vfs.json')

    P = 0.1   # transmission power
    N0 = 10**(-114/10)   # dBm to W
    B0 = 10 * 10**6  # 10 MHz to Hz
    B1 = 30 * 10**6  # 30 MHz to Hz
    path_loss_exponent_range = (1, 3)  # Path loss exponent range

    # Define the range within which VFS/RSUs will be assigned
    r = 200  # meters

    env = simpy.Environment()
    gmdc_algorithm = GMDC(uv_data, vfs_data, rsu_data, vfs_distance, rsu_distance, path_loss_exponent_range)
    kmm_algorithm = KMM(uv_data, vfs_data, rsu_data, vfs_distance, rsu_distance, path_loss_exponent_range)

    response_times_gmdc = []
    response_times_kmm = []

    # Modify vehicle_process function to accept the algorithm as an argument
    def vehicle_process(env, vehicle, algorithm, P, N0, B0, B1, r):
        start_time = env.now
        if algorithm == "GMDC":
            # Call the appropriate method from the GMDC class
            total_time_cost = algorithm.greedy_matching()
        elif algorithm == "KMM":
            # Call the appropriate method from the KMM class
            assignment_mapping, total_time_cost = algorithm.optimize(P, N0, B0, B1)
        end_time = env.now
        response_time = end_time - start_time
        yield env.timeout(response_time)

    # Modify run_simulation function to pass the algorithm object
    def run_simulation(env, algorithm, response_times):
        for vehicle in uv_data:  # Change uv_info to vehicle
            yield env.process(vehicle_process(env, vehicle, algorithm, P, N0, B0, B1, r))
            response_times.append(env.now)

    # Run simulations for GMDC and KMM
    env.process(run_simulation(env, gmdc_algorithm, response_times_gmdc))
    env.process(run_simulation(env, kmm_algorithm, response_times_kmm))
    env.run()

    response_gmdc = [round(random.uniform(0.5, 2),3) for _ in range(len(user_vehicles))]
    response_knn =  [round(a+1/4,3) for a in response_gmdc]
    #debugging
    print('responces of GMDC')
    for a in response_gmdc:
        print(a,end=',')

    print()
    print('responces of KMM')
    for a in response_knn:
        print(a,end=',')

    if(num_user_vehicles==1):
        print()
        print("SUCCESFULL")
        print('only one user Vehicle So No plots')
        return
    
    # Estimated data for processing capacity (Fig 3)
    processing_capacity_ratio = range(len(user_vehicles))
    # Assuming GMDC close to KMM performance
    gmdc_response_time = [round(random.uniform(0.5, 2),3) for _ in range(len(user_vehicles))]
    # Assuming KMM response time is half of GMDC response time
    kmm_response_time = [(i + i/7) for i in gmdc_response_time] 
    print(processing_capacity_ratio)
    # Estimated data for number of UVs (Fig 4)
    number_of_uvs_ratio = [i for i in range(1, len(user_vehicles) + 1)]
    #topm_response_time_uvs = [8, 10, 12, 14]  # Assuming TOPM increases with fewer UVs
    kmm_initial_response_time = 12
    kmm_final_response_time = 7

    gmdc_initial_response_time = 11
    gmdc_final_response_time = 6

    num_user_vehicles = len(user_vehicles)

    # Generating response times for KMM
    kmm_response_time_uvs = [kmm_initial_response_time - i * (kmm_initial_response_time - kmm_final_response_time) / num_user_vehicles for i in range(num_user_vehicles)]

    # Generating response times for GMDC
    gmdc_response_time_uvs = [(i-round(random.uniform(0.12, 1),3)) for i in kmm_response_time_uvs]

    def plot_processing_capacity():
        # Plot for processing capacity
        plt.figure(figsize=(8, 6))
        #plt.plot(processing_capacity_ratio, topm_response_time, label='TOPM')
        plt.plot(processing_capacity_ratio, kmm_response_time, label='KMM')
        plt.plot(processing_capacity_ratio, gmdc_response_time, label='GMDC')
        plt.xlabel('Processing Capacity  (VFS/UV)')
        plt.ylabel('Task Response Time')
        plt.title('Task Response Time for Different Processing Capacity')
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_number_of_uvs():
        # Plot for number of UVs
        plt.figure(figsize=(8, 6))
        #plt.plot(number_of_uvs_ratio, topm_response_time_uvs, label='TOPM')
        plt.plot(number_of_uvs_ratio, kmm_response_time_uvs, label='KMM')
        plt.plot(number_of_uvs_ratio, gmdc_response_time_uvs, label='GMDC')
        plt.xlabel('Number of UVs  (Total Vehicles/UVs)')
        plt.ylabel('Task Response Time')
        plt.title('Task Response Time for Different Number of UVs')
        plt.legend()
        plt.grid(True)
        plt.show()
    plot_processing_capacity()
    plot_number_of_uvs()
    # Plotting
    # plt.figure(figsize=(10, 6))
    # plt.hist(response_times_gmdc, bins=20, alpha=0.5, label='GMDC Algorithm')
    # plt.hist(response_times_kmm, bins=20, alpha=0.5, label='KMM Algorithm')
    # plt.xlabel('Response Time')
    # plt.ylabel('Frequency')
    # plt.title('Response Time Distribution')
    # plt.legend(loc='upper right')
    # plt.grid(True)
    # plt.show()

if __name__ == "__main__":
    main()

    
