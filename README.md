
# A dynamic task offloading algorithm based on greedy matching in vehicle network

The project implemented a dynamic task offloading algorithm for vehicular ad hoc networks (VANETs) based on greedy matching principles. The algorithm can dynamically assign tasks to communication endpoints based on real-time network conditions, task characteristics, and resource availability. This approach aims to optimize task assignment, minimize response times, and enhance service quality for cloud-based applications in vehicular environments. Existing task offloading approaches may not fully address the unique challenges caused by VANETs, necessitating innovative solutions.

## Team Members
- Irigi Yuva Kumar  - 232CS013
- Dhananjani Jayarukshi- 232CS014
- Himanshu Thakkar - 232CS035

## Algorithms used
1. KMM
2. GMDC

## Greedy Matching Algorithm

- Uses a greedy matching approach to assign tasks to available VFSs or RSUs. 
- It is for adapting the dynamics of traffic environment.
- In GMDC, the user vehicle appears randomly.
- It initializes parameters like communication range, transmission power, noise power, available bandwidth, and path loss exponent range. 
- Data preparation involves loading vehicle, VFS, and RSU data, and assigning tasks based on communication ranges and delay time. 
- The algorithm minimizes total response time by selecting the best match for each vehicle.

## KMM Algorithm
- The KMM algorithm is a sophisticated task assignment optimization method. 
- It involves initializing parameters and loading data, determining potential matches with VFSs and RSUs, optimizing the assignment to minimize total response time, and evaluating the algorithm's effectiveness by analyzing assignment results and total response time.
- It uses a two-time selection mechanism to select the offload server and uses the Kuhn-Munkras (KM) algorithm for the final decision. 


## Requirements
 - Python 3.10.12
 - Required Python packages (simpy, matplotlib)
   
## Installation
1. Clone the repository to your local machine:

```sh
git clone https://github.com/dhana1023/Cloud-Project.git
```

2. Navigate to the project directory:
```sh
cd your_project
```
3. Install the required python packages:
   - simpy
   - matplotlib
     
## To execute 
`python3 simulate.py`

It will generate the data in the form of json files.

- RSU.json
- vfs.json
- user_vehicle.json
- distances_rsu.json
- distances_vfs.json
- vehicles.json


This project is designed to simulate vehicle movement and generate various data files and plots based on the simulation. The simulation is executed through the simulate.py script.
