import gc
import math
import multiprocessing as mp
import os
import pickle
import pdb
import pandas as pd
import sys
import cProfile
from functools import partial
from itertools import product

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..\..','Simulations'))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..\..','Framework'))
from Location import Location
from LoRaPacket import *
import SimulationProcess
from GlobalConfig import *

# The console attempts to auto-detect the width of the display area, but when that fails it defaults to 80
# characters. This behavior can be overridden with:
desired_width = 320
pd.set_option('display.width', desired_width)

class RunSettings:
    def __init__(self, mac_schemes, cell_size, num_nodes, sf, tx_intervall,
            num_simulations=1, p_sizes=10, std_jitter=[0, 0], guard_time=[0]):
        self.mac_schemes = mac_schemes
        self.cell_size = cell_size
        self.num_nodes = num_nodes
        self.sf = sf
        self.num_simulations = num_simulations
        self.p_sizes = p_sizes
        self.tx_intervall = TX_Intervall
        self.std_jitter = std_jitter
        self.guard_time = guard_time

#Payload Size Combinations
payload_sizes = [10, 51, 221] #SF 7
#payload_sizes = [10, 51, 115] #SF 9
#payload_sizes = [10, 51] #SF 12

#Network Cell Size
cell_size=1500
# Spreading Factor
sf=[7] #SF = 7,8,9,10,11,12

#Number of nodes Combinations
num_nodes = [500, 1000]

#Transmission Intervals Combinations (in minutes)
TX_Intervall =[3, 5, 8, 10, 12, 15, 17, 20, 25, 30] #SF 7 
#TX_Intervall =[5, 10, 15, 20, 25, 30, 35, 40, 45, 50] #SF 9
#TX_Intervall =[16, 25, 35, 45, 60, 75, 90, 105, 120, 135] #SF 12



std_jitter=[[0, (680e-6)]] #array of array synchonization error distribution combinations (680us Gaussian distribution)
#std_jitter=[[0, 0],[0, (582e-6)/3],[1, 582e-6]] #[perfect synchonization, 582us Gaussian Distribution, 1 ms Uniform Distribution]
num_simulations = 2 #Number of simulations runs for each combination of (num_nodes, TX_Interval, std_jitter,)
guard_times = [3] #S-ALOHA guard time in ms; the array must contain the same number of elements of std_jitter

#Configuration of the S-ALOHA run 
s_aloha_gt = RunSettings(mac_schemes=['SLOTTED_ALOHA'], cell_size=cell_size, num_nodes=num_nodes, sf=sf, tx_intervall=TX_Intervall, std_jitter=std_jitter, guard_time=guard_times, p_sizes=payload_sizes) 

#Configuration of the P-ALOHA run 
p_aloha = RunSettings(mac_schemes=['ALOHA'], cell_size=cell_size, num_nodes=num_nodes, sf=sf, tx_intervall=TX_Intervall, p_sizes=payload_sizes)		

runs = [p_aloha, s_aloha_gt]

def touch(path):
    with open(path, 'a'):
        os.utime(path, None)

def generate_locations(nodes, cell_size):
    locations_file = "Locations/"+"{}_locations_{}_cellsize_{}_sim.pkl".format(nodes, cell_size, num_simulations)
    locations_per_simulation = list()
    for num_sim in range(num_simulations):
        locations = list()
        for i in range(nodes):
            loc = Location(cell_size, indoor=False)
            locations.append(loc)
        locations_per_simulation.append(locations)

    if os.path.exists(locations_file):
        os.remove(locations_file)
    touch(locations_file)
    print('Generating {} locations with cell size {} for {} simulations'.format(nodes, cell_size, num_simulations))

    with open(locations_file, 'wb') as f:
        pickle.dump(locations_per_simulation, f)

def setup(nodes, payload_size, tx_intervall, cell_size, mac, sf, std_jitter=0, guard_time=0):
    if start_with_fixed_sf and mac == 'SLOTTED_ALOHA':
        results_file = "Results/{}_{}_{}_{}_{}_{}_{}_SF_{}_GT_{}_JITTER_{}.p".format(mac, adr, confirmed_messages, nodes, payload_size, tx_intervall, num_simulations, sf, guard_time, std_jitter)
    elif start_with_fixed_sf:
        results_file = "Results/{}_{}_{}_{}_{}_{}_{}_SF_{}.p".format(mac, adr, confirmed_messages, nodes, payload_size, tx_intervall, num_simulations, sf)
    else:
        results_file = "Results/{}_{}_{}_{}_{}_{}_{}_SF_random.p".format(mac, adr, confirmed_messages, nodes, payload_size, tx_intervall, num_simulations)
    locations_file = "Locations/"+"{}_locations_{}_cellsize_{}_sim.pkl".format(nodes, cell_size, num_simulations)
    with open(locations_file, 'rb') as file_handler:
        locations_per_simulation = pickle.load(file_handler)
        num_of_simulations = len(locations_per_simulation)
        check = len(locations_per_simulation[0])
    resume_from_simulation = 0

    if os.path.isfile(results_file) and load_prev_simulation_results:
        _results = pickle.load(open(results_file, "rb"))
        if 'idx_of_simulations_done' in _results:
            resume_from_simulation = _results['idx_of_simulations_done'] + 1
    else:
        os.makedirs(os.path.dirname(results_file), exist_ok=True)
        _results = {
            'cell_size': cell_size,
            'adr': adr,
            'confirmed_messages': confirmed_messages,
            'num_simulations': num_simulations,
            'total_devices': nodes,
            'nodes': dict(),
            'clocks': dict(),
            'gateway': dict(),
            'air_interface': dict(),
            'path_loss_variances': path_loss_variances,
            'payload_sizes': payload_sizes,
            'mean_energy': dict(),
            'std_energy': dict(),
            'num_of_simulations_done': 0
        }

    for payload_size in payload_sizes:
        _results['nodes'] = dict()
        _results['gateway'] = dict()
        _results['air_interface'] = dict()
        _results['mean_energy'] = dict()
        _results['std_energy'] = dict()

    return resume_from_simulation, locations_per_simulation, _results, results_file

def process_results(results, num_nodes, sim_idx, r):
    sim_idx = str(sim_idx)

    if sim_idx not in results['nodes']: 
        results['nodes'][sim_idx] = r['mean_nodes'] 
        results['gateway'][sim_idx] = r['gateway'] 
        results['air_interface'][sim_idx] = r['air_interface']
        results['mean_energy'][sim_idx] = np.mean(r['mean_energy_all_nodes']) 
        results['std_energy'][sim_idx] = np.std(r['mean_energy_all_nodes']) 
    else:
        results['nodes'][sim_idx] = results['nodes'][sim_idx] + r[
            'mean_nodes'] 
        results['gateway'][sim_idx] = results['gateway'][sim_idx] + r[
            'gateway'] 
        results['air_interface'][sim_idx] = results['air_interface'][sim_idx] + r[
            'air_interface'] 
        results['mean_energy'][sim_idx] = results['mean_energy'][sim_idx] + np.mean(
            r['mean_energy_all_nodes']) 
        results['std_energy'][sim_idx] = results['std_energy'][sim_idx] + np.std(
            r['mean_energy_all_nodes']) 
			
def run_parallel(nodes, sf, payload_size, tx_intervall, num_simulations,cell_size, mac, path_loss_variances, confirmed_messages, adr, std_jitter=0, guard_time=0):
	resume_from_simulation, locations_per_simulation, _results, results_file = setup(nodes, payload_size, tx_intervall, cell_size, mac, sf, std_jitter, guard_time)
	for n_sim in range(resume_from_simulation, num_simulations):
		print('\t\t\t\t\tSimulation: {} nodes, SF {}, TX-Interval {} min, Payload {} B,   #{} '.format(nodes, sf, tx_intervall, payload_size, n_sim + 1))
		locations = locations_per_simulation[n_sim]
		path_loss_variance = path_loss_variances[0] 
		# transmission rate is going to be outdated
		gateway_location = Location(cell_size, indoor=False, gateway=True)
		#r_list = cProfile.run('SimulationProcess.run(locations, mac, payload_size, tx_intervall, path_loss_variance, gateway_location, nodes, sf,confirmed_messages, adr, std_jitter, guard_time)')
		r_list = SimulationProcess.run(locations, mac, payload_size, tx_intervall, path_loss_variance, gateway_location, nodes, sf,confirmed_messages, adr, std_jitter, guard_time)
		gc.collect()
		
		_sim_idx = n_sim
		process_results(_results, nodes, _sim_idx, r_list)
		# update Results
		_results['idx_of_simulations_done'] = n_sim
		pickle.dump(_results, open(results_file, "wb"))

if __name__ == '__main__':
	pool = mp.Pool(mp.cpu_count())
	print('Generating Location for Cell Size {} and Nodes {}'.format(cell_size, num_nodes))
	for nodes in num_nodes:
			generate_locations(nodes, cell_size)
		
	for run in runs:
		for mac in run.mac_schemes:
			print('Performing {} simulation'.format(mac))
			print('\tUsing SF: {}'.format(run.sf))
			print('\t\tSimulating: {} nodes in parallel'.format(run.num_nodes))
			if mac == 'SLOTTED_ALOHA':
				j=0
				for guard_time in run.guard_time:
					print('\t\t\tUsing guard time: {} ms'.format(guard_time))
					print('\t\t\t\tUsing jitter distr: {} std: {:.3f} ms'.format(std_jitter[0],(std_jitter[1]*1000)))
					iterable = product(run.num_nodes, run.sf, run.p_sizes, run.tx_intervall)
					func = partial(run_parallel, num_simulations=num_simulations, cell_size=run.cell_size, mac=mac, path_loss_variances=path_loss_variances, confirmed_messages=confirmed_messages, adr=adr, std_jitter=std_jitter, guard_time=guard_time)
					pool.starmap(func, iterable)
					j+=1
			else:
				iterable = product(run.num_nodes, run.sf, run.p_sizes, run.tx_intervall)
				func = partial(run_parallel, num_simulations=num_simulations,cell_size=cell_size, mac=mac, path_loss_variances=path_loss_variances, confirmed_messages=confirmed_messages, adr=adr, std_jitter=[0,0], guard_time=0)
				pool.starmap(func, iterable)
	pool.close()

