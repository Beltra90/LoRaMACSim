import simpy
import pdb
import math
import PropagationModel
from AirInterface import AirInterface
from EnergyProfile import EnergyProfile
from Gateway import Gateway
from LoRaParameters import LoRaParameters
from Node import *
from SNRModel import SNRModel
from GlobalConfig import *
from LoRaPacket import *

tx_power_mW = {2: 91.8, 5: 95.9, 8: 101.6, 11: 120.8, 14: 146.5}
rx_measurements = {'pre_mW': 8.2, 'pre_ms': 3.4, 'rx_lna_on_mW': 39,
                   'rx_lna_off_mW': 34,
                   'post_mW': 8.3, 'post_ms': 10.7}

class SimulationSettings:
    
    def __init__(self, num_tx_in_sim, msg_air_time, mac,tx_intervall, dc, std_jitter, guard_time, cad_time, cad_proc_time):
        self.msg_air_time = msg_air_time
        self.mac = mac
        self.cad_time = cad_time
        self.cad_proc_time = cad_proc_time
        self.std_jitter = std_jitter[1]
        self.std_jitter_distr = std_jitter[0]
        self.guard_time = guard_time 
        self.dc = dc
        self.slot_size = msg_air_time + self.guard_time
        self.sim_time = 60000 * tx_intervall * num_tx_in_sim #duration of the simulation in milliseconds
        self.tx_intervall = tx_intervall

def run_helper(args):
    return run(*args)


def run(locs, mac, p_size, tx_intervall, sigma, gateway_location, num_nodes, sf, confirmed_messages, adr, std_jitter, guard_time=0):
    sim_env = simpy.Environment()
    gateway = Gateway(sim_env, gateway_location, max_snr_adr=True, avg_snr_adr=False)
    nodes = []
    air_interface = AirInterface(gateway, PropagationModel.ETSI(), SNRModel(), sim_env)

    for node_id in range(num_nodes):
        energy_profile = EnergyProfile(5.7e-3, 15, tx_power_mW,
                                       rx_power=rx_measurements)
        _sf = np.random.choice(LoRaParameters.SPREADING_FACTORS)
        if start_with_fixed_sf:
            _sf = sf

        lora_param = LoRaParameters(freq=LoRaParameters.DEFAULT_CHANNELS[0],
                                    sf=_sf,
                                    bw=125, cr=4, crc_enabled=1, de_enabled=0, header_implicit_mode=0, tp=14)
									
        msg_air_time = time_on_air(p_size, lora_param)#Calculates the message air time
        dc = msg_air_time/(60000*tx_intervall)#Calculates the duty-cycle of the devices (check that the duty-cycle is smaller than the maximum allowed by the sub-band)
        sleep_time = msg_air_time * (100 - dc*100)#Rough approximation of the time a node is not transmitting during a transmission period
        
        t_sym = (2.0 ** lora_param.sf) / lora_param.bw #Symbol time
		
        num_tx_in_sim = 10000/num_nodes*20 #Scale the number of transmission per node (and subsequently the simulation time) according to the number of nodes (10000 Nodes-> 20 TXs per nodes; 5000-> 40 TXs per nodes)

        settings = SimulationSettings(num_tx_in_sim, msg_air_time, mac, tx_intervall, dc,
                std_jitter, guard_time, cad_time=2*t_sym, cad_proc_time=0.5*t_sym)
				
		# CAD depends on SF but we here simplified to 2 * t_sym according to
        # https://www.semtech.com/uploads/documents/an1200.21_std.pdf
		
        node = Node.factory(mac, node_id, energy_profile, lora_param, sleep_time,
                    process_time=5,
                    adr=adr,
                    location=locs[node_id],
                    base_station=gateway, env=sim_env, payload_size=p_size, air_interface=air_interface,
                    settings=settings, confirmed_messages=confirmed_messages)
        nodes.append(node) 
        sim_env.process(node.run())


    sim_env.run(until=settings.sim_time)
    # Simulation is done.
    # Process the results

    mean_energy_per_bit_list = list()
    for n in nodes:
        mean_energy_per_bit_list.append(n.energy_per_bit())

    data_mean_nodes = Node.get_mean_simulation_data_frame(nodes, name=sigma) / (
        num_nodes)

    data_gateway = gateway.get_simulation_data(name=sigma)

    data_air_interface = air_interface.get_simulation_data(name=sigma) / (
        num_nodes)


    return {
        'mean_nodes': data_mean_nodes,
        'gateway': data_gateway,
        'air_interface': data_air_interface,
        'path_loss_std': sigma,
        'payload_size': p_size,
        'mean_energy_all_nodes': mean_energy_per_bit_list
    }
