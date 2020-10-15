from GlobalConfig import *
import PropagationModel
from Location import Location
from Gateway import Gateway
from LoRaPacket import UplinkMessage
import matplotlib.pyplot as plt
import pandas as pd

from SNRModel import SNRModel
import gc
import math

class AirInterface:
    def __init__(self, gateway: Gateway, prop_model: PropagationModel, snr_model: SNRModel, env):
            
        self.prop_measurements = {}
        self.num_of_packets_collided = 0
        self.num_of_packets_send = 0
        self.total_collision_checks = 0
        self.gateway = gateway
        self.packages_in_air = list()
        self.color_per_node = dict()
        self.prop_model = prop_model
        self.snr_model = snr_model
        self.env = env

    @staticmethod
    def frequency_collision(p1: UplinkMessage, p2: UplinkMessage):
        """frequencyCollision, conditions
                |f1-f2| <= 120 kHz if f1 or f2 has bw 500
                |f1-f2| <= 60 kHz if f1 or f2 has bw 250
                |f1-f2| <= 30 kHz if f1 or f2 has bw 125
        """

        p1_freq = p1.lora_param.freq
        p2_freq = p2.lora_param.freq

        p1_bw = p1.lora_param.bw
        p2_bw = p2.lora_param.bw

        if abs(p1_freq - p2_freq) <= 120 and (p1_bw == 500 or p2_bw == 500):
            if   PRINT_ENABLED:
                print("frequency coll 500")
            return True
        elif abs(p1_freq - p2_freq) <= 60 and (p1_bw == 250 or p2_bw == 250):
            if   PRINT_ENABLED:
                print("frequency coll 250")
            return True
        elif abs(p1_freq - p2_freq) <= 30 and (p1_bw == 125 or p2_bw == 125):
            if   PRINT_ENABLED:
                print("frequency coll 125")
            return True

        if   PRINT_ENABLED:
            print("no frequency coll")
        return False

    @staticmethod
    def sf_collision(p1: UplinkMessage, p2: UplinkMessage):
        #
        # sfCollision, conditions
        #
        #       sf1 == sf2
        #
        if p1.lora_param.sf == p2.lora_param.sf:
            if   PRINT_ENABLED:
                print("collision sf node {} and node {}".format(p1.node.id, p2.node.id))
            return True
        if   PRINT_ENABLED:
            print("no sf collision")
        return False

    @staticmethod
    def timing_collision(me: UplinkMessage, other: UplinkMessage):
        # packet p1 collides with packet p2 when it overlaps in its critical section

        sym_duration = 2 ** me.lora_param.sf / (1.0 * me.lora_param.bw)
        num_preamble = 8
        critical_section_start = me.start_on_air + sym_duration * (num_preamble - 5)
        critical_section_end = me.start_on_air + me.my_time_on_air()

        if   PRINT_ENABLED:
            print('P1 has a critical section in [{} - {}]'.format(critical_section_start, critical_section_end))

        other_end = other.start_on_air + other.my_time_on_air()

        if other_end < critical_section_start or other.start_on_air > critical_section_end:
            # all good
            me_time_collided = False
        else:
            # timing collision
            me_time_collided = True
            #print('Node {} and {} time collided'.format(me.node.id, other.node.id))
            #print('Node {} start on air: {}'.format(me.node.id, me.start_on_air))
            #print('Node {} start on air: {}'.format(other.node.id, other.start_on_air))

        sym_duration = 2 ** other.lora_param.sf / (1.0 * other.lora_param.bw)
        num_preamble = 8
        critical_section_start = other.start_on_air + sym_duration * (num_preamble - 5)
        critical_section_end = other.start_on_air + other.my_time_on_air()

        if   PRINT_ENABLED:
            print('P2 has a critical section in [{} - {}]'.format(critical_section_start, critical_section_end))

        me_end = me.start_on_air + me.my_time_on_air()

        if me_end < critical_section_start or me.start_on_air > critical_section_end:
            # all good
            other_time_collided = False
        else:
            # timing collision
            other_time_collided = True
            #print('Node {} and {} time collided'.format(me.node.id, other.node.id))
            #print('Node {} start on air: {}'.format(me.node.id, me.start_on_air))
            #print('Node {} start on air: {}'.format(other.node.id, other.start_on_air))

        # return who has been time collided (can be with each other)
        if me_time_collided and other_time_collided:
            return (me, other)
        elif me_time_collided:
            return (me,)
        elif other_time_collided:
            return (other_time_collided,)
        else:
            return None

    @staticmethod
    def power_collision(me: UplinkMessage, other: UplinkMessage, time_collided_nodes):
        power_threshold = 1  # dB
        if   PRINT_ENABLED:
            print(
                "pwr: node {0.node.id} {0.rss:3.2f} dBm node {1.node.id} {1.rss:3.2f} dBm; diff {2:3.2f} dBm".format(me,
                                                                                                                     other,
                                                                                                                     round(
                                                                                                                         me.rss - other.rss,
                                                                                                                         2)))
        if abs(me.rss - other.rss) < power_threshold:
            if   PRINT_ENABLED:
                print("collision pwr both node {} and node {} (too close to each other)".format(me.node.id,
                                                                                                other.node.id))
            if me in time_collided_nodes:
                me.collided = True
                #print('Node {} and {} power collided'.format(me.node.id, other.node.id))
                #print('Node {} start on air: {}'.format(me.node.id, me.start_on_air))
                #print('Node {} start on air: {}'.format(other.node.id, other.start_on_air))
            if other in time_collided_nodes:
                other.collided = True
                #print('Node {} and {} power collided'.format(me.node.id, other.node.id))
                #print('Node {} start on air: {}'.format(me.node.id, me.start_on_air))
                #print('Node {} start on air: {}'.format(other.node.id, other.start_on_air))

        elif me.rss - other.rss < power_threshold:
            # me has been overpowered by other
            # me will collided if also time_collided

            if me in time_collided_nodes:
                if   PRINT_ENABLED:
                    print("collision pwr both node {} has collided by node {}".format(me.node.id, other.node.id))
                me.collided = True
        else:
            # other was overpowered by me
            if other in time_collided_nodes:
                if   PRINT_ENABLED:
                    print("collision pwr both node {} has collided by node {}".format(other.node.id, me.node.id))
                other.collided = True

    def collision(self, packet) -> bool:
        if  PRINT_ENABLED:
            print("CHECK node {} (sf:{} bw:{} freq:{:.6e}) #others: {}".format(
                packet.node.id, packet.lora_param.sf, packet.lora_param.bw, packet.lora_param.freq,
                len(self.packages_in_air)))
        if packet.collided:
            self.total_collision_checks += 1
            packet.collided = True
            #return True
        pkts_in_air = len(self.packages_in_air) - 1
        #print('{}: Marking up collisions'.format(packet.node.id))
        #print('{}: Currently {} packages in air'.format(packet.node.id, pkts_in_air))
        num_simple_checks = 0
        for other in self.packages_in_air:
            if other.node.id != packet.node.id:
                '''
                print("{}: Collision with: node {} (sf:{} bw:{} freq:{:.6e})".format(
                    packet.node.id, other.node.id, other.lora_param.sf, other.lora_param.bw,
                    other.lora_param.freq))
                '''
                if PRINT_ENABLED:
                    print(">> node {} (sf:{} bw:{} freq:{:.6e})".format(
                        other.node.id, other.lora_param.sf, other.lora_param.bw,
                        other.lora_param.freq))
                if AirInterface.frequency_collision(packet, other):
                    #print('{}: Frequency collision'.format(packet.node.id))
                    if AirInterface.sf_collision(packet, other):
                        msg_time_overlap = abs(packet.start_on_air - other.start_on_air)
                        has_tx_on_rx = math.isclose(msg_time_overlap,
                                packet.node.settings.msg_air_time, abs_tol=1e-5)
                        if not has_tx_on_rx:
                            if SIMPLE_COLLISIONS:
                                packet.collided = True
                                other.collided = True
                            else:
                                #print('{}: Checking time collision'.format(packet.node.id))
                                time_collided_nodes = AirInterface.timing_collision(packet, other)
                                if time_collided_nodes is not None:
                                    AirInterface.power_collision(packet, other, time_collided_nodes)
        #print('{}: Marked {}/{} packets in air as collided'.format(packet.node.id, num_simple_checks, pkts_in_air))
        return packet.collided

    color_values = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f']

    def packet_in_air(self, packet: UplinkMessage):
        self.num_of_packets_send += 1
        # id = packet.node.id
        # if id not in self.color_per_node:
        #     self.color_per_node[id] = '#' + random.choice(AirInterface.color_values) + random.choice(
        #         AirInterface.color_values) + random.choice(AirInterface.color_values) + random.choice(
        #         AirInterface.color_values) + random.choice(AirInterface.color_values) + random.choice(
        #         AirInterface.color_values)

        from_node = packet.node
        node_id = from_node.id
        rss = self.prop_model.tp_to_rss(from_node.location.indoor, packet.lora_param.tp,
                                        Location.distance(self.gateway.location, packet.node.location))
        if node_id not in self.prop_measurements:
            self.prop_measurements[node_id] = {'rss': [], 'snr': [], 'time': []}
        packet.rss = rss
        snr = self.snr_model.rss_to_snr(rss)
        packet.snr = snr

        #print('Gateway location: ({}, {})'.format(self.gateway.location.x, self.gateway.location.y))
        #print('Node {} location: ({}, {} ->{})'.format(packet.node.id, packet.node.location.x, packet.node.location.y, Location.distance(self.gateway.location, packet.node.location)))
        #print('RSS: {} SNR: {}'.format(rss, snr))
        self.prop_measurements[node_id]['time'].append(self.env.now)
        self.prop_measurements[node_id]['rss'].append(rss)
        self.prop_measurements[node_id]['snr'].append(snr)

        self.packages_in_air.append(packet)
        gc.disable()

    def mark_collisions(self, packet: UplinkMessage):
        for other in self.packages_in_air:
            if not other.collided and other.node.id != packet.node.id:
                packet.collided = True
                other.collided = True
        gc.disable()

    def packet_received(self, packet: UplinkMessage) -> bool:
        """Packet has fully received by the gateway
            This method checks if this packet has collided
            and remove from in the air
            :return bool (True collided or False not collided)"""
        #print('{}: Packet received at GW at time {}'.format(packet.node.id, self.env.now))
        #print('{}: Packet was sent on freq: {}, sf: {}'.format(packet.node.id, packet.lora_param.freq,
        #    packet.lora_param.sf))
        #print('{}: Checking for collisions at time: {}'.format(packet.node.id, self.env.now))
        collided = self.collision(packet)
        if collided:
            self.num_of_packets_collided += 1
            #print('{}: Our packet has collided'.format(packet.node.id))
        self.packages_in_air.remove(packet)
        #print('{} Removing packet from air at time {}'.format(packet.node.id, self.env.now))
        for other in self.packages_in_air:
            if not other.collided and other.node.id != packet.node.id:
                '''
                print('{}: Found other packet (node id: {}, freq: {}, sf: {}, bw: {}) not collided'.
                        format(packet.node.id, other.node.id, 
                            other.node.lora_param.freq, other.node.lora_param.sf, other.node.lora_param.bw))
                '''
        gc.disable()
        return collided

    def plot_packets_in_air(self):
        plt.figure()
        ax = plt.gca()
        plt.axis('off')
        ax.grid(False)
        for package in self.packages_in_air:
            node_id = package.node.id
            plt.hlines(package.lora_param.freq, package.start_on_air, package.start_on_air + package.my_time_on_air(),
                       color=self.color_per_node[node_id],
                       linewidth=2.0)
        plt.show()

    def log(self):
        print('Total number of packets in the air {}'.format(self.num_of_packets_send))
        print('Total number of packets collided {} {:2.2f}%'.format(self.num_of_packets_collided,
                                                                    self.num_of_packets_collided * 100 / self.num_of_packets_send))

    def get_prop_measurements(self, node_id):
        return self.prop_measurements[node_id]

    def get_simulation_data(self, name) -> pd.Series:
        series = pd.Series([self.num_of_packets_collided, self.num_of_packets_send, self.total_collision_checks],
                           index=['NumberOfPacketsCollided', 'NumberOfPacketsOnAir', 'TotalCollisionChecks'])
        series.name = name
        return series.transpose()
