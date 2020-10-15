import numpy as np
import math
import random
from enum import Enum
from LoRaParameters import *

class SlottedAloha:
    PRINT_ENABLED = False

class CSMA:
    PRINT_ENABLED = False
	
	
############### SIMULATION SPECIFIC PARAMETERS ###############
start_with_fixed_sf = True
start_sf = 10


adr = False
confirmed_messages = False

acks_enabled = False
SIMPLE_COLLISIONS = False
DISABLE_DC = True
path_loss_variances = [7.9]  # [0, 5, 7.8, 15, 20]

MAC_IMPROVEMENT = False

############### DEFAULT PARAMETERS ###############
LOG_ENABLED = True
PRINT_ENABLED = False
track_changes = True
load_prev_simulation_results = True

