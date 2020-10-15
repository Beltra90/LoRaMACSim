import os
import pickle

from GlobalConfig import *
from Location import Location

# USE THE GLOBAL CONFIG!
#num_locations = 500
#cell_size = 1000
#num_of_simulations = 1000

locations_per_simulation = list()
print('Generating {} locations for {} simulations'.format(num_locations, num_of_simulations))
for num_sim in range(num_of_simulations):
    locations = list()
    for i in range(num_locations):
        locations.append(Location(max=cell_size, indoor=False))
    locations_per_simulation.append(locations)


os.makedirs(os.path.dirname(locations_file), exist_ok=True)
with open(locations_file, 'wb') as f:
    pickle.dump(locations_per_simulation, f)

# just to test the code
# with open('locations_10000_locations_1000_sim.pkl', 'rb') as filehandler:
#     print(pickle.load(filehandler))
