import numpy as np


class SNRModel:
    def __init__(self, GRID_SIZE=2):
        self.noise = -80  # mean_mean_values
        self.std_noise = 6  # mean_std_values
        
        # noise floor according to https://www.semtech.com/uploads/documents/an1200.22.pdf

        self.noise_figure = 6
        self.noise_floor = -174 + self.noise_figure + 10 * np.log10(125e3)
    
    def rss_to_snr(self, rss: float):
        # TODO make a better noise assumptionS

        return rss - self.noise_floor


def roundup(x, GRID_SIZE):
    x = np.divide(x, GRID_SIZE)
    return np.ceil(x).astype(int) * GRID_SIZE
