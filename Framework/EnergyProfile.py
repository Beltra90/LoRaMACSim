
class EnergyProfile:
    def __init__(self, sleep_power, proc_power, tx_power, rx_power):
        self.sleep_power_mW = sleep_power
        self.proc_power_mW = proc_power
        self.tx_power_mW = tx_power
        self.rx_power = rx_power
        self.cad_power_mW = 10.8 * (2 * 1.8)
        self.cad_proc_power_mW = 5.6 * (2 * 1.8)
