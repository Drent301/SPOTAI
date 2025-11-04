import time
import os
from core.statebus import StateBus

# Simuleert de ruwe hardware input
class BridgeReceiver:
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        print("BridgeReceiver (Fase B.2 - Unitree ROS) geïnitialiseerd.")
        
    def run_hardware_loop(self):
        """
        De hoge-frequentie lus die ruwe hardware-data (IMU, LiDAR, Grove) leest
        en deze naar de statebus schrijft in ROS-compatibel formaat.
        """
        # Hier zou de logica komen om de ReSpeaker, Grove sensoren en IMU te lezen.
        
        # Simuleer IMU data (ROS topic /imu/data)
        self.statebus.set_value("imu_data", {"accel": [0.1, 0.0, 9.8], "gyro": [0.0, 0.0, 0.0]})
        
        # Dit is de lus die later de 'bridge_receiver.py' aanstuurt
        print("[BridgeReceiver] Hardware monitoring gestart. Wacht op data...")

if __name__ == "__main__":
    bus = StateBus()
    receiver = BridgeReceiver(bus)
    receiver.run_hardware_loop()