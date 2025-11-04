import time
import os
import random
from core.statebus import StateBus
from typing import Dict, Any

# Hoge-frequentie loop voor sensoren (bijvoorbeeld 100 Hz, of 200 Hz voor IMU)
SENSOR_LOOP_HZ = 100 
LOOP_INTERVAL = 1.0 / SENSOR_LOOP_HZ

class BridgeReceiver:
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self._running = True
        print(f"BridgeReceiver (Fase B.2 - Unitree ROS) geïnitialiseerd op {SENSOR_LOOP_HZ} Hz.")

    def run_sensor_loop(self):
        """
        De hoge-frequentie lus die ruwe hardware-data (IMU, LiDAR, Grove) leest
        en deze naar de statebus schrijft in ROS-compatibel formaat (Unitree Topics).
        """
        print("[BridgeReceiver] Sensor Loop gestart. Druk op Ctrl+C om te stoppen.")
        
        while self._running:
            start_time = time.time()
            
            # --- STAP 1: Simuleer Ruwe Sensor Data ---
            
            # Unitree/ROS IMU Topic Conveties: /imu/data
            imu_accel_x = 0.0 + random.uniform(-0.01, 0.01)
            imu_gyro_z = 0.0 + random.uniform(-0.005, 0.005)

            # Unitree/ROS Voetkracht Topic: /foot_force
            foot_force_front_left = 100.0 + random.uniform(-5, 5)

            # Ruwe Grove Sensor Data (Touch/Nabijheid)
            touch_sensor_back = random.choice([True, False])

            # --- STAP 2: Schrijf naar StateBus in Unitree ROS-Format ---
            
            # A. IMU Data
            self.statebus.set_value(
                "imu_data", 
                {"accel": [imu_accel_x, 0.0, 9.8], "gyro": [0.0, 0.0, imu_gyro_z]}
            )
            
            # B. Voetkracht Data (nodig voor balans en Bandit-learning)
            self.statebus.set_value(
                "foot_force",
                {"fl": foot_force_front_left, "fr": 100.0, "rl": 100.0, "rr": 100.0}
            )

            # C. Grove Sensors
            self.statebus.set_value(
                "touch_data",
                {"back": touch_sensor_back}
            )

            # Zorgt voor 100 Hz synchronisatie
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # print(f"[BridgeReceiver] Data verstuurd. Lag: {elapsed*1000:.2f} ms")


    def stop(self):
        self._running = False
        print("[BridgeReceiver] Gestopt.")

if __name__ == "__main__":
    # Test om te zien of de loop werkt
    from core.statebus import StateBus # Nodig voor de test
    
    bus = StateBus()
    receiver = BridgeReceiver(bus)
    
    t = threading.Thread(target=receiver.run_sensor_loop)
    t.start()
    
    time.sleep(1) 
    
    # Check een gelogde waarde
    print(f"Laatste IMU-waarde: {bus.get_value('imu_data')}")
    
    receiver.stop()
    t.join()