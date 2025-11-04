import time
import os
import random
import threading # Nodig voor de test
from core.statebus import StateBus
from core.config_manager import ConfigManager # Nodig voor get_setting

# Hoge-frequentie loop voor sensoren (bijvoorbeeld 100 Hz)
SENSOR_LOOP_HZ = 100 
LOOP_INTERVAL = 1.0 / SENSOR_LOOP_HZ

class BridgeReceiver:
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager()
        self._running = True
        print(f"BridgeReceiver (Fase B.2 - Unitree ROS) geïnitialiseerd op {SENSOR_LOOP_HZ} Hz.")

    def run_sensor_loop(self):
        """
        De hoge-frequentie lus die ruwe hardware-data (IMU, LiDAR, Grove) leest
        en deze naar de statebus schrijft in ROS-compatibel formaat (Unitree Topics).
        """
        print("[BridgeReceiver] Sensor Loop gestart. Druk op Ctrl+C om te stoppen.")
        
        # De lus om sensors te lezen is continu en moet snel zijn
        while self._running:
            start_time = time.time()
            
            # --- STAP 1: Simuleer Ruwe Sensor Data ---
            
            # Unitree/ROS IMU Topic Conveties: /imu/data
            imu_accel_x = 0.0 + random.uniform(-0.01, 0.01)
            imu_gyro_z = 0.0 + random.uniform(-0.005, 0.005)

            # Ruwe Grove Sensor Data (Touch/Nabijheid)
            touch_sensor_back = random.choice([True, False])
            
            # Lees configuratie voor gevoeligheid (Fase B.4)
            gezichtsgevoeligheid = self.config_manager.get_setting("GEZICHTSDETECTIEGEVOELIGHEID")

            # --- STAP 2: Schrijf naar StateBus in Unitree ROS-Format ---
            
            # A. IMU Data (voor balans)
            self.statebus.set_value(
                "imu_data", 
                {"accel": [imu_accel_x, 0.0, 9.8], "gyro": [0.0, 0.0, imu_gyro_z]}
            )
            
            # B. Grove Sensors (voor fysieke veiligheid)
            self.statebus.set_value(
                "touch_data",
                {"back": touch_sensor_back, "sensor_sensitivity": gezichtsgevoeligheid}
            )

            # Zorgt voor 100 Hz synchronisatie
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            

    def stop(self):
        self._running = False
        print("[BridgeReceiver] Gestopt.")

if __name__ == "__main__":
    # Test om te zien of de loop werkt
    bus = StateBus()
    receiver = BridgeReceiver(bus)
    
    # Simuleer een threading.Thread om de loop te laten draaien
    class MockThread:
        def __init__(self, target):
            self._target = target
            
        def start(self):
            # Voer de loop slechts 1 seconde uit voor de test
            for _ in range(SENSOR_LOOP_HZ):
                self._target()
                
    t = MockThread(target=receiver.run_sensor_loop)
    
    t.start()
    
    # Check een gelogde waarde
    print(f"Laatste IMU-waarde: {bus.get_value('imu_data')}")
    
    receiver.stop()