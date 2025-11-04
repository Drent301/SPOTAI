import time
import os
import sys
import threading
from core.statebus import StateBus
from core.config_manager import ConfigManager
# Importeer onze nieuwe, aparte I2C driver
from core.i2c_driver import I2CDriver

# Hoge-frequentie loop voor sensoren (bijvoorbeeld 100 Hz)
SENSOR_LOOP_HZ = 100 
LOOP_INTERVAL = 1.0 / SENSOR_LOOP_HZ

class BridgeReceiver:
    """
    Fase B.2: Leest ruwe hardware-data (IMU, Touch) van de I2CDriver
    en schrijft deze naar de StateBus.
    """
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager()
        self._running = True
        
        # Initialiseer de ECHTE (gesimuleerde) hardware driver
        try:
            self.driver = I2CDriver()
        except Exception as e:
            print(f"[BridgeReceiver] FATALE FOUT: Kon I2C-driver niet laden: {e}")
            self.driver = None
            return
            
        print(f"BridgeReceiver (Fase B.2) geïnitialiseerd op {SENSOR_LOOP_HZ} Hz.")

    def run_sensor_loop(self):
        """
        De hoge-frequentie lus die ruwe hardware-data leest.
        """
        if not self.driver:
            print("[BridgeReceiver] Kan loop niet starten, driver is niet geladen.")
            return
            
        print("[BridgeReceiver] Sensor Loop gestart.")
        
        while self._running:
            start_time = time.time()
            
            # --- STAP 1: Lees Ruwe Sensor Data van de Drivers ---
            imu_data = self.driver.get_imu_data()
            touch_data_raw = self.driver.get_touch_data()

            # --- STAP 2: Schrijf naar StateBus ---
            
            # A. IMU Data (voor balans)
            self.statebus.set_value("imu_data", imu_data)
            
            # B. Grove Sensors (voor fysieke veiligheid)
            # Voeg de config-waarde toe voor de IntentEngine
            touch_data_raw["sensor_sensitivity"] = self.config_manager.get_setting("GEZICHTSDETECTIEGEVOELIGHEID")
            self.statebus.set_value("touch_data", touch_data_raw)

            # Zorgt voor 100 Hz synchronisatie
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        self._running = False
        print("[BridgeReceiver] Gestopt.")

if __name__ == "__main__":
    bus = StateBus()
    receiver = BridgeReceiver(bus)

    if receiver.driver:
        t = threading.Thread(target=receiver.run_sensor_loop)
        t.start()
        
        # Laat de loop 2 seconden draaien
        time.sleep(2)
        
        receiver.stop()
        t.join()
        
        # Check een gelogde waarde
        print(f"Test voltooid. Laatste IMU-waarde: {bus.get_value('imu_data')}")
        print(f"Test voltooid. Laatste Touch-waarde: {bus.get_value('touch_data')}")
    else:
        print("Test mislukt, processor kon niet initialiseren.")