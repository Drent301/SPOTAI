import time
import os
import sys
import threading 

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.statebus import StateBus
from core.config_manager import ConfigManager 
# Importeer onze nieuwe, aparte driver
from perception.hailo_driver import HailoDriver

# De perceptieloop is langzamer (bijv. 10 Hz) vanwege video- en AI-verwerking
PERCEPTION_LOOP_HZ = 10 
LOOP_INTERVAL = 1.0 / PERCEPTION_LOOP_HZ

class PerceptionProcessor:
    """
    Fase B.4: Centrale module voor visuele perceptie.
    Deze module haalt data op van de HailoDriver en stuurt deze naar de StateBus.
    """
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager()
        self._running = True
        
        # Initialiseer de ECHTE (gesimuleerde) hardware driver
        try:
            self.driver = HailoDriver()
        except Exception as e:
            print(f"[PerceptionProcessor] FATALE FOUT: Kon Hailo-driver niet laden: {e}")
            self.driver = None
            return

        self.gezichtsgevoeligheid = self.config_manager.get_setting("GEZICHTSDETECTIEGEVOELIGHEID")
        self.camera_res = self.config_manager.get_setting("CAMERA_RESOLUTIE")
        
        print(f"PerceptionProcessor (Fase B.4 - Hailo-8) geïnitialiseerd.")
        print(f"Config: Gevoeligheid={self.gezichtsgevoeligheid}, Resolutie={self.camera_res}")

    def run_perception_loop(self):
        """
        De hoofdloop voor het verwerken van visuele data.
        """
        if not self.driver:
            print("[PerceptionProcessor] Kan loop niet starten, driver is niet geladen.")
            return

        print(f"[PerceptionProcessor] Loop gestart op {PERCEPTION_LOOP_HZ} Hz.")
        
        while self._running:
            start_time = time.time()
            
            # 1. Haal de AI-inferentie op van de hardware driver
            detections = self.driver.get_inference()
            
            # 2. Schrijf resultaten naar StateBus (voor Intent Engine)
            self.statebus.set_value("detections", detections)
            
            if detections:
                # Optioneel: schrijf naar /tmp/dets.json (zoals in manifest)
                # self._write_to_tmp(detections)
                pass

            # Synchronisatie
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        self._running = False
        if self.driver:
            self.driver.stop()
        print("[PerceptionProcessor] Gestopt.")

if __name__ == "__main__":
    bus = StateBus()
    processor = PerceptionProcessor(bus)
    
    if processor.driver:
        t = threading.Thread(target=processor.run_perception_loop)
        t.start()
        
        # Laat de processor 3 seconden draaien
        time.sleep(3) 
        
        processor.stop()
        t.join()
        
        # Controleer of de data is aangekomen
        print(f"Laatste Detections in StateBus: {bus.get_value('detections')}")
    else:
        print("Test mislukt, processor kon niet initialiseren.")