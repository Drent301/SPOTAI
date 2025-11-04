import time
import os
import random
import threading 
from core.statebus import StateBus
from core.config_manager import ConfigManager 

# De perceptieloop is langzamer (bijv. 10 Hz) vanwege video- en AI-verwerking
PERCEPTION_LOOP_HZ = 10 
LOOP_INTERVAL = 1.0 / PERCEPTION_LOOP_HZ

class PerceptionProcessor:
    """
    Fase B.4: Centrale module voor visuele perceptie (Hailo-8 / YOLOv8 / DeepFace).
    Verwerkt videofeed en schrijft gedetailleerde detections naar de statebus.
    """
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager()
        self._running = True
        
        self.gezichtsgevoeligheid = self.config_manager.get_setting("GEZICHTSDETECTIEGEVOELIGHEID")
        self.camera_res = self.config_manager.get_setting("CAMERA_RESOLUTIE")
        
        print(f"PerceptionProcessor (Fase B.4 - Hailo-8) geïnitialiseerd.")
        print(f"Config: Gevoeligheid={self.gezichtsgevoeligheid}, Resolutie={self.camera_res}")

    def _simulate_hailo_inference(self) -> list:
        """
        Simuleert de inferentie van de Hailo-8 accelerator.
        (YOLOv8 voor objecten, DeepFace voor gezichten/emoties)
        """
        detections = []
        current_time = time.time()
        
        # Gezichtsdetectie (Afhankelijk van Gevoeligheid)
        if random.random() < 0.6:
            detections.append({
                "type": "gezicht",
                "id": 1,
                "positie": [100, 100, 50, 50], # BBox: x, y, w, h
                "naam": "Gebruiker",
                "emotie": random.choice(["blij", "neutraal", "verbaasd"]),
                "confidence": random.uniform(self.gezichtsgevoeligheid, 1.0)
            })

        # Objectdetectie (YOLOv8)
        if random.random() < 0.2:
            detections.append({
                "type": "object",
                "label": "speelgoed",
                "positie": [300, 200, 80, 80],
                "confidence": 0.85
            })

        # Schrijf de ruwe detections naar de StateBus
        self.statebus.set_value("raw_detections_timestamp", current_time)
        return detections

    def run_perception_loop(self):
        """
        De hoofdloop voor het verwerken van visuele data.
        """
        print(f"[PerceptionProcessor] Loop gestart op {PERCEPTION_LOOP_HZ} Hz.")
        
        while self._running:
            start_time = time.time()
            
            # 1. Hailo-8 Inferentie
            detections = self._simulate_hailo_inference()
            
            # 2. Schrijf resultaten naar StateBus (voor Intent Engine)
            self.statebus.set_value("detections", detections)
            
            if detections:
                pass
                # print(f"[Perception] {len(detections)} detections verstuurd.")

            # Synchronisatie
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        self._running = False
        print("[PerceptionProcessor] Gestopt.")

if __name__ == "__main__":
    bus = StateBus()
    processor = PerceptionProcessor(bus)
    
    t = threading.Thread(target=processor.run_perception_loop)
    t.start()
    
    # Laat de processor 3 seconden draaien
    time.sleep(3) 
    
    processor.stop()
    t.join()
    
    # Controleer of de data is aangekomen
    print(f"Laatste Detections in StateBus: {bus.get_value('detections')}")