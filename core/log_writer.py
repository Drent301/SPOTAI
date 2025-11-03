import json
import os
import time
from typing import Dict, Any

# Bepaal het pad naar de datamap
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, 'data')
EXPERIENCE_FILE = os.path.join(DATA_DIR, 'experience.ndjson')
LOG_FILE_PATH_PREFIX = os.path.join(DATA_DIR, 'offline_logs', 'reflect_')


class LogWriter:
    def __init__(self):
        os.makedirs(os.path.join(DATA_DIR, 'offline_logs'), exist_ok=True)
        print("LogWriter geïnitialiseerd. Klaar om te loggen naar experience.ndjson.")

    def log_event(self, event_type: str, details: Dict[str, Any], context: Dict[str, Any] = None):
        """
        Logt een gestructureerd evenement naar experience.ndjson.
        Dit volgt de Unitree IL_lerobot JSONL/JSON Lines structuur voor datasets.
        """
        
        # Haal de huidige status van de StateBus (als dit nodig is voor context)
        # We importeren de StateBus niet direct om circular imports te voorkomen.
        
        log_entry = {
            "ts": time.time(),
            "event_type": event_type,
            "details": details,
            "context": context if context is not None else {},
        }
        
        # JSON Lines formaat: één JSON object per regel
        try:
            with open(EXPERIENCE_FILE, 'a') as f:
                f.write(json.dumps(log_entry) + '\n')
            
            print(f"[LogWriter] Event gelogd: {event_type}")
        except IOError as e:
            print(f"LogWriter: Kon niet schrijven naar {EXPERIENCE_FILE}: {e}")

    def log_offline_reflect_batch(self, batch_data: str, timestamp: float):
        """
        Slaat een batch offline logs op, voor latere synchronisatie.
       
        """
        filename = f"{LOG_FILE_PATH_PREFIX}{int(timestamp)}.json"
        
        try:
            with open(filename, 'w') as f:
                f.write(batch_data)
            print(f"[LogWriter] Offline reflectie opgeslagen in {filename}")
        except IOError as e:
            print(f"LogWriter: Kon offline batch niet opslaan: {e}")

if __name__ == "__main__":
    print("LogWriter module test...")
    lw = LogWriter()
    
    # Simuleer een sensor event (bijvoorbeeld YOLOv8 detectie)
    lw.log_event(
        event_type="VISION_DETECTED",
        details={"object": "persoon", "confidence": 0.92, "model": "YOLOv8"},
        context={"mode": "social"}
    )
    
    # Simuleer een actie van de robot
    lw.log_event(
        event_type="ACTION_EXECUTED",
        details={"action": "social_greet", "status": "success"},
    )
    
    # Simuleer een offline log batch
    dummy_offline_data = json.dumps({"reflect_output": "Geen cloudverbinding, maar motoriek was stabiel."})
    lw.log_offline_reflect_batch(dummy_offline_data, time.time())