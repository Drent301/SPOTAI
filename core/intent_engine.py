import time
import requests
import json
import os
import random
import threading 
from core.statebus import StateBus
from core.config_manager import ConfigManager
from core.behavior_tree_manager import BehaviorTreeManager # <-- GEWIJZIGD
from core.log_writer import LogWriter

# De Rasa NLU server draait (in de toekomst) lokaal
RASA_URL = "http://localhost:5005/model/parse"
INTENT_LOOP_HZ = 10 # 10 Hz loop

class IntentEngine:
    """
    Verwerkt ruwe data (spraak, perceptie, sensoren) tot een geconsolideerde
    intentie en gebruikt een Behavior Tree om de finale actie te bepalen.
    """
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager()
        self.log_writer = LogWriter()
        
        # Initialiseer de Behavior Tree Manager
        self.bt_manager = BehaviorTreeManager(self.statebus)
        self._running = True
        
        # Configuratie voor beslissingslogica
        self.gezichtsgevoeligheid = self.config_manager.get_setting("GEZICHTSDETECTIEGEVOELIGHEID", 0.7)
        self.autonomy_level = self.config_manager.get_setting("AUTONOMY_LEVEL", 0.7)

        print("IntentEngine geïnitialiseerd met Behavior Tree.")

    def get_rasa_intent(self, command: str) -> dict:
        """Haalt de intentie op van de mock Rasa server."""
        try:
            response = requests.post(
                'http://127.0.0.1:5005/model/parse', 
                json={'text': command},
                timeout=0.5
            )
            data = response.json()
            return {"intent": data.get('intent', {}).get('name', 'unknown_intent'), "confidence": data.get('intent', {}).get('confidence', 0.5)}
        except requests.exceptions.RequestException:
            return {"intent": "offline_fallback", "confidence": 1.0}


    def _determine_priority_intent(self, speech: dict, detections: list, touch: dict) -> dict:
        """
        Consolideert alle inputs tot één actuele, geordende intentie.
        """
        # A. Fysieke veiligheid (Hoogste Prioriteit)
        if touch and touch.get("back", False):
            return {"type": "emergency", "action": "stop_motors", "reason": "Fysieke aanraking", "confidence": 1.0}
            
        # B. Spraakintentie (Hoog)
        if speech and speech.get("confidence", 0) >= 0.8:
            if speech['intent'] == 'move_command':
                 return {"type": "motor_action", "action": "walk_forward", "source": "speech", "confidence": speech['confidence']}
            if speech['intent'] == 'query_name':
                 return {"type": "speech_action", "action": "answer_name", "source": "speech", "confidence": speech['confidence']}
            if speech['intent'] == 'offline_fallback':
                 return {"type": "speech_action", "action": "answer_offline_fallback", "source": "speech", "confidence": 1.0}
        
        # C. Visuele intentie (Midden)
        if detections:
            for det in detections:
                if det.get("type") == "gezicht" and det.get("emotie") == "blij" and det.get("confidence", 0) > self.gezichtsgevoeligheid:
                    return {"type": "social_action", "action": "approach_and_greet", "source": "perception", "confidence": det['confidence']}
        
        # D. Standaard / Leeg (Laag)
        return {"type": "idle", "action": "monitor", "source": "none", "confidence": 1.0}


    def run_intent_loop(self):
        """De hoofdloop van de IntentEngine (10 Hz)."""
        print(f"[IntentEngine] Loop gestart op {INTENT_LOOP_HZ} Hz.")
        
        LOOP_INTERVAL = 1.0 / INTENT_LOOP_HZ
        
        while self._running:
            start_time = time.time()
            
            # --- 1. Inputs Ophalen ---
            # De Behavior Tree's blackboard haalt zelf de volledige state op,
            # maar we halen hier specifieke waarden op voor de intentieconsolidatie.
            speech_result = self.statebus.get_value("latest_intent")
            detections = self.statebus.get_value("detections")
            touch_data = self.statebus.get_value("touch_data")
            
            # --- 2. CONSOLIDATIE (IntentEngine Logic) ---
            consolidated_intent = self._determine_priority_intent(
                speech_result, detections, touch_data
            )
            
            # Schrijf de geconsolideerde intentie naar de StateBus zodat de BT het kan lezen
            self.statebus.set_value("current_consolidated_intent", consolidated_intent)
            
            # --- 3. MODUS BEPALING (Behavior Tree Logic) ---
            # De Behavior Tree wordt "getikt" en bepaalt de finale actie.
            self.bt_manager.tick()
            
            # --- 4. Logging ---
            final_action = self.statebus.get_value("robot_action")
            self.log_writer.log_event(
                event_type="INTENT_CONSOLIDATED",
                details={"final_action": final_action, "intent_type": consolidated_intent["type"]},
                context={"current_bt_status": self.bt_manager.tree.root.status.value}
            )
            
            # Synchronisatie
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    def stop(self):
        self._running = False
        print("[IntentEngine] Gestopt.")

def main():
    bus = StateBus()
    engine = IntentEngine(bus)
    
    try:
        engine.run_intent_loop()
    except KeyboardInterrupt:
        engine.stop()
        print("IntentEngine gestopt door gebruiker.")

if __name__ == "__main__":
    main()
