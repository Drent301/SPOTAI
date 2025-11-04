import time
import os
import random
import threading 
from core.statebus import StateBus
from core.config_manager import ConfigManager 

# De Intent Engine werkt op een midden-frequentie
INTENT_LOOP_HZ = 10
LOOP_INTERVAL = 1.0 / INTENT_LOOP_HZ

class IntentEngine:
    """
    Fase B.5a: Verwerkt ruwe data (spraak, perceptie, sensoren) tot een geconsolideerde 
    intentie voor de Mode Arbiter.
    """
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager()
        self._running = True
        
        # Leest configuratie voor gedragsprioriteiten (Fase B.0)
        self.autonomy_level = self.config_manager.get_setting("AUTONOMY_LEVEL")
        self.reflection_priority = self.config_manager.get_setting("REFLECTIE_PRIORITEIT")

        print(f"IntentEngine (Fase B.5a) geïnitialiseerd. Autonomie: {self.autonomy_level}.")
        print(f"Prioriteit: {self.reflection_priority}")

    def run_intent_loop(self):
        """
        De hoofdloop die alle inputs combineert en een actuele intentie bepaalt.
        """
        print(f"[IntentEngine] Loop gestart op {INTENT_LOOP_HZ} Hz.")
        
        while self._running:
            start_time = time.time()
            
            # --- 1. Lees alle Inputs van de StateBus ---
            latest_intent_speech = self.statebus.get_value("latest_intent")
            detections = self.statebus.get_value("detections")
            touch_data = self.statebus.get_value("touch_data")
            
            # --- 2. Beslis de geconsolideerde Intentie ---
            consolidated_intent = self._determine_priority_intent(
                latest_intent_speech, detections, touch_data
            )
            
            # --- 3. Schrijf naar StateBus voor Mode Arbiter ---
            self.statebus.set_value("current_consolidated_intent", consolidated_intent)
            
            # Synchronisatie
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _determine_priority_intent(self, speech: dict, detections: list, touch: dict) -> dict:
        """
        Consolideert alle inputs tot één actuele, geordende intentie.
        Veiligheid heeft altijd prioriteit.
        """
        
        # A. Fysieke veiligheid (Hoogste Prioriteit)
        if touch.get("back", False):
            return {"type": "emergency", "action": "stop_motors", "reason": "Fysieke aanraking", "confidence": 1.0}
            
        # B. Spraakintentie (Hoog)
        if speech and speech.get("confidence", 0) > 0.8:
            # Als spraak 'move_command' is, vertaal naar motoractie
            if speech['intent'] == 'move_command':
                 return {"type": "motor_action", "action": "walk_forward", "source": "speech", "confidence": speech['confidence']}
            # Als spraak 'query_name' is, vertaal naar spraakactie
            if speech['intent'] == 'query_name':
                 return {"type": "speech_action", "action": "answer_name", "source": "speech", "confidence": speech['confidence']}
        
        # C. Visuele intentie (Midden)
        if detections:
            for det in detections:
                if det.get("type") == "gezicht" and det.get("emotie") == "blij" and det.get("confidence", 0) > self.gezichtsgevoeligheid:
                    return {"type": "social_action", "action": "approach_and_greet", "source": "perception", "confidence": det['confidence']}
        
        # D. Standaard / Leeg (Laag)
        return {"type": "idle", "action": "monitor", "source": "none", "confidence": 1.0}


    def stop(self):
        self._running = False
        print("[IntentEngine] Gestopt.")

if __name__ == "__main__":
    # Test vereist dat de StateBus gevuld is met gesimuleerde data
    bus = StateBus()
    
    # 1. Simuleer inputs (zoals door B.2, B.3, B.4 geschreven)
    bus.set_value("latest_intent", {"intent": "move_command", "confidence": 0.9, "action": "forward"})
    bus.set_value("detections", [{"type": "gezicht", "emotie": "blij", "confidence": 0.95}])
    bus.set_value("touch_data", {"back": False})
    
    engine = IntentEngine(bus)
    
    t = threading.Thread(target=engine.run_intent_loop)
    t.start()
    
    time.sleep(1) # Laat de lus draaien
    
    # Controleer de uitvoer
    print(f"Laatste Geconsolideerde Intentie: {bus.get_value('current_consolidated_intent')}")
    
    engine.stop()
    t.join()