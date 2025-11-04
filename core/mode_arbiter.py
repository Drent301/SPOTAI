import time
import threading
from core.statebus import StateBus
from core.config_manager import ConfigManager
from typing import Dict, Any

# De frequentie van deze loop is gelijk aan die van de IntentEngine (bijv. 10 Hz)
LOOP_INTERVAL = 0.1 

class ModeArbiter:
    """
    Fase B.5b: De centrale beslissingsmodule.
    Vertaalt de geconsolideerde intentie en systeemprioriteiten naar de finale actie.
    """
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager()
        self._running = True
        self.current_mode = "idle"
        
        # Configuratie: drempel voor zelfbeslissingen
        self.autonomy_level = self.config_manager.get_setting("AUTONOMY_LEVEL")

        print(f"ModeArbiter (Fase B.5b) geïnitialiseerd. Autonomie: {self.autonomy_level}.")

    def _determine_final_action(self, is_charging: bool, network_state: str, intent: Dict[str, Any]) -> str:
        """
        De prioriteitsmatrix voor acties (hoogste prioriteit wint).
        Dit is de kern van de beslissingslogica van de robot.
        """
        
        # A. Hoogste Prioriteit: Veiligheid/Onderhoud
        if intent["type"] == "emergency":
            self.current_mode = "emergency_stop"
            return intent["action"]
            
        if is_charging:
            self.current_mode = "charging"
            return "system_check" # Onderhoudsactie tijdens opladen

        # B. Netwerk/Online Status (Offline Fallback)
        if network_state == "offline":
            self.current_mode = "offline_mode"
            # Offline spraak antwoord heeft prioriteit over idle
            if intent["type"] == "speech_action": 
                 return "answer_offline_fallback"

        # C. Intentie-Actie (Motoriek, Spraak, Sociale Actie)
        # De intentie moet een minimum confidence hebben (de autonomie drempel)
        if intent["confidence"] >= self.autonomy_level:
            self.current_mode = intent["type"]
            return intent["action"]

        # D. Laagste Prioriteit: Standaardactie
        self.current_mode = "idle"
        return "monitor_sensors"

    def get_mode(self) -> str:
        """Publieke getter voor de huidige modus (nodig voor de IntentEngine)."""
        return self.current_mode
    
    def run_arbiter_loop(self):
        """
        De hoofdloop voor de Mode Arbiter (in de definitieve architectuur wordt 
        deze code aangeroepen vanuit de IntentEngine of gpt_agent).
        """
        print(f"[ModeArbiter] Loop gestart.")
        
        while self._running:
            start_time = time.time()
            
            # Lees de geconsolideerde intentie uit de statebus
            current_state = self.statebus.get_state()
            intent = current_state.get("current_consolidated_intent", {"type": "idle", "action": "monitor", "confidence": 1.0})
            
            final_action = self._determine_final_action(
                current_state.get("is_charging", False), 
                current_state.get("network_state", "online"), 
                intent
            )
            
            # Schrijf de finale actie naar de StateBus
            self.statebus.set_value("robot_action", final_action)

            # Synchronisatie
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    def stop(self):
        self._running = False
        print("[ModeArbiter] Gestopt.")

if __name__ == "__main__":
    # Vereist dat de IntentEngine de geconsolideerde intentie heeft geschreven
    bus = StateBus()
    
    # Simuleer een hoog-prioriteit commando: Stop
    bus.set_value("current_consolidated_intent", {"type": "emergency", "action": "stop_motors", "reason": "Test", "confidence": 1.0})
    
    arbiter = ModeArbiter(bus)
    
    t = threading.Thread(target=arbiter.run_arbiter_loop)
    t.start()
    
    time.sleep(1) 
    
    # Controleer de uitvoer
    print(f"Laatste Robot Actie: {bus.get_value('robot_action')}")
    
    arbiter.stop()
    t.join()