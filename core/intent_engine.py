import time
import requests
import json
import os
from core.statebus import StateBus
from core.mode_arbiter import ModeArbiter

# BELANGRIJKE IMPORTS VOOR DE FIX
import sys
from core.config_manager import ConfigManager
from core.log_writer import LogWriter
# We hebben de oude parse_and_execute_advice niet meer nodig omdat LangGraph het overneemt.

# --- Mock Functies voor Externe Services ---
# De Rasa NLU server draait (in de toekomst) lokaal
RASA_URL = "http://localhost:5005/model/parse"

class IntentEngine:
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        # NIEUW: Initialiseer de ConfigManager en LogWriter
        self.config_manager = ConfigManager()
        self.log_writer = LogWriter() 
        self.arbiter = ModeArbiter(self.statebus)
        self.loop_interval = 0.05 # 20 Hz loop
        print("IntentEngine (Fase 2) geïnitialiseerd. Start 20Hz loop...")

    def get_rasa_intent(self, command: str) -> str:
        """Haalt de intentie op van de mock Rasa server."""
        try:
            response = requests.post(
                'http://127.0.0.1:5005/model/parse', 
                json={'text': command},
                timeout=0.5
            )
            data = response.json()
            return data.get('intent', {}).get('name', 'unknown_intent')
        except requests.exceptions.RequestException as e:
            print(f"[IntentEngine] ERROR: Rasa server niet bereikbaar: {e}")
            return "unknown_intent"

    def run_loop(self):
        """De hoofdloop van de IntentEngine (20 Hz)."""
        try:
            while True:
                start_time = time.time()
                
                # Herlaad de staat vanaf schijf (voor de laatste sensorwaarden)
                self.statebus.reload_state()
                
                # *** FIX: Haal alle config-waarden op via de ConfigManager ***
                # Dit is de juiste manier om configuratiewaarden te gebruiken.
                current_config = self.config_manager.config
                
                # --- Stap 1: Haal de huidige status op ---
                voice_command = self.statebus.get_value("pending_voice_command")
                is_person_nearby = self.statebus.get_value("vision_person_nearby")
                
                # --- Stap 2: Bepaal de huidige Modus (Mode Arbiter) ---
                mode = self.arbiter.get_mode() 
                self.statebus.set_value("robot_mode", mode)
                
                # --- Stap 3: Log de huidige status (voor reflectie) ---
                print(f"[IntentEngine] Mode: {mode}. Person: {is_person_nearby}. Command: {voice_command}. Speed: {current_config.get('OBSTACLE_RESPONSE_SPEED', 'N/A')}")
                
                # --- Stap 4: Handel af op basis van modus ---
                action = None
                
                if mode == "listening" and voice_command:
                    self.statebus.set_value("pending_voice_command", None) 
                    intent = self.get_rasa_intent(voice_command)
                    action = f"execute_{intent}"
                    self.statebus.set_value("robot_action", action)

                elif mode == "social":
                    action = "social_greet"
                    self.statebus.set_value("robot_action", action)
                
                elif mode == "idle":
                    action = "idle_wander"
                    self.statebus.set_value("robot_action", action)
                
                # *** LOG HET EVENEMENT NA HET BEPALEN VAN DE ACTIE (Fase A.3) ***
                if action:
                    self.log_writer.log_event(
                        event_type="ACTION_DETERMINED",
                        details={"action": action, "mode": mode},
                        context={"config_speed": current_config.get('OBSTACLE_RESPONSE_SPEED')}
                    )

                # --- Stap 5: Wacht tot de volgende lus ---
                elapsed_time = time.time() - start_time
                sleep_time = self.loop_interval - elapsed_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nIntentEngine gestopt door gebruiker.")
        except Exception as e:
            print(f"\nIntentEngine FATALE FOUT: {e}")


if __name__ == "__main__":
    bus = StateBus()
    # Initialiseer de status
    bus.set_value("pending_voice_command", "Stop met bewegen")
    bus.set_value("vision_person_nearby", True)
    
    # Start de engine
    engine = IntentEngine(bus)
    engine.run_loop()
