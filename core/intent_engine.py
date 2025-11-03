import time
import requests
import json
import os

# We moeten het pad vertellen waar de 'core' map is
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


from core.statebus import StateBus
from core.mode_arbiter import ModeArbiter
from core.log_writer import LogWriter # NIEUWE IMPORT

# --- Mock Functies voor Externe Services ---
# Deze functies simuleren de communicatie met externe services (Rasa/GPT)

def get_rasa_intent(command: str) -> str:
    """
    Simuleert het aanroepen van de lokale Rasa NLU-server.
    In de echte implementatie: requests.post('http://localhost:5005/model/parse', json={'text': command})
    """
    if "ga naar" in command.lower():
        return "move_to_location"
    elif "stop" in command.lower() or "houd" in command.lower():
        return "stop_movement"
    else:
        # Dit zou de fallback zijn als Rasa de intentie niet herkent
        return "unknown_intent"


class IntentEngine:
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.arbiter = ModeArbiter(self.statebus)
        self.log_writer = LogWriter() # INITIALISATIE VAN DE LOG WRITER
        self.loop_interval = 0.05 # 20 Hz loop (zoals in het plan)
        print("IntentEngine (Fase 2) geïnitialiseerd. Start 20Hz loop...")

    def get_rasa_intent(self, command: str) -> str:
        """
        Haalt de intentie op van de mock Rasa server.
        Dit is de lokale, snelle intentie-classificatie.
        """
        # Vervang door echte aanroep in de Pi-omgeving
        try:
            response = requests.post(
                'http://127.0.0.1:5005/model/parse', 
                json={'text': command},
                timeout=0.5
            )
            data = response.json()
            # De mock-server geeft direct de intent terug
            return data.get('intent', {}).get('name', 'unknown_intent')
        except requests.exceptions.RequestException as e:
            # Fallback bij fout (bijv. server niet bereikbaar)
            print(f"[IntentEngine] ERROR: Rasa server niet bereikbaar: {e}")
            return "unknown_intent"

    def run_loop(self):
        """
        De hoofdloop van de IntentEngine (20 Hz).
        Deze bepaalt wat de robot op elk moment moet doen.
        """
        try:
            while True:
                start_time = time.time()
                
                # --- Stap 1: Haal de huidige status op ---
                # Haal alle relevante data uit de StateBus
                voice_command = self.statebus.get_value("pending_voice_command")
                is_person_nearby = self.statebus.get_value("vision_person_nearby")
                current_config = self.statebus.get_all_configs() # Configs uit de bus, inclusief LangGraph wijzigingen
                
                # --- Stap 2: Bepaal de huidige Modus (Mode Arbiter) ---
                # De Arbiter beslist de hoogste prioriteit: social, maintenance, idle
                mode = self.arbiter.determine_mode(
                    is_person_nearby=is_person_nearby,
                    battery_low=self.statebus.get_value("battery_soc") < 0.10,
                    error_state=self.statebus.get_value("system_error")
                )
                self.statebus.set_value("robot_mode", mode)
                
                # --- Stap 3: Log de huidige status (voor reflectie) ---
                # Dit wordt periodiek gebruikt door de GPT-Agent (agent/agent_runtime.py)
                print(f"[IntentEngine] Mode: {mode}. Person: {is_person_nearby}. Command: {voice_command}. Battery: {current_config.get('battery_soc', 'N/A')}")
                
                # --- Stap 4: Handel af op basis van modus ---
                action = None
                
                if mode == "listening" and voice_command:
                    # Wis de commando om te voorkomen dat het opnieuw wordt uitgevoerd
                    self.statebus.set_value("pending_voice_command", None) 
                    
                    # Vraag Rasa om de intentie te classificeren
                    intent = self.get_rasa_intent(voice_command)
                    action = f"execute_{intent}"
                    self.statebus.set_value("robot_action", action) # Bijv. 'execute_move_to_location'

                elif mode == "social":
                    action = "social_greet"
                    self.statebus.set_value("robot_action", action)
                
                elif mode == "idle":
                    action = "idle_wander"
                    self.statebus.set_value("robot_action", action)
                
                elif mode == "maintenance":
                    # Actie wordt gezet door de arbiter of gpt_agent
                    action = self.statebus.get_value("robot_action") 
                    if not action:
                        action = "system_check" # Standaard maintenance actie
                        self.statebus.set_value("robot_action", action)
                
                # *** LOG HET EVENEMENT NA HET BEPALEN VAN DE ACTIE (Fase A.3) ***
                if action:
                    self.log_writer.log_event(
                        event_type="ACTION_DETERMINED",
                        details={"action": action, "mode": mode},
                        context=current_config # Huidige config als context
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
    # Test op basis van de mock hardware/statebus
    bus = StateBus()
    # Initialiseer de status
    bus.set_value("pending_voice_command", "Stop met bewegen")
    bus.set_value("vision_person_nearby", True)
    
    # Start de engine
    engine = IntentEngine(bus)
    engine.run_loop()