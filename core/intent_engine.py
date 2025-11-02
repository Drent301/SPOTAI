import time
import requests
from core.statebus import StateBus
from core.mode_arbiter import ModeArbiter
from agent.agent_runtime import parse_and_execute_advice  # BELANGRIJKE IMPORT

# De Rasa NLU server draait (in de toekomst) lokaal
RASA_URL = "http://localhost:5005/model/parse"

class IntentEngine:
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.arbiter = ModeArbiter(self.statebus)
        self.loop_interval = 0.05 # 20 Hz loop
        print("IntentEngine (Fase 2) geïnitialiseerd. Start 20Hz loop...")

    def get_rasa_intent(self, text):
        """Simuleert een aanroep naar de lokale Rasa server."""
        print(f"[IntentEngine] Rasa NLU aanroep voor: '{text}'")
        try:
            response = requests.post(RASA_URL, json={"text": text})
            if response.status_code == 200:
                intent = response.json().get("intent", {}).get("name", "unknown")
                print(f"[IntentEngine] Rasa intent ontvangen: {intent}")
                return intent
            else:
                print(f"[IntentEngine] Rasa serverfout: {response.status_code}")
                return "nlu_error"
        except requests.ConnectionError:
            print("[IntentEngine] Kan Rasa server op localhost:5005 niet bereiken.")
            return "nlu_offline"

    def run_loop(self):
        """De hoofd 'intent-loop'."""
        try:
            while True:
                start_time = time.time()
                
                # Herlaad de staat vanaf schijf
                self.statebus.reload_state()
                
                # --- Stap 1: Verwerk GPT Advies ---
                # Kijk of de 'denker' (agent_runtime) advies heeft
                gpt_advice = self.statebus.get_value("last_gpt_advice")
                if gpt_advice:
                    print(f"[IntentEngine] Nieuw GPT Advies Gevonden: '{gpt_advice}'")
                    # Gebruik de parser uit de agent_runtime om het advies uit te voeren
                    parse_and_execute_advice(gpt_advice)
                    # Wis het advies na verwerking
                    self.statebus.set_value("last_gpt_advice", None)
                
                # --- Stap 2: Bepaal Modus ---
                mode = self.arbiter.get_mode()
                
                # --- Stap 3: Haal Sensor Data op ---
                voice_command = self.statebus.get_value("pending_voice_command")
                
                # --- Stap 4: Handel af op basis van modus ---
                if mode == "listening" and voice_command:
                    self.statebus.set_value("pending_voice_command", None)
                    intent = self.get_rasa_intent(voice_command)
                    self.statebus.set_value("robot_action", f"execute_{intent}")

                elif mode == "social":
                    self.statebus.set_value("robot_action", "social_greet")
                
                elif mode == "idle":
                    self.statebus.set_value("robot_action", "idle_wander")

                # Wacht de rest van de 0.05s cyclus
                elapsed = time.time() - start_time
                if elapsed < self.loop_interval:
                    time.sleep(self.loop_interval - elapsed)
                else:
                    print(f"[IntentEngine] WAARSCHUWING: Loop duurde te lang: {elapsed:.2f}s")

        except KeyboardInterrupt:
            print("IntentEngine gestopt.")

if __name__ == "__main__":
    bus = StateBus()
    engine = IntentEngine(bus)
    
    print("IntentEngine klaar. Starten van run_loop()...")
    engine.run_loop()