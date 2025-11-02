import time
import requests # Voor het aanroepen van Rasa
from core.statebus import StateBus
from core.mode_arbiter import ModeArbiter

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

                # *** DE FIX ***
                # Herlaad de staat vanaf schijf aan het begin van *elke* loop
                self.statebus.reload_state()

                # 1. Bepaal modus
                mode = self.arbiter.get_mode()

                # 2. Haal sensor/input data op
                voice_command = self.statebus.get_value("pending_voice_command")

                # 3. Handel af op basis van modus
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
    engine.run_loop() # Start de loop direct
