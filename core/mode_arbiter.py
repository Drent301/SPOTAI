import time
import sys
import os

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.statebus import StateBus

# Drempelwaarde voor een lage batterij
LOW_BATTERY_THRESHOLD = 20.0 # in procent

class ModeArbiter:
    """
    De "Eindbaas": neemt de uiteindelijke beslissing over de actie van de robot.
    Leest de intentie en systeemstatussen van de StateBus en bepaalt de `robot_action`.
    """
    def __init__(self, bus: StateBus):
        self.bus = bus
        self.last_action = None
        print("ModeArbiter geïnitialiseerd.")

    def run_decision_loop(self):
        """De hoofdloop die continu beslissingen neemt."""
        print("ModeArbiter: Beslissingsloop gestart (draait elke 2 seconden).")

        while True:
            try:
                # --- Rapporteer eigen status ---
                status = {"state": "running", "timestamp": time.time()}
                self.bus.set_value("mode_arbiter_status", status)

                # --- Verzamel Systeeminformatie ---
                power_state = self.bus.get_value("power_state", default={"percentage": 100.0})
                consolidated_intent = self.bus.get_value("consolidated_intent", default="no_intent")

                # --- Pas Beslissingslogica toe ---
                # Dit is een hiërarchie: de belangrijkste regels komen eerst.

                chosen_action = "monitor_sensors" # Standaard fallback-actie

                # 1. Kritieke Batterijstatus?
                if power_state.get("percentage", 100.0) < LOW_BATTERY_THRESHOLD:
                    chosen_action = "return_to_base"

                # 2. Is er een duidelijke, directe intentie?
                elif consolidated_intent and consolidated_intent != "no_intent":
                    # Hier zou je complexere vertalingen kunnen doen,
                    # maar voor nu nemen we de intentie direct over.
                    chosen_action = consolidated_intent

                # --- Publiceer de Definitieve Actie ---
                # Publiceer alleen als de actie verandert om ruis op de bus te verminderen.
                if chosen_action != self.last_action:
                    self.bus.set_value("robot_action", chosen_action)
                    self.last_action = chosen_action
                    print(f"ModeArbiter: Nieuwe actie ingesteld -> {chosen_action}")

            except Exception as e:
                print(f"[ModeArbiter] Fout in beslissingsloop: {e}")

            time.sleep(2) # Neem elke 2 seconden een beslissing

def main():
    """Start de Mode Arbiter."""
    # Wacht even zodat Redis zeker is opgestart
    time.sleep(1)

    bus = StateBus()
    if not bus._is_connected():
        print("[ModeArbiter] Kan niet starten, geen verbinding met StateBus (Redis).")
        return

    arbiter = ModeArbiter(bus)
    arbiter.run_decision_loop()

if __name__ == "__main__":
    main()
