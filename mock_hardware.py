import time
import json
import os
import sys

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.statebus import StateBus

def simulate():
    print("--- Hardware Simulator Gestart (met Redis StateBus) ---")

    bus = StateBus()

    # Controleer of we verbinding hebben
    if not bus._is_connected():
        print("[MockHardware] Kan niet doorgaan zonder Redis-verbinding.")
        return

    # Start-condities
    print("\n--- Setup: Initialisatie Testcondities ---")
    bus.set_value("is_charging", False)
    bus.set_value("vision_detections", []) # Lege lijst, geen detecties
    bus.set_value("latest_intent", {}) # Leeg, geen spraak

    try:
        print("\n--- TEST 1: Wacht 5s (Robot is 'idle') ---")
        time.sleep(5)

        print("\n--- TEST 2: Simuleer 'mens gezien' ---")
        bus.set_value("vision_detections", [{"type": "gezicht", "emotie": "blij", "confidence": 0.95}])
        time.sleep(5) # Wacht 5s (Robot moet 'social' zijn)

        print("\n--- TEST 3: Simuleer 'spraakopdracht' ---")
        bus.set_value("latest_intent", {"intent": "move_command", "confidence": 0.9})
        time.sleep(5) # Wacht 5s (Robot moet 'listening' zijn en Rasa aanroepen)

        print("\n--- TEST 4: Simuleer 'opladen' ---")
        bus.set_value("is_charging", True)
        time.sleep(5) # Wacht 5s (Robot moet 'charging' zijn)

        print("\n--- Simulatie Voltooid ---")
        # Reset state voor de volgende run
        bus.set_value("is_charging", False)
        bus.set_value("vision_detections", [])
        bus.set_value("latest_intent", {})


    except KeyboardInterrupt:
        print("Hardware simulator gestopt.")

if __name__ == "__main__":
    simulate()
