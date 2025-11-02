import time
import json
import os

# Gebruik hetzelfde pad als de StateBus
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, 'data')
STATE_FILE = os.path.join(DATA_DIR, 'statebus.json')

def write_state(data):
    """Een simpele 'statebus' schrijver."""
    try:
        # Zorg dat de map bestaat
        os.makedirs(DATA_DIR, exist_ok=True)
        # Lees de huidige staat
        current_state = {}
        if os.path.exists(STATE_FILE):
            with open(STATE_FILE, 'r') as f:
                current_state = json.load(f)

        # Update de staat
        current_state.update(data)

        # Schrijf de nieuwe staat
        with open(STATE_FILE, 'w') as f:
            json.dump(current_state, f, indent=4)

        print(f"[MockHardware] STATEBUS BIJGEWERKT: {data}")

    except Exception as e:
        print(f"[MockHardware] Fout bij schrijven statebus: {e}")

def simulate():
    print("--- Hardware Simulator Gestart ---")
    print(f"Schrijft naar: {STATE_FILE}")

    # Start-condities
    write_state({"is_charging": False, "last_human_seen_ts": None, "pending_voice_command": None})

    try:
        print("\n--- TEST 1: Wacht 5s (Robot is 'idle') ---")
        time.sleep(5)

        print("\n--- TEST 2: Simuleer 'mens gezien' ---")
        write_state({"last_human_seen_ts": time.time()})
        time.sleep(5) # Wacht 5s (Robot moet 'social' zijn)

        print("\n--- TEST 3: Simuleer 'spraakopdracht' ---")
        write_state({
            "last_voice_command_ts": time.time(),
            "pending_voice_command": "wat is het weer vandaag"
        })
        time.sleep(5) # Wacht 5s (Robot moet 'listening' zijn en Rasa aanroepen)

        print("\n--- TEST 4: Simuleer 'opladen' ---")
        write_state({"is_charging": True})
        time.sleep(5) # Wacht 5s (Robot moet 'charging' zijn)

        print("\n--- Simulatie Voltooid ---")

    except KeyboardInterrupt:
        print("Hardware simulator gestopt.")

if __name__ == "__main__":
    simulate()
