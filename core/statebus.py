import json
import time
import os 

# Bepaal het pad naar het state-bestand
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, 'data')
STATE_FILE = os.path.join(DATA_DIR, 'statebus.json')

class StateBus:
    def __init__(self):
        # Zorg dat de data map bestaat
        os.makedirs(DATA_DIR, exist_ok=True)

        self.state = self.load_state()
        print("StateBus geïnitialiseerd.")

    def load_state(self):
        """Laadt de huidige staat uit het JSON-bestand."""
        try:
            with open(STATE_FILE, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"StateBus: {STATE_FILE} niet gevonden, maak een nieuwe aan.")
            return {"network_state": "unknown", "last_update": 0}
        except json.JSONDecodeError:
            # Dit kan gebeuren als twee scripts tegelijk schrijven
            print(f"StateBus: Fout bij lezen {STATE_FILE}, gebruik lege staat.")
            return {"network_state": "unknown", "last_update": 0}

    def save_state(self):
        """Slaat de huidige staat op naar het JSON-bestand."""
        try:
            with open(STATE_FILE, 'w') as f:
                json.dump(self.state, f, indent=4)
        except IOError as e:
            print(f"StateBus: Kon niet naar {STATE_FILE} schrijven: {e}")

    def set_value(self, key, value):
        """Werkt een waarde bij in de staat en slaat deze op."""
        self.state[key] = value
        self.state['last_update'] = time.time()
        self.save_state()
        print(f"StateBus: {key} ingesteld op {value}")

    def get_value(self, key):
        """Haalt een waarde op uit de (in-memory) staat."""
        return self.state.get(key, None)

    def reload_state(self):
        """
        *** NIEUWE FUNCTIE ***
        Herlaadt de staat vanaf de schijf.
        Dit is cruciaal om data van externe processen te zien.
        """
        self.state = self.load_state()

if __name__ == "__main__":
    print("StateBus module test...")
    bus = StateBus()
    bus.set_value("robot_mode", "idle")
    print(f"Huidige robot_mode: {bus.get_value('robot_mode')}")
