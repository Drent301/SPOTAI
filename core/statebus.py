import json
import time

[cite_start]STATE_FILE = '../data/statebus.json' # Gebaseerd op je structuur [cite: 222]

class StateBus:
    def __init__(self):
        self.state = self.load_state()
        print("StateBus geïnitialiseerd.")

    def load_state(self):
        """Laadt de huidige staat uit het JSON-bestand."""
        try:
            with open(STATE_FILE, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            print("StateBus: statebus.json niet gevonden, maak een nieuwe aan.")
            return {"network_state": "unknown", "last_update": 0}
        except json.JSONDecodeError:
            print("StateBus: Fout bij lezen statebus.json, maak een nieuwe aan.")
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
        """Haalt een waarde op uit de staat."""
        return self.state.get(key, None)

if __name__ == "__main__":
    # Dit is om de module direct te testen
    bus = StateBus()
    bus.set_value("robot_mode", "idle")
    print(f"Huidige robot_mode: {bus.get_value('robot_mode')}")
    [cite_start]bus.set_value("network_state", "online") # [cite: 5]