import json
import os
from typing import Dict, Any, Optional

# Bepaal het pad naar de datamap
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, 'data')
MEMORY_FILE = os.path.join(DATA_DIR, 'memory.json')

# Standaard geheugenstructuur (een lijst van gebruikers of contexten)
DEFAULT_MEMORY = {
    "preferences": {},  # Algemene geleerde parameters (bv. wat te zeggen bij fout)
    "users": {}         # Gebruiker-specifiek geheugen (naam, leeftijd, voorkeuren)
}

class MemoryManager:
    def __init__(self):
        os.makedirs(DATA_DIR, exist_ok=True)
        self.memory = self.load_memory()
        print("MemoryManager geïnitialiseerd.")

    def load_memory(self) -> Dict[str, Any]:
        """Laadt het geheugen uit het JSON-bestand."""
        try:
            with open(MEMORY_FILE, 'r') as f:
                saved_memory = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            saved_memory = {}

        # Zorgt ervoor dat de basisstructuur altijd aanwezig is
        merged_memory = DEFAULT_MEMORY.copy()
        merged_memory.update(saved_memory)
        return merged_memory

    def save_memory(self):
        """Slaat het actuele geheugen op naar disk."""
        try:
            with open(MEMORY_FILE, 'w') as f:
                json.dump(self.memory, f, indent=4)
        except IOError as e:
            print(f"MemoryManager: Kon niet naar {MEMORY_FILE} schrijven: {e}")

    def get_user_data(self, user_id: str) -> Optional[Dict[str, Any]]:
        """Haalt data op voor een specifieke gebruiker."""
        return self.memory["users"].get(user_id)

    def set_user_data(self, user_id: str, data: Dict[str, Any]):
        """Slaat data op voor een gebruiker en schrijft weg."""
        user_entry = self.memory["users"].get(user_id, {})
        user_entry.update(data)
        self.memory["users"][user_id] = user_entry
        self.save_memory()
        print(f"MemoryManager: Data voor gebruiker '{user_id}' opgeslagen.")

    def log_preference(self, key: str, value: Any):
        """Slaat een algemene voorkeur op."""
        self.memory["preferences"][key] = value
        self.save_memory()
        print(f"MemoryManager: Algemene voorkeur '{key}' opgeslagen.")

if __name__ == "__main__":
    print("MemoryManager module test...")
    mm = MemoryManager()
    
    # Test opslaan van een gebruiker
    mm.set_user_data("Jules", {"leeftijd": 8, "favoriete_kleur": "blauw"})
    print(f"Data voor Jules: {mm.get_user_data('Jules')}")
    
    # Test opslaan van een algemene voorkeur
    mm.log_preference("standaard_begroeting", "Hallo daar, vriend!")
    
    # Simuleer herstart
    del mm
    mm_reboot = MemoryManager()
    print(f"Na reboot is de voorkeur: {mm_reboot.memory['preferences'].get('standaard_begroeting')}")