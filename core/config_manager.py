import json
import os
from typing import Any, Dict

# Bepaal het pad naar de datamap
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, 'data')
CONFIG_FILE = os.path.join(DATA_DIR, 'config.json')

# Standaardinstellingen (gebaseerd op documenten)
DEFAULT_CONFIG = {
    "MOTOR_LOOP_HZ": 20, # De intent-loop
    "REFLECTION_INTERVAL_SEC": 30,
    "AUTONOMY_LEVEL": 0.5,
    "SPEECH_TONE": "vrolijk",
    "OBSTACLE_RESPONSE_SPEED": 1.2, # Standaard vertraagd
    "PID_BALANCE_GAIN": 0.35, # Basiswaarde voor tune_gain
}

class ConfigManager:
    def __init__(self):
        os.makedirs(DATA_DIR, exist_ok=True)
        self.config = self.load_config()
        print("ConfigManager geïnitialiseerd.")

    def load_config(self) -> Dict[str, Any]:
        """Laadt de configuratie, merge met defaults."""
        try:
            with open(CONFIG_FILE, 'r') as f:
                saved_config = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            saved_config = {}

        # Merge saved settings over de defaults
        merged_config = DEFAULT_CONFIG.copy()
        merged_config.update(saved_config)
        return merged_config

    def save_config(self):
        """Slaat de actuele configuratie op naar disk."""
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(self.config, f, indent=4)
        except IOError as e:
            print(f"ConfigManager: Kon niet naar {CONFIG_FILE} schrijven: {e}")

    def get_setting(self, key: str) -> Any:
        """Haalt een instelling op."""
        return self.config.get(key)

    def set_setting(self, key: str, value: Any):
        """Slaat een instelling op en schrijft weg."""
        if key not in DEFAULT_CONFIG:
            print(f"WAARSCHUWING: '{key}' is geen bekende configuratieparameter.")
        
        self.config[key] = value
        self.save_config()
        print(f"ConfigManager: '{key}' persistent opgeslagen als {value}")

if __name__ == "__main__":
    # Test om te zien of de manager werkt en waarden opslaat
    print("ConfigManager module test...")
    cm = ConfigManager()
    
    # Test set_setting
    cm.set_setting("SPEECH_TONE", "neutraal")
    print(f"Nieuwe tone: {cm.get_setting('SPEECH_TONE')}")
    
    # Simuleer herstart
    del cm
    cm_reboot = ConfigManager()
    print(f"Na reboot is tone: {cm_reboot.get_setting('SPEECH_TONE')}")