import json
import os
from typing import Any, Dict

# Bepaal het pad naar de datamap
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, 'data')
CONFIG_FILE = os.path.join(DATA_DIR, 'config.json')

# Standaardinstellingen (UITGEBREID met alle parameters uit GPT-ControlMatrix)
DEFAULT_CONFIG = {
    # I. Configuratie & Gedragsparameters
    "LOOPSNELHEID": 0.5, 
    "ACCELERATIE": 0.2,
    "BOCHTRADIUS": 0.8,
    "VOETDRUKCORRECTIE": 0.0,
    "PID_BALANCE_GAIN": 0.35, # Basiswaarde voor tune_gain
    "SPEECH_TONE": "vrolijk",
    "SPEECH_PITCH": 1.0,
    "SPEECH_SNELHEID": 1.0,
    "SPEECH_EMOTIE_KLEUR": "groen",
    "EMOTIE_DISPLAY_KLEUR": "#00ff00",
    "EMOTIE_DISPLAY_EXPRESSIE": "neutraal",
    "GEZICHTSDETECTIEGEVOELIGHEID": 0.85, 
    "CAMERA_RESOLUTIE": "1280x720",
    "AUTONOMY_LEVEL": 0.5,
    
    # II. Leren & Reflectie
    "EXPLORATIEGRAAD": 0.2, # Parameter voor Bandit-learning
    "REFLECTIE_PRIORITEIT": "mensen eerst", 
    "REFLECTIE_BATCH_GROOTTE": 100, 

    # III. Systeem & Optimalisatie (Bestaande)
    "OBSTACLE_RESPONSE_SPEED": 1.2, # Standaard vertraagd (GPT geoptimaliseerd)
    "REFLECTION_INTERVAL_SEC": 30, # De loop frequentie van de GPT-agent
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
        # WAARSCHUWING: Geen key check, zodat GPT zelf nieuwe keys kan toevoegen, 
        # maar dit wordt gevalideerd door LangGraph's Guardrails in agent/agent_runtime.py
        
        self.config[key] = value
        self.save_config()
        print(f"ConfigManager: '{key}' persistent opgeslagen als {value}")

if __name__ == "__main__":
    print("ConfigManager module test...")
    cm = ConfigManager()
    
    # Test instelling van GPT-ControlMatrix
    cm.set_setting("EXPLORATIEGRAAD", 0.5)
    print(f"Nieuwe exploratiegraad: {cm.get_setting('EXPLORATIEGRAAD')}")
    
    # Simuleer herstart
    del cm
    cm_reboot = ConfigManager()
    print(f"Na reboot is gevoeligheid: {cm_reboot.get_setting('GEZICHTSDETECTIEGEVOELIGHEID')}")