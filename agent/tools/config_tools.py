import sys
import os
from langchain_core.tools import tool

# Voeg de hoofdmap toe zodat de tool de core modules kan importeren
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.config_manager import ConfigManager
from core.statebus import StateBus

@tool
def set_config(key: str, value: str) -> str:
    """
    (Niveau 1) Past een configuratieparameter veilig aan met persistentie.
    Vereist een geldige sleutel (key) en een waarde (value).
    """
    
    cm = ConfigManager()
    
    # --- Guardrail: Waardebereik Validatie (Niveau 1) ---
    # Controleer of de waarde binnen veilige marges ligt voor kritieke parameters.
    if key == "OBSTACLE_RESPONSE_SPEED":
        try:
            numeric_value = float(value)
            if not (0.1 <= numeric_value <= 2.0):
                return f"ERROR: Waarde {value} voor {key} buiten veilige marge (0.1-2.0s)."
        except ValueError:
            return f"ERROR: Waarde {value} voor {key} moet een nummer zijn."
    
    # Voer de configuratiewijziging uit (Persistentie)
    cm.set_setting(key, value)
    
    # Vuur een snelle update af via de StateBus voor realtime modules
    bus = StateBus()
    bus.set_value(f"config_{key}", value)
    
    return f"CONFIG_UPDATED: '{key}' persistent opgeslagen als '{value}'. De motorische laag kan deze nu lezen."