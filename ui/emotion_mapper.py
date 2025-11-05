import sys
import os
import json
import time

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.statebus import StateBus
from core.config_manager import ConfigManager

# Het bestand dat de 'hyperpixel_overlay' leest
UI_STATE_FILE = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data', 'ui_state.json')

# Kleurdefinities gebaseerd op de documenten
KLEUR_OFFLINE = (102, 204, 255)  # Zachtblauw
KLEUR_IDLE = (0, 255, 0)         # Groen (default)
KLEUR_EMERGENCY = (255, 0, 0)     # Rood

class EmotionMapper:
    """
    Vertaalt de StateBus-status naar een simpele JSON voor de PyGame UI.
    Dit draait als een aparte, lichtgewicht service.
    """
    def __init__(self):
        self.bus = StateBus()
        self.config = ConfigManager()
        self._running = True
        print("[EmotionMapper] Gestart. Monitort StateBus...")

    def _get_ui_state(self) -> dict:
        """Haalt de robotstatus op en vertaalt dit naar een UI-staat."""
        
        # Haal de huidige modus op van de ModeArbiter
        robot_action = self.bus.get_value("robot_action") 
        # Gebruik 'robot_mode' als fallback, hoewel 'robot_action' specifieker is
        robot_mode = self.bus.get_value("robot_mode", "idle") 
        
        network = self.bus.get_value("network_state")

        # Prioriteit 1: Noodstop
        if "emergency" in str(robot_action) or "stop" in str(robot_action):
            return {"emotion": "error", "color": KLEUR_EMERGENCY}

        # Prioriteit 2: Offline status
        if network == "offline":
            return {"emotion": "offline", "color": KLEUR_OFFLINE}

        # TODO: Voeg hier meer emoties toe (blij, luisteren, etc.)
        # bijv. if robot_action == "approach_and_greet": ...

        # Default: Idle
        default_color_hex = self.config.get_setting("EMOTIE_DISPLAY_KLEUR")
        
        # Converteer hex (e.g., #00ff00) naar RGB tuple (0, 255, 0)
        if default_color_hex and default_color_hex.startswith('#'):
            h = default_color_hex.lstrip('#')
            rgb_color = tuple(int(h[i:i+2], 16) for i in (0, 2, 4))
        else:
            rgb_color = KLEUR_IDLE # Fallback
            
        return {"emotion": "idle", "color": rgb_color}

    def run_loop(self):
        """Hoofdloop, ververst de UI-status met 5 Hz."""
        while self._running:
            try:
                state = self._get_ui_state()
                
                # Schrijf de UI-staat naar de StateBus voor de overlay
                self.bus.set_value("ui_state", state)
                    
            except Exception as e:
                print(f"[EmotionMapper] Fout bij schrijven UI-staat: {e}")
            
            time.sleep(0.2) # Update met 5 Hz

    def stop(self):
        self._running = False
        print("[EmotionMapper] Gestopt.")

def main():
    mapper = EmotionMapper()
    try:
        mapper.run_loop()
    except KeyboardInterrupt:
        mapper.stop()

if __name__ == "__main__":
    main()