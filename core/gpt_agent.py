import requests
import json
import time
import os
from typing import Optional
from core.statebus import StateBus 
from core.config_manager import ConfigManager # NIEUWE IMPORT

# !!! VERVANG DIT MET JE ECHTE DIGITALOCEAN IP-ADRES !!!
# Let op: de IP-adres in de code is ter demonstratie en moet het IP van uw Droplet zijn.
REFLECTOR_API_URL = "http://159.223.217.1/reflect"

class GptAgent:
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager() # Voor eventuele config-waarden
        print("GptAgent (Fase 4) geïnitialiseerd.")

    def get_recent_logs(self, lines=20) -> str:
        """Laadt de laatste 'n' regels uit het experience.ndjson log."""
        log_path = os.path.join('data', 'experience.ndjson')
        try:
            with open(log_path, 'r') as f:
                all_logs = f.readlines()
                recent_logs = all_logs[-lines:]
                return "".join(recent_logs)
        except FileNotFoundError:
            return "Geen logs beschikbaar."
        except Exception as e:
            return f"Log leesfout: {e}"

    def check_connection(self) -> bool:
        """
        Controleert de verbinding met de reflector_api op DigitalOcean.
        Dit is de basis voor de network_state-logica.
        """
        try:
            # Probeer de root-endpoint aan te roepen
            response = requests.get(REFLECTOR_API_URL.replace("/reflect", ""), timeout=5)
            response.raise_for_status()
            
            # Als de verbinding succesvol is, stel de status in
            self.statebus.set_value("network_state", "online")
            return True
        except requests.exceptions.RequestException:
            # Als de verbinding mislukt (timeout, 4xx/5xx, etc.)
            print("[GptAgent] Connectie Fout: Kan DigitalOcean niet bereiken.")
            self.statebus.set_value("network_state", "offline")
            return False

    def trigger_reflection(self, task_type: str = "strategy") -> Optional[dict]:
        """
        Activeert een reflectie-call naar de DigitalOcean API.
        task_type: 'strategy' (GPT-4o) of 'basic_talk' (GPT-3.5-turbo)
        """
        # --- Stap 1: Model Switching (Kostenoptimalisatie) ---
        model_map = {
            "strategy": "gpt-4o",
            "basic_talk": "gpt-3.5-turbo"
        }
        model_to_use = model_map.get(task_type, "gpt-4o")

        print(f"Start GPT-reflectie (model: {model_to_use}, taak: {task_type})...")
        
        # --- Stap 2: Check Verbinding ---
        if not self.check_connection():
            print("[GptAgent] Offline. Reflectie geannuleerd.")
            return None

        # --- Stap 3: Data Voorbereiden en Roep API aan ---
        current_state = self.statebus.load_state()
        logs_data = self.get_recent_logs()
        
        payload = {
            "state": current_state,
            "logs": logs_data,
            "model": model_to_use
        }
        
        try:
            start_time = time.time()
            response = requests.post(REFLECTOR_API_URL, json=payload, timeout=15.0)
            latency = time.time() - start_time
            
            response.raise_for_status() 
            
            result = response.json()
            reflection_json = json.loads(result.get("reflection", "{}"))
            
            print(f"Reflectie succesvol ontvangen (in {latency:.2f}s).")
            print(f"Analyse: {reflection_json.get('analyse')}")
            
            advies = reflection_json.get('advies')
            if advies:
                # Advies opslaan in statebus voor de AgentRuntime/IntentEngine
                self.statebus.set_value("last_gpt_advice", advies)
            
            return reflection_json

        except requests.exceptions.RequestException as e:
            print(f"Connectie Fout tijdens reflectie: {e}. Gaat offline.")
            self.statebus.set_value("network_state", "offline")
            return None


if __name__ == "__main__":
    # Dit is een test-script om te controleren of de verbinding werkt
    
    print("GptAgent module test (Fase A.4)...")
    
    # Zorg dat de data-map en statebus bestaan voor de test
    os.makedirs('data', exist_ok=True)
    if not os.path.exists('data/statebus.json'):
        with open('data/statebus.json', 'w') as f:
            json.dump({"test_mode": True}, f)
        
    # Maak dummy data/experience.ndjson aan voor de test
    with open('data/experience.ndjson', 'w') as f:
        f.write('{"ts": 12345, "event": "robot_gestart"}\n')
        f.write('{"ts": 12346, "event": "obstakel_gezien"}\n')
    
    bus = StateBus()
    agent = GptAgent(bus)
    
    print(f"Test-aanroep (Strategy) naar {REFLECTOR_API_URL}...")
    agent.trigger_reflection(task_type="strategy")