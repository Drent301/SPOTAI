import requests
import json
import time
import os
# We importeren de StateBus klasse uit het statebus.py bestand in dezelfde map
from core.statebus import StateBus 

# !!! VERVANG DIT MET JE ECHTE DIGITALOCEAN IP-ADRES !!!
REFLECTOR_API_URL = "http://159.223.217.1/reflect"

class GptAgent:
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        print("GptAgent (Fase 4) geïnitialiseerd.")

    def get_recent_logs(self, lines=20):
        """Laadt de laatste 'n' regels uit het experience.ndjson log."""
        # We bouwen het pad naar het databestand
        log_path = os.path.join('data', 'experience.ndjson')
        try:
            with open(log_path, 'r') as f:
                # Lees alle regels en neem de laatste 'n'
                all_logs = f.readlines()
                recent_logs = all_logs[-lines:]
                return "".join(recent_logs) # Stuur als één string
        except FileNotFoundError:
            print(f"GptAgent: {log_path} nog niet gevonden.")
            return "Geen logs beschikbaar."
        except Exception as e:
            print(f"GptAgent: Fout bij lezen logs: {e}")
            return f"Log leesfout: {e}"

    def trigger_reflection(self, model="gpt-4-mini"):
        """
        Activeert een reflectie-call naar de DigitalOcean API.
        Dit is een 'event-triggered' reflectie.
        """
        print(f"Start GPT-reflectie (model: {model})...")
        
        # 1. Haal huidige staat op
        current_state = self.statebus.load_state()
        
        # 2. Haal recente logs op
        logs_data = self.get_recent_logs()
        
        # 3. Bereid de data voor
        payload = {
            "state": current_state,
            "logs": logs_data,
            "model": model
        }
        
        # 4. Roep de cloud API aan
        try:
            start_time = time.time()
            response = requests.post(REFLECTOR_API_URL, json=payload, timeout=15.0)
            latency = time.time() - start_time
            
            # Stopt als de HTTP-status 4xx of 5xx is
            response.raise_for_status() 
            
            result = response.json()
            # De API stuurt een JSON-string terug, we parsen die
            reflection_json = json.loads(result.get("reflection", "{}"))
            
            print(f"Reflectie succesvol ontvangen (in {latency:.2f}s).")
            print(f"Analyse: {reflection_json.get('analyse')}")
            
            # Verwerk het advies
            advies = reflection_json.get('advies')
            if advies:
                print(f"Advies ontvangen: {advies}")
                # We slaan het advies op in de statebus voor de runtime
                self.statebus.set_value("last_gpt_advice", advies)
            
            return reflection_json

        except requests.exceptions.HTTPError as e:
            # Fout van de server (bijv. 500)
            print(f"HTTP Fout tijdens reflectie: {e.response.status_code} {e.response.text}")
            self.statebus.set_value("network_state", "offline")
        except requests.exceptions.ConnectionError:
            # Server is niet bereikbaar
            print(f"Connectie Fout: Kan API op {REFLECTOR_API_URL} niet bereiken. Gaat offline.")
            self.statebus.set_value("network_state", "offline")
        except Exception as e:
            print(f"Onbekende fout tijdens reflectie: {e}")

if __name__ == "__main__":
    # Dit is een test-script om te controleren of de verbinding werkt
    # Voer dit uit vanuit de HOOFDMAP (SPOTAI) met: python -m core.gpt_agent
    
    print("GptAgent module test...")
    
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
    
    print(f"Test-aanroep naar {REFLECTOR_API_URL}...")
    agent.trigger_reflection(model="gpt-4o")