import time
import re
import os

# We moeten het pad vertellen waar de 'core' map is
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.statebus import StateBus
from core.gpt_agent import GptAgent
from core.config_manager import ConfigManager # NIEUWE IMPORT voor persistentie

# --- Veilige Tools (Fase 4 / ControlMatrix Niveau 1-2) ---
# Dit zijn de functies die de GPT-agent mag aanroepen.
# set_config() is nu persistent.

def set_config(key, value):
    """(Niveau 1) Past een configuratieparameter veilig aan met persistentie."""
    
    # Initialiseer de manager en de tools voor validatie
    cm = ConfigManager()
    
    # Guardrail: Valideer hier de input (bv. voor obstakelsnelheid)
    if key == "OBSTACLE_RESPONSE_SPEED":
        try:
            # We verwachten een numerieke string, converteer deze
            numeric_value = float(value)
            # Guardrail: Check of de waarde binnen een veilige marge ligt
            if not (0.1 <= numeric_value <= 2.0):
                return f"ERROR: Waarde {value} voor {key} buiten veilige marge (0.1-2.0s)."
        except ValueError:
            return f"ERROR: Waarde {value} voor {key} moet een nummer zijn."
    
    # Schrijf de nieuwe setting weg (Persistente opslag in config.json)
    cm.set_setting(key, value)
    print(f"[AgentRuntime] TOOL GEVOERD: set_config({key}, {value})")
    
    # Update de statebus om de intent_engine op de hoogte te stellen van de wijziging
    bus = StateBus()
    bus.set_value(f"config_{key}", value)
    
    return f"CONFIG_UPDATED: '{key}' persistent opgeslagen als '{value}'. De motorische laag kan deze nu lezen."

def restart_service(service_name):
    """(Niveau 2) Herstart een specifieke systemd-service."""
    print(f"[AgentRuntime] TOOL: restart_service({service_name})")
    # Toekomstige implementatie: os.system(f"systemctl --user restart {service_name}")
    pass

def tune_gain(servo_id, gain_value):
    """(Niveau 2) Past de gain van een servo aan."""
    print(f"[AgentRuntime] TOOL: tune_gain({servo_id}, {gain_value})")
    pass

def log_reflection(message):
    """(Niveau 2) Logt een specifieke reflectie van de agent."""
    print(f"[AgentRuntime] TOOL: log_reflection({message})")
    pass
    
def enable_network():
    """(Niveau 2) Dummy functie gebaseerd op het advies dat we kregen."""
    print(f"[AgentRuntime] TOOL: enable_network() aangeroepen.")
    pass

# --- Geavanceerde Tools (ControlMatrix Niveau 5) ---
# WAARSCHUWING: Deze functies zijn optioneel en moeten in een 
# sandbox draaien, zoals beschreven in het ControlMatrix-document.
#
AGENT_ALLOW_PATCHES = False # Standaard uitgeschakeld

def edit_code(file, section, patch):
    """(Niveau 5) Past code aan (standaard UIT)."""
    if AGENT_ALLOW_PATCHES:
        print(f"[AgentRuntime] TOOL (N5): edit_code({file}, {section}, ...)")
    else:
        print(f"[AgentRuntime] TOOL (N5): edit_code() genegeerd (AGENT_ALLOW_PATCHES=False).")

def rollback(file):
    """(Niveau 5) Rolt een wijziging terug (standaard UIT)."""
    if AGENT_ALLOW_PATCHES:
        print(f"[AgentRuntime] TOOL (N5): rollback({file})")
    else:
        print(f"[AgentRuntime] TOOL (N5): rollback() genegeerd (AGENT_ALLOW_PATCHES=False).")

def suggest_update(description):
    """(Niveau 5) Schrijft een voorstel naar updates/pending.txt."""
    print(f"[AgentRuntime] TOOL (N5): suggest_update({description})")
    # Dit is veiliger en kan altijd aan staan.
    try:
        os.makedirs('updates', exist_ok=True)
        with open('updates/pending.txt', 'a') as f:
            f.write(f"[{time.ctime()}] {description}\n")
    except Exception as e:
        print(f"[AgentRuntime] Fout bij schrijven suggest_update: {e}")

def generate_doc(changes):
    """(Niveau 5) Documenteert veranderingen."""
    print(f"[AgentRuntime] TOOL (N5): generate_doc({changes})")
    pass

# --- Tool Registry ---
# Mapt de advies-functienamen naar de daadwerkelijke Python-functies
AVAILABLE_TOOLS = {
    "set_config": set_config,
    "restart_service": restart_service,
    "tune_gain": tune_gain,
    "log_reflection": log_reflection,
    "enable_network": enable_network,
    "edit_code": edit_code,
    "rollback": rollback,
    "suggest_update": suggest_update,
    "generate_doc": generate_doc,
}

# --- Advies Parser ---
def parse_and_execute_advice(advice_string):
    """
    Parse het advies en voert het veilig uit.
    Deze parser zoekt naar functie-aanroepen zoals 'functie()' of 'functie("arg1", 123)'.
    """
    if not advice_string:
        return

    # Zoek naar 'functie()' of 'functie(...)' in de adviestekst
    matches = re.findall(r"\'?(\w+)\((.*?)\)\'?", advice_string)
    
    if not matches:
        print(f"[AgentRuntime] Geen uitvoerbaar commando gevonden in advies.")
        return

    for match in matches:
        tool_name = match[0]
        args_str = match[1]
        
        if tool_name in AVAILABLE_TOOLS:
            try:
                # Simpele argument-parser (split op komma)
                args = []
                if args_str:
                    # Verwijder quotes en spaties
                    args = [arg.strip().strip("'\"") for arg in args_str.split(',')]
                
                print(f"[AgentRuntime] Voert tool uit: {tool_name} met args {args}")
                AVAILABLE_TOOLS[tool_name](*args)
                
            except Exception as e:
                print(f"[AgentRuntime] Fout bij uitvoeren tool {tool_name}: {e}")
        else:
            print(f"[AgentRuntime] Onbekende tool in advies: {tool_name}")

# --- Hoofdloop ---
def main_runtime_loop():
    print("AgentRuntime (Fase 4) gestart. Druk op Ctrl+C om te stoppen.")
    bus = StateBus()
    agent = GptAgent(bus)
    
    # Async batching interval (Fase 8)
    REFLECTION_INTERVAL_SEC = 30 
    
    try:
        while True:
            print(f"\n--- Runtime Loop: Wacht {REFLECTION_INTERVAL_SEC}s ---")
            time.sleep(REFLECTION_INTERVAL_SEC)
            
            print("[RuntimeLoop] 1. Start periodieke reflectie...")
            # Gebruik gpt-4o, want 4-mini gaf een fout
            agent.trigger_reflection(model="gpt-4o")
            
            print("[RuntimeLoop] 2. Haal laatste advies op van StateBus...")
            advice = bus.get_value("last_gpt_advice")
            
            if advice:
                print(f"[RuntimeLoop] 3. Verwerk advies: '{advice}'")
                parse_and_execute_advice(advice)
                
                # Wis het advies na verwerking, zodat het niet opnieuw wordt uitgevoerd
                bus.set_value("last_gpt_advice", None) 
            else:
                print("[RuntimeLoop] 3. Geen nieuw advies gevonden.")

    except KeyboardInterrupt:
        print("\nAgentRuntime gestopt door gebruiker.")

if __name__ == "__main__":
    main_runtime_loop()