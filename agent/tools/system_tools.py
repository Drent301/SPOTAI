import os
from langchain_core.tools import tool

# Dit is een mock voor de echte systemctl/supervisorctl aanroep op de Raspberry Pi.
# In de definitieve Pi-omgeving wordt dit een veilige subprocess.run() met systemctl.

def _execute_service_command(service_name: str, command: str) -> bool:
    """
    Simuleert de uitvoering van een systemd-commando op de robot.
    In de echte omgeving moet dit via een veilige, niet-root systemctl/supervisorctl.
    """
    # Whitelist van toegestane services (cruciale Guardrail)
    whitelisted_services = [
        "perception_processor",
        "speech_processor",
        "motor_bridge",
        "intent_engine"
    ]
    
    if service_name not in whitelisted_services:
        print(f"[ERROR] Service '{service_name}' staat niet op de whitelist en kan niet worden geherstart.")
        return False
        
    print(f"[SERVICE_CONTROL] Command '{command}' sent to service '{service_name}'.")
    # Simuleer een succesvolle herstart
    return True

@tool
def restart_service(service_name: str) -> str:
    """
    (Niveau 2) Herstart een robot-service (zoals 'speech_processor' of 'perception_processor') 
    bij een gedetecteerde fout of crash, ten behoeve van zelfherstel.
    """
    # Guardrail: Controleer of de service op de whitelist staat
    if not _execute_service_command(service_name, "check_access"):
        return f"ERROR: De service '{service_name}' is geen geldige of geautoriseerde service om te herstarten."

    # Voer de herstart uit
    if _execute_service_command(service_name, "restart"):
        return f"SERVICE_RESTARTED: De service '{service_name}' is succesvol opnieuw opgestart."
    else:
        return f"ERROR: Het herstarten van service '{service_name}' is mislukt. Controleer de logs."