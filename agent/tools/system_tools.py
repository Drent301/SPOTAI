import os
from langchain_core.tools import tool

# Dit is een mock voor de echte systemctl/supervisorctl aanroep op de Raspberry Pi.

def _execute_service_command(service_name: str, command: str) -> bool:
    """
    Simuleert de uitvoering van een systemd-commando op de robot.
    Dit is de whitelisting-logica.
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
    return True

@tool
def restart_service(service_name: str) -> str:
    """
    (Niveau 2) Herstart een robot-service (zoals 'speech_processor' of 'perception_processor') 
    bij een gedetecteerde fout of crash, ten behoeve van zelfherstel.
    """
    if not _execute_service_command(service_name, "check_access"):
        return f"ERROR: De service '{service_name}' is geen geldige of geautoriseerde service om te herstarten."

    if _execute_service_command(service_name, "restart"):
        return f"SERVICE_RESTARTED: De service '{service_name}' is succesvol opnieuw opgestart."
    else:
        return f"ERROR: Het herstarten van service '{service_name}' is mislukt. Controleer de logs."

@tool
def log_reflection(message: str) -> str:
    """
    (Niveau 2) Logt een belangrijke reflectie of beslissing van de agent 
    naar het centrale logsysteem voor audit en analyse.
    """
    # Dit is een mock voor de echte logwriter (Fase A.3)
    print(f"[AUDIT_LOG] GPT Reflection: {message}")
    return f"LOG_RECORDED: Bericht is vastgelegd in het systeemlogboek."