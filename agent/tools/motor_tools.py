import sys
import os
from langchain_core.tools import tool

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.statebus import StateBus

@tool
def tune_gain(servo_id: str, gain_value: str) -> str:
    """
    (Niveau 2) Past de gain (PID-instelling) van een servo aan. 
    Dit wordt gebruikt door de GPT-agent om het Bandit-learning systeem te optimaliseren.
    """
    
    # Guardrail: Controleer of de gain_value een geldig formaat heeft
    try:
        float(gain_value)
    except ValueError:
        return f"ERROR: Gain waarde '{gain_value}' is geen geldig nummer."

    # Update de statebus voor de Motor Bridge
    bus = StateBus()
    bus.set_value(f"servo_gain_{servo_id}", gain_value)
    
    return f"GAIN_TUNED: Servo '{servo_id}' gain is ingesteld op {gain_value}. Motor Bridge zal dit toepassen."