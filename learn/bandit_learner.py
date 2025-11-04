import os
import sys
import json
import time
from typing import Dict, Any

# Voeg de hoofdmap toe
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Importeer kernmodules
from core.statebus import StateBus
from core.config_manager import ConfigManager

class BanditLearner:
    """
    Simuleert de Tensorforce (Bandit-learning) Agent voor lokale PID-gain optimalisatie.
    Deze agent draait op de Raspberry Pi en optimaliseert snel lage-niveau parameters.
    """
    def __init__(self, state_bus: StateBus):
        self.bus = state_bus
        self.config_manager = ConfigManager()
        
        # Dit simuleert de Tensorforce Agent en zijn interne model
        self.current_gains: Dict[str, float] = self._load_initial_gains()
        self.exploration_rate: float = self.config_manager.get_config("BANDIT_EXPLORATION_RATE", 0.1)
        
        print(f"[BanditLearner] Geïnitialiseerd. Exploratiegraad: {self.exploration_rate}")
        
    def _load_initial_gains(self) -> Dict[str, float]:
        """Laadt de initiële, veilige gains van de config file."""
        # Dit zijn de gains die geoptimaliseerd moeten worden
        return {
            "hip_pitch_gain": self.config_manager.get_config("HIP_PITCH_GAIN", 0.5),
            "knee_pitch_gain": self.config_manager.get_config("KNEE_PITCH_GAIN", 0.7),
            "wheel_gain": self.config_manager.get_config("WHEEL_GAIN", 0.4)
        }

    def _get_reward(self) -> float:
        """
        Simuleert het berekenen van de beloning op basis van de StateBus.
        Beloning is gebaseerd op de stabiliteit en energie-efficiëntie.
        """
        # Voorbeeld: Negatieve beloning voor hoge trillingen (root mean square) of te veel energieverbruik
        vibration = self.bus.get_value("imu_vibration_rms", 0.0) 
        energy_use = self.bus.get_value("motor_energy_load", 0.0)
        
        # De beloningsfunctie: een combinatie van negatieve trilling en energie.
        # Een lage waarde is goed (hoge beloning).
        reward = 1.0 - (vibration * 0.5 + energy_use * 0.5) 
        return max(0.0, reward) # Beloning moet minimaal 0 zijn

    def step(self):
        """
        Voert één leercyclus uit:
        1. Meet beloning.
        2. Kiest een nieuwe actie (gain).
        3. Past de gain aan (simuleert het updaten van de Tensorforce-state).
        """
        # 1. Beloning meten voor de huidige actie (deze wordt door de motor-bridge direct toegepast)
        reward = self._get_reward()
        
        # 2. Beslissen welke gain aan te passen (e-greedy of softmax-actie)
        # We vereenvoudigen: Kies willekeurig een gain om te exploreren
        target_gain_key = list(self.current_gains.keys())[int(time.time() * 100) % len(self.current_gains)]
        current_value = self.current_gains[target_gain_key]

        # 3. Nieuwe actie genereren (Simuleert de Bandit Agent)
        new_value = current_value
        if reward < 0.8 or time.time() % 10 < self.exploration_rate * 10: # Lage beloning of Exploratie
            # Kleine, willekeurige stap (bijv. +/- 0.05)
            step = (os.urandom(1)[0] / 255.0 - 0.5) / 10.0 
            new_value = max(0.2, min(0.9, current_value + step)) # Blijf binnen veilige marges (0.2 tot 0.9)
            
            print(f"[Bandit] Gain {target_gain_key} aangepast. Oude: {current_value:.3f}, Nieuw: {new_value:.3f}. Beloning: {reward:.3f}")
            
            # 4. De nieuwe actie toepassen
            self.current_gains[target_gain_key] = new_value
            
            # Update de StateBus, zodat de MotorBridge het oppikt
            self.bus.set_value(target_gain_key, new_value)
            
            # Log de actie voor offline GPT-reflectie (Fase D.3)
            self.bus.set_value("last_bandit_action", {
                "timestamp": time.time(),
                "action": "tune_gain",
                "target": target_gain_key,
                "value": new_value,
                "reward": reward
            })
            
    
# Test code voor lokale uitvoering
if __name__ == '__main__':
    bus = StateBus()
    # Simuleer een basisbelasting in de StateBus
    bus.set_value("imu_vibration_rms", 0.2)
    bus.set_value("motor_energy_load", 0.3)
    
    learner = BanditLearner(bus)
    
    print("\nStart Bandit-learning simulatie...")
    for i in range(5):
        learner.step()
        time.sleep(1)
        
    print("\nEindwaarden na simulatie:")
    print(learner.current_gains)
    print("Laatste actie in StateBus:")
    print(bus.get_value("last_bandit_action"))