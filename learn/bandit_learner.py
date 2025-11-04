import os
import sys
import json
import time
import random
from typing import Dict, Any

# Voeg de hoofdmap toe
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Importeer kernmodules
from core.statebus import StateBus
from core.config_manager import ConfigManager

class BanditLearner:
    """
    Fase D.1: Simuleert de Tensorforce (Bandit-learning) Agent voor lokale PID-gain optimalisatie.
    Deze agent draait op de Raspberry Pi en optimaliseert snel lage-niveau parameters.
    """
    def __init__(self, state_bus: StateBus):
        self.bus = state_bus
        self.config_manager = ConfigManager()
        
        # Dit simuleert de Tensorforce Agent en zijn interne model
        self.current_gains: Dict[str, float] = self._load_initial_gains()
        
        # Leest de exploratiegraad uit de ControlMatrix configuratie
        self.exploration_rate: float = self.config_manager.get_setting("EXPLORATIEGRAAD")
        
        print(f"[BanditLearner] Geïnitialiseerd. Exploratiegraad: {self.exploration_rate}")
        
    def _load_initial_gains(self) -> Dict[str, float]:
        """Laadt de initiële, veilige gains van de config file (of defaults)."""
        # Gebruikt de algemene PID gain uit de ControlMatrix
        initial_gain = self.config_manager.get_setting("PID_BALANCE_GAIN")
        return {
            "hip_pitch_gain": initial_gain,
            "knee_pitch_gain": initial_gain,
            "wheel_gain": 0.4
        }

    def _get_reward(self) -> float:
        """
        Simuleert het berekenen van de beloning op basis van de StateBus.
        Beloning is gebaseerd op de stabiliteit en energie-efficiëntie.
        """
        # Data die van de BridgeReceiver (imu_data, motor_load) moet komen
        vibration = self.bus.get_value("imu_vibration_rms", 0.2) 
        energy_use = self.bus.get_value("motor_energy_load", 0.3)
        
        # De beloningsfunctie
        reward = 1.0 - (vibration * 0.5 + energy_use * 0.5) 
        return max(0.0, reward) 

    def step(self):
        """
        Voert één leercyclus uit:
        1. Meet beloning.
        2. Kiest een nieuwe actie (gain) op basis van de Bandit-strategie.
        3. Past de gain aan (simuleert het updaten van de Tensorforce-state).
        """
        reward = self._get_reward()
        target_gain_key = random.choice(list(self.current_gains.keys()))
        current_value = self.current_gains[target_gain_key]

        new_value = current_value
        
        if reward < 0.8 or random.random() < self.exploration_rate: # Lage beloning of Exploratie
            # Kleine, willekeurige stap (bijv. +/- 0.05)
            step = random.uniform(-0.05, 0.05) 
            new_value = max(0.2, min(0.9, current_value + step)) # Blijf binnen veilige marges 
            
            print(f"[Bandit] Gain {target_gain_key} aangepast. Nieuw: {new_value:.3f}. Beloning: {reward:.3f}")
            
            # 4. De nieuwe actie toepassen
            self.current_gains[target_gain_key] = new_value
            
            # Update de StateBus (MotorBridge en tune_gain tool kijken hiernaar)
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