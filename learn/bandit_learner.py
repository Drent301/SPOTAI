import os
import sys
import json
import time
import random
import logging
from typing import Dict, List, Tuple

# Voeg de hoofdmap toe
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Importeer kernmodules
from core.statebus import StateBus
from core.config_manager import ConfigManager

# Basis logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - [%(levelname)s] - %(message)s')

class EpsilonGreedyBandit:
    """
    Een robuuste, niet-gesimuleerde implementatie van een Epsilon-Greedy Bandit algoritme.
    Dit algoritme wordt gebruikt om de beste PID-gains te vinden door exploratie en exploitatie
    in balans te houden.
    """
    def __init__(self, gain_names: List[str], initial_gain: float, exploration_rate: float, step_size: float = 0.05):
        self.gain_names = gain_names
        self.epsilon = exploration_rate  # De kans op het kiezen van een willekeurige actie (exploratie)
        self.step_size = step_size

        # Houd de geschatte waarde (Q-value) en het aantal keren dat elke gain is gekozen bij
        self.q_values: Dict[str, float] = {name: initial_gain for name in self.gain_names}
        self.action_counts: Dict[str, int] = {name: 0 for name in self.gain_names}

        logging.info(f"EpsilonGreedyBandit geïnitialiseerd voor {gain_names} met epsilon={self.epsilon:.2f}.")

    def choose_action(self) -> str:
        """ Kiest een gain om aan te passen op basis van de epsilon-greedy strategie. """
        if random.random() < self.epsilon:
            # Exploratie: kies een willekeurige gain
            chosen_gain = random.choice(self.gain_names)
            logging.info(f"Bandit kiest voor exploratie: '{chosen_gain}'")
            return chosen_gain
        else:
            # Exploitatie: kies de gain met de hoogste geschatte waarde
            # (We implementeren dit als het kiezen van de minst beloonde actie om te verbeteren)
            # Dit is een variant die focust op het verbeteren van de zwakste schakel.
            worst_gain = min(self.q_values, key=self.q_values.get)
            logging.info(f"Bandit kiest voor exploitatie (verbeteren): '{worst_gain}'")
            return worst_gain

    def update_q_value(self, gain_name: str, reward: float):
        """ Werkt de Q-waarde voor een gekozen gain bij op basis van de ontvangen beloning. """
        self.action_counts[gain_name] += 1
        n = self.action_counts[gain_name]

        # Incrementele update van de Q-waarde (gemiddelde beloning)
        old_q = self.q_values[gain_name]
        new_q = old_q + (1/n) * (reward - old_q)
        self.q_values[gain_name] = new_q
        logging.info(f"Q-waarde voor '{gain_name}' bijgewerkt naar {new_q:.3f} (n={n}, reward={reward:.3f}).")


class BanditLearner:
    """
    Implementeert de Tensorforce (Bandit-learning) Agent voor lokale PID-gain optimalisatie
    met een Epsilon-Greedy strategie.
    """
    def __init__(self, state_bus: StateBus):
        self.bus = state_bus
        self.config_manager = ConfigManager()
        
        self.exploration_rate: float = self.config_manager.get_setting("EXPLORATIEGRAAD", 0.15)
        initial_gain = self.config_manager.get_setting("PID_BALANCE_GAIN", 0.6)
        
        gain_config = {
            "hip_pitch_gain": {"min": 0.4, "max": 0.9, "step": 0.05},
            "knee_pitch_gain": {"min": 0.4, "max": 0.9, "step": 0.05},
            "wheel_gain": {"min": 0.2, "max": 0.7, "step": 0.02}
        }
        self.gain_names = list(gain_config.keys())
        self.gain_config = gain_config
        self.current_gains = {name: initial_gain for name in self.gain_names}

        self.bandit = EpsilonGreedyBandit(
            gain_names=self.gain_names,
            initial_gain=initial_gain,
            exploration_rate=self.exploration_rate
        )

        logging.info(f"[BanditLearner] Geïnitialiseerd met Epsilon-Greedy strategie.")

    def _get_reward(self) -> float:
        """
        Berekent de beloning op basis van echte data uit de StateBus.
        Een hogere beloning betekent betere prestaties.
        """
        vibration = self.bus.get_value("imu_vibration_rms", 0.5) # Default naar een neutrale slechte waarde
        energy_use = self.bus.get_value("motor_energy_load", 0.5)
        
        reward = 1.0 - (vibration * 0.6 + energy_use * 0.4) # Geef stabiliteit meer gewicht
        return max(0.0, reward)

    def step(self):
        """ Voert één leercyclus uit. """
        reward = self._get_reward()

        # 1. Bandit kiest welke gain te tunen
        target_gain = self.bandit.choose_action()

        # 2. Werk de Q-waarde van de gekozen gain bij
        self.bandit.update_q_value(target_gain, reward)

        # 3. Pas de daadwerkelijke gain aan
        current_value = self.current_gains[target_gain]
        step = self.gain_config[target_gain]["step"]
        
        # Bepaal de richting: verbeter als de beloning laag is, anders verken
        direction = 1 if reward > self.bandit.q_values[target_gain] else -1

        new_value = current_value + direction * step

        # Zorg dat de gain binnen veilige limieten blijft
        min_val = self.gain_config[target_gain]["min"]
        max_val = self.gain_config[target_gain]["max"]
        new_value = max(min_val, min(max_val, new_value))

        if abs(new_value - current_value) > 0.001:
            logging.info(f"Gain '{target_gain}' aangepast: {current_value:.3f} -> {new_value:.3f} (reward: {reward:.3f})")
            
            self.current_gains[target_gain] = new_value
            self.bus.set_value(target_gain, new_value)
            
            self.bus.set_value("last_bandit_action", {
                "timestamp": time.time(),
                "action": "tune_gain",
                "target": target_gain,
                "value": new_value,
                "reward": reward
            })

# Test code
if __name__ == '__main__':
    bus = StateBus()
    bus.set_value("imu_vibration_rms", 0.3)
    bus.set_value("motor_energy_load", 0.4)
    
    learner = BanditLearner(bus)
    
    print("\nStart Bandit-learning Epsilon-Greedy simulatie...")
    for i in range(10):
        # Simuleer veranderende condities
        bus.set_value("imu_vibration_rms", random.uniform(0.1, 0.6))
        learner.step()
        time.sleep(0.5)
        
    print("\nEindwaarden na simulatie:")
    print(json.dumps(learner.current_gains, indent=2))
    print("\nFinale Q-waarden:")
    print(json.dumps(learner.bandit.q_values, indent=2))
