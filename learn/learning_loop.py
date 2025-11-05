import os
import sys
import time
import random
import threading
from typing import Dict, Any

# Voeg de hoofdmap toe voor imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Importeer alle leercomponenten
from core.statebus import StateBus
from core.log_writer import LogWriter
from learn.bandit_learner import BanditLearner # D.1: Lokale gain optimalisatie
from learn.offline_reflector import OfflineReflector # D.2: Lokale Llama/FAISS


# Hoofdfrequentie: Leren hoeft niet realtime, maar moet sneller dan GPT-cloud (1 Hz)
LEARNING_LOOP_HZ = 1 
LOOP_INTERVAL = 1.0 / LEARNING_LOOP_HZ

class LearningLoop:
    """
    Fase D.3: Orchestrator voor alle lokale leer- en reflectieprocessen.
    Draait op de Pi en voert Bandit-updates en Offline-reflectie uit.
    """
    def __init__(self, state_bus: StateBus):
        self.bus = state_bus
        self.log_writer = LogWriter()
        self.bandit_learner = BanditLearner(state_bus)
        self.offline_reflector = OfflineReflector(state_bus)
        self._running = True
        
        # Simuleer de start van de leerprocessen
        self.bus.set_value("learning_loop_status", "active")
        print("[LearningLoop] Geïnitialiseerd. Zelflerende capaciteit actief.")

    def run_loop(self):
        """De centrale leerloop (1 Hz)."""
        print(f"[LearningLoop] Loop gestart op {LEARNING_LOOP_HZ} Hz. Druk op Ctrl+C om te stoppen.")
        
        while self._running:
            start_time = time.time()
            
            # --- STAP 1: Lokale Optimalisatie (Bandit-Learning) ---
            # De BanditLearner voert een step uit en update direct de StateBus met nieuwe gains.
            self.bandit_learner.step()
            
            # --- STAP 2: Offline Cognitie (Llama-3 / Mini-Reflectie) ---
            # Voert mini-reflectie uit als de robot offline is
            self.offline_reflector.trigger_mini_reflection()
            
            # --- STAP 3: Log de leercyclus ---
            self.log_writer.log_event(
                event_type="LEARNING_CYCLE_COMPLETED",
                details={"bandit_gains": self.bandit_learner.current_gains}
            )

            # --- STAP 4: Periodieke Reflectie Logging ---
            # Dit is een voorbeeld om de LogWriter te testen met een batch.
            if random.random() < 0.05: # 5% kans om te loggen, of eenmaal per minuut
                self.log_writer.log_offline_reflect_batch(
                    f"Mini-reflectie uitgevoerd. Bandit-gains zijn: {self.bandit_learner.current_gains}", 
                    time.time()
                )
            
            # Synchronisatie
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        self._running = False
        self.bus.set_value("learning_loop_status", "inactive")
        print("[LearningLoop] Gestopt.")

def main():
    bus = StateBus()
    learner = LearningLoop(bus)
    
    try:
        learner.run_loop()
    except KeyboardInterrupt:
        learner.stop()
        print("LearningLoop gestopt door gebruiker.")

if __name__ == '__main__':
    main()