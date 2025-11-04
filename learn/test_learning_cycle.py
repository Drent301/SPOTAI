import sys
import os
import time
import threading

# Voeg de hoofdmap toe voor imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.statebus import StateBus
from core.config_manager import ConfigManager
from learn.learning_loop import LearningLoop

# Definieert het logbestand dat gecontroleerd moet worden
DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')
EXPERIENCE_FILE = os.path.join(DATA_DIR, 'experience.ndjson')

def setup_simulation_state(bus: StateBus, config: ConfigManager):
    """Initialiseert de StateBus met waarden die leren triggeren."""
    
    print("\n--- Setup: Initialisatie Testcondities ---")
    
    # 1. Simuleer een slechte robotstatus (hoge trilling = slechte beloning)
    bus.set_value("imu_vibration_rms", 0.5)
    bus.set_value("motor_energy_load", 0.7)
    
    # 2. Simuleer Offline Status (om de Llama-3 reflectie te activeren)
    bus.set_value("network_state", "offline")
    bus.set_value("robot_mode", "idle")
    
    # 3. Zorg dat de Bandit config correct is
    config.set_setting("PID_BALANCE_GAIN", 0.4) # Beginwaarde
    config.set_setting("EXPLORATIEGRAAD", 0.9)  # Hoge exploratie om sneller te leren
    
    print(f"Status: Robot is ingesteld op OFF-LINE mode. Trilling: {bus.get_value('imu_vibration_rms')}")


def run_full_learning_validation():
    bus = StateBus()
    config = ConfigManager()
    
    # Wis oude logbestanden voor een schone test
    if os.path.exists(EXPERIENCE_FILE):
        os.remove(EXPERIENCE_FILE)
        
    setup_simulation_state(bus, config)
    
    learner = LearningLoop(bus)
    
    # Start de Learning Loop in een aparte thread
    t = threading.Thread(target=learner.run_loop)
    t.start()
    
    print("\n--- Validatie: Wacht 5 seconden op de Learning Loop (5 cycli) ---")
    time.sleep(5)
    
    learner.stop()
    t.join()
    
    # --- Assertie 1: Bandit-Learning Succes ---
    current_gain = bus.get_value("hip_pitch_gain")
    print(f"\n[RESULT 1: Bandit Gain] Huidige Gain: {current_gain:.3f}")
    if current_gain != 0.4:
        print("✅ SUCCESS: Bandit Learner heeft de PID-gain aangepast (Leercyclus is actief).")
    else:
        print("❌ FAILED: Bandit Learner heeft de gain niet aangepast.")

    # --- Assertie 2: Offline Reflectie Geactiveerd ---
    last_offline_query_ts = bus.get_value("last_local_query_ts")
    if last_offline_query_ts:
        print(f"✅ SUCCESS: Offline Reflector (Llama-3) is geactiveerd (Timestamp: {last_offline_query_ts}).")
    else:
        print("❌ FAILED: Offline Reflector is niet geactiveerd.")

    # --- Assertie 3: Logging (Unitree JSONL) ---
    if os.path.exists(EXPERIENCE_FILE) and os.path.getsize(EXPERIENCE_FILE) > 0:
        print("✅ SUCCESS: Experience.ndjson is aangemaakt en gevuld (D.3 LogWriter is OK).")
    else:
        print("❌ FAILED: Experience.ndjson is niet aangemaakt.")


if __name__ == '__main__':
    run_full_learning_validation()