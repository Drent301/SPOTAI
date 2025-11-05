import time
import os
import sys
import threading
from core.config_manager import ConfigManager
from core.statebus import StateBus
from typing import Dict, Any

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Importeer onze nieuwe, aparte motor driver
from core.motor_driver import MotorDriver

# 1 kHz loop (1 ms latency) voor de Reflex Layer
MOTOR_LOOP_HZ = 1000 
LOOP_INTERVAL = 1.0 / MOTOR_LOOP_HZ

class MotorBridge:
    """
    Fase B.1: De 1kHz (low-latency) reflex-laag.
    Leest de StateBus, berekent OCS2/MPC (simulatie) en stuurt 
    commando's naar de MotorDriver.
    """
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager()
        self._running = True
        
        # Initialiseer de ECHTE (gesimuleerde) hardware driver
        try:
            self.driver = MotorDriver(num_joints=2)
        except Exception as e:
            print(f"[MotorBridge] FATALE FOUT: Kon motor-driver niet laden: {e}")
            self.driver = None
            return

        print(f"MotorBridge (Fase B.1) geïnitialiseerd op {MOTOR_LOOP_HZ} Hz.")

    def run_motor_loop(self):
        """
        De low-latency, 1 kHz control loop.
        """
        if not self.driver:
            print("[MotorBridge] Kan loop niet starten, driver is niet geladen.")
            return

        print("[MotorBridge] Motor Loop gestart.")
        
        while self._running:
            start_time = time.time()
            
            # --- STAP 1: Lees Hardware Feedback ---
            # Dit moet EERST, om de laatste positie te weten (Unitree-stijl)
            feedback = self.driver.get_feedback()
            self.statebus.set_value("joint_states", feedback)
            
            # --- STAP 2: Lees Systeem Staat & Config ---
            self.statebus.reload_state()
            current_state = self.statebus.state
            action = current_state.get("robot_action", "idle_wander")
            obstacle_speed = self.config_manager.get_setting("OBSTACLE_RESPONSE_SPEED")
            
            # --- STAP 3: Bereken Motor Commando (MPC/OCS2) ---
            joint_torque_cmd = self._calculate_control(action, obstacle_speed, feedback)
            
            # --- STAP 4: Stuur Commando naar Hardware ---
            self.driver.send_command(joint_torque_cmd)

            # --- Synchronisatie ---
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            if elapsed > LOOP_INTERVAL * 1.5:
                print(f"[MotorBridge] WAARSCHUWING: Lag! Loop duurde {elapsed*1000:.2f} ms.")

    def _calculate_control(self, action: str, speed_limit: float, feedback: Dict) -> Dict[str, float]:
        """
        Simuleert de MPC (OCS2) of PID-berekening.
        In de echte versie gebruikt deze 'feedback' om de volgende torque te berekenen.
        """
        # Unitree/ROS topic-conventie: joint_torque_cmd
        if "walk_forward" in action or "execute" in action:
            # Simuleer een simpele P-controller om naar positie 0.5 te gaan
            target_pos = 0.5
            current_pos = feedback.get("pos_j1", 0.0)
            gain = 0.8
            torque = (target_pos - current_pos) * gain
            return {"joint_1_torque": torque, "speed_limit": speed_limit}
            
        elif "social" in action:
            # Ga langzaam naar positie 0.1
            target_pos = 0.1
            current_pos = feedback.get("pos_j1", 0.0)
            gain = 0.5
            torque = (target_pos - current_pos) * gain
            return {"joint_1_torque": torque, "speed_limit": speed_limit}
            
        # Default: Ga terug naar 0 (idle)
        target_pos = 0.0
        current_pos = feedback.get("pos_j1", 0.0)
        gain = 0.2
        torque = (target_pos - current_pos) * gain
        return {"joint_1_torque": torque, "speed_limit": speed_limit}

    def stop(self):
        self._running = False
        if self.driver:
            self.driver.stop()
        print("[MotorBridge] Gestopt.")

if __name__ == "__main__":
    bus = StateBus()
    cm = ConfigManager()
    cm.set_setting("OBSTACLE_RESPONSE_SPEED", 0.8) 
    
    bridge = MotorBridge(bus)
    
    if bridge.driver:
        # Simuleer een actie voor 2 seconden
        bus.set_value("robot_action", "walk_forward")
        
        t = threading.Thread(target=bridge.run_motor_loop)
        t.start()
        
        print("Test: 'walk_forward' voor 2 seconden...")
        time.sleep(2)
        
        print("\nTest: 'social' voor 2 seconden...")
        bus.set_value("robot_action", "social_greet")
        time.sleep(2)
        
        bridge.stop()
        t.join()
        
        print(f"\nTest voltooid. Laatste joint_states: {bus.get_value('joint_states')}")
    else:
        print("Test mislukt, processor kon niet initialiseren.")