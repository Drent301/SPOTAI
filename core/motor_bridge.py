import time
import os
import json
import threading
from core.config_manager import ConfigManager
from core.statebus import StateBus
from typing import Dict, Any

# 1 kHz loop (1 ms latency) voor de Reflex Layer
MOTOR_LOOP_HZ = 1000 
LOOP_INTERVAL = 1.0 / MOTOR_LOOP_HZ

class MotorBridge:
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager()
        self._running = True
        print(f"MotorBridge (Fase B.1 - Unitree/VESC) geïnitialiseerd op {MOTOR_LOOP_HZ} Hz.")

    def run_motor_loop(self):
        """
        De low-latency, 1 kHz control loop voor motor drivers (VESC/ODrive).
        Gebaseerd op de cyclic update structuur van Unitree SDK.
        """
        print("[MotorBridge] Motor Loop gestart. Druk op Ctrl+C om te stoppen.")
        
        while self._running:
            start_time = time.time()
            
            # Stap 1: Lees huidige robotstaat
            self.statebus.reload_state()
            current_state = self.statebus.state
            
            # Stap 2: Lees het commando van de IntentEngine
            action = current_state.get("robot_action", "idle_wander")
            
            # Stap 3: Lees configuratie (voor bijv. beveiliging)
            obstacle_speed = self.config_manager.get_setting("OBSTACLE_RESPONSE_SPEED")
            
            # Stap 4: Bereken motorcommando's (Torque/Positie/Snelheid)
            # In een echte Pi/C++ implementatie zou hier de FLORES/OCS2 MPC draaien.
            joint_torque_cmd = self._calculate_control(action, obstacle_speed)
            
            # Stap 5: Stuur commando's naar hardware
            self._send_command(joint_torque_cmd)
            
            # Stap 6: Lees sensorfeedback (Encoder/IMU)
            self._update_sensor_feedback()

            # Zorgt voor 1 kHz synchronisatie
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # Toekomstige logica voor lag-analyse
            if elapsed > LOOP_INTERVAL * 1.5:
                print(f"[MotorBridge] WAARSCHUWING: Lag! Loop duurde {elapsed*1000:.2f} ms.")

    def _calculate_control(self, action: str, speed_limit: float) -> Dict[str, float]:
        """Simuleert de MPC (OCS2) of PID-berekening."""
        # Unitree/ROS topic-conventie: joint_torque_cmd
        if "execute" in action:
            return {"joint_1_torque": 0.5, "speed_limit": speed_limit}
        elif "social" in action:
            return {"joint_1_torque": 0.05, "speed_limit": speed_limit}
        return {"joint_1_torque": 0.0, "speed_limit": speed_limit}

    def _send_command(self, cmd: Dict[str, float]):
        """Simuleert het sturen van UDP-pakketten naar de VESC/ODrive drivers."""
        # Dit zou de lage-level UDP/CAN bus communicatie zijn.
        # Voor de simulatie: we loggen het commando.
        self.statebus.set_value("motor_cmd_sent", cmd)
        # print(f"[MotorBridge] Commando verstuurd: {cmd}")

    def _update_sensor_feedback(self):
        """Simuleert het bijwerken van de sensor feedback (ROS topic /joint_states)."""
        # Dit wordt later gebruikt door de bridge_receiver.py (Fase B.2)
        current_pose = {"pos_j1": 0.1, "vel_j1": 0.05}
        self.statebus.set_value("joint_states", current_pose)
        
    def stop(self):
        self._running = False
        print("[MotorBridge] Gestopt.")

if __name__ == "__main__":
    bus = StateBus()
    # Maak dummy config-waarde aan
    cm = ConfigManager()
    cm.set_setting("OBSTACLE_RESPONSE_SPEED", 0.8) 
    
    # Test
    bridge = MotorBridge(bus)
    
    # Simuleer een actie voor 1 seconde
    bus.set_value("robot_action", "execute_move_to_location")
    
    t = threading.Thread(target=bridge.run_motor_loop)
    t.start()
    
    time.sleep(1) 
    
    bridge.stop()
    t.join()