import time
import random
import math
from typing import Dict, Any

# Dit bestand simuleert de ECHTE 'pyvesc' (VESC) of 'odrive' (ODrive) libraries.
# Wanneer we dit op de Pi 5 testen, vervangen we dít bestand
# door de daadwerkelijke seriële/CAN-bus aanroepen.

class MotorDriver:
    """
    Een placeholder (mock) voor de echte VESC, ODrive, of SimpleFOC motor drivers.
    Deze klasse wordt aangeroepen door de MotorBridge op 1000 Hz.
    """
    def __init__(self, num_joints: int = 2):
        # In een echte implementatie:
        # - self.serial_port = serial.Serial('/dev/ttyACM0', 115200)
        # - self.vesc_comms = pyvesc.VESC(self.serial_port)
        print("[MotorDriver] Verbonden met VESC/ODrive hardware (simulatie).")
        self.num_joints = num_joints
        self._running = True

        # Simuleer de huidige staat van de motor-encoders
        self._joint_positions = [0.0] * num_joints
        self._joint_velocities = [0.0] * num_joints
        self._last_cmd_time = time.time()

    def send_command(self, joint_torque_cmd: Dict[str, float]):
        """ 
        Simuleert het versturen van torque/snelheid commando's naar de hardware.
        """
        torque = joint_torque_cmd.get("joint_1_torque", 0.0)

        # Simuleer het effect van het commando op de motor
        dt = time.time() - self._last_cmd_time
        self._last_cmd_time = time.time()

        # Simpele fysica: torque -> acceleratie -> snelheid -> positie
        # (Dit is een zeer grove simulatie)
        acceleration = torque * 5.0 # Gesimuleerde motorconstante
        self._joint_velocities[0] += acceleration * dt
        # Wrijving
        self._joint_velocities[0] *= 0.95 
        self._joint_positions[0] += self._joint_velocities[0] * dt

        # print(f"DEBUG: Torque={torque}, Vel={self._joint_velocities[0]}, Pos={self._joint_positions[0]}")

    def get_feedback(self) -> Dict[str, float]:
        """
        Simuleert het lezen van telemetrie (encoder/positie) van de hardware.
        """
        # Voeg een beetje willekeurige ruis toe aan de gesimuleerde waarden
        pos_noise = random.uniform(-0.001, 0.001)
        vel_noise = random.uniform(-0.005, 0.005)

        return {
            "pos_j1": self._joint_positions[0] + pos_noise,
            "vel_j1": self._joint_velocities[0] + vel_noise,
            "current_j1": abs(self._joint_velocities[0]) * 0.5 # Gesimuleerd stroomverbruik
        }

    def stop(self):
        print("[MotorDriver] Hardware commando's gestopt (fail-safe).")
        self._running = False