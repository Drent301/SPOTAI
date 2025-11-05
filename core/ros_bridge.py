import sys
import os
import json
import time
import threading
import random

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.statebus import StateBus

# --- Dit is de "Simulatie" ---
# Op de Pi 5 vervangen we deze sectie door:
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Imu, JointState

class SimROS2Node:
    """ Simuleert de ROS 2 Node (rclpy) functionaliteit. """
    def __init__(self, node_name):
        print(f"[SimROS2] Node '{node_name}' gestart.")
        self._callbacks = {}
        self._publishers = {}

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        print(f"[SimROS2] Abonneert op topic: {topic}")
        self._callbacks[topic] = callback

    def create_publisher(self, msg_type, topic, qos_profile):
        print(f"[SimROS2] Maakt publisher voor topic: {topic}")
        self._publishers[topic] = None # Sla de publisher op

    def publish(self, topic, msg):
        """ Simuleert het publiceren van een bericht. """
        if topic in self._publishers:
            # print(f"DEBUG: Publiceer naar {topic}: {msg}")
            pass
        else:
            print(f"WAARSCHUWING: Poging tot publiceren op onbekend topic {topic}")

    def spin_once(self):
        """ Simuleert het controleren op nieuwe berichten. """
        # Simuleer een IMU-bericht
        if "/imu/data" in self._callbacks:
            msg = {"data": "sim_imu_data", "linear_acceleration_y": random.uniform(-0.1, 0.1)}
            self._callbacks["/imu/data"](msg)

        # Simuleer een JointState-bericht
        if "/joint_states" in self._callbacks:
            msg = {"name": ["j1", "j2"], "position": [random.uniform(-0.5, 0.5), 0]}
            self._callbacks["/joint_states"](msg)

def init_ros():
    print("[SimROS2] rclpy.init() aangeroepen.")

def shutdown_ros():
    print("[SimROS2] rclpy.shutdown() aangeroepen.")

# --- Einde Simulatie ---


class ROSBridge:
    """
    De centrale "Tolk" tussen de StateBus (ons brein) en ROS 2 (ons zenuwstelsel).
    """
    def __init__(self, statebus):
        self.bus = statebus
        self._running = True

        # Initialiseer de (gesimuleerde) ROS 2 Node
        init_ros()
        self.node = SimROS2Node("spotai_ros_bridge")

        # --- 1. StateBus (Output) -> ROS 2 (Input) ---
        # We publiceren commando's (zoals /cmd_vel) naar ROS 2
        self.cmd_vel_pub = self.node.create_publisher("Twist", "/cmd_vel", 10)

        # --- 2. ROS 2 (Output) -> StateBus (Input) ---
        # We abonneren op sensor-topics en schrijven ze naar de StateBus
        self.node.create_subscription("Imu", "/imu/data", self._imu_callback, 10)
        self.node.create_subscription("JointState", "/joint_states", self._joint_states_callback, 10)

        print("[ROSBridge] Succesvol geïnitialiseerd.")

    # --- Callbacks: Data van ROS 2 naar StateBus ---

    def _imu_callback(self, msg):
        """ Ontvangt IMU-data van ROS 2 en schrijft dit naar de StateBus. """
        # print(f"DEBUG: Ontvangen /imu/data: {msg}")
        # Vertaal ROS-msg naar StateBus-formaat
        accel_y = msg.get("linear_acceleration_y", 0.0)
        self.bus.set_value("imu_data", {"accel_y": accel_y})

    def _joint_states_callback(self, msg):
        """ Ontvangt motor-telemetrie van ROS 2 en schrijft dit naar de StateBus. """
        # print(f"DEBUG: Ontvangen /joint_states: {msg}")
        self.bus.set_value("joint_states", msg)

    # --- Publish Loop: Data van StateBus naar ROS 2 ---

    def run_publish_loop(self):
        """
        Leest de StateBus en publiceert de gewenste actie naar ROS 2.
        Draait op een lagere frequentie (10 Hz), want dit is de "denk"-snelheid.
        """
        print("[ROSBridge] Publish-loop (StateBus -> ROS 2) gestart op 10 Hz.")
        while self._running:
            self.bus.reload_state()
            action = self.bus.get_value("robot_action", "monitor_sensors")

            # Vertaal StateBus-actie naar ROS 2 /cmd_vel bericht
            msg = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"z": 0.0}}

            if action == "walk_forward":
                msg["linear"]["x"] = 0.5 # Ga vooruit
            elif action == "turn_left":
                msg["angular"]["z"] = 0.8 # Draai linksom

            self.node.publish("/cmd_vel", msg)
            time.sleep(0.1) # 10 Hz

    def run_spin_loop(self):
        """
        Draait de (gesimuleerde) ROS 2-callbacks op hoge frequentie.
        Dit haalt sensor-data op.
        """
        print("[ROSBridge] Spin-loop (ROS 2 -> StateBus) gestart op 100 Hz.")
        while self._running:
            self.node.spin_once()
            time.sleep(0.01) # 100 Hz

    def stop(self):
        self._running = False
        shutdown_ros()
        print("[ROSBridge] Gestopt.")

if __name__ == "__main__":
    bus = StateBus()
    bridge = ROSBridge(bus)

    # Start de twee loops in aparte threads
    pub_thread = threading.Thread(target=bridge.run_publish_loop, daemon=True)
    spin_thread = threading.Thread(target=bridge.run_spin_loop, daemon=True)

    pub_thread.start()
    spin_thread.start()

    try:
        print("ROSBridge draait. Simuleer een 'walk' commando voor 3 seconden...")
        bus.set_value("robot_action", "walk_forward")
        time.sleep(3)

        print("ROSBridge draait. Simuleer een 'stop' commando...")
        bus.set_value("robot_action", "monitor_sensors")
        time.sleep(5)

    except KeyboardInterrupt:
        print("[Main] Stop-signaal ontvangen...")
    finally:
        bridge.stop()
        print("[Main] Wacht op threads...")
        pub_thread.join(timeout=1)
        spin_thread.join(timeout=1)
        print("[Main] Afgesloten.")