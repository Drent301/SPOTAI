import sys
import os
import json
import time
import threading

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.statebus import StateBus

# --- ECHTE ROS 2 IMPORTS ---
# De simulatiecode is verwijderd.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
# --- Einde ROS 2 Imports ---


class ROSBridge(Node): # <-- Erft nu direct van de ROS 2 Node
    """
    De centrale "Tolk" tussen de StateBus (ons brein) en ROS 2 (ons zenuwstelsel).
    """
    def __init__(self, statebus):
        self.bus = statebus
        self._running = True

        # Initialiseer de ECHTE ROS 2 Node
        super().__init__("spotai_ros_bridge")
        self.get_logger().info("ROS 2 Bridge Node (spotai_ros_bridge) gestart.")

        # --- 1. StateBus (Output) -> ROS 2 (Input) ---
        # We publiceren commando's (zoals /cmd_vel) naar ROS 2
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10) # <-- Gebruikt echt Twist-type

        # --- 2. ROS 2 (Output) -> StateBus (Input) ---
        # We abonneren op sensor-topics en schrijven ze naar de StateBus
        self.create_subscription(Imu, "/imu/data", self._imu_callback, 10) # <-- Gebruikt echt Imu-type
        self.create_subscription(JointState, "/joint_states", self._joint_states_callback, 10)

        self.get_logger().info("ROSBridge Succesvol geïnitialiseerd (publishers/subscribers gemaakt).")

    # --- Callbacks: Data van ROS 2 naar StateBus ---

    def _imu_callback(self, msg: Imu):
        """ Ontvangt IMU-data van ROS 2 en schrijft dit naar de StateBus. """
        # Vertaal ROS-msg (Imu object) naar StateBus-formaat
        accel_y = msg.linear_acceleration.y
        
        # TODO: Bereken hier de RMS-trilling (vibration_rms) indien nodig
        
        self.bus.set_value("imu_data", {"accel_y": accel_y})

    def _joint_states_callback(self, msg: JointState):
        """ Ontvangt motor-telemetrie van ROS 2 en schrijft dit naar de StateBus. """
        # Vertaal ROS-msg (JointState object) naar StateBus-formaat
        # Converteer numpy arrays (als die er zijn) naar standaard Python lijsten voor JSON
        state_dict = {
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort)
        }
        self.bus.set_value("joint_states", state_dict)

    # --- Publish Loop: Data van StateBus naar ROS 2 ---

    def run_publish_loop(self):
        """
        Leest de StateBus en publiceert de gewenste actie naar ROS 2.
        Draait op een lagere frequentie (10 Hz), want dit is de "denk"-snelheid.
        """
        self.get_logger().info("Publish-loop (StateBus -> ROS 2) gestart op 10 Hz.")
        while self._running:
            self.bus.reload_state()
            action = self.bus.get_value("robot_action", "monitor_sensors")

            # Vertaal StateBus-actie naar ROS 2 /cmd_vel bericht
            msg = Twist() # <-- Maak een echt Twist-bericht object

            if action == "walk_forward":
                msg.linear.x = 0.5 # Ga vooruit
            elif action == "turn_left":
                msg.angular.z = 0.8 # Draai linksom
            # Anders blijft msg (linear.x=0.0, angular.z=0.0) wat "stop" betekent

            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1) # 10 Hz
        
        self.get_logger().info("Publish-loop gestopt.")


    def run_spin_loop(self):
        """
        Draait de ROS 2-callbacks (spin) op hoge frequentie.
        Dit haalt sensor-data op.
        """
        self.get_logger().info("Spin-loop (ROS 2 -> StateBus) gestart.")
        while self._running and rclpy.ok():
            # rclpy.spin_once verwerkt alle wachtende callbacks
            rclpy.spin_once(self, timeout_sec=0.01) # ~100 Hz
        
        self.get_logger().info("Spin-loop gestopt.")

    def stop(self):
        self._running = False
        self.get_logger().info("Stop-signaal ontvangen. Node wordt afgesloten.")
        self.destroy_node() # Maak de node resources vrij

if __name__ == "__main__":
    rclpy.init() # Initialiseer ROS 2
    
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
        rclpy.shutdown() # Sluit ROS 2 netjes af