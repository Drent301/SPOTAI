import rclpy
from rclpy.node import Node
import cv2
import json
import time

# --- Echte Hardware Imports ---
# (Deze moeten geïnstalleerd zijn op de Pi 5)
# import hailo_platform 
# import numpy as np
# from hailo_sdk_client import ClientRunner

# Importeer de ROS 2 standaard berichttypes
from sensor_msgs.msg import Image # Voor het publiceren van ruwe beelden
from std_msgs.msg import String # We gebruiken een JSON-string voor de detecties

class HailoRunnerMock:
    """ 
    Mock-klasse om de Hailo-inferentie te simuleren. 
    Vervang dit door de echte hailo_sdk_client.ClientRunner.
    """
    def __init__(self):
        print("[VisionNode] HailoRunnerMock geïnitialiseerd.")
    
    def infer(self, frame):
        # Simuleer dat het een 'persoon' detecteert
        time.sleep(0.05) # Simuleer inferentietijd
        return [
            {
                "type": "gezicht", 
                "confidence": 0.95, 
                "bbox": [100, 150, 200, 350],
                "emotie": "blij" # TODO: Voeg emotie-herkenning toe
            }
        ]

class VisionNode(Node):
    """
    ROS 2 Node voor perceptie.
    Leest de camera, gebruikt de Hailo-8 chip voor detecties, en publiceert de resultaten.
    """
    def __init__(self):
        super().__init__("spotai_vision_node")
        self.get_logger().info("Vision Node gestart.")

        # --- Initialiseer Hardware ---
        try:
            # 1. Camera (OpenCV)
            self.capture = cv2.VideoCapture(0)
            if not self.capture.isOpened():
                raise IOError("Kan camera 0 niet openen.")
            self.get_logger().info("Camera succesvol geïnitialiseerd.")

            # 2. Hailo AI Accelerator
            # TODO: Vervang dit door de echte Hailo initialisatie
            self.hailo_device = HailoRunnerMock() 
            self.get_logger().info("Hailo-8 AI accelerator succesvol geïnitialiseerd.")

        except Exception as e:
            self.get_logger().error(f"Hardware-initialisatiefout: {e}")
            return

        # --- ROS 2 Publishers ---
        # 1. Publisher voor ruwe beelden (optioneel, voor debuggen)
        self.image_pub = self.create_publisher(Image, "/vision/raw_image", 10)
        
        # 2. Publisher voor de detecties (als JSON-string)
        # De ROSBridge moet hierop abonneren (of de IntentEngine direct)
        self.detections_pub = self.create_publisher(String, "/vision/detections", 10)
        
        self.get_logger().info("ROS 2 publishers aangemaakt.")

    def run_inference_loop(self):
        """ De hoofdloop die frames leest en inferentie uitvoert. """
        self.get_logger().info("Inference loop gestart...")
        
        while rclpy.ok():
            try:
                ret, frame = self.capture.read()
                if not ret:
                    self.get_logger().warn("Frame kon niet worden gelezen, probeer opnieuw...")
                    time.sleep(0.5)
                    continue

                # --- Stap 1: Inferentie (Hailo) ---
                # (In een echte setup wil je dit misschien in een aparte thread)
                detections = self.hailo_device.infer(frame)

                if detections:
                    # --- Stap 2: Publiceer Detecties ---
                    # Converteer de detectie-lijst naar een JSON-string
                    detection_msg = String()
                    detection_msg.data = json.dumps(detections)
                    
                    self.detections_pub.publish(detection_msg)
                    
                    # Log alleen als er iets wordt gezien
                    # self.get_logger().info(f"Gepubliceerd: {detection_msg.data}")

                # --- Stap 3: Publiceer Ruw Beeld (Optioneel) ---
                # (Dit kost veel bandbreedte, alleen aanzetten voor debuggen)
                # self.image_pub.publish(...) 
                
                # (We hebben geen spin_once nodig als we alleen publiceren)
                
            except Exception as e:
                self.get_logger().error(f"Fout in inference loop: {e}")
                time.sleep(1)

    def stop(self):
        self.get_logger().info("Vision Node stoppen...")
        self.capture.release()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        vision_node = VisionNode()
        vision_node.run_inference_loop() # Deze functie is de blokkerende loop
    except KeyboardInterrupt:
        print("[Main] Stop-signaal ontvangen.")
    finally:
        if 'vision_node' in locals():
            vision_node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()