import rclpy
from rclpy.node import Node
import cv2
import json
import time
import os
import sys

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.config_manager import ConfigManager

# --- Echte Hardware Imports ---
# Zorg ervoor dat de Hailo SDK correct geïnstalleerd is
try:
    import hailo.hailortcli as hailort
    from hailo_sdk_client import ClientRunner
    HAILO_ENABLED = True
except ImportError:
    print("[VisionNode] WAARSCHUWING: Hailo SDK niet gevonden. Kan de VisionNode niet starten.")
    HAILO_ENABLED = False

# Importeer de ROS 2 standaard berichttypes
from sensor_msgs.msg import Image
from std_msgs.msg import String

# --- CONFIGURATIE ---
# Het pad naar het Hailo-model wordt nu centraal beheerd in data/config.json
config = ConfigManager()
HAILO_HEF_PATH = config.get_setting("hailo_model_path", "models/default.hef")


class HailoRunner:
    """ Echte implementatie van de Hailo-inferentie. """
    def __init__(self, hef_path):
        if not HAILO_ENABLED:
            raise ImportError("De Hailo SDK is niet geïnstalleerd of beschikbaar in de Python-omgeving.")

        print("[VisionNode] Initialiseren van Hailo ClientRunner...")

        # Controleer of het HEF-bestand bestaat
        if not os.path.exists(hef_path):
            raise FileNotFoundError(f"HEF-bestand niet gevonden op: {hef_path}")

        # Scan voor Hailo-apparaten
        scan_result = hailort.scan()
        if not scan_result:
            raise RuntimeError("Geen Hailo-apparaat gevonden.")

        device_id = scan_result[0]['device_id']
        print(f"[VisionNode] Hailo-apparaat gevonden met ID: {device_id}")

        # Laad het HEF-model
        self.runner = ClientRunner(hef_path=hef_path, device_ids=[device_id])
        print("[VisionNode] Hailo ClientRunner succesvol geladen.")

    def infer(self, frame):
        """ Voert inferentie uit en formatteert de output. """
        # 1. Pre-processing: Converteer en resize het frame
        # De Hailo-runner verwacht vaak een specifiek formaat (bijv. RGB) en dimensie.
        input_shape = self.runner.get_input_shapes()[0][1:3] # (height, width)
        resized_frame = cv2.resize(frame, (input_shape[1], input_shape[0]))
        rgb_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)

        # 2. Inferentie
        # Stuur het frame naar de Hailo-8. De `infer` methode verwacht een lijst van frames.
        raw_results = self.runner.infer([rgb_frame])

        # 3. Post-processing: Vertaal de ruwe output naar een bruikbaar formaat.
        # Dit is sterk afhankelijk van het model, maar we simuleren een standaard objectdetectie-output.
        detections = []
        for result in raw_results:
            # Voorbeeld-structuur; pas aan op basis van je model-output.
            # Stel dat 'result' een numpy array is met [class_id, confidence, x_min, y_min, x_max, y_max]
            for detection in result:
                confidence = detection[1]
                if confidence > 0.5: # Filter op betrouwbaarheid
                    box = detection[2:6]
                    # Schaal de bounding box terug naar de originele framegrootte
                    original_h, original_w, _ = frame.shape
                    scaled_box = [
                        int(box[0] * original_w),
                        int(box[1] * original_h),
                        int(box[2] * original_w),
                        int(box[3] * original_h)
                    ]

                    detections.append({
                        "type": "gezicht", # Placeholder-label
                        "confidence": float(confidence),
                        "bbox": scaled_box,
                        "emotie": "blij" # Placeholder-emotie
                    })
        return detections

class VisionNode(Node):
    """
    ROS 2 Node voor perceptie.
    Leest de camera, gebruikt de Hailo-8 chip voor detecties, en publiceert de resultaten.
    """
    def __init__(self):
        super().__init__("spotai_vision_node")
        self.get_logger().info("Vision Node gestart.")
        self.hailo_device = None
        self.capture = None

        # --- Initialiseer Hardware ---
        try:
            # 1. Camera (OpenCV)
            self.capture = cv2.VideoCapture(0)
            if not self.capture.isOpened():
                raise IOError("Kan camera 0 niet openen.")
            self.get_logger().info("Camera succesvol geïnitialiseerd.")

            # 2. Hailo AI Accelerator
            self.hailo_device = HailoRunner(HAILO_HEF_PATH)
            self.get_logger().info("Hailo-8 AI accelerator succesvol geïnitialiseerd.")

        except (IOError, ImportError, FileNotFoundError, RuntimeError) as e:
            self.get_logger().error(f"KRITISCHE FOUT bij hardware-initialisatie: {e}")
            # Stop de node als hardware faalt
            if self.capture:
                self.capture.release()
            # Zet rclpy.ok() op False om de main loop te stoppen
            if rclpy.ok():
                rclpy.shutdown()
            return

        # --- ROS 2 Publishers ---
        self.detections_pub = self.create_publisher(String, "/vision/detections", 10)
        self.get_logger().info("ROS 2 publisher voor /vision/detections aangemaakt.")

    def run_inference_loop(self):
        """ De hoofdloop die frames leest en inferentie uitvoert. """
        if not rclpy.ok():
             self.get_logger().error("Node is niet correct geïnitialiseerd, inference loop wordt niet gestart.")
             return

        self.get_logger().info("Inference loop gestart...")
        while rclpy.ok():
            try:
                ret, frame = self.capture.read()
                if not ret:
                    self.get_logger().warn("Frame kon niet worden gelezen, probeer opnieuw...")
                    time.sleep(0.5)
                    continue

                # --- Stap 1: Inferentie ---
                detections = self.hailo_device.infer(frame)

                # --- Stap 2: Publiceer Detecties ---
                if detections:
                    detection_msg = String()
                    detection_msg.data = json.dumps(detections)
                    self.detections_pub.publish(detection_msg)

            except Exception as e:
                self.get_logger().error(f"Fout in inference loop: {e}")
                time.sleep(1)

    def stop(self):
        self.get_logger().info("Vision Node stoppen...")
        if self.capture:
            self.capture.release()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    vision_node = None
    try:
        vision_node = VisionNode()
        # Controleer of de node correct is geïnitialiseerd voordat de loop start
        if rclpy.ok():
            vision_node.run_inference_loop()
    except KeyboardInterrupt:
        print("[Main] Stop-signaal ontvangen.")
    finally:
        if vision_node:
            vision_node.stop()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
