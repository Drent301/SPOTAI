import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
import vosk
import json
import queue
import os
import sys
import threading

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.config_manager import ConfigManager

# --- Configuratie ---
config = ConfigManager()
MODEL_PATH = config.get_setting("vosk_model_path", "models/vosk-model")
# De ID van de input device wordt nu ook centraal beheerd.
# Laat op None in JSON voor de default microfoon.
DEVICE_ID = config.get_setting("sound_device_id", None)
SAMPLE_RATE = 16000
BLOCK_SIZE = 8000 # Verhoogd voor minder frequente callbacks

class AsrNode(Node):
    """
    ROS 2 Node voor ASR (Spraak-naar-Tekst).
    Luistert naar een microfoon en publiceert de herkende tekst.
    """
    def __init__(self):
        super().__init__("spotai_asr_node")
        self.get_logger().info("ASR Node (Oren) gestart.")
        
        self.q = queue.Queue()
        self.stream = None # Initialiseer stream als None
        self._running = True
        
        # --- Initialiseer Hardware/Modellen ---
        try:
            # 1. Vosk Model
            if not os.path.exists(MODEL_PATH):
                raise FileNotFoundError(f"Vosk-model niet gevonden op: '{MODEL_PATH}'. Download een model en werk data/config.json bij.")
            self.model = vosk.Model(MODEL_PATH)
            self.recognizer = vosk.KaldiRecognizer(self.model, SAMPLE_RATE)
            self.get_logger().info(f"Vosk model '{MODEL_PATH}' succesvol geladen.")
            
            # 2. Microfoon (SoundDevice)
            if DEVICE_ID is None:
                self.get_logger().warn("Geen 'sound_device_id' ingesteld in config.json, gebruik default input.")
            else:
                self.get_logger().info(f"Gebruik sound device met ID: {DEVICE_ID}")

            self.stream = sd.RawInputStream(
                samplerate=SAMPLE_RATE, 
                blocksize=BLOCK_SIZE, 
                device=DEVICE_ID, 
                dtype='int16',
                channels=1, 
                callback=self._audio_callback
            )
            self.get_logger().info("Microfoon-stream geïnitialiseerd.")

        except Exception as e:
            self.get_logger().error(f"KRITISCHE FOUT bij hardware/model-initialisatie: {e}")
            if rclpy.ok():
                rclpy.shutdown()
            return

        # --- ROS 2 Publishers ---
        self.publisher_ = self.create_publisher(String, "/speech/asr_text", 10)
        self.get_logger().info("Publisher op /speech/asr_text aangemaakt.")

    def _audio_callback(self, indata, frames, time, status):
        """ Dit wordt aangeroepen door de audio-stream voor elk audio-blok. """
        if status:
            self.get_logger().warn(f"Audio stream status: {status}")
        if self._running:
            self.q.put(bytes(indata))

    def run_recognition_loop(self):
        """ De hoofdloop die audio verwerkt en publiceert. """
        if not self.stream or not rclpy.ok():
            self.get_logger().error("Node niet correct geïnitialiseerd. Herkenningsloop wordt niet gestart.")
            return

        self.get_logger().info("Starten met spraakherkenning...")
        self.stream.start()
        
        while rclpy.ok() and self._running:
            try:
                data = self.q.get(timeout=0.1)

                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(self.recognizer.FinalResult())
                    text = result.get('text', '')
                    
                    if text:
                        self.get_logger().info(f"Herkend: '{text}'")
                        msg = String()
                        msg.data = text
                        self.publisher_.publish(msg)

            except queue.Empty:
                pass # Ga door als er geen data is
            except Exception as e:
                self.get_logger().error(f"Fout in herkenningsloop: {e}")

    def stop(self):
        self.get_logger().info("ASR Node stoppen...")
        self._running = False
        if self.stream:
            self.stream.stop()
            self.stream.close()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    asr_node = None
    try:
        asr_node = AsrNode()
        if rclpy.ok():
            asr_node.run_recognition_loop()
    except KeyboardInterrupt:
        print("[Main] Stop-signaal ontvangen.")
    finally:
        if asr_node:
            asr_node.stop()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
