import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
import vosk
import json
import queue
import os

# --- Configuratie ---
# Het pad naar het Vosk-model. Download een model van https://alphacephei.com/vosk/models
# en plaats het in een 'models' map in de root van dit project.
# Voorbeeld: 'models/vosk-model-small-nl-0.22'
MODEL_PATH = "models/vosk-model" # TODO: Zorg ervoor dat dit pad klopt!

# De ID van de input device. Laat op None voor de default microfoon.
# Om de juiste ID te vinden, run: python3 -m sounddevice
DEVICE_ID = None
SAMPLE_RATE = 16000
BLOCK_SIZE = 8000

class AsrNode(Node):
    """
    ROS 2 Node voor ASR (Spraak-naar-Tekst).
    Luistert naar een microfoon en publiceert de herkende tekst.
    """
    def __init__(self):
        super().__init__("spotai_asr_node")
        self.get_logger().info("ASR Node (Oren) gestart.")
        
        self.q = queue.Queue()
        
        # --- Initialiseer Hardware/Modellen ---
        try:
            # 1. Vosk Model
            if not os.path.exists(MODEL_PATH):
                raise FileNotFoundError(f"Vosk-model niet gevonden op: '{MODEL_PATH}'. Download een model en update het pad.")
            self.model = vosk.Model(MODEL_PATH)
            self.get_logger().info(f"Vosk model '{MODEL_PATH}' succesvol geladen.")
            
            # 2. Microfoon (SoundDevice)
            if DEVICE_ID is None:
                self.get_logger().warn("Geen DEVICE_ID ingesteld, gebruik default input.")
            
            self.stream = sd.RawInputStream(
                samplerate=SAMPLE_RATE, 
                blocksize=BLOCK_SIZE, 
                device=DEVICE_ID, 
                dtype='int16',
                channels=1, 
                callback=self._audio_callback
            )
            self.recognizer = vosk.KaldiRecognizer(self.model, SAMPLE_RATE)
            self.get_logger().info("Microfoon-stream geïnitialiseerd.")

        except Exception as e:
            self.get_logger().error(f"Hardware/Model-initialisatiefout: {e}")
            return

        # --- ROS 2 Publishers ---
        self.publisher_ = self.create_publisher(String, "/speech/asr_text", 10)
        self.get_logger().info("Publisher op /speech/asr_text aangemaakt.")

    def _audio_callback(self, indata, frames, time, status):
        """ Dit wordt aangeroepen door de audio-stream voor elk audio-blok. """
        if status:
            self.get_logger().warn(f"Audio stream status: {status}")
        self.q.put(bytes(indata))

    def run_recognition_loop(self):
        """ De hoofdloop die audio verwerkt en publiceert. """
        self.get_logger().info("Starten met spraakherkenning...")
        self.stream.start()
        
        while rclpy.ok():
            try:
                data = self.q.get()
                if not data:
                    time.sleep(0.01)
                    continue

                if self.recognizer.AcceptWaveform(data):
                    # Volledige zin herkend
                    result = self.recognizer.FinalResult()
                    result_json = json.loads(result)
                    text = result_json.get('text', '')
                    
                    if text:
                        self.get_logger().info(f"Herkend: '{text}'")
                        msg = String()
                        msg.data = text
                        self.publisher_.publish(msg)
                else:
                    # Tussentijds resultaat
                    # partial = json.loads(self.recognizer.PartialResult())
                    # self.get_logger().info(f"Partial: {partial.get('partial')}")
                    pass
            
            except queue.Empty:
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Fout in herkenningsloop: {e}")

    def stop(self):
        self.get_logger().info("ASR Node stoppen...")
        self.stream.stop()
        self.stream.close()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        asr_node = AsrNode()
        asr_node.run_recognition_loop() # Blokkerende loop
    except KeyboardInterrupt:
        print("[Main] Stop-signaal ontvangen.")
    finally:
        if 'asr_node' in locals():
            asr_node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()