import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import shutil
import sys
import threading

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.config_manager import ConfigManager

# --- Configuratie ---
config = ConfigManager()
PIPER_MODEL_PATH = config.get_setting("piper_model_path", "models/piper-model.onnx")
SPEAKER_ID = config.get_setting("tts_speaker_id", 0)
# Het audio output device. Gebruik 'aplay -l' om de juiste naam te vinden.
AUDIO_DEVICE = config.get_setting("tts_audio_device", "default")

class PiperTTS:
    """ Robuuste implementatie van Piper TTS via de command-line. """
    def __init__(self, model_path, speaker_id, audio_device, logger):
        self.logger = logger
        self.model_path = model_path
        self.speaker_id = str(speaker_id)
        self.audio_device = audio_device
        self.is_speaking = threading.Lock() # Lock om te voorkomen dat meerdere zinnen tegelijk worden uitgesproken

        # Controleer of het model bestaat
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Piper-model niet gevonden op: '{self.model_path}'. Controleer 'piper_model_path' in config.json.")

        # Controleer of de benodigde executables bestaan
        if not shutil.which("piper"):
            raise FileNotFoundError("'piper' executable niet gevonden. Zorg ervoor dat Piper TTS correct geïnstalleerd is.")
        if not shutil.which("aplay"):
             raise FileNotFoundError("'aplay' executable niet gevonden. Zorg ervoor dat ALSA utilities (alsa-utils) geïnstalleerd zijn.")

        self.logger.info(f"PiperTTS (Stem) geïnitialiseerd met model '{os.path.basename(self.model_path)}' en speaker ID {self.speaker_id}.")

    def speak(self, text_to_speak):
        """ Genereert en speelt audio af. Blokkeert tot het afspelen is voltooid. """
        if self.is_speaking.locked():
            self.logger.warn("TTS is al bezig met spreken, negeer nieuw verzoek.")
            return

        with self.is_speaking:
            self.logger.info(f"Genereert audio voor: '{text_to_speak}'")
            output_file = "/tmp/spotai_tts_output.wav"
            
            try:
                # 1. Genereer audio
                piper_process = subprocess.run(
                    ['piper', '-m', self.model_path, '-s', self.speaker_id, '-f', output_file],
                    input=text_to_speak,
                    text=True,
                    check=True,
                    capture_output=True # Vang stdout/stderr op
                )
                if piper_process.stderr:
                    self.logger.warn(f"[Piper stderr]: {piper_process.stderr}")

                # 2. Speel audio af
                aplay_process = subprocess.run(
                    ['aplay', '-D', self.audio_device, output_file],
                    check=True,
                    capture_output=True
                )
                if aplay_process.stderr:
                    self.logger.warn(f"[aplay stderr]: {aplay_process.stderr}")

                self.logger.info("Audio succesvol afgespeeld.")

            except FileNotFoundError:
                # Dit zou niet moeten gebeuren na de __init__ check, maar voor de zekerheid
                self.logger.error(f"'{output_file}' niet gevonden. Is de audio generatie mislukt?")
            except subprocess.CalledProcessError as e:
                self.logger.error(f"Fout tijdens TTS-proces: {e.stderr}")
            except Exception as e:
                self.logger.error(f"Onverwachte fout tijdens TTS-generatie of afspelen: {e}")
            finally:
                # 3. Opruimen
                if os.path.exists(output_file):
                    os.remove(output_file)

class TtsNode(Node):
    """
    ROS 2 Node voor TTS (Tekst-naar-Spraak).
    Luistert naar een topic en spreekt de tekst uit met Piper.
    """
    def __init__(self):
        super().__init__("spotai_tts_node")
        self.get_logger().info("TTS Node (Stem) gestart.")
        self.tts_engine = None

        # --- Initialiseer Hardware/Modellen ---
        try:
            self.tts_engine = PiperTTS(PIPER_MODEL_PATH, SPEAKER_ID, AUDIO_DEVICE, self.get_logger())
        except FileNotFoundError as e:
            self.get_logger().error(f"KRITISCHE FOUT bij initialisatie: {e}")
            if rclpy.ok():
                rclpy.shutdown()
            return

        # --- ROS 2 Subscribers ---
        self.subscription = self.create_subscription(
            String,
            "/tts/speak", # Het topic waar we op luisteren
            self._speak_callback,
            10)
        self.get_logger().info(f"Luistert naar /tts/speak. Audio output: '{AUDIO_DEVICE}'.")

    def _speak_callback(self, msg):
        """ Wordt aangeroepen zodra er een bericht binnenkomt. """
        if not self.tts_engine:
            self.get_logger.error("TTS engine niet beschikbaar, kan verzoek niet verwerken.")
            return

        text = msg.data
        if text:
            # Voer de 'speak' methode uit in een aparte thread om de ROS 2 callback niet te blokkeren.
            speak_thread = threading.Thread(target=self.tts_engine.speak, args=(text,), daemon=True)
            speak_thread.start()

    def stop(self):
        self.get_logger().info("TTS Node stoppen...")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    tts_node = None
    try:
        tts_node = TtsNode()
        if rclpy.ok():
            rclpy.spin(tts_node)
    except KeyboardInterrupt:
        print("[Main] Stop-signaal ontvangen.")
    finally:
        if tts_node:
            tts_node.stop()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
