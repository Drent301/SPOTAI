import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import shutil
import sys

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.config_manager import ConfigManager

# --- Configuratie ---
config = ConfigManager()
PIPER_MODEL_PATH = config.get_setting("piper_model_path", "models/piper-model.onnx")
SPEAKER_ID = 0
# Het audio output device. Gebruik 'aplay -l' om de juiste naam te vinden, bijv. 'plughw:1,0'.
AUDIO_DEVICE = "default"

class PiperTTS:
    """ Robuuste implementatie van Piper TTS via de command-line. """
    def __init__(self, logger):
        self.logger = logger

        # Controleer of het model bestaat
        if not os.path.exists(PIPER_MODEL_PATH):
            raise FileNotFoundError(f"Piper-model niet gevonden op: '{PIPER_MODEL_PATH}'.")

        # Controleer of de benodigde executables bestaan
        if not shutil.which("piper"):
            raise FileNotFoundError("'piper' executable niet gevonden. Zorg ervoor dat Piper TTS geïnstalleerd is.")
        if not shutil.which("aplay"):
             raise FileNotFoundError("'aplay' executable niet gevonden. Zorg ervoor dat ALSA utilities (alsa-utils) geïnstalleerd zijn.")

        self.logger.info("PiperTTS (Stem) succesvol geïnitialiseerd.")

    def speak(self, text_to_speak):
        self.logger.info(f"Piper TTS genereert audio voor: '{text_to_speak}'")
        try:
            # Dit is een gesimuleerde manier om Piper aan te roepen.
            # Het genereert een WAV-bestand, speelt het af met 'aplay', en verwijdert het.
            
            output_file = "/tmp/tts_output.wav"
            
            # 1. Genereer audio
            # (Dit vereist dat 'piper' in het systeem-PATH staat)
            subprocess.run(
                ['piper', '-m', PIPER_MODEL_PATH, '-s', str(SPEAKER_ID), '-f', output_file],
                input=text_to_speak,
                text=True,
                check=True
            )

            # 2. Speel audio af (op het specifieke device)
            # (Dit vereist dat 'aplay' geïnstalleerd is)
            subprocess.run(['aplay', '-D', AUDIO_DEVICE, output_file], check=True)
            
            # 3. Opruimen
            os.remove(output_file)
            
            self.logger.info("Audio afgespeeld.")

        except Exception as e:
            self.logger.error(f"Fout tijdens TTS-generatie of afspelen: {e}")

class TtsNode(Node):
    """
    ROS 2 Node voor TTS (Tekst-naar-Spraak).
    Luistert naar een topic en spreekt de tekst uit met Piper.
    """
    def __init__(self):
        super().__init__("spotai_tts_node")
        self.get_logger().info("TTS Node (Stem) gestart.")

        # --- Initialiseer Hardware/Modellen ---
        try:
            self.tts_engine = PiperTTS(self.get_logger())
        except FileNotFoundError as e:
            self.get_logger().error(f"Initialisatiefout: {e}")
            # Zorg ervoor dat de node niet verder start als de engine faalt
            self.tts_engine = None
            return

        # --- ROS 2 Subscribers ---
        self.subscription = self.create_subscription(
            String,
            "/tts/speak", # Het topic waar we op luisteren
            self._speak_callback,
            10)
        self.get_logger().info("Luistert naar /tts/speak topic.")

    def _speak_callback(self, msg):
        """ Wordt aangeroepen zodra er een bericht binnenkomt op /tts/speak. """
        if self.tts_engine is None:
            self.get_logger().error("TTS engine is niet geïnitialiseerd, kan tekst niet uitspreken.")
            return

        text = msg.data
        if text:
            self.get_logger().info(f"Bericht ontvangen om te spreken: '{text}'")
            self.tts_engine.speak(text)

    def stop(self):
        self.get_logger().info("TTS Node stoppen...")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tts_node = TtsNode()
        # rclpy.spin() houdt de node levend om op berichten te wachten
        rclpy.spin(tts_node) 
    except KeyboardInterrupt:
        print("[Main] Stop-signaal ontvangen.")
    finally:
        if 'tts_node' in locals():
            tts_node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()