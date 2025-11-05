import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

# --- Configuratie ---
PIPER_MODEL_PATH = "pad/naar/je/piper-model.onnx" # TODO: Vervang dit
SPEAKER_ID = 0 # TODO: Pas aan indien nodig voor je model
AUDIO_DEVICE = "default" # TODO: Pas aan naar je ReSpeaker output, bijv. 'plughw:1,0'

class PiperTTSMock:
    """ 
    Mock-klasse om Piper TTS te simuleren via de command-line.
    Vervang dit door de echte Piper Python library (indien beschikbaar) 
    of een robuustere subprocess-aanroep.
    """
    def __init__(self, logger):
        self.logger = logger
        self.logger.info("PiperTTSMock (Stem) geïnitialiseerd.")

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
        self.tts_engine = PiperTTSMock(self.get_logger())

        # --- ROS 2 Subscribers ---
        self.subscription = self.create_subscription(
            String,
            "/tts/speak", # Het topic waar we op luisteren
            self._speak_callback,
            10)
        self.get_logger().info("Luistert naar /tts/speak topic.")

    def _speak_callback(self, msg):
        """ Wordt aangeroepen zodra er een bericht binnenkomt op /tts/speak. """
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