import time
import json
import sys
import os
import threading

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.statebus import StateBus
from core.config_manager import ConfigManager

# Importeer onze nieuwe, aparte drivers
from speech.asr_driver import ASRDriver
from speech.tts_driver import TTSDriver

# Mock-import voor de RasaClient (deze blijft zoals hij was)
class RasaClient:
    def get_intent(self, text):
        if "loop naar voren" in text.lower():
            return {"intent": "move_command", "confidence": 0.95, "action": "forward"}
        if "hoi spot" in text.lower() or "wat is je naam" in text.lower():
            return {"intent": "query_name", "confidence": 0.85, "action": "answer_name"}
        return {"intent": "offline_fallback", "confidence": 0.7, "action": "answer_offline_fallback"}


class SpeechProcessor:
    """
    Fase B.3: Orchestrator voor spraak (ASR, NLU, TTS).
    Leest de robot_action en audio-input, en stuurt events naar de StateBus.
    """
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager()
        self._running = True
        
        # Initialisatie van ECHTE (gesimuleerde) drivers
        try:
            self.asr = ASRDriver()
            self.tts = TTSDriver()
        except Exception as e:
            print(f"[SpeechProcessor] FATALE FOUT: Kon spraakdrivers niet laden: {e}")
            self.asr = None
            self.tts = None
            return

        self.nlu = RasaClient() # NLU blijft voor nu de mock
        
        print("SpeechProcessor (Fase B.3) geïnitialiseerd (met ASR/TTS-drivers).")

    def _execute_speech_action(self, action: str):
        """
        Voert de spraakactie uit die door de Mode Arbiter is gekozen.
        """
        if not self.tts:
            return

        if action == "answer_name":
            response_text = "Mijn naam is Spot-AI. Ik ben een open-source sociale robot."
            self.tts.synthesize(response_text)
            
        elif action == "answer_offline_fallback":
            response_text = "Ik ben momenteel offline, maar ik luister. Vraag iets anders!"
            self.tts.synthesize(response_text)
            
        elif action == "speak_agent_response":
            gpt_response = self.statebus.get_value("gpt_response_text", "Ik heb geen reactie ontvangen.")
            self.tts.synthesize(gpt_response)

    def run_speech_loop(self):
        """De hoofdloop van de SpeechProcessor."""
        if not self.asr or not self.tts:
            print("[SpeechProcessor] Kan loop niet starten, drivers niet geladen.")
            return

        print("[SpeechProcessor] Spraak-loop gestart.")
        
        LOOP_INTERVAL = 0.2 # Luister 5x per seconde
        
        while self._running:
            start_time = time.time()
            
            # 1. Spraakherkenning (ASR)
            # De ASRDriver handelt de audio-input en detectie intern af
            transcription = self.asr.listen_for_audio()
            
            if transcription:
                # 2. Intentie Bepaling (NLU)
                intent_data = self.nlu.get_intent(transcription)
                
                # 3. Schrijf de intentie naar de StateBus
                self.statebus.set_value("latest_intent", intent_data)
                
            # 4. Spraakuitvoer (TTS) - Luister naar de Mode Arbiter
            robot_action = self.statebus.get_value("robot_action")
            self._execute_speech_action(robot_action)
            
            # 5. Synchronisatie
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        self._running = False
        if self.asr:
            self.asr.stop()
        print("[SpeechProcessor] Gestopt.")

if __name__ == "__main__":
    bus = StateBus()
    bus.set_value("robot_action", "answer_name")
    
    processor = SpeechProcessor(bus)
    
    if processor.asr:
        t = threading.Thread(target=processor.run_speech_loop)
        t.start()
        
        # Laat 10 seconden draaien om spraak te simuleren
        print("SpeechProcessor test loopt 10s... (wacht op gesimuleerde spraak)")
        time.sleep(10) 
        
        processor.stop()
        t.join()
        
        print(f"Test voltooid. Laatste Intentie geschreven: {bus.get_value('latest_intent')}")
    else:
        print("Test mislukt, processor kon niet initialiseren.")