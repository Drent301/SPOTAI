import time
import json
import threading
from core.statebus import StateBus
from core.config_manager import ConfigManager

# Mock-imports voor de speech-componenten
# In de definitieve Pi-setup zijn dit Vosk, Piper en een Rasa HTTP-client
class VoskASR:
    def recognize(self, audio_data):
        # Simuleert offline spraakherkenning
        if b"move forward" in audio_data:
            return "Loop naar voren."
        if b"hello" in audio_data:
            return "Hoi Spot."
        return None

class PiperTTS:
    def synthesize(self, text):
        # Simuleert vloeiende spraaksynthese
        print(f"[TTS] Synthesizing: '{text}'...")
        # Simuleer de latency van de spraakuitvoer
        time.sleep(len(text) / 20) 
        return b"Audio data for " + text.encode()

class RasaClient:
    def get_intent(self, text):
        # Simuleert lokale NLU (Natural Language Understanding)
        if "loop naar voren" in text.lower():
            return {"intent": "move_command", "confidence": 0.95, "action": "forward"}
        if "hoi spot" in text.lower() or "wat is je naam" in text.lower():
            return {"intent": "query_name", "confidence": 0.85, "action": "answer_name"}
        # Offline fallback voor onbekende zinnen
        return {"intent": "offline_fallback", "confidence": 0.7, "action": "answer_offline_fallback"}

class SpeechProcessor:
    """
    Fase B.3: De centrale orchestrator voor offline spraak (ASR, NLU, TTS).
    Leest de robot_action en voert de juiste spraak/taak uit.
    """
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager()
        self._running = True
        
        # Initialisatie van lokale componenten
        self.asr = VoskASR()
        self.tts = PiperTTS()
        self.nlu = RasaClient()
        
        # Microfooninstellingen
        self.mic_sensitivity = self.config_manager.get_setting("MICROFOONGEVOELIGHEID")
        self.vad_threshold = self.config_manager.get_setting("VAD_THRESHOLD")
        
        print("SpeechProcessor (Fase B.3) geïnitialiseerd (Vosk/Piper/Rasa).")

    def _listen_for_audio(self):
        """Simuleert continu luisteren via ReSpeaker HAT."""
        # Dit zou in een aparte thread/loop op de Pi draaien.
        print("[ASR] Luistert op de achtergrond...")
        # Simuleer luister-events: een commando na 5 seconden.
        time.sleep(5) 
        # Simuleer ontvangen audiobuffer (met commando)
        return b"Some background noise... move forward ... end of speech."
        
    def _execute_speech_action(self, action: str):
        """
        Voert de actie uit die door de Mode Arbiter is gekozen (via StateBus).
        """
        if action == "answer_name":
            response_text = "Mijn naam is Spot-AI. Ik ben een open-source sociale robot."
            self.tts.synthesize(response_text)
            
        elif action == "answer_offline_fallback":
            response_text = "Ik ben momenteel offline, maar ik luister. Vraag iets anders!"
            self.tts.synthesize(response_text)
            
        elif action == "speak_agent_response":
            # Wordt gebruikt als de GPT Agent een antwoord heeft gegenereerd
            gpt_response = self.statebus.get_value("gpt_response_text", "Ik heb geen reactie ontvangen van de cognitieve kern.")
            self.tts.synthesize(gpt_response)
            
        elif action == "monitor_sensors":
            pass # Geen spraakuitvoer, enkel monitoring
            
        else:
            # Onbekende acties (bijv. motoriek) worden hier genegeerd
            pass

    def run_speech_loop(self):
        """De hoofdloop van de SpeechProcessor."""
        LOOP_INTERVAL = 1.0 / 5.0 # 5 Hz loop
        
        while self._running:
            start_time = time.time()
            
            # 1. Spraakherkenning (ASR)
            raw_audio = self._listen_for_audio() 
            transcription = self.asr.recognize(raw_audio)
            
            if transcription:
                # 2. Intentie Bepaling (NLU)
                intent_data = self.nlu.get_intent(transcription)
                
                # 3. Schrijf de intentie naar de StateBus voor de IntentEngine (B.5a)
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
        print("[SpeechProcessor] Gestopt.")

if __name__ == "__main__":
    # Test vereist dat de IntentEngine de final actie bepaalt, maar hier focussen we op de Speech-loop zelf.
    bus = StateBus()
    
    # Simuleer een actie die de Arbiter zou kunnen kiezen
    bus.set_value("robot_action", "answer_name")
    
    processor = SpeechProcessor(bus)
    
    t = threading.Thread(target=processor.run_speech_loop)
    t.start()
    
    time.sleep(7) # Geef tijd voor luisteren en spreken
    
    # De processor heeft nu de intentie van "move forward" naar de StateBus geschreven
    print(f"Laatste Intentie geschreven: {bus.get_value('latest_intent')}")
    
    processor.stop()
    t.join()