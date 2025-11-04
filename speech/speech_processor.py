import time
import os
import random
import threading 
from core.statebus import StateBus
from core.config_manager import ConfigManager 

# Spraakverwerking is trager dan de sensorloop (bijvoorbeeld 5 Hz)
SPEECH_LOOP_HZ = 5 
LOOP_INTERVAL = 1.0 / SPEECH_LOOP_HZ

class SpeechProcessor:
    """
    Fase B.3: Centrale module voor spraakverwerking (ASR, TTS, NLU).
    Gebruikt Vosk/Piper/Rasa in de uiteindelijke implementatie.
    """
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.config_manager = ConfigManager()
        self._running = True
        
        # Leest configuratie voor spraakparameters (Fase B.0)
        self.pitch = self.config_manager.get_setting("SPEECH_PITCH")
        self.tone = self.config_manager.get_setting("SPEECH_TONE")
        print(f"SpeechProcessor (Fase B.3) geïnitialiseerd. Huidige toon: {self.tone} (Pitch: {self.pitch}).")

    def _simulate_vosk_asr(self) -> str:
        """Simuleert Vosk ASR (Speech-to-Text)."""
        # In een echte implementatie leest deze de microfoon en Vosk model.
        if random.random() < 0.1: # 10% kans op een spraakcommando
            return random.choice(["loop naar voren", "wat is jouw naam", "ben je moe"])
        return ""

    def _simulate_rasa_nlu(self, text: str) -> dict:
        """Simuleert Rasa NLU (Natural Language Understanding) of Intent Engine."""
        if not text:
            return {"intent": "none", "confidence": 1.0}

        # Stuurt spraak naar de intent_engine (later te implementeren)
        if "loop" in text:
            return {"intent": "move_command", "confidence": 0.95, "action": "forward"}
        if "naam" in text:
            return {"intent": "query_name", "confidence": 0.88}
        
        return {"intent": "unknown", "confidence": 0.5}

    def _simulate_piper_tts(self, response: str):
        """Simuleert Piper TTS (Text-to-Speech) op basis van configuratie."""
        if response:
            print(f"[TTS - {self.tone}, Pitch={self.pitch:.1f}] Zegt: '{response}'")

    def run_speech_loop(self):
        """
        De hoofdloop voor het verwerken van spraak:
        Luisteren -> NLU -> Response genereren -> Spreken.
        """
        print(f"[SpeechProcessor] Loop gestart op {SPEECH_LOOP_HZ} Hz.")
        
        while self._running:
            start_time = time.time()
            
            # 1. Spraak Herkennen (ASR)
            transcribed_text = self._simulate_vosk_asr()
            
            if transcribed_text:
                print(f"\n[ASR] Gehoord: {transcribed_text}")
                
                # 2. Intent Herkennen (NLU)
                nlu_result = self._simulate_rasa_nlu(transcribed_text)
                
                # Schrijf intentie naar de StateBus (voor de Mode Arbiter)
                self.statebus.set_value("latest_intent", nlu_result)
                print(f"[NLU] Intentie: {nlu_result['intent']} (Confidence: {nlu_result['confidence']:.2f})")
                
                # 3. Response Genereren (TTS)
                response = self._generate_response(nlu_result)
                
                # 4. Spraak Synthese (TTS)
                self._simulate_piper_tts(response)

            # Synchronisatie
            elapsed = time.time() - start_time
            sleep_time = LOOP_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _generate_response(self, nlu_result: dict) -> str:
        """Eenvoudige logica om een antwoord te genereren op basis van de intentie."""
        intent = nlu_result['intent']
        if intent == "move_command":
            return "Ik beweeg nu. Waar moet ik naartoe?"
        if intent == "query_name":
            return "Mijn naam is Spot AI, en ik ben hier om te leren."
        if intent == "unknown":
            return "Dat heb ik niet begrepen. Kunt u dat herhalen?"
        return ""

    def stop(self):
        self._running = False
        print("[SpeechProcessor] Gestopt.")

if __name__ == "__main__":
    bus = StateBus()
    processor = SpeechProcessor(bus)
    
    t = threading.Thread(target=processor.run_speech_loop)
    t.start()
    
    # Laat de processor 5 seconden draaien
    time.sleep(5) 
    
    processor.stop()
    t.join()
    
    print(f"Laatste intentie in StateBus: {bus.get_value('latest_intent')}")