import os
import sys
import json
import time
import random
from typing import Dict, Any, Optional

# Voeg de hoofdmap toe
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.statebus import StateBus
from core.memory_manager import MemoryManager

# Simuleert het gebruik van de lokale LLM omgeving (Ollama + Llama-3)
class OllamaClient:
    """Mock-client voor de lokale Llama-3 LLM (llama-3-8b-instruct)."""
    def reflect(self, prompt: str) -> str:
        # Simuleer Llama-3 (8B) antwoordtijd van ~0.5 - 1.5 seconden
        time.sleep(random.uniform(0.5, 1.5))
        if "fout" in prompt.lower():
            return "Ik heb een onbekende fout gedetecteerd. Schakel over naar veilige modus."
        return "Ik kan deze vraag lokaal beantwoorden: De zon is een ster."

class FAISSCache:
    """Mock voor de FAISS vector database cache."""
    def __init__(self, memory_manager: MemoryManager):
        self.memory = memory_manager
        # In een echte implementatie zou hier een FAISS index geladen worden.
        self.context_cache: Dict[str, str] = {
            "wat is de beste manier om te leren": "Je leert het beste door herhaling en reflectie.",
            "waarom ben je traag": "Ik ben traag omdat de OBSTACLE_RESPONSE_SPEED op 1.2 staat."
        }

    def retrieve(self, query: str) -> Optional[str]:
        """Haalt relevant antwoord op uit de cache op basis van de query."""
        for key, value in self.context_cache.items():
            if key in query.lower():
                return value
        return None


class OfflineReflector:
    """
    Fase D.2: Beheert lokale LLM (Ollama) en FAISS cache voor snelle, offline antwoorden.
    """
    def __init__(self, state_bus: StateBus):
        self.bus = state_bus
        self.memory_manager = MemoryManager()
        self.ollama_client = OllamaClient()
        self.faiss_cache = FAISSCache(self.memory_manager)
        
        print("[OfflineReflector] Geïnitialiseerd. Ollama/FAISS is klaar.")

    def run_local_query(self, query: str) -> str:
        """
        Voert een offline query uit met een hybride aanpak (Cache -> LLM).
        """
        self.bus.set_value("last_local_query_ts", time.time())
        
        # 1. Probeer eerst de FAISS Cache
        cached_response = self.faiss_cache.retrieve(query)
        if cached_response:
            print("[OfflineReflector] Antwoord gevonden in FAISS Cache.")
            return cached_response
        
        # 2. Vraag de lokale LLM (Llama-3)
        print("[OfflineReflector] Cache mist. Vraag lokale LLM...")
        llm_response = self.ollama_client.reflect(query)
        
        # 3. Log de nieuwe interactie (voor toekomstige caching)
        self.memory_manager.log_preference(f"offline_query_{time.time()}", query)
        
        return llm_response

    def trigger_mini_reflection(self):
        """
        Voert een periodieke, zeer lichte reflectie uit over de laatste staat.
        """
        if self.bus.get_value("network_state") == "offline":
            self.bus.set_value("last_local_query_ts", time.time())
            print("[OfflineReflector] Offline Mini-reflectie gestart...")
            
            last_error = self.bus.get_value("last_error_log", "Geen fouten gevonden.")
            
            prompt = f"De laatste systeemsituatie is: {self.bus.get_value('robot_mode')}. Laatste fout: {last_error}. Hoe kan ik dit lokaal oplossen?"
            
            llm_response = self.ollama_client.reflect(prompt)
            
            print(f"[OfflineReflector] Llama-3 advies: {llm_response}")
            
            # Log de reflectie voor latere synchronisatie (Fase A.3)
            
            return llm_response
        return ""

# Test code voor lokale uitvoering
if __name__ == '__main__':
    bus = StateBus()
    reflector = OfflineReflector(bus)
    
    # TEST 1: Cache Hit
    print("\n--- TEST 1: Cache Hit (FAISS) ---")
    response_hit = reflector.run_local_query("wat is de beste manier om te leren")
    print(f"Antwoord (Cache Hit): {response_hit}")
    
    # TEST 2: LLM Miss
    print("\n--- TEST 2: LLM Call (Ollama) ---")
    response_miss = reflector.run_local_query("hoe snel moet ik lopen")
    print(f"Antwoord (LLM Miss): {response_miss}")
    
    # TEST 3: Mini-reflectie (vereist offline status)
    bus.set_value("network_state", "offline")
    bus.set_value("robot_mode", "error")
    print("\n--- TEST 3: Offline Reflectie ---")
    offline_advice = reflector.trigger_mini_reflection()
    print(f"Llama-3 Offline Advies: {offline_advice}")