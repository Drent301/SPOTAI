import os
import sys
import json
import time
import random
import logging
from typing import Dict, Any, Optional

# Voeg de hoofdmap toe
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.statebus import StateBus
from core.memory_manager import MemoryManager

# --- ECHTE IMPLEMENTATIE IMPORTS ---
try:
    import ollama
    OLLAMA_ENABLED = True
except ImportError:
    OLLAMA_ENABLED = False

try:
    import faiss
    from sentence_transformers import SentenceTransformer
    FAISS_ENABLED = True
except ImportError:
    FAISS_ENABLED = False

# --- CONFIGURATIE ---
OLLAMA_MODEL = "llama3:8b-instruct-q4_K_M" # Model voor lokale reflectie
OLLAMA_HOST = "http://localhost:11434"
FAISS_MODEL = 'all-MiniLM-L6-v2'
FAISS_CONFIDENCE_THRESHOLD = 0.75

# Basisconfiguratie voor logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- KLASSEN ---

class OllamaClient:
    """Echte client voor de lokale Llama-3 LLM via Ollama."""
    def __init__(self, logger_instance):
        self.logger = logger_instance
        if not OLLAMA_ENABLED:
            raise ImportError("De 'ollama' library is niet geïnstalleerd. Draai 'pip install ollama'.")

        try:
            self.client = ollama.Client(host=OLLAMA_HOST)
            self.client.list()
            self.logger.info(f"Verbonden met Ollama op {OLLAMA_HOST}.")
        except Exception as e:
            self.logger.error(f"Kon niet verbinden met Ollama op {OLLAMA_HOST}: {e}")
            raise ConnectionError(f"Ollama-verbinding gefaald.")

    def reflect(self, prompt: str) -> str:
        """Stuur een prompt naar het lokale LLM en retourneer het antwoord."""
        try:
            response = self.client.chat(
                model=OLLAMA_MODEL,
                messages=[{'role': 'user', 'content': prompt}],
                stream=False
            )
            return response['message']['content']
        except Exception as e:
            self.logger.error(f"Fout tijdens aanroep naar Ollama model '{OLLAMA_MODEL}': {e}")
            return "Er was een fout bij de lokale LLM-aanroep."

class FAISSCache:
    """Echte implementatie van de FAISS vector database cache."""
    def __init__(self, logger_instance, memory_manager: MemoryManager):
        self.logger = logger_instance
        self.memory = memory_manager
        if not FAISS_ENABLED:
            raise ImportError("Installeer 'faiss-cpu' en 'sentence-transformers'.")

        self.model = SentenceTransformer(FAISS_MODEL)
        self.dimension = self.model.get_sentence_embedding_dimension()
        self.index = faiss.IndexFlatL2(self.dimension)
        self.documents: Dict[int, Dict[str, str]] = {}
        self.next_id = 0

        self._initialize_cache()

    def _initialize_cache(self):
        """Vult de cache met initiële data."""
        initial_data = {
            "wat is de beste manier om te leren": "Je leert het beste door herhaling en reflectie.",
            "waarom ben je traag": "Ik ben traag omdat de OBSTACLE_RESPONSE_SPEED op 1.2 staat."
        }
        for question, answer in initial_data.items():
            self.add(question, answer)
        self.logger.info(f"FAISS cache geïnitialiseerd met {len(self.documents)} documenten.")

    def add(self, question: str, answer: str):
        """Voegt een vraag-antwoord paar toe aan de index."""
        embedding = self.model.encode([question])[0]
        self.index.add(embedding.reshape(1, -1))
        self.documents[self.next_id] = {"question": question, "answer": answer}
        self.logger.info(f"Document {self.next_id} toegevoegd aan FAISS cache: '{question}'")
        self.next_id += 1

    def retrieve(self, query: str) -> Optional[str]:
        """Zoekt naar de meest relevante vraag en geeft het antwoord terug."""
        if self.index.ntotal == 0:
            return None # Lege index

        query_embedding = self.model.encode([query])[0]
        distances, indices = self.index.search(query_embedding.reshape(1, -1), 1)

        if indices.size > 0:
            best_match_id = indices[0][0]
            distance = distances[0][0]

            similarity = 1 / (1 + distance)

            if similarity > FAISS_CONFIDENCE_THRESHOLD:
                self.logger.info(f"FAISS hit (id: {best_match_id}) met similariteit {similarity:.2f}.")
                return self.documents[best_match_id]["answer"]

        self.logger.info(f"FAISS miss voor query: '{query}'")
        return None

class OfflineReflector:
    """
    Fase D.2: Beheert lokale LLM (Ollama) en FAISS cache voor snelle, offline antwoorden.
    """
    def __init__(self, state_bus: StateBus, logger_instance=logger):
        self.bus = state_bus
        self.logger = logger_instance
        self.memory_manager = MemoryManager()
        self.is_initialized = False
        
        try:
            self.ollama_client = OllamaClient(self.logger)
            self.faiss_cache = FAISSCache(self.logger, self.memory_manager)
            self.logger.info("OfflineReflector succesvol geïnitialiseerd.")
            self.is_initialized = True
        except (ImportError, ConnectionError) as e:
            self.logger.warning(f"Initialisatie van OfflineReflector mislukt: {e}. Functionaliteit is uitgeschakeld.")
            self.ollama_client = None
            self.faiss_cache = None

    def run_local_query(self, query: str) -> str:
        """
        Voert een offline query uit met een hybride aanpak (Cache -> LLM).
        """
        if not self.is_initialized:
            return "Offline reflectie is niet beschikbaar."

        self.bus.set_value("last_local_query_ts", time.time())
        
        cached_response = self.faiss_cache.retrieve(query)
        if cached_response:
            return cached_response
        
        llm_response = self.ollama_client.reflect(query)
        
        # Voeg de nieuwe kennis toe aan de FAISS cache
        self.faiss_cache.add(query, llm_response)

        # Log ook in de algemene memory manager
        self.memory_manager.log_preference(f"offline_qa_{time.time()}", {"question": query, "answer": llm_response})
        
        return llm_response

    def trigger_mini_reflection(self):
        """
        Voert een periodieke, zeer lichte reflectie uit over de laatste staat.
        """
        if not self.is_initialized:
            return ""

        if self.bus.get_value("network_state") == "offline":
            self.bus.set_value("last_local_query_ts", time.time())
            self.logger.info("Offline Mini-reflectie gestart...")
            
            last_error = self.bus.get_value("last_error_log", "Geen fouten gevonden.")
            
            prompt = f"De laatste systeemsituatie is: {self.bus.get_value('robot_mode')}. Laatste fout: {last_error}. Hoe kan ik dit lokaal oplossen?"
            
            llm_response = self.ollama_client.reflect(prompt)
            
            self.logger.info(f"Llama-3 advies: {llm_response}")
            
            return llm_response
        return ""

# Test code voor lokale uitvoering
if __name__ == '__main__':
    bus = StateBus()
    reflector = OfflineReflector(bus)

    if reflector.is_initialized:
        # TEST 1: Cache Hit
        print("\n--- TEST 1: Cache Hit (FAISS) ---")
        response_hit = reflector.run_local_query("hoe leer ik het beste") # Query is anders, maar semantisch gelijk
        print(f"Antwoord (Cache Hit): {response_hit}")

        # TEST 2: LLM Miss, gevolgd door Cache Hit
        print("\n--- TEST 2: LLM Call (Ollama) & Cache vullen ---")
        query_miss = "wat is de hoofdstad van nederland"
        response_miss = reflector.run_local_query(query_miss)
        print(f"Antwoord (LLM Miss): {response_miss}")

        print("\n--- TEST 2b: Verificatie Cache Hit ---")
        response_verify = reflector.run_local_query(query_miss)
        print(f"Antwoord (Cache Verify): {response_verify}")

        # TEST 3: Mini-reflectie (vereist offline status)
        bus.set_value("network_state", "offline")
        bus.set_value("robot_mode", "error")
        print("\n--- TEST 3: Offline Reflectie ---")
        offline_advice = reflector.trigger_mini_reflection()
        print(f"Llama-3 Offline Advies: {offline_advice}")
    else:
        print("\nOfflineReflector kon niet worden geïnitialiseerd. Controleer Ollama-server en dependencies.")
