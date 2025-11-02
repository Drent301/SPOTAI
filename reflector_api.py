import os
import time
from fastapi import FastAPI, HTTPException, Request
from openai import OpenAI
import logging
from dotenv import load_dotenv

# Laad de variabelen uit het .env bestand (zoals OPENAI_API_KEY)
load_dotenv()

# --- Configuratie ---
# De key wordt nu VEILIG uit de .env file geladen
API_KEY = os.environ.get("OPENAI_API_KEY")

try:
    client = OpenAI(api_key=API_KEY)
    if not API_KEY:
        raise ValueError("OPENAI_API_KEY niet gevonden in .env file.")
except Exception as e:
    print(f"FOUT: Kon OpenAI client niet initialiseren: {e}")
    client = None

# Logging instellen
logging.basicConfig(level=logging.INFO, filename='reflector_api.log',
                    format='%(asctime)s %(levelname)s:%(message)s')

app = FastAPI()
print("Reflector API (Fase 6) gestart.")
logging.info("Reflector API (Fase 6) gestart.")

# --- Kosten Log Functie (Fase 8) ---
def log_cost(model, input_tokens, output_tokens):
    """Logt de kosten van een GPT call, gebaseerd op DOC-20251102-WA0003."""
    # Kostenstructuur
    costs = {
        "gpt-3.5-turbo": (0.0005, 0.0015),
        "gpt-4-mini": (0.005, 0.015),
        "gpt-4o": (0.01, 0.03),
        "gpt-5": (0.02, 0.06), 
    }
    
    if model not in costs:
        logging.warning(f"Kosten voor model {model} onbekend.")
        return

    in_cost, out_cost = costs[model]
    total_cost = ((input_tokens / 1000) * in_cost) + ((output_tokens / 1000) * out_cost)
    
    # Log de kosten
    logging.info(f"COST_LOG: model={model}, input_tokens={input_tokens}, "
                 f"output_tokens={output_tokens}, total_cost=${total_cost:.6f}")
    return total_cost

# --- API Endpoint ---
@app.post("/reflect")
async def handle_reflection(request: Request):
    """
    Ontvangt data van de Pi (gpt_agent.py), stuurt het naar GPT
    voor reflectie, en stuurt advies terug.
    """
    if not client:
        logging.error("OpenAI client niet geïnitialiseerd.")
        raise HTTPException(status_code=500, detail="OpenAI API key niet geconfigureerd op server.")
        
    try:
        data = await request.json()
        logs = data.get("logs", "")
        state = data.get("state", {})
        model = data.get("model", "gpt-4o") # Client stuurt gpt-4o

        logging.info(f"Reflectie-aanvraag ontvangen. Model: {model}. Log-lengte: {len(logs)}")

        # Systeemprompt voor de GPT-agent (Aangepast voor set_config test)
        system_prompt = """
        Jij bent de cognitieve kern (GPT-agent) voor een Spot-AI robot.
        Je analyseert logs en robotstatus. Je doel is reflecteren en
        concrete verbeteringen voorstellen in JSON-formaat.
        
        Je output MOET een JSON-object zijn met 'analyse' (je gedachten) 
        en 'advies' (een concreet commando).

        De logs die je nu ziet ('obstakel_gezien') tonen dat de
        obstakeldetectie werkt, maar traag reageert (standaard 1.2s).
        Je MOET adviseren om dit sneller te maken.

        Beschikbare commando's:
        - set_config(key, value)
        - restart_service(service_name)
        - enable_network()

        Voorbeeld:
        {
            "analyse": "De logs tonen 1.2s latency bij obstakeldetectie. Dit is te traag.",
            "advies": "set_config('OBSTACLE_RESPONSE_SPEED', 0.8)"
        }
        """

        # Roep de OpenAI API aan
        start_time = time.time()
        response = client.chat.completions.create(
            model=model,
            response_format={"type": "json_object"},
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Analyseer de volgende data:\nSTATE:\n{state}\n\nLOGS:\n{logs}"}
            ]
        )
        
        latency = time.time() - start_time
        logging.info(f"OpenAI API call voltooid in {latency:.2f}s")

        # Verwerk en log kosten
        if response.usage:
            log_cost(model, response.usage.prompt_tokens, response.usage.completion_tokens)
        
        gpt_result_json = response.choices[0].message.content
        
        return {"status": "success", "reflection": gGpt_result_json, "latency_ms": latency * 1000}

    except Exception as e:
        logging.error(f"Fout tijdens reflectie: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/")
def read_root():
    return {"project": "Spot-AI Reflector API", "status": "online"}