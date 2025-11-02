from fastapi import FastAPI, Request
import uvicorn
import time

app = FastAPI()

@app.post("/model/parse")
async def handle_parse(request: Request):
    """
    Simuleert het /model/parse endpoint van een Rasa NLU server.
    """
    try:
        data = await request.json()
        text = data.get("text", "").lower()
        
        print(f"[MockRasa] Verzoek ontvangen: '{text}'")
        
        # Simpele logica om een intent te bepalen
        intent_name = "intent_algemeen"
        if "weer" in text:
            intent_name = "intent_weer"
        elif "hallo" in text:
            intent_name = "intent_groet"
            
        # Dit is de datastructuur die de intent_engine verwacht
        response_json = {
            "intent": {
                "name": intent_name,
                "confidence": 0.95 
            },
            "entities": [],
            "text": text
        }
        
        print(f"[MockRasa] Antwoord: {intent_name}")
        return response_json
        
    except Exception as e:
        print(f"[MockRasa] Fout: {e}")
        return {"error": str(e)}, 500

@app.get("/")
def read_root():
    return {"status": "Mock Rasa NLU Server Draait"}

if __name__ == "__main__":
    print("--- Mock Rasa NLU Server starten op http://localhost:5005 ---")
    # De poort 5005 is de standaardpoort die Rasa gebruikt
    uvicorn.run(app, host="127.0.0.1", port=5005)
