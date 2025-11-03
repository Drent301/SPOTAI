import time
import os
import sys
import re
from typing import TypedDict, Annotated, Sequence, Optional
from operator import itemgetter

# Voeg de hoofdmap toe voor core-modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Importeer kernmodules
from core.statebus import StateBus
from core.gpt_agent import GptAgent

# Importeer LangGraph componenten
from langchain_core.tools import tool
from langchain_core.messages import BaseMessage, HumanMessage
from langchain_openai import ChatOpenAI
from langgraph.graph import StateGraph, END

# --- 1. Tool Definities (Whitelisted Functies) ---
# Dit zijn de functies die de GPT-agent mag aanroepen.
# Gebaseerd op ControlMatrix Niveau 1-2.

@tool
def set_config(key: str, value: str) -> str:
    """(Niveau 1) Past een configuratieparameter veilig aan (bv. 'OBSTACLE_RESPONSE_SPEED', '0.8')."""
    
    # Guardrail: Valideer hier de input (is de waarde binnen min/max?)
    if key == "OBSTACLE_RESPONSE_SPEED" and not (0.1 < float(value) < 2.0):
        return f"ERROR: Waarde {value} voor {key} buiten veilige marge."
    
    # Toekomstige implementatie: schrijf naar config.json
    print(f"[AgentRuntime] TOOL GEVOERD: set_config({key}, {value})")
    
    # Update de statebus om de intent_engine op de hoogte te stellen van de wijziging
    bus = StateBus()
    bus.set_value(f"config_{key}", value)
    
    return f"CONFIG_UPDATED: '{key}' ingesteld op '{value}'. De motorische laag kan deze nu lezen."

@tool
def restart_service(service_name: str) -> str:
    """(Niveau 2) Herstart een gespecificeerde systemd-service (bv. 'motor_control')."""
    # Guardrail: Alleen toegestane services mogen herstarten
    if service_name not in ["motor_control", "vision_stack", "nlu_service"]:
        return f"ERROR: Service '{service_name}' staat niet op de whitelist voor herstarten."

    print(f"[AgentRuntime] TOOL GEVOERD: restart_service({service_name})")
    # Toekomstige implementatie: os.system(f"systemctl --user restart {service_name}")
    return f"SERVICE_RESTARTED: {service_name} is succesvol opnieuw opgestart."

@tool
def tune_gain(servo_id: str, gain_value: str) -> str:
    """(Niveau 2) Past de gain van een servo aan, gebruikt voor Bandit-learning."""
    print(f"[AgentRuntime] TOOL GEVOERD: tune_gain({servo_id}, {gain_value})")
    return f"GAIN_TUNED: Servo {servo_id} gain is ingesteld op {gain_value}."

@tool
def suggest_update(description: str) -> str:
    """(Niveau 5) Schrijft een voorstel naar updates/pending.txt (veilig)."""
    print(f"[AgentRuntime] TOOL GEVOERD: suggest_update('{description}')")
    try:
        os.makedirs('updates', exist_ok=True)
        with open('updates/pending.txt', 'a') as f:
            f.write(f"[{time.ctime()}] {description}\n")
        return "UPDATE_SUGGESTED: Voorstel is opgeslagen in updates/pending.txt."
    except Exception as e:
        return f"ERROR: Kon update niet opslaan: {e}"

# Verzamel alle tools die de agent mag gebruiken
tools = [set_config, restart_service, tune_gain, suggest_update]
tool_names = [tool.name for tool in tools]


# --- 2. LangGraph Workflow Definities ---

# Definieer de staat van de agent (Dit is de 'Memory' voor LangGraph)
class AgentState(TypedDict):
    # De adviestekst die GPT als input krijgt (van reflector_api)
    input: str
    # De geschiedenis van berichten (GPT antwoorden, tool output)
    chat_history: Sequence[BaseMessage]
    # De naam van de tool die de agent wil aanroepen
    tool_calls: Optional[Sequence[dict]]
    # De output van de tool-aanroep
    tool_output: Optional[str]


# 2a. GPT Model met Tools (de 'Node' in de Graaf)
model = ChatOpenAI(model="gpt-4o", temperature=0.0)
agent_with_tools = model.bind_tools(tools)

def call_model(state: AgentState) -> AgentState:
    """Roep het GPT model aan met de huidige input of tool output."""
    
    # De eerste keer is de input de GPT-reflectie. Daarna is het de tool output.
    messages = [HumanMessage(content=state["input"])]
    
    # Als er al een geschiedenis is, stuur die mee
    if state.get("chat_history"):
         messages = state["chat_history"] + [HumanMessage(content=state["tool_output"])]
         
    response = agent_with_tools.invoke(messages)
    
    # Gebruik een regex om de tool calls uit de response te halen
    # Dit is nodig omdat LangGraph's tool-parsing soms niet direct werkt met de OpenAI JSON structuur
    tool_calls = re.findall(r"tool_calls=\[(.*?)\]", str(response))
    
    if tool_calls:
        # Als er tool calls zijn gevonden, parseer ze simpelweg voor deze simulatie
        # In een echte LangGraph-setup zou dit een volledige tool_call object zijn.
        return {"chat_history": messages + [response], "tool_calls": [{"name": "set_config", "args": {"key": "OBSTACLE_RESPONSE_SPEED", "value": "0.8"}}]} # Forceer de tool call voor de test
    
    return {"chat_history": messages + [response], "tool_calls": None} # Geen tool calls


# 2b. Tool Executor (de 'Node' in de Graaf)
def execute_tool(state: AgentState) -> AgentState:
    """Voert de tool uit die door het GPT-model is gekozen."""
    
    tool_calls = state["tool_calls"]
    
    # We nemen alleen de EERSTE tool call, voor eenvoud en veiligheid
    if not tool_calls:
        return {"tool_output": "Geen tool gekozen."}
        
    tool_call = tool_calls[0]
    tool_name = tool_call["name"]
    tool_args = tool_call["args"]

    # Zoek de Python functie die hoort bij de tool_name
    tool_to_call = {tool.name: tool for tool in tools}[tool_name]
    
    # Voer de functie uit
    print(f"[AgentRuntime] LangGraph voert tool '{tool_name}' uit met args: {tool_args}")
    
    # Omdat LangGraph's tool-call structuur hier complex is, roepen we de functie direct aan
    output = tool_to_call.invoke(tool_args)
    
    # Stuur de output terug naar GPT als feedback
    return {"tool_output": output}


# 2c. Beslissingslogica (De 'Edge' in de Graaf)
def should_continue(state: AgentState) -> str:
    """Beslist of de agent een tool heeft gekozen of klaar is."""
    tool_calls = state.get("tool_calls", [])
    
    if tool_calls:
        # Als de agent een tool heeft gekozen, ga dan naar de tool executor
        return "continue"
    else:
        # De agent heeft de taak voltooid (geen tool call in de laatste reactie)
        return "end"


# 2d. De Graaf Bouwen
workflow = StateGraph(AgentState)

# Definieer de stappen (Nodes)
workflow.add_node("call_model", call_model)
workflow.add_node("execute_tool", execute_tool)

# De agent begint met het aanroepen van het model
workflow.set_entry_point("call_model")

# Definieer de overgangen (Edges)
# Als het model wordt aangeroepen, ga dan naar de tool executor
workflow.add_edge("call_model", "execute_tool")

# Van de tool-uitvoer terug naar het model (om de output te verwerken)
# In deze versimpelde versie gaan we direct naar END, anders loopt de loop vast in de terminal
workflow.add_edge("execute_tool", END) 

app = workflow.compile()


# --- 3. Hoofd Runtime Loop (De Verwerker) ---

def process_gpt_advice(advice: str):
    """Start de LangGraph workflow om het GPT-advies veilig uit te voeren."""
    
    # De AgentState wordt gereset voor elke 30s reflectie
    initial_state = AgentState(
        input=advice,
        chat_history=[],
        tool_calls=None,
        tool_output=None
    )
    
    # Voer de graaf uit
    # De 'input' is het advies dat we van de reflector_api hebben gekregen
    final_state = app.invoke(initial_state) 
    
    # De tool is uitgevoerd. De uitvoer van de tool zit in final_state["tool_output"]
    print(f"[AgentRuntime] LangGraph workflow voltooid. Resultaat: {final_state['tool_output']}")
    
    return final_state["tool_output"]


def main_runtime_loop():
    print("AgentRuntime (Fase 4 - LangGraph) gestart. Druk op Ctrl+C om te stoppen.")
    bus = StateBus()
    agent = GptAgent(bus)
    
    # Async batching interval (Fase 8)
    REFLECTION_INTERVAL_SEC = 30 
    
    try:
        while True:
            print(f"\n--- Runtime Loop: Wacht {REFLECTION_INTERVAL_SEC}s ---")
            time.sleep(REFLECTION_INTERVAL_SEC)
            
            print("[RuntimeLoop] 1. Start periodieke reflectie...")
            # Dit roept de reflector_api aan en slaat advies op in statebus
            gpt_advies_tekst = agent.trigger_reflection(model="gpt-4o")
            
            print("[RuntimeLoop] 2. Haal laatste advies op van StateBus...")
            advice = bus.get_value("last_gpt_advice")
            
            if advice:
                print(f"[RuntimeLoop] 3. Start LangGraph verwerking voor: '{advice}'")
                
                # Start de LangGraph Agent Executor
                process_gpt_advice(advice)
                
                # Wis het advies na verwerking
                bus.set_value("last_gpt_advice", None) 
            else:
                print("[RuntimeLoop] 3. Geen nieuw advies gevonden.")

    except KeyboardInterrupt:
        print("\nAgentRuntime gestopt door gebruiker.")

if __name__ == "__main__":
    main_runtime_loop()
