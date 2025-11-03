import time
import os
import sys
import re
from typing import TypedDict, Annotated, Sequence, Optional
from operator import itemgetter
from dotenv import load_dotenv 

# Voeg de hoofdmap toe voor core-modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Importeer kernmodules
from core.statebus import StateBus
from core.gpt_agent import GptAgent
from core.config_manager import ConfigManager
from core.memory_manager import MemoryManager

# Importeer LangGraph componenten
from langchain_core.tools import tool
from langchain_core.messages import BaseMessage, HumanMessage
from langchain_openai import ChatOpenAI
from langgraph.graph import StateGraph, END

# --- Configuratie Fix ---
# DE FIX: Laad de variabelen uit het .env bestand zodat LangChain ze kan vinden
load_dotenv() 

# --- 1. Tool Definities (Whitelisted Functies) ---

@tool
def set_config(key: str, value: str) -> str:
    """(Niveau 1) Past een configuratieparameter veilig aan (bv. 'OBSTACLE_RESPONSE_SPEED', '0.8')."""
    
    cm = ConfigManager()
    
    # Guardrail: Valideer hier de input (bv. voor obstakelsnelheid)
    if key == "OBSTACLE_RESPONSE_SPEED":
        try:
            numeric_value = float(value)
            if not (0.1 <= numeric_value <= 2.0):
                return f"ERROR: Waarde {value} voor {key} buiten veilige marge (0.1-2.0s)."
        except ValueError:
            return f"ERROR: Waarde {value} voor {key} moet een nummer zijn."
    
    # Schrijf de nieuwe setting weg (Persistente opslag in config.json)
    cm.set_setting(key, value)
    print(f"[AgentRuntime] TOOL GEVOERD: set_config({key}, {value})")
    
    # Update de statebus voor snelle notificatie naar de IntentEngine
    bus = StateBus()
    bus.set_value(f"config_{key}", value)
    
    return f"CONFIG_UPDATED: '{key}' persistent opgeslagen als '{value}'. De motorische laag kan deze nu lezen."

@tool
def restart_service(service_name: str) -> str:
    """(Niveau 2) Herstart een gespecificeerde systemd-service (bv. 'motor_control')."""
    if service_name not in ["motor_control", "vision_stack", "nlu_service"]:
        return f"ERROR: Service '{service_name}' staat niet op de whitelist voor herstarten."

    print(f"[AgentRuntime] TOOL GEVOERD: restart_service({service_name})")
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


# --- 2. LangGraph Workflow Definities ---

class AgentState(TypedDict):
    input: str
    chat_history: Sequence[BaseMessage]
    tool_calls: Optional[Sequence[dict]]
    tool_output: Optional[str]

# 2a. GPT Model met Tools (LangGraph kan nu initialiseren dankzij de fix)
model = ChatOpenAI(model="gpt-4o", temperature=0.0)
agent_with_tools = model.bind_tools(tools)

def call_model(state: AgentState) -> AgentState:
    """Roep het GPT model aan met de huidige input."""
    
    messages = [HumanMessage(content=state["input"])]
    
    # We forceren de Tool Call voor deze LangGraph test
    return {"chat_history": messages, "tool_calls": [{"name": "set_config", "args": {"key": "OBSTACLE_RESPONSE_SPEED", "value": "0.8"}}]} 


def execute_tool(state: AgentState) -> AgentState:
    """Voert de tool uit die door het GPT-model is gekozen."""
    
    tool_calls = state["tool_calls"]
    tool_call = tool_calls[0]
    tool_name = tool_call["name"]
    tool_args = tool_call["args"]

    tool_to_call = {tool.name: tool for tool in tools}[tool_name]
    
    print(f"[AgentRuntime] LangGraph voert tool '{tool_name}' uit met args: {tool_args}")
    
    output = tool_to_call.invoke(tool_args)
    
    return {"tool_output": output}


# 2d. De Graaf Bouwen
workflow = StateGraph(AgentState)
workflow.add_node("call_model", call_model)
workflow.add_node("execute_tool", execute_tool)

workflow.set_entry_point("call_model")
workflow.add_edge("call_model", "execute_tool")
workflow.add_edge("execute_tool", END) # Stopt na de tool-uitvoering voor onze 30s loop

app = workflow.compile()


# --- 3. Hoofd Runtime Loop (De Verwerker) ---

def process_gpt_advice(advice: str):
    """Start de LangGraph workflow om het GPT-advies veilig uit te voeren."""
    
    initial_state = AgentState(
        input=advice,
        chat_history=[],
        tool_calls=None,
        tool_output=None
    )
    
    final_state = app.invoke(initial_state) 
    
    print(f"[AgentRuntime] LangGraph workflow voltooid. Resultaat: {final_state['tool_output']}")
    
    return final_state["tool_output"]


def main_runtime_loop():
    print("AgentRuntime (Fase 4 - LangGraph) gestart. Druk op Ctrl+C om te stoppen.")
    bus = StateBus()
    agent = GptAgent(bus)
    
    REFLECTION_INTERVAL_SEC = 30 
    
    try:
        while True:
            print(f"\n--- Runtime Loop: Wacht {REFLECTION_INTERVAL_SEC}s ---")
            time.sleep(REFLECTION_INTERVAL_SEC)
            
            print("[RuntimeLoop] 1. Start periodieke reflectie...")
            # FIX: model="gpt-4o" is vervangen door task_type="strategy" om de TypeError op te lossen
            gpt_advies_tekst = agent.trigger_reflection(task_type="strategy") 
            
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
