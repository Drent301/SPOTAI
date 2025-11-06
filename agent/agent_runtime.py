import time
import os
import sys
import re
from typing import TypedDict, Annotated, Sequence, Optional
from operator import itemgetter
from dotenv import load_dotenv 

# Voeg de hoofdmap toe voor core-modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

# --- NIEUWE IMPORTS: Importeer alle tools uit de submappen ---
from agent.tools.config_tools import set_config 
from agent.tools.system_tools import restart_service, log_reflection
from agent.tools.motor_tools import tune_gain
from agent.tools.code_tools import edit_code, rollback, generate_doc

# Importeer kernmodules
from core.statebus import StateBus
from core.gpt_agent import GptAgent
from core.config_manager import ConfigManager
from core.memory_manager import MemoryManager

# Importeer LangGraph componenten
from langchain_core.tools import tool # Hier niet direct nodig, maar voor de types
from langchain_core.messages import BaseMessage, HumanMessage
from langchain_openai import ChatOpenAI
from langgraph.graph import StateGraph, END

# --- Configuratie Fix ---
load_dotenv() 

# --- 1. Tool Definities (Verzameling) ---

# Verzamel alle tools die de LangGraph Agent mag gebruiken
# Deze worden nu uit de geïmplementeerde bestanden gehaald:
tools = [
    set_config, 
    restart_service, 
    tune_gain, 
    log_reflection,
    # Niveau 5 Tools (Placeholders):
    edit_code,
    rollback,
    generate_doc
]


# --- 2. LangGraph Workflow Definities ---

class AgentState(TypedDict):
    input: str
    chat_history: Sequence[BaseMessage]
    tool_calls: Optional[Sequence[dict]]
    tool_output: Optional[str]

# 2a. GPT Model met Tools (LangGraph kan nu initialiseren)
model = ChatOpenAI(model="gpt-4o", temperature=0.0)
# FIX: Bind alle geëxporteerde tools aan het GPT-model
agent_with_tools = model.bind_tools(tools)

def call_model(state: AgentState) -> AgentState:
    """Roep het GPT model aan met de huidige input."""
    
    messages = [HumanMessage(content=state["input"])]
    
    # Roep het model aan om de tool call te genereren
    response = agent_with_tools.invoke(messages)

    return {"chat_history": messages, "tool_calls": response.tool_calls}


def execute_tool(state: AgentState) -> AgentState:
    """Voert de tool uit die door het GPT-model is gekozen."""
    
    tool_calls = state["tool_calls"]
    tool_call = tool_calls[0]
    tool_name = tool_call["name"]
    tool_args = tool_call["args"]

    # FIX: Haal de tool op uit de globale lijst van tools
    tool_to_call = next((t for t in tools if t.name == tool_name), None)
    
    if tool_to_call is None:
        raise ValueError(f"LangGraph Error: Tool '{tool_name}' niet gevonden in de tools lijst.")
    
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


def main():
    print("AgentRuntime (Fase C.2 - LangGraph) gestart. Druk op Ctrl+C om te stoppen.")
    bus = StateBus()
    agent = GptAgent(bus)
    
    REFLECTION_INTERVAL_SEC = 30 
    
    try:
        while True:
            print(f"\n--- Runtime Loop: Wacht {REFLECTION_INTERVAL_SEC}s ---")
            time.sleep(REFLECTION_INTERVAL_SEC)
            
            print("[RuntimeLoop] 1. Start periodieke reflectie...")
            # Dit roept de reflector_api aan en slaat advies op in statebus
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
    main()
