import sys
import os
from langchain_core.tools import tool

# WAARSCHUWING: Deze vlag moet op False staan om de veiligheid te garanderen
AGENT_ALLOW_PATCHES = False 

@tool
def edit_code(file: str, section: str, patch: str) -> str:
    """
    (Niveau 5) Past code aan (Zelfherstel/Code-Patching). 
    STANDAARD UITGESCHAKELD vanwege veiligheidsrisico's.
    """
    if not AGENT_ALLOW_PATCHES:
        return f"ACCESS_DENIED: Code-aanpassing is uitgeschakeld. Voorstel is: {file} patch."
    
    # In een echte implementatie zou hier auto-diff en validatie plaatsvinden.
    return f"CODE_PATCHED: Bestand {file} is aangepast."

@tool
def rollback(file: str) -> str:
    """
    (Niveau 5) Herstelt de vorige versie van een bestand na een mislukte code-patch.
    STANDAARD UITGESCHAKELD.
    """
    if not AGENT_ALLOW_PATCHES:
        return f"ACCESS_DENIED: Rollback is uitgeschakeld. Kan geen wijzigingen terugdraaien."

    return f"ROLLBACK_SUCCESS: Bestand {file} is hersteld naar vorige versie."

@tool
def generate_doc(changes: str) -> str:
    """
    (Niveau 5) Documenteert veranderingen. Wordt gebruikt na een succesvolle aanpassing of leercyclus.
    """
    print(f"[DOC_GEN] Documentatie gegenereerd voor: {changes}")
    return "DOCUMENTATION_CREATED: Documentatie voor wijzigingen is bijgewerkt."