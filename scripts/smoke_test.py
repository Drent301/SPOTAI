import redis
import time
import sys
import os

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.statebus import StateBus

def run_smoke_test():
    """
    Een eenvoudige smoke test om de basisfunctionaliteit van Spot-AI te controleren.
    """
    print("--- Spot-AI Smoke Test ---")

    # --- Stap 1: Test Redis Verbinding ---
    try:
        bus = StateBus()
        bus.redis_client.ping()
        print("[OK] Stap 1: Redis verbinding succesvol.")
    except redis.exceptions.ConnectionError as e:
        print(f"[FAIL] Stap 1: Kon niet verbinden met Redis: {e}")
        print("Zorg ervoor dat de Redis server draait.")
        sys.exit(1)

    # --- Stap 2: Wacht op Mode Arbiter ---
    print("\n--- Stap 2: Wachten op kerncomponenten (max 15s) ---")
    start_time = time.time()
    arbiter_ready = False

    while time.time() - start_time < 15:
        # De ModeArbiter zou zijn status moeten schrijven bij het opstarten
        status = bus.get_value("mode_arbiter_status")
        if status and status.get("state") == "running":
            print("[OK] Mode Arbiter is actief en rapporteert status op de StateBus.")
            arbiter_ready = True
            break

        print("Wachten op Mode Arbiter...")
        time.sleep(2)

    if not arbiter_ready:
        print("[FAIL] Mode Arbiter heeft zijn status niet gerapporteerd op de StateBus.")
        print("Controleer of de 'mode_arbiter' node correct wordt gestart in de launch file.")
        sys.exit(1)

    # --- Stap 3: Controleer of een actie wordt ingesteld ---
    print("\n--- Stap 3: Controleren of een standaard actie wordt ingesteld (max 10s) ---")
    start_time = time.time()
    action_set = False

    while time.time() - start_time < 10:
        action = bus.get_value("robot_action")
        if action:
            print(f"[OK] Een robot_action is ingesteld op de StateBus: '{action}'")
            action_set = True
            break

        print("Wachten op robot_action...")
        time.sleep(1)

    if not action_set:
        print("[FAIL] Er is geen 'robot_action' ingesteld op de StateBus.")
        print("De Mode Arbiter lijkt geen beslissing te nemen.")
        sys.exit(1)

    print("\n--- Alle tests geslaagd! ---")
    print("De basisfunctionaliteit van het cognitieve brein lijkt correct te werken.")
    sys.exit(0)

if __name__ == "__main__":
    run_smoke_test()
