#!/bin/bash

echo "--- SpotAI Systeem Start ---"

# Navigeer naar de map waar het script staat
# Dit zorgt ervoor dat alle paden (zoals /data) correct werken
cd "$(dirname "$0")"

# Maak de data-map, voor het geval die mist
mkdir -p data

# Start alle kernservices op de achtergrond

echo "[Launcher] Start Kernlogica (Intent, Arbiter, Agent)..."
python3 core/intent_engine.py &
python3 agent/agent_runtime.py &

echo "[Launcher] Start Leermodules (Bandit, Reflector)..."
python3 learn/learning_loop.py &

echo "[Launcher] Start Hardware Bridges (Sensoren, Motoren)..."
python3 core/bridge_receiver.py &
python3 core/motor_bridge.py &

echo "[Launcher] Start Perceptie & Spraak..."
python3 perception/perception_processor.py &
python3 speech/speech_processor.py &

echo "[Launcher] Start UI (Ogen & Mapper)..."
python3 ui/emotion_mapper.py &
python3 ui/hyperpixel_overlay.py &

echo "[Launcher] Start Systeembeheer (Stroom, Koeling)..."
python3 scripts/power_manager.py &
python3 scripts/cooling_control.py &

echo "--- Alle services gestart ---"

# Dit commando houdt het script "levend" zodat we de logs kunnen zien
# Druk op Ctrl+C om alles te stoppen
wait