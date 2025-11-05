#!/bin/bash

echo "--- SpotAI Systeem Start (ROS 2 Hybride Modus) ---"

# Navigeer naar de map waar het script staat
cd "$(dirname "$0")"
mkdir -p data

# --- Cognitieve Laag (Ons Brein) ---
echo "[Launcher] Start Kernlogica (Intent, Agent)..."
python3 core/intent_engine.py &
python3 agent/agent_runtime.py &

echo "[Launcher] Start Leermodules (Bandit, Reflector)..."
python3 learn/learning_loop.py &

echo "[Launcher] Start UI (Ogen & Mapper)..."
python3 ui/emotion_mapper.py &
python3 ui/hyperpixel_overlay.py &

echo "[Launcher] Start Systeembeheer (Stroom, Koeling)..."
python3 scripts/power_manager.py &
python3 scripts/cooling_control.py &

# --- ROS 2 Brug (Het Zenuwstelsel) ---
echo "[Launcher] Start de ROS 2 Bridge..."
python3 core/ros_bridge.py &

# WAARSCHUWING: De volgende scripts zijn nu LEEG en vervangen door de ROS Bridge
# Ze worden hier niet meer gestart:
# - core/bridge_receiver.py
# - core/motor_bridge.py
# - perception/perception_processor.py
# - speech/speech_processor.py

echo "--- Alle services gestart ---"
wait