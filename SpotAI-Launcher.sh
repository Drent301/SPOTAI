#!/bin_bash

echo "--- SpotAI Systeem Start (ROS 2 Hybride Modus) ---"

# Navigeer naar de map waar het script staat
cd "$(dirname "$0")"
mkdir -p data

# --- Cognitieve Laag (Ons Brein) ---
echo "[Launcher] Start Kernlogica (Intent, Agent)..."
python3 core/intent_engine.py &
python3 agent/agent_runtime.py &

echo "[Launcher] Start Lokale API (Reflector)..."
# Zorg dat het .env bestand met OPENAI_API_KEY in de root staat
uvicorn reflector_api:app --host 127.0.0.1 --port 8000 &

echo "[Launcher] Start Leermodules (Bandit, Reflector)..."
python3 learn/learning_loop.py &

echo "[Launcher] Start UI (Ogen & Mapper)..."
python3 ui/emotion_mapper.py &
python3 ui/hyperpixel_overlay.py &

echo "[Launcher] Start Systeembeheer (Stroom, Koeling)..."
python3 scripts/power_manager.py &
python3 scripts/cooling_control.py &

# --- ROS 2 Brug & Zintuigen ---
echo "[Launcher] Start de ROS 2 Bridge (Tolk)..."
python3 core/ros_bridge.py &

echo "[Launcher] Start de ROS 2 Vision Node (Ogen)..."
python3 perception/ros2_vision_node.py &

echo "[Launcher] Start de ROS 2 Speech Nodes (Oren & Stem)..."
python3 speech/ros2_asr_node.py &
python3 speech/ros2_tts_node.py &


# WAARSCHUWING: De volgende scripts zijn nu LEEG en vervangen:
# - perception/perception_processor.py
# - speech/speech_processor.py

echo "--- Alle services gestart ---"
wait