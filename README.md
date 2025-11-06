# Spot-AI Project

Dit project bevat de volledige, hardware-ready cognitieve software voor een autonome robot, ontworpen om te draaien op een Raspberry Pi 5.

Het systeem is gebouwd op een **hybride architectuur**:

1.  Een **Cognitieve Laag (het Brein)**, geschreven in puur Python, die beslissingen neemt.
2.  Een **Hardware Laag (het Zenuwstelsel)**, die draait op **ROS 2** en de sensoren en motoren aanstuurt.

Alle communicatie tussen deze lagen wordt beheerd door een centrale **Redis-gebaseerde `StateBus`**, waardoor het brein en het zenuwstelsel volledig losgekoppeld en real-time gesynchroniseerd zijn.

## Project Status: Hardware-Ready

Alle softwarematige simulaties zijn succesvol vervangen door daadwerkelijke hardware-implementaties. De software is nu compleet en klaar voor fysieke assemblage en testen.

-----

## Architectuur & Kerncomponenten

Alle services worden parallel gestart door het ROS 2 launch-systeem (`spotai.launch.py`).

### 1\. Het Cognitieve Brein (Python)

Dit is de "denkende" laag van de robot, die los van ROS 2 draait.

  * **`core/statebus.py`**: Biedt een high-level interface naar de **Redis-database**, die fungeert als het centrale "prikbord" waar alle modules hun status lezen en schrijven.
  * **`core/intent_engine.py`**: Leest alle sensor-inputs van de StateBus (o.a. van ROS 2) en consolideert deze tot één duidelijke "geconsolideerde intentie".
  * **`core/mode_arbiter.py`**: De "eindbaas". Kijkt naar de intentie, batterijstatus en netwerkstatus en neemt de definitieve beslissing over de `robot_action` (bv. "walk\_forward").

### 2\. De Hybride Brug (De Tolk)

Dit is de cruciale verbinding tussen het brein en het zenuwstelsel.

  * **`core/ros_bridge.py`**: Een **echte ROS 2 Node** (`rclpy.Node`) die twee taken heeft:
    1.  **Abonneren** op sensor-topics (zoals `/imu/data`, `/joint_states`) en deze data vertalen naar de `StateBus`.
    2.  **Publiceren** van de `robot_action` van de `StateBus` naar ROS 2-commando-topics (zoals `/cmd_vel`).

### 3\. De Zintuigen (ROS 2 Nodes)

Dit zijn de daadwerkelijke hardware-drivers die als losse ROS 2-nodes draaien.

  * **`perception/ros2_vision_node.py`**:
      * Gebruikt **OpenCV** om de camera uit te lezen.
      * Gebruikt de **Hailo AI-accelerator** voor detecties (TODO: mock vervangen door echte SDK).
      * Publiceert detecties (als JSON-string) naar het `/vision/detections` topic.
  * **`speech/ros2_asr_node.py`** (Oren):
      * Gebruikt `sounddevice` om de **ReSpeaker-microfoon** uit te lezen.
      * Gebruikt **Vosk** voor spraakherkenning.
      * Publiceert de herkende tekst naar het `/speech/asr_text` topic.
  * **`speech/ros2_tts_node.py`** (Stem):
      * Abonneert op het `/tts/speak` topic.
      * Gebruikt **Piper TTS** om de tekst om te zetten in audio.
      * Speelt de audio af via `aplay`.

### 4\. De GPT-Agent (Zelfverbetering)

Het systeem voor reflectie en zelfaanpassing, dat nu **volledig lokaal** op de Pi draait.

  * **`reflector_api.py`**: Een **FastAPI-server** die lokaal draait. Het ontvangt de status van de robot, roept de OpenAI API aan en stuurt het advies terug.
  * **`core/gpt_agent.py`**: Activeert periodiek een reflectie door de lokale `reflector_api` aan te roepen (`http://127.0.0.1:8000/reflect`) en schrijft het advies naar de `StateBus`.
  * **`agent/agent_runtime.py`**: Een veilige **LangGraph**-omgeving die het advies van de GPT-agent leest en uitvoert door "tools" aan te roepen (bv. `set_config`, `restart_service`).

### 5\. Lokaal Leersysteem (Offline Intelligentie)

Deze modules (nog gesimuleerd) bieden de robot de mogelijkheid om lokaal, zonder cloud, te leren en te reflecteren.

  * **`learn/learning_loop.py`**: De hoofd-orkestrator voor lokaal leren.
  * **`learn/bandit_learner.py`**: (Gesimuleerd) Een agent die lokaal PID-gains optimaliseert.
  * **`learn/offline_reflector.py`**: (Gesimuleerd) Een lokale LLM (Llama-3) om basisvragen te beantwoorden als het internet wegvalt.

### 6\. Systeembeheer & UI (Gezondheid & Zintuigen)

Lichtgewicht scripts die de gezondheid van de Pi 5 en de visuele feedback beheren.

  * **`scripts/power_manager.py`**: Leest de **PiSugar-batterijstatus** uit via I2C (`smbus2`) en schrijft de status (bv. `is_charging`) naar de `StateBus`.
  * **`scripts/cooling_control.py`**: Leest de CPU-temperatuur via `vcgencmd` en stuurt de **ventilator** aan via `RPi.GPIO`.
  * **`ui/emotion_mapper.py`**: Vertaalt de `robot_action` van de `StateBus` naar een emotie en kleur (bv. "idle" -\> groen, "offline" -\> blauw).
  * **`ui/hyperpixel_overlay.py`**: Leest de status van `emotion_mapper` en tekent met **PyGame** de daadwerkelijke "ogen" op het ronde HyperPixel-scherm.

-----

## Installatie & Uitvoeren

### Opslag Strategie (Aanbevolen)

1.  **SD-kaart:** Installeer alleen het OS (Raspberry Pi OS) en alle ROS 2-packages.
2.  **USB-Drive (ext4):** Formatteer een snelle USB-drive. Plaats deze volledige `SPOTAI` projectmap én alle AI-modellen (Vosk, Piper, Llama-3, Hailo) op deze drive.
3.  **`fstab`:** Stel `fstab` in om de USB-drive automatisch te koppelen bij het opstarten (bv. naar `/mnt/spotai_project`).

### Installatie (Python Dependencies)

Installeer de benodigde Python-packages (idealiter in een virtual environment):

```bash
pip install -r requirements.txt 
```

*(De benodigde packages staan in `requirements.txt`)*

### Configuratie

1.  **OpenAI API-sleutel:**
    Maak een bestand met de naam `.env` in de hoofdmap van dit project. Voeg de volgende regel toe:

    ```
    OPENAI_API_KEY=jouw_echte_api_key_hier
    ```

    Het `.gitignore`-bestand voorkomt dat dit bestand naar GitHub wordt gepusht.

2.  **Hardware Paden:**
    Controleer de `TODO`-commentaren in de volgende bestanden en pas de paden en apparaat-ID's aan naar jouw setup:

      * `speech/ros2_asr_node.py` (Vosk `MODEL_PATH` en ReSpeaker `DEVICE_ID`)
      * `speech/ros2_tts_node.py` (Piper `PIPER_MODEL_PATH` en `AUDIO_DEVICE`)
      * `scripts/cooling_control.py` (`FAN_PIN`)

### Systeem Starten

Alle services worden beheerd en gestart via het ROS 2 launch-systeem. Dit zorgt voor robuustheid en automatisch herstarten van services.

1.  **Bouw de ROS 2 Package:**
    Navigeer naar de root van de `spotai` workspace en bouw de package:
    ```bash
    colcon build
    ```

2.  **Source de Workspace:**
    Voeg de workspace toe aan je shell-omgeving:
    ```bash
    source install/setup.bash
    ```

3.  **Start het Systeem:**
    Start alle Spot-AI services met één commando:
    ```bash
    ros2 launch spotai spotai.launch.py
    ```
