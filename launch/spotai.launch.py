import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    """
    Genereert de ROS 2 Launch Description voor het volledige Spot-AI systeem.
    Dit vervangt het oude SpotAI-Launcher.sh script.
    """

    ld = LaunchDescription()

    # --- Cognitieve Laag ---
    mode_arbiter_node = Node(
        package='spotai', executable='mode_arbiter',
        name='mode_arbiter',
        respawn=True,
        output='screen'
    )

    gpt_agent_node = Node(
        package='spotai', executable='gpt_agent',
        name='gpt_agent',
        respawn=True,
        output='screen'
    )

    intent_engine_node = Node(
        package='spotai', executable='intent_engine',
        name='intent_engine',
        respawn=True,
        output='screen'
    )

    agent_runtime_node = Node(
        package='spotai', executable='agent_runtime',
        name='agent_runtime',
        respawn=True,
        output='screen'
    )

    # Uvicorn wordt gestart met ExecuteProcess omdat het geen standaard ROS node is
    reflector_api_proc = ExecuteProcess(
        cmd=['uvicorn', 'reflector_api:app', '--host', '127.0.0.1', '--port', '8000'],
        name='reflector_api',
        output='screen'
    )

    learning_loop_node = Node(
        package='spotai', executable='learning_loop',
        name='learning_loop',
        respawn=True,
        output='screen'
    )

    # --- UI ---
    emotion_mapper_node = Node(
        package='spotai', executable='emotion_mapper',
        name='emotion_mapper',
        respawn=True,
        output='screen'
    )

    hyperpixel_overlay_node = Node(
        package='spotai', executable='hyperpixel_overlay',
        name='hyperpixel_overlay',
        respawn=True,
        output='screen'
    )

    # --- Systeembeheer ---
    power_manager_node = Node(
        package='spotai', executable='power_manager',
        name='power_manager',
        respawn=True,
        output='screen'
    )

    cooling_control_node = Node(
        package='spotai', executable='cooling_control',
        name='cooling_control',
        respawn=True,
        output='screen'
    )

    # --- ROS 2 Brug & Zintuigen ---
    ros_bridge_node = Node(
        package='spotai', executable='ros_bridge',
        name='ros_bridge',
        respawn=True,
        output='screen'
    )

    vision_node = Node(
        package='spotai', executable='vision_node',
        name='vision_node',
        respawn=True,
        output='screen'
    )

    asr_node = Node(
        package='spotai', executable='asr_node',
        name='asr_node',
        respawn=True,
        output='screen'
    )

    tts_node = Node(
        package='spotai', executable='tts_node',
        name='tts_node',
        respawn=True,
        output='screen'
    )

    # --- Voeg alle nodes en processen toe aan de Launch Description ---
    ld.add_action(mode_arbiter_node)
    ld.add_action(gpt_agent_node)
    ld.add_action(intent_engine_node)
    ld.add_action(agent_runtime_node)
    ld.add_action(reflector_api_proc)
    ld.add_action(learning_loop_node)
    ld.add_action(emotion_mapper_node)
    ld.add_action(hyperpixel_overlay_node)
    ld.add_action(power_manager_node)
    ld.add_action(cooling_control_node)
    ld.add_action(ros_bridge_node)
    ld.add_action(vision_node)
    ld.add_action(asr_node)
    ld.add_action(tts_node)

    return ld
