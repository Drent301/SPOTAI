from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'spotai'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- Voeg de launch-bestanden toe ---
        (os.path.join('share', package_name, 'launch'), glob('*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Agent',
    maintainer_email='agent@example.com',
    description='Cognitieve software voor de Spot-AI autonome robot.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Hier definiëren we de 'executables' die de launch file kan aanroepen
            'intent_engine = core.intent_engine:main',
            'agent_runtime = agent.agent_runtime:main',
            'learning_loop = learn.learning_loop:main',
            'emotion_mapper = ui.emotion_mapper:main',
            'hyperpixel_overlay = ui.hyperpixel_overlay:main',
            'power_manager = scripts.power_manager:main',
            'cooling_control = scripts.cooling_control:main',
            'ros_bridge = core.ros_bridge:main',
            'vision_node = perception.ros2_vision_node:main',
            'asr_node = speech.ros2_asr_node:main',
            'tts_node = speech.ros2_tts_node:main',
        ],
    },
)
