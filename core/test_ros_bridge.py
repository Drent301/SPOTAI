import unittest
import json
import sys
from unittest.mock import MagicMock
import os

# --- Expliciete Mocking van ROS 2-bibliotheken ---
# We definiëren onze eigen nep-Node klasse om de 'StopIteration' fout te vermijden
# en om de logger-mock betrouwbaar te kunnen controleren.
class MockNode:
    """Een expliciete mock die rclpy.node.Node imiteert voor onze tests."""
    def __init__(self, node_name):
        # Elke Node heeft een logger. We maken hier een mock van zodat we
        # kunnen controleren of er fouten worden gelogd.
        self.logger = MagicMock(spec=['info', 'error', 'warn'])

    def get_logger(self):
        return self.logger

    def create_publisher(self, *args, **kwargs):
        return MagicMock() # We testen de publisher niet, dus een simpele mock is genoeg.

    def create_subscription(self, *args, **kwargs):
        return MagicMock() # Idem voor de subscriber.

# Plaats de mocks in sys.modules VOORDAT we de te testen code importeren.
# Zo dwingen we Python om onze nep-bibliotheken te gebruiken.
sys.modules['rclpy'] = MagicMock()
mock_node_module = MagicMock()
mock_node_module.Node = MockNode # Gebruik onze expliciete MockNode
sys.modules['rclpy.node'] = mock_node_module
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()

# Nu importeren we veilig de code die we willen testen.
from core.ros_bridge import ROSBridge
from core.statebus import StateBus

class TestROSBridge(unittest.TestCase):
    """Test de logica van de ROSBridge zonder een echte ROS 2-omgeving."""

    def setUp(self):
        """Wordt voor elke test uitgevoerd."""
        self.mock_bus = MagicMock(spec=StateBus)
        # De ROSBridge erft nu van onze MockNode, dus we kunnen de logger later inspecteren.
        self.bridge = ROSBridge(self.mock_bus)

    def test_vision_callback_updates_statebus_correctly(self):
        """
        Controleert of de _vision_callback een geldige JSON-string correct
        naar de StateBus schrijft.
        """
        # Arrange
        test_payload = [{"type": "gezicht", "confidence": 0.99}]
        mock_ros_msg = MagicMock()
        mock_ros_msg.data = json.dumps(test_payload)

        # Act
        self.bridge._vision_callback(mock_ros_msg)

        # Assert
        self.mock_bus.set_value.assert_called_once_with("vision_detections", test_payload)
        print("test_vision_callback_updates_statebus_correctly: PASSED")

    def test_vision_callback_handles_invalid_json(self):
        """
        Controleert of de _vision_callback een fout logt en de StateBus niet
        aanroept bij ongeldige JSON.
        """
        # Arrange
        mock_ros_msg = MagicMock()
        mock_ros_msg.data = "dit is { geen > json"

        # Act
        self.bridge._vision_callback(mock_ros_msg)

        # Assert
        # Controleer of de StateBus NIET is aangeroepen.
        self.mock_bus.set_value.assert_not_called()
        # Controleer of de logger (die deel uitmaakt van onze MockNode) is gebruikt
        # om een fout te loggen.
        self.bridge.get_logger().error.assert_called_once()
        print("test_vision_callback_handles_invalid_json: PASSED")

if __name__ == '__main__':
    # Voeg de project-root toe aan het pad zodat 'from core...' werkt.
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    unittest.main()
