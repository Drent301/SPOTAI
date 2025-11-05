import py_trees
import time
import operator
from typing import Dict, Any

from core.statebus import StateBus
from core.config_manager import ConfigManager

# --- Gedragsdefinities (Behaviours) ---

class SetRobotAction(py_trees.behaviour.Behaviour):
    """Een actie-node die de 'robot_action' op de StateBus instelt."""
    def __init__(self, name: str, action: str, statebus: StateBus):
        super(SetRobotAction, self).__init__(name)
        self.action = action
        self.statebus = statebus

    def update(self):
        self.statebus.set_value("robot_action", self.action)
        self.logger.debug(f"  {self.name} [Setting Action: {self.action}]")
        return py_trees.common.Status.SUCCESS

class CheckConsolidatedIntentType(py_trees.behaviour.Behaviour):
    """Controleert of het 'consolidated_intent' type overeenkomt met het gegeven type."""
    def __init__(self, name: str, intent_type: str):
        super(CheckConsolidatedIntentType, self).__init__(name)
        self.intent_type = intent_type
        self.blackboard = None

    def update(self):
        intent = self.blackboard.consolidated_intent
        if intent and intent.get("type") == self.intent_type:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class CheckStateValue(py_trees.behaviour.Behaviour):
    """Controleert of een sleutel in de 'current_state' dictionary overeenkomt met de gegeven waarde."""
    def __init__(self, name: str, state_key: str, expected_value: Any):
        super(CheckStateValue, self).__init__(name)
        self.state_key = state_key
        self.expected_value = expected_value
        self.blackboard = None

    def update(self):
        state = self.blackboard.current_state
        if state and state.get(self.state_key) == self.expected_value:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class CheckIntentConfidence(py_trees.behaviour.Behaviour):
    """Controleert of de 'consolidated_intent' confidence >= een drempel is."""
    def __init__(self, name: str, threshold: float):
        super(CheckIntentConfidence, self).__init__(name)
        self.threshold = threshold
        self.blackboard = None

    def update(self):
        intent = self.blackboard.consolidated_intent
        if intent and intent.get("confidence", 0.0) >= self.threshold:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

# --- Behavior Tree Manager ---

class BehaviorTreeManager:
    """Beheert de creatie en uitvoering van de beslissings-Behavior Tree."""

    class Blackboard:
        """Een simpele blackboard-implementatie om data te delen binnen de boom."""
        def __init__(self, statebus: StateBus):
            self.statebus = statebus
            self.current_state: Dict[str, Any] = {}
            self.consolidated_intent: Dict[str, Any] = {}

        def update(self):
            """Update het blackboard met de laatste data van de StateBus."""
            self.current_state = self.statebus.get_all_values() or {}
            self.consolidated_intent = self.current_state.get("current_consolidated_intent",
                                                             {"type": "idle", "action": "monitor", "confidence": 1.0})

    def __init__(self, statebus: StateBus):
        self.bus = statebus
        self.config = ConfigManager()
        self.autonomy_level = self.config.get_setting("AUTONOMY_LEVEL", 0.7)
        self.blackboard = self.Blackboard(statebus)
        self.tree = self._create_tree()

    def _create_tree(self) -> py_trees.behaviour.Behaviour:
        """Bouwt de volledige Behavior Tree op."""

        root = py_trees.composites.Selector("ModeArbiter", memory=True)

        # Prioriteit 1: Noodstop
        emergency_check = py_trees.composites.Sequence("EmergencySequence", memory=True)
        is_emergency = CheckConsolidatedIntentType(name="IsEmergency?", intent_type="emergency")
        set_emergency_action = SetRobotAction("SetEmergencyStop", "stop_motors", self.bus)
        emergency_check.add_children([is_emergency, set_emergency_action])

        # Prioriteit 2: Opladen
        charging_check = py_trees.composites.Sequence("ChargingSequence", memory=True)
        is_charging = CheckStateValue(name="IsCharging?", state_key="is_charging", expected_value=True)
        set_charging_action = SetRobotAction("SetSystemCheck", "system_check", self.bus)
        charging_check.add_children([is_charging, set_charging_action])

        # Prioriteit 3: Actie gebaseerd op intentie
        intent_action_check = py_trees.composites.Sequence("IntentActionSequence", memory=True)
        is_intent_confident = CheckIntentConfidence(name=f"IsIntentConfident? (>{self.autonomy_level})", threshold=self.autonomy_level)

        class ExecuteIntentAction(py_trees.behaviour.Behaviour):
            def __init__(self, name, statebus, blackboard):
                super(ExecuteIntentAction, self).__init__(name)
                self.statebus = statebus
                self.blackboard = blackboard
            def update(self):
                action = self.blackboard.consolidated_intent.get("action", "monitor_sensors")
                self.statebus.set_value("robot_action", action)
                return py_trees.common.Status.SUCCESS

        execute_intent = ExecuteIntentAction("ExecuteIntentAction", self.bus, self.blackboard)
        intent_action_check.add_children([is_intent_confident, execute_intent])

        # Laagste Prioriteit / Default: Monitoren
        set_default_action = SetRobotAction("SetDefaultMonitor", "monitor_sensors", self.bus)

        root.add_children([emergency_check, charging_check, intent_action_check, set_default_action])
        return root

    def tick(self):
        """Update het blackboard en voert één cyclus ('tick') van de boom uit."""
        self.blackboard.update()

        for node in self.tree.iterate():
            if hasattr(node, "blackboard"):
                node.blackboard = self.blackboard

        self.tree.tick_once()

# --- Test code ---
if __name__ == "__main__":
    print("Behavior Tree Manager Test")
    bus = StateBus()
    bt_manager = BehaviorTreeManager(bus)

    print("\n--- Behavior Tree ---")
    print(py_trees.display.ascii_tree(bt_manager.tree, show_status=False))

    def run_test_scenario(scenario_name, state):
        print(f"\n--- SCENARIO: {scenario_name} ---")
        for key, value in state.items():
            bus.set_value(key, value)
        bt_manager.tick()
        print(f"==> Finale Robot Actie: {bus.get_value('robot_action')}")

    run_test_scenario("Noodstop", {"current_consolidated_intent": {"type": "emergency", "action": "stop_motors", "confidence": 1.0}})
    run_test_scenario("Opladen", {"is_charging": True, "current_consolidated_intent": {"type": "idle", "confidence": 1.0}})
    run_test_scenario("Zekere Intentie", {"is_charging": False, "current_consolidated_intent": {"type": "motor_action", "action": "walk_forward", "confidence": 0.9}})
    run_test_scenario("Onzekere Intentie", {"is_charging": False, "current_consolidated_intent": {"type": "motor_action", "action": "walk_forward", "confidence": 0.5}})
    run_test_scenario("Default State", {"is_charging": False, "current_consolidated_intent": {"type": "idle", "confidence": 1.0}})
