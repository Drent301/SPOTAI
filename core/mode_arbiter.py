import time
from core.statebus import StateBus

class ModeArbiter:
    def __init__(self, statebus: StateBus):
        self.statebus = statebus
        self.current_mode = "idle"
        print("ModeArbiter geïnitialiseerd.")

    def get_mode(self):
        """Bepaalt en retourneert de huidige robot-modus."""

        # Haal relevante data uit de statebus (dit wordt later gevuld)
        last_voice_command = self.statebus.get_value("last_voice_command_ts")
        last_human_seen = self.statebus.get_value("last_human_seen_ts")
        is_charging = self.statebus.get_value("is_charging")

        current_time = time.time()

        # 1. Prioriteit: Opladen
        if is_charging:
            self.current_mode = "charging"
            return self.current_mode

        # 2. Prioriteit: Reageren op spraak
        if last_voice_command and (current_time - last_voice_command < 10):
            self.current_mode = "listening"
            return self.current_mode

        # 3. Prioriteit: Interactie met mens
        if last_human_seen and (current_time - last_human_seen < 30):
            self.current_mode = "social"
            return self.current_mode

        # 4. Fallback: Verkennen of niets doen
        self.current_mode = "idle"
        return self.current_mode

if __name__ == "__main__":
    # Test script
    print("ModeArbiter module test...")
    bus = StateBus()
    arbiter = ModeArbiter(bus)

    print(f"Huidige modus: {arbiter.get_mode()}")

    print("Simuleer zien van mens...")
    bus.set_value("last_human_seen_ts", time.time())
    print(f"Nieuwe modus: {arbiter.get_mode()}")
