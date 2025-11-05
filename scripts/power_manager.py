import sys
import os
import time
import random

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.statebus import StateBus

class PowerDriver:
    """
    Een placeholder (mock) voor de echte PiSugar 3 (I2C) of INA260 sensor.
    """
    def __init__(self):
        # In een echte implementatie:
        # - Verbind met I2C
        # - Initialiseer de sensor
        print("[PowerDriver] PiSugar 3 / INA260 geïnitialiseerd (simulatie).")
        # Simuleer een langzaam leeglopende batterij
        self.current_voltage = 4.2
        self.current_percentage = 100.0

    def get_power_stats(self) -> dict:
        """ Simuleert het uitlezen van de batterijstatus. """

        # Simuleer langzaam verbruik
        self.current_voltage -= 0.001
        self.current_percentage -= 0.05

        if self.current_percentage < 0:
            self.current_percentage = 100.0
            self.current_voltage = 4.2

        return {
            "voltage": round(self.current_voltage, 2),
            "percentage": round(self.current_percentage, 1),
            "is_charging": random.random() < 0.1 # 10% kans dat hij "oplaadt"
        }

def run_power_monitor():
    """
    De hoofdloop voor de power manager service.
    """
    bus = StateBus()
    driver = PowerDriver()

    print("[PowerManager] Service gestart. Monitort batterijstatus.")

    while True:
        try:
            stats = driver.get_power_stats()

            # Schrijf de status naar de StateBus
            bus.set_value("power_state", stats)
            bus.set_value("is_charging", stats["is_charging"]) # Ook apart voor de Arbiter

            if stats["percentage"] < 10.0:
                print(f"[PowerManager] WAARSCHUWING: Batterij bijna leeg! {stats['percentage']}%")
                # De IntentEngine/ModeArbiter zal hierop reageren

        except Exception as e:
            print(f"[PowerManager] Fout bij uitlezen batterij: {e}")

        time.sleep(10) # Batterijstatus hoeft niet realtime

if __name__ == "__main__":
    try:
        run_power_monitor()
    except KeyboardInterrupt:
        print("[PowerManager] Gestopt.")