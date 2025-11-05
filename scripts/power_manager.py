import sys
import os
import time
import random

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.statebus import StateBus

# --- ECHTE HARDWARE IMPORTS ---
# Zorg ervoor dat 'smbus2' is geïnstalleerd: pip install smbus2
try:
    from smbus2 import SMBus
    I2C_BUS = 1 # Pi 5 gebruikt bus 1
    PISUGAR_ADDR = 0x57 # Standaard I2C-adres voor PiSugar 3
    I2C_ENABLED = True
except ImportError:
    print("[PowerDriver] WAARSCHUWING: 'smbus2' niet gevonden. Start in simulatiemodus.")
    I2C_ENABLED = False


class PowerDriver:
    """
    Bedient de PiSugar 3 batterijmonitor via I2C.
    """
    def __init__(self):
        if I2C_ENABLED:
            try:
                self.bus = SMBus(I2C_BUS)
                # Test-leesactie om te controleren of het apparaat aanwezig is
                self.bus.read_byte_data(PISUGAR_ADDR, 0x00) 
                print(f"[PowerDriver] PiSugar 3 (I2C) succesvol verbonden op adres {hex(PISUGAR_ADDR)}.")
            except Exception as e:
                print(f"[PowerDriver] FOUT: Kon I2C-apparaat niet vinden op {hex(PISUGAR_ADDR)}: {e}")
                print("[PowerDriver] Val terug op simulatiemodus.")
                self.bus = None
        else:
            self.bus = None # Draai in simulatiemodus
        
        # Simulatie fallback waarden
        self.sim_percentage = 100.0

    def _read_i2c_register(self, register):
        """ Leest een specifiek register van de PiSugar. """
        # TODO: Implementeer hier de specifieke I2C-leescommando's voor PiSugar 3
        # Dit is een placeholder-implementatie
        if register == "voltage":
            # (Vervang dit door echte bus.read_word_data() etc.)
            return random.uniform(3.7, 4.2) 
        if register == "percentage":
             # (Vervang dit door echte bus.read_byte_data() etc.)
            return random.uniform(80.0, 81.0) # Simuleer 80%
        if register == "is_charging":
             # (Vervang dit door echte bus.read_byte_data() etc.)
            return False
        return 0

    def get_power_stats(self) -> dict:
        """ Leest de batterijstatus uit van de I2C-bus. """
        
        if self.bus is None:
            # --- Simulatie Modus ---
            self.sim_percentage -= 0.05
            if self.sim_percentage < 1: self.sim_percentage = 100.0
            return {
                "voltage": 3.8 + (self.sim_percentage / 100 * 0.4),
                "percentage": round(self.sim_percentage, 1),
                "is_charging": False
            }

        # --- Echte Hardware Modus ---
        try:
            # TODO: Vervang dit door de echte register-adressen
            voltage = self._read_i2c_register("voltage")
            percentage = self._read_i2c_register("percentage")
            is_charging = self._read_i2c_register("is_charging")

            return {
                "voltage": round(voltage, 2),
                "percentage": round(percentage, 1),
                "is_charging": is_charging
            }
        except Exception as e:
            print(f"[PowerDriver] Fout bij uitlezen I2C: {e}. Val terug op simulatie.")
            self.bus = None # Stop met proberen als het faalt
            return self.get_power_stats() # Roep opnieuw aan (nu in simulatiemodus)


def main():
    """
    De hoofdloop voor de power manager service.
    """
    bus = StateBus()
    driver = PowerDriver()

    print("[PowerManager] Service gestart. Monitort batterijstatus.")

    try:
        while True:
            try:
                stats = driver.get_power_stats()

                # Schrijf de status naar de StateBus
                bus.set_value("power_state", stats)
                bus.set_value("is_charging", stats["is_charging"]) # Ook apart voor de Arbiter

                if stats["percentage"] < 10.0:
                    print(f"[PowerManager] WAARSCHUWING: Batterij bijna leeg! {stats['percentage']}%")

            except Exception as e:
                print(f"[PowerManager] Fout in hoofdloop: {e}")

            time.sleep(10) # Batterijstatus hoeft niet realtime
    except KeyboardInterrupt:
        print("[PowerManager] Gestopt.")

if __name__ == "__main__":
    main()