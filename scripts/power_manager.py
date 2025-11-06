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
        # --- ECHTE I2C IMPLEMENTATIE ---
        if self.bus is None:
            # Dit zou niet aangeroepen moeten worden als er geen bus is, maar als fallback.
            return 0

        try:
            if register == "voltage":
                # Registers 0x22 (hoog) en 0x23 (laag) voor voltage in mV
                high_byte = self.bus.read_byte_data(PISUGAR_ADDR, 0x22)
                low_byte = self.bus.read_byte_data(PISUGAR_ADDR, 0x23)
                voltage_mv = (high_byte << 8) | low_byte
                return voltage_mv / 1000.0

            if register == "percentage":
                # Register 0x2A voor batterijpercentage
                percentage = self.bus.read_byte_data(PISUGAR_ADDR, 0x2A)
                return percentage

            if register == "is_charging":
                # De 7e bit van register 0x02 geeft externe stroom aan
                status_byte = self.bus.read_byte_data(PISUGAR_ADDR, 0x02)
                is_powered = (status_byte >> 7) & 1
                return bool(is_powered)

        except Exception as e:
            print(f"[PowerDriver] Fout bij het lezen van register '{register}': {e}")
            # Val terug op simulatiemodus bij een I2C-fout
            self.bus = None
            return 0 # Geef een veilige waarde terug

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