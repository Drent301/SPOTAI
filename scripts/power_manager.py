import sys
import os
import time

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
    print("[PowerDriver] WAARSCHUWING: 'smbus2' niet gevonden. Kan de PowerDriver niet starten.")
    I2C_ENABLED = False


class PowerDriver:
    """
    Bedient de PiSugar 3 batterijmonitor via I2C.
    """
    def __init__(self):
        if not I2C_ENABLED:
            raise ImportError("De 'smbus2' library is niet gevonden. Installeer met 'pip install smbus2'.")

        try:
            self.bus = SMBus(I2C_BUS)
            # Test-leesactie om te controleren of het apparaat aanwezig is
            self.bus.read_byte_data(PISUGAR_ADDR, 0x00)
            print(f"[PowerDriver] PiSugar 3 (I2C) succesvol verbonden op adres {hex(PISUGAR_ADDR)}.")
        except FileNotFoundError:
            self.bus = None
            raise ConnectionError(f"I2C-bus {I2C_BUS} niet gevonden. Controleer de Raspberry Pi configuratie.")
        except Exception as e:
            self.bus = None
            raise ConnectionError(f"Kon I2C-apparaat niet vinden op {hex(PISUGAR_ADDR)}: {e}")

    def _read_i2c_register(self, register):
        """ Leest een specifiek register van de PiSugar. """
        try:
            if register == "voltage":
                high_byte = self.bus.read_byte_data(PISUGAR_ADDR, 0x22)
                low_byte = self.bus.read_byte_data(PISUGAR_ADDR, 0x23)
                voltage_mv = (high_byte << 8) | low_byte
                return voltage_mv / 1000.0
            elif register == "percentage":
                return self.bus.read_byte_data(PISUGAR_ADDR, 0x2A)
            elif register == "is_charging":
                status_byte = self.bus.read_byte_data(PISUGAR_ADDR, 0x02)
                return bool((status_byte >> 7) & 1)
            else:
                raise ValueError(f"Onbekend register: {register}")
        except Exception as e:
            # Her-raise de exceptie met meer context
            raise IOError(f"Fout bij het lezen van I2C-register '{register}': {e}") from e

    def get_power_stats(self) -> dict:
        """ Leest de batterijstatus uit van de I2C-bus. """
        try:
            voltage = self._read_i2c_register("voltage")
            percentage = self._read_i2c_register("percentage")
            is_charging = self._read_i2c_register("is_charging")

            return {
                "voltage": round(voltage, 2),
                "percentage": round(percentage, 1),
                "is_charging": is_charging,
                "error": None
            }
        except (IOError, ValueError) as e:
            print(f"[PowerDriver] Fout bij ophalen power stats: {e}")
            # Geef een duidelijke foutstatus terug die de rest van het systeem kan gebruiken
            return {
                "voltage": 0.0,
                "percentage": 0.0,
                "is_charging": False,
                "error": str(e)
            }


def main():
    """
    De hoofdloop voor de power manager service.
    """
    try:
        bus = StateBus()
        driver = PowerDriver()
    except (ImportError, ConnectionError) as e:
        print(f"[PowerManager] KRITISCHE FOUT bij initialisatie: {e}")
        print("[PowerManager] Service wordt gestopt.")
        return # Stop de service als de driver niet kan initialiseren

    print("[PowerManager] Service gestart. Monitort batterijstatus.")

    try:
        while True:
            try:
                stats = driver.get_power_stats()

                # Schrijf de status naar de StateBus
                bus.set_value("power_state", stats)
                bus.set_value("is_charging", stats["is_charging"]) # Ook apart voor de Arbiter

                if stats.get("error"):
                     print(f"[PowerManager] Fout bij uitlezen batterij: {stats['error']}")
                elif stats["percentage"] < 10.0:
                    print(f"[PowerManager] WAARSCHUWING: Batterij bijna leeg! {stats['percentage']}%")

            except Exception as e:
                print(f"[PowerManager] Onverwachte fout in hoofdloop: {e}")

            time.sleep(10) # Batterijstatus hoeft niet realtime
    except KeyboardInterrupt:
        print("[PowerManager] Gestopt.")

if __name__ == "__main__":
    main()
