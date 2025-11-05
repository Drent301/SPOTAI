import time
import os
import sys

# --- ECHTE HARDWARE IMPORTS ---
# Zorg ervoor dat 'RPi.GPIO' is geïnstalleerd: pip install RPi.GPIO
try:
    import RPi.GPIO as GPIO
    GPIO_ENABLED = True
except (ImportError, RuntimeError):
    print("[CoolingDriver] WAARSCHUWING: 'RPi.GPIO' niet gevonden. Start in simulatiemodus.")
    GPIO_ENABLED = False

# --- Configuratie ---
FAN_ON_TEMP = 65.0  # Temp in C° om ventilator aan te zetten
FAN_OFF_TEMP = 55.0 # Temp in C° om ventilator uit te zetten
FAN_PIN = 18        # BCM pin-nummer voor de ventilator (pas dit aan)

class CoolingDriver:
    """
    Beheert de CPU-temperatuur en de ventilator-GPIO.
    """
    def __init__(self):
        self.is_fan_on = False
        self.sim_temp = 50.0

        if GPIO_ENABLED:
            try:
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(FAN_PIN, GPIO.OUT)
                GPIO.output(FAN_PIN, GPIO.LOW) # Zorg dat ventilator uit is bij start
                print(f"[CoolingDriver] Ventilator (GPIO {FAN_PIN}) geïnitialiseerd.")
            except Exception as e:
                print(f"[CoolingDriver] FOUT: Kon GPIO niet initialiseren: {e}")
                self.gpio_ok = False
        else:
            self.gpio_ok = False

    def get_cpu_temp(self) -> float:
        """ Leest de CPU-temperatuur uit m.b.v. 'vcgencmd'. """
        try:
            # Voer het vcgencmd commando uit en lees de output
            temp_str = os.popen("vcgencmd measure_temp").readline()
            # De output is "temp=XX.X'C"
            return float(temp_str.split("=")[1].split("'")[0])
        except Exception as e:
            # Fallback naar simulatie als vcgencmd faalt
            print(f"[CoolingDriver] FOUT: Kan 'vcgencmd' niet lezen: {e}. Val terug op simulatie.")
            self.sim_temp += 0.5 if not self.is_fan_on else -1.5
            return self.sim_temp

    def set_fan(self, state: bool):
        """ Zet de ventilator GPIO-pin aan of uit. """
        if state == self.is_fan_on:
            return # Geen wijziging

        self.is_fan_on = state
        
        if not self.gpio_ok:
            if state:
                print("[CoolingDriver] Ventilator AAN (Simulatie)")
            else:
                print("[CoolingDriver] Ventilator UIT (Simulatie)")
            return

        # --- Echte Hardware Actie ---
        try:
            if state:
                print("[CoolingDriver] Ventilator AAN")
                GPIO.output(FAN_PIN, GPIO.HIGH)
            else:
                print("[CoolingDriver] Ventilator UIT")
                GPIO.output(FAN_PIN, GPIO.LOW)
        except Exception as e:
            print(f"[CoolingDriver] FOUT: Kon ventilator-GPIO niet schakelen: {e}")
            self.gpio_ok = False # Stop met proberen

def main():
    """
    De hoofdloop voor de cooling control service.
    """
    driver = CoolingDriver()
    print("[CoolingControl] Service gestart. Monitort CPU-temperatuur.")

    try:
        while True:
            temp = driver.get_cpu_temp()

            if temp > FAN_ON_TEMP:
                driver.set_fan(True)
            elif temp < FAN_OFF_TEMP:
                driver.set_fan(False)

            # print(f"DEBUG: Temp={temp}°C, FanOn={driver.is_fan_on}")
            time.sleep(5) # Temperatuur hoeft niet realtime

    except KeyboardInterrupt:
        print("[CoolingControl] Gestopt.")
    finally:
        if GPIO_ENABLED:
            print("[CoolingControl] GPIO opschonen...")
            GPIO.cleanup() # Zet pinnen terug naar input

if __name__ == "__main__":
    main()