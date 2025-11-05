import time
import random

# Drempelwaarden
FAN_ON_TEMP = 65.0
FAN_OFF_TEMP = 55.0

class CoolingDriver:
    """
    Een placeholder (mock) voor de echte 'vcgencmd' en 'RPi.GPIO' libraries.
    """
    def __init__(self):
        # In een echte implementatie:
        # - GPIO.setmode(GPIO.BCM)
        # - GPIO.setup(FAN_PIN, GPIO.OUT)
        print("[CoolingDriver] Ventilator (GPIO) geïnitialiseerd (simulatie).")
        self.is_fan_on = False
        self.current_temp = 50.0

    def get_cpu_temp(self) -> float:
        """ Simuleert het lezen van 'vcgencmd measure_temp'. """
        # Simuleer temperatuurschommelingen
        if self.is_fan_on:
            self.current_temp -= 1.5 # Koelt af
        else:
            self.current_temp += 0.5 # Warmt op

        # Zorg dat het binnen realistische grenzen blijft
        self_heating = random.uniform(0.1, 0.5)
        self.current_temp = max(45.0, min(75.0, self.current_temp + self_heating))
        return self.current_temp

    def set_fan(self, state: bool):
        """ Simuleert het aan/uit zetten van de ventilator GPIO-pin. """
        if state == self.is_fan_on:
            return # Geen wijziging

        self.is_fan_on = state
        if state:
            print("[CoolingDriver] Ventilator AAN")
            # Echte code: GPIO.output(FAN_PIN, GPIO.HIGH)
        else:
            print("[CoolingDriver] Ventilator UIT")
            # Echte code: GPIO.output(FAN_PIN, GPIO.LOW)

def run_cooling_loop():
    """
    De hoofdloop voor de cooling control service.
    """
    driver = CoolingDriver()
    print("[CoolingControl] Service gestart. Monitort CPU-temperatuur.")

    while True:
        temp = driver.get_cpu_temp()

        if temp > FAN_ON_TEMP:
            driver.set_fan(True)
        elif temp < FAN_OFF_TEMP:
            driver.set_fan(False)

        # print(f"DEBUG: Temp={temp}°C, FanOn={driver.is_fan_on}")
        time.sleep(5) # Temperatuur hoeft niet realtime

if __name__ == "__main__":
    try:
        run_cooling_loop()
    except KeyboardInterrupt:
        print("[CoolingControl] Gestopt.")