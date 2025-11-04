import time
import random

# Dit bestand simuleert de ECHTE 'smbus2' (I2C) library
# en de specifieke MPU6050 (IMU) en Grove (Touch) sensor libraries.
# Wanneer we dit op de Pi 5 testen, vervangen we dít bestand
# door de daadwerkelijke I2C-aanroepen.

class I2CDriver:
    """
    Een placeholder (mock) voor de echte I2C-sensoren.
    Beheert de IMU (balans) en Grove Touch (veiligheid) sensoren.
    """
    def __init__(self):
        # In een echte implementatie:
        # - bus = smbus2.SMBus(1)
        # - self.imu = MPU6050(bus)
        # - self.touch = GroveTouch(bus, 0x2A)
        print("[I2CDriver] Verbonden met I2C-bus (simulatie).")
        print("[I2CDriver] IMU (MPU6050) en Grove Touch sensoren geïnitialiseerd (simulatie).")
        self.running = True

    def get_imu_data(self) -> dict:
        """
        Simuleert het uitlezen van de IMU (gyroscoop en accelerometer).
        """
        # Unitree/ROS IMU Topic Conveties: /imu/data
        imu_accel_x = 0.0 + random.uniform(-0.01, 0.01)
        imu_gyro_z = 0.0 + random.uniform(-0.005, 0.005)

        return {"accel": [imu_accel_x, 0.0, 9.8], "gyro": [0.0, 0.0, imu_gyro_z]}

    def get_touch_data(self) -> dict:
        """
        Simuleert het uitlezen van de touch-sensor.
        """
        # Willekeurig 1 op de 100 keer een "touch" simuleren
        touch_sensor_back = random.random() < 0.01 
        return {"back": touch_sensor_back}