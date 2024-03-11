import time
from module_itg3205 import i2c_itg3205

gyro = i2c_itg3205(1)  # порт может быть различным в зависимости от вашей конфигурации Raspberry Pi
azimuth = 0

try:
    while True:
        start_time = time.time()
        (x, y, z) = gyro.getDegPerSecAxes()
        time.sleep(0.01)
        end_time = time.time()
        sleep_time = end_time - start_time
        azimuth += (z + 0.37537391304347356) * sleep_time
        azimuth = azimuth % 360  # удерживаем азимут в пределах [0, 360)
        print(f"Azimuth: {azimuth} degrees")
except KeyboardInterrupt:
    pass
