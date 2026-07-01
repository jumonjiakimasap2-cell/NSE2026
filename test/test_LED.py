import time
from gpiozero import LED
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device

# バックエンドを指定
Device.pin_factory = LGPIOFactory()

# 物理40番ピン = BCM 21番
led = LED(21)

print("LEDを5秒間点灯します...")
led.on()
time.sleep(5)
led.off()
print("消灯しました。")
