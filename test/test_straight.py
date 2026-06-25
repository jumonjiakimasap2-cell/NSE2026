from gpiozero import Motor, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import time
Device.pin_factory = LGPIOFactory()
# ピン設定（BCM番号）
pwm_a = PWMOutputDevice(13)  # PWMA
pwm_b = PWMOutputDevice(18)  # PWMB
motor_a = Motor(forward=5, backward=6)   # AIN1, AIN2
motor_b = Motor(forward=23, backward=24)  # BIN1, BIN2
SPEED    = 0.8   # 速度（0.0〜1.0）
DURATION = 10   # 前進する秒数
def stop():
    pwm_a.value = 0
    pwm_b.value = 0
    motor_a.stop()
    motor_b.stop()
def forward(speed):
    pwm_a.value = speed
    pwm_b.value = speed
    motor_a.forward()
    motor_b.forward()
try:
    print(f"直進テスト開始：{DURATION}秒間前進します")
    forward(SPEED)
    time.sleep(DURATION)
    stop()
    print("直進テスト完了")
except KeyboardInterrupt:
    print("\n強制終了")
finally:
    stop()
    pwm_a.close(); pwm_b.close()
    motor_a.close(); motor_b.close()
