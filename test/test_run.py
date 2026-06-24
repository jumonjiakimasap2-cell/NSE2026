import RPi.GPIO as GPIO
import time
import sys

# ===== ピン設定 =====
PWMA = 33
AIN1 = 29
AIN2 = 31
PWMB = 18
BIN1 = 12
BIN2 = 16

frequency = 100  # PWM周波数（少し上げた方が安定）

GPIO.setmode(GPIO.BCM)

GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)

# PWM
pwmA = GPIO.PWM(PWMA, frequency)
pwmB = GPIO.PWM(PWMB, frequency)

pwmA.start(0)
pwmB.start(0)

speed = 80  # デフォルト速度（0〜100）

# ===== モーター制御 =====
def stop():
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    GPIO.output(AIN1, 0)
    GPIO.output(AIN2, 0)
    GPIO.output(BIN1, 0)
    GPIO.output(BIN2, 0)

def forward():
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)
    GPIO.output(AIN1, 1)
    GPIO.output(AIN2, 0)
    GPIO.output(BIN1, 1)
    GPIO.output(BIN2, 0)

def backward():
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)
    GPIO.output(AIN1, 0)
    GPIO.output(AIN2, 1)
    GPIO.output(BIN1, 0)
    GPIO.output(BIN2, 1)

def left():
    pwmA.ChangeDutyCycle(speed * 0.5)
    pwmB.ChangeDutyCycle(speed)
    GPIO.output(AIN1, 1)
    GPIO.output(AIN2, 0)
    GPIO.output(BIN1, 1)
    GPIO.output(BIN2, 0)

def right():
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed * 0.5)
    GPIO.output(AIN1, 1)
    GPIO.output(AIN2, 0)
    GPIO.output(BIN1, 1)
    GPIO.output(BIN2, 0)

# ===== キー入力（1文字取得）=====
def get_key():
    try:
        import termios, tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch.lower()
    except:
        return input().strip().lower()

# ===== メイン =====
try:
    print("操作開始：W/A/S/Dで移動、spaceで停止、qで終了")

    while True:
        cmd = get_key()

        if cmd == 'w':
            print("Forward")
            forward()

        elif cmd == 's':
            print("Backward")
            backward()

        elif cmd == 'a':
            print("Left")
            left()

        elif cmd == 'd':
            print("Right")
            right()

        elif cmd == ' ':
            print("Stop")
            stop()

        elif cmd == 'q':
            print("終了")
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    print("強制終了")

finally:
    stop()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
