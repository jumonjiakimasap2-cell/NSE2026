"""
rpi_blink.py
============
【ラズパイ側】SSH切断中LED点滅 & 自動SSH復活スクリプト

NSE2026/test/rpi_blink.py

このスクリプトはPCのtest_onoff_pc.pyから
ラズパイにアップロードされ、nohupで実行される。

動作:
    1. SSHサービスを停止
    2. SSH_OFF_SEC 秒間 LED を点滅 (1秒周期)
    3. SSHサービスを再起動
    4. LED を RECONNECT_SEC 秒間点灯して終了

重要:
    - nohup で SSH セッションとは独立して動作する
    - gpiozero + LGPIOFactory (NSE2026全体と統一)
    - BCM21番ピン
"""

import time
import subprocess
import sys

from gpiozero import LED, Device
from gpiozero.pins.lgpio import LGPIOFactory

# ── 設定 (PC側スクリプトから引数で上書き可能) ──
LED_PIN       = 21
SSH_OFF_SEC   = 60.0   # 点滅時間 [s]
RECONNECT_SEC = 10.0   # SSH復活後の点灯時間 [s]
BLINK_ON      = 0.5    # 点灯時間 [s]
BLINK_OFF     = 0.5    # 消灯時間 [s]

# コマンドライン引数で上書き (test_onoff_pc.py が渡す)
if len(sys.argv) >= 3:
    SSH_OFF_SEC   = float(sys.argv[1])
    RECONNECT_SEC = float(sys.argv[2])

# ── GPIO 初期化 ──
Device.pin_factory = LGPIOFactory()
led = LED(LED_PIN)

def log(msg):
    """タイムスタンプ付きログ (nohup実行時はファイルに残る)"""
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)

log(f"rpi_blink.py 起動  OFF={SSH_OFF_SEC}s  RECONNECT={RECONNECT_SEC}s")

try:
    # ── Step1: SSHサービス停止 ──
    log("SSHサービス停止中...")
    subprocess.run(["sudo", "systemctl", "stop", "ssh"], timeout=10)
    log("SSHサービス停止完了")

    # ── Step2: 点滅ループ (SSH_OFF_SEC 秒間) ──
    log(f"LED点滅開始 ({SSH_OFF_SEC:.0f}秒間)")
    deadline = time.time() + SSH_OFF_SEC
    while time.time() < deadline:
        led.on()
        time.sleep(min(BLINK_ON,  max(0, deadline - time.time())))
        led.off()
        time.sleep(min(BLINK_OFF, max(0, deadline - time.time())))
    led.off()
    log("LED点滅終了")

    # ── Step3: SSHサービス再起動 ──
    log("SSHサービス再起動中...")
    subprocess.run(["sudo", "systemctl", "start", "ssh"], timeout=15)
    log("SSHサービス再起動完了")

    # ── Step4: 再接続確認用 点灯 ──
    log(f"LED点灯 ({RECONNECT_SEC:.0f}秒間) — PC側が再接続できます")
    led.on()
    time.sleep(RECONNECT_SEC)
    led.off()
    log("完了。LED消灯。")

except Exception as e:
    log(f"エラー: {e}")
    # エラー時も必ずSSHを復活させる
    subprocess.run(["sudo", "systemctl", "start", "ssh"], timeout=15)
    led.off()

finally:
    led.close()
