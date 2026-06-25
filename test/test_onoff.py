"""
test_onoff.py
=============
SSH 接続オン/オフ 切り替え試験プログラム

NSE2026/test/test_onoff.py

シーケンス:
    Step 1 [SSH ON  /  10 s]
        BCM21 番ピンの LED を点灯しっぱなし。
        SSH 接続は有効のままで動作確認できる。

    Step 2 [SSH OFF /  30 s]
        systemctl stop ssh で SSH サービスを停止。
        LED を 1 秒間隔で点滅させ「接続オフ中」を知らせる。

    Step 3 [SSH 復活 → 再接続待機 → LED ON / 10 s]
        systemctl start ssh で SSH サービスを再起動。
        ポート 22 が LISTEN 状態になるまで待機し、
        復活を確認できたら LED を再び点灯しっぱなし。
        10 秒後に LED 消灯して終了。

GPIO:
    LED_PIN = BCM 21  (gpiozero LED, lgpio バックエンド)
    ← 全テストコードと同じ gpiozero + LGPIOFactory

SSH 制御:
    sudo systemctl stop  ssh   (または sshd)
    sudo systemctl start ssh
    ※ ラズパイで passwordless sudo が必要。
      /etc/sudoers に以下を追加しておくこと:
        pi ALL=(ALL) NOPASSWD: /bin/systemctl start ssh, /bin/systemctl stop ssh

動作確認 (SSH なし環境でのドライランは SSH_DRY_RUN=True で可能):
    SSH_DRY_RUN = True にするとサービス停止/起動をスキップし、
    LED タイミングだけを確認できる。
"""

import subprocess
import socket
import time
import datetime
import sys

# --- gpiozero (test_run.py / fall.py / test_straight.py と同じバックエンド) ---
from gpiozero import LED, Device
from gpiozero.pins.lgpio import LGPIOFactory

# ===========================================================================
# 設定
# ===========================================================================

LED_PIN         = 21       # BCM 番号

SSH_ON_SEC      = 10.0    # Step1: SSH ON 中に LED 点灯する時間 [s]
SSH_OFF_SEC     = 30.0    # Step2: SSH OFF 中に LED 点滅する時間 [s]
BLINK_INTERVAL  = 1.0     # Step2: 点滅周期 [s]  (ON:0.5s / OFF:0.5s)
RECONNECT_SEC   = 10.0    # Step3: SSH 復活後に LED 点灯する時間 [s]

SSH_SERVICE     = "ssh"   # systemctl のサービス名 ("sshd" の環境もあり)
SSH_PORT        = 22
RECONNECT_TIMEOUT = 30.0  # SSH が LISTEN 状態になるまで待つ最大時間 [s]
RECONNECT_POLL  = 0.5     # 再接続確認のポーリング間隔 [s]

# True にすると systemctl stop/start をスキップして LED タイミングだけ確認できる
SSH_DRY_RUN     = False

# ===========================================================================
# SSH 制御ユーティリティ
# ===========================================================================

def _run(cmd: list[str]) -> bool:
    """コマンドを実行。成功 True / 失敗 False を返す。"""
    try:
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=10
        )
        if result.returncode != 0:
            print(f"[WARN] コマンド失敗: {' '.join(cmd)}")
            print(f"       stderr: {result.stderr.decode().strip()}")
            return False
        return True
    except Exception as e:
        print(f"[ERROR] コマンド実行エラー: {e}")
        return False


def ssh_stop() -> bool:
    """SSH サービスを停止する。"""
    if SSH_DRY_RUN:
        print("[DRY-RUN] SSH サービス停止をスキップ")
        return True
    print(f"[SSH] サービス停止: sudo systemctl stop {SSH_SERVICE}")
    return _run(["sudo", "systemctl", "stop", SSH_SERVICE])


def ssh_start() -> bool:
    """SSH サービスを起動する。"""
    if SSH_DRY_RUN:
        print("[DRY-RUN] SSH サービス起動をスキップ")
        return True
    print(f"[SSH] サービス起動: sudo systemctl start {SSH_SERVICE}")
    return _run(["sudo", "systemctl", "start", SSH_SERVICE])


def wait_ssh_listen(timeout: float = RECONNECT_TIMEOUT) -> bool:
    """
    SSH がポート 22 で LISTEN するまでポーリングで待機する。

    Returns
    -------
    bool
        True  : LISTEN 確認
        False : タイムアウト
    """
    if SSH_DRY_RUN:
        print("[DRY-RUN] SSH LISTEN 確認をスキップ (0.5 s 待機)")
        time.sleep(0.5)
        return True

    print(f"[SSH] ポート {SSH_PORT} の LISTEN を待機中 (最大 {timeout:.0f} s)...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", SSH_PORT), timeout=1.0):
                return True   # 接続できた = LISTEN 中
        except (ConnectionRefusedError, OSError):
            pass              # まだ起動していない
        time.sleep(RECONNECT_POLL)

    print(f"[WARN] {timeout:.0f} s 待っても SSH が LISTEN しませんでした。")
    return False

# ===========================================================================
# LED ヘルパー
# ===========================================================================

def blink_for(led: LED, duration: float, interval: float):
    """
    duration 秒間、interval 秒周期 (ON:半分 / OFF:半分) で点滅する。
    Ctrl+C が来た場合は呼び出し元に例外を伝播させる。
    """
    half     = interval / 2.0
    deadline = time.time() + duration
    while time.time() < deadline:
        led.on()
        remaining = deadline - time.time()
        time.sleep(min(half, max(0, remaining)))
        led.off()
        remaining = deadline - time.time()
        time.sleep(min(half, max(0, remaining)))

# ===========================================================================
# メイン
# ===========================================================================

def main():
    # --- gpiozero バックエンド設定 (全テストコードと統一) ---
    Device.pin_factory = LGPIOFactory()

    led = LED(LED_PIN)

    print("=" * 60)
    print("  test_onoff.py  SSH オン/オフ 切り替え試験")
    print(f"  {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    if SSH_DRY_RUN:
        print("  *** DRY-RUN モード: SSH サービス操作はスキップされます ***")
    print("=" * 60)

    try:
        # ============================================================
        # Step 1 : SSH ON — LED 点灯  (SSH_ON_SEC 秒)
        # ============================================================
        print(f"\n[Step1] SSH ON  — LED 点灯 ({SSH_ON_SEC:.0f} 秒)")
        led.on()
        time.sleep(SSH_ON_SEC)

        # ============================================================
        # Step 2 : SSH 停止 → LED 点滅  (SSH_OFF_SEC 秒)
        # ============================================================
        print(f"\n[Step2] SSH 停止 → LED 点滅 ({SSH_OFF_SEC:.0f} 秒 / {BLINK_INTERVAL:.0f} s 間隔)")
        led.off()
        ssh_stop()

        blink_for(led, SSH_OFF_SEC, BLINK_INTERVAL)
        led.off()   # 点滅後は確実に消灯

        # ============================================================
        # Step 3 : SSH 復活 → LISTEN 待機 → LED 点灯  (RECONNECT_SEC 秒)
        # ============================================================
        print(f"\n[Step3] SSH 復活中...")
        ssh_start()

        listen_ok = wait_ssh_listen(timeout=RECONNECT_TIMEOUT)

        if listen_ok:
            print(f"[Step3] SSH LISTEN 確認！ — LED 点灯 ({RECONNECT_SEC:.0f} 秒)")
        else:
            print(f"[Step3] タイムアウト。SSH が応答しませんでしたが続行します。")

        led.on()
        time.sleep(RECONNECT_SEC)
        led.off()

        print("\n[INFO] 試験完了。")

    except KeyboardInterrupt:
        print("\n\n[INFO] Ctrl+C — 中断します。")
        # SSH が停止中に中断された場合でも必ず再起動する
        print("[INFO] SSH サービスを念のため再起動します...")
        ssh_start()

    finally:
        led.off()
        led.close()
        print("[INFO] LED リソースを解放しました。")


# ===========================================================================
if __name__ == "__main__":
    main()
