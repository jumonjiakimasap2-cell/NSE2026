"""
test_onoff_pc.py
================
【PC側】SSH接続オン/オフ 切り替え試験  ── Windows/Mac/Linux 対応

NSE2026/test/test_onoff_pc.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
【前回コードの問題点と対処】

問題1: ヒアドキュメント(cat << 'EOF')がWindowsのcmdで使えない
    → paramiko の sftp.put() でファイルを転送する方式に変更。
      ラズパイ側スクリプト rpi_blink.py を事前にアップロードして実行する。

問題2: python3 -c "..." の複数行コマンドが SSH 経由で壊れる
    → 複数行 Python を -c で渡すのは引用符のエスケープが環境依存で壊れやすい。
      スクリプトファイルとして転送して python3 ファイル名 で実行する方式に統一。

問題3: exec_command() はノンブロッキングで即座に返る
    → バックグラウンド起動 (&) をしているが SSH セッションが閉じると
      子プロセスが SIGHUP で死ぬ場合がある。
      → nohup + 出力リダイレクトで完全にセッションから切り離す。

問題4: SSHを止めた後に ssh.connect() が即座に失敗しない
    → paramiko はソケットキャッシュを保持するため、
      古い ssh オブジェクトを再利用すると誤判定する。
      → 再接続ループで毎回 SSHClient() を新規生成する。

問題5: Step2で stop命令を出した後 exec_command がタイムアウトする
    → stop と blink を同じセッションで連続実行すると
      stop が即座に効いて次の命令が届かない。
      → blink スクリプト(nohup) を先に起動 → 1秒待機 → stop を実行
        という順序に固定する。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

必要ライブラリ (PC側):
    pip install paramiko

ラズパイ側の事前準備:
    1. sudo visudo で以下を追加:
       pi ALL=(ALL) NOPASSWD: /bin/systemctl start ssh, /bin/systemctl stop ssh
    2. このスクリプトを実行すると rpi_blink.py が自動転送される。
       rpi_blink.py は test_onoff_pc.py と同じフォルダに置くこと。
"""

import time
import socket
import os
import sys

try:
    import paramiko
except ImportError:
    print("[ERROR] paramiko が見つかりません。")
    print("        pip install paramiko  を実行してください。")
    sys.exit(1)

# ===========================================================================
# 設定
# ===========================================================================

RPI_HOST      = "raspberrypi.local"   # または IPアドレス (例: "192.168.1.xx")
RPI_USER      = "pi"
RPI_PASS      = "pi"                  # 必要に応じて変更

LED_PIN       = 21                    # BCM番号 (NSE2026 全体と統一)

SSH_ON_SEC    = 10.0                  # Step1: SSH ON 中の LED 点灯時間 [s]
SSH_OFF_SEC   = 60.0                  # Step2: SSH OFF 中の LED 点滅時間 [s]
RECONNECT_SEC = 10.0                  # Step3: SSH 復活後の LED 点灯時間 [s]

# ラズパイ上のスクリプト配置先
REMOTE_SCRIPT = "/home/pi/rpi_blink.py"
LOCAL_SCRIPT  = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "rpi_blink.py")

# 再接続待機設定
RECONNECT_TIMEOUT = 90.0   # SSH復活待ちの最大時間 [s]
RECONNECT_POLL    = 2.0    # ポーリング間隔 [s]

# ===========================================================================
# ユーティリティ
# ===========================================================================

def new_ssh() -> paramiko.SSHClient:
    """毎回新しい SSHClient を返す (キャッシュ誤判定を防ぐ)"""
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    return client


def ssh_connect(host, user, passwd, timeout=10) -> paramiko.SSHClient | None:
    """接続試行。成功したら SSHClient を返す。失敗したら None。"""
    client = new_ssh()
    try:
        client.connect(host, username=user, password=passwd,
                       timeout=timeout,
                       allow_agent=False,     # Windowsでエージェントエラーを防ぐ
                       look_for_keys=False)   # 鍵ファイルを探さない (パスワード認証に専念)
        return client
    except Exception as e:
        try:
            client.close()
        except Exception:
            pass
        return None


def run_remote(ssh, cmd: str, timeout: int = 15) -> tuple[str, str]:
    """リモートコマンドを実行して (stdout, stderr) を返す。"""
    try:
        _, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
        out = stdout.read().decode("utf-8", errors="replace").strip()
        err = stderr.read().decode("utf-8", errors="replace").strip()
        return out, err
    except Exception as e:
        return "", str(e)


def upload_script(ssh) -> bool:
    """rpi_blink.py をラズパイへ SFTP 転送する。"""
    if not os.path.exists(LOCAL_SCRIPT):
        print(f"[ERROR] ローカルに rpi_blink.py が見つかりません: {LOCAL_SCRIPT}")
        print("        test_onoff_pc.py と同じフォルダに rpi_blink.py を置いてください。")
        return False
    try:
        sftp = ssh.open_sftp()
        sftp.put(LOCAL_SCRIPT, REMOTE_SCRIPT)
        sftp.close()
        print(f"  [転送OK] {LOCAL_SCRIPT} → {RPI_HOST}:{REMOTE_SCRIPT}")
        return True
    except Exception as e:
        print(f"  [ERROR] ファイル転送失敗: {e}")
        return False

# ===========================================================================
# 表示ヘルパー
# ===========================================================================

def hdr(title):
    print(f"\n{'─' * 60}")
    print(f"  {title}")
    print(f"{'─' * 60}")


def countdown(sec: float, label: str):
    """sec 秒のカウントダウンを表示する。"""
    for i in range(int(sec), 0, -1):
        print(f"   ⏳ {label}  あと {i:3d} 秒 ...", end="\r")
        time.sleep(1)
    print()

# ===========================================================================
# メイン
# ===========================================================================

def main():
    print("=" * 60)
    print("   SSH & LED 切り替え試験  (PC 側コントローラ)")
    print(f"   ターゲット : {RPI_HOST}  USER={RPI_USER}")
    print(f"   LED        : BCM {LED_PIN}")
    print(f"   タイムライン: ON {SSH_ON_SEC:.0f}s → OFF {SSH_OFF_SEC:.0f}s → ON {RECONNECT_SEC:.0f}s")
    print("=" * 60)

    # ======================================================
    # Step 1 : 初期接続 & LED 点灯  (SSH_ON_SEC 秒)
    # ======================================================
    hdr("Step 1 : SSH 接続 & LED 常時点灯")

    print(f"  接続中... {RPI_HOST}")
    ssh = ssh_connect(RPI_HOST, RPI_USER, RPI_PASS)
    if ssh is None:
        print("[ERROR] ラズパイに接続できません。")
        print("        ・IPアドレスまたはホスト名を確認してください")
        print("        ・ラズパイの SSH が有効か確認してください")
        print("        ・同じネットワークにいるか確認してください")
        return

    print("  [🟢 接続成功]")

    # rpi_blink.py をアップロード
    print("  rpi_blink.py をラズパイへ転送中...")
    if not upload_script(ssh):
        ssh.close()
        return

    # Step1: LED 点灯 (SSH セッションから独立して動かす必要はないので直接実行)
    led_on_cmd = (
        f"python3 -c '"
        f"from gpiozero import LED, Device; "
        f"from gpiozero.pins.lgpio import LGPIOFactory; "
        f"Device.pin_factory = LGPIOFactory(); "
        f"led = LED({LED_PIN}); "
        f"led.on(); "
        f"import time; time.sleep({SSH_ON_SEC}); "
        f"led.off()"
        f"' &"
    )
    run_remote(ssh, led_on_cmd)
    print(f"  [💡 LED 点灯] BCM{LED_PIN} を {SSH_ON_SEC:.0f} 秒間点灯します")
    countdown(SSH_ON_SEC, "Step1 (LED点灯中)")

    # ======================================================
    # Step 2 : rpi_blink.py を nohup 起動 → SSH停止
    # ======================================================
    hdr("Step 2 : SSH 停止 & LED 点滅 (ラズパイ自律動作)")

    # ── nohup でラズパイ側スクリプトを完全独立起動 ──
    # SSH セッションが閉じても nohup + & で生き続ける
    nohup_cmd = (
        f"nohup python3 {REMOTE_SCRIPT} "
        f"{SSH_OFF_SEC} {RECONNECT_SEC} "
        f"> /tmp/rpi_blink.log 2>&1 &"
    )
    print(f"  ラズパイ側スクリプトを nohup 起動中...")
    run_remote(ssh, nohup_cmd)

    # ★ 点滅スクリプトが起動して LED が光り始めるまで少し待つ ★
    # (この間に rpi_blink.py がSSHを止めるので、その前に確認)
    time.sleep(2.0)

    # スクリプトが起動したか PID を確認
    out, _ = run_remote(ssh, "pgrep -f rpi_blink.py")
    if out:
        print(f"  [OK] rpi_blink.py 起動確認 (PID: {out.strip()})")
    else:
        print("  [WARN] PID確認できず。スクリプトが起動しているか不明です。")

    # SSH セッションを明示的に閉じる
    ssh.close()
    print("  [🔒 切断] PC ← → ラズパイ の SSH セッションを切断しました")
    print(f"  [🔴 点滅中] ラズパイが自律的に {SSH_OFF_SEC:.0f} 秒間 LED を点滅させます")
    print(f"  ※ この間はラズパイの SSH に接続できません")
    countdown(SSH_OFF_SEC, "Step2 (LED点滅中 / SSH切断中)")

    # ======================================================
    # Step 3 : SSH 復活待機 & 再接続 & LED 点灯確認
    # ======================================================
    hdr("Step 3 : SSH 復活待機 & 再接続確認")

    print(f"  ラズパイの SSH が復活するまで待機中 (最大 {RECONNECT_TIMEOUT:.0f} s)...")
    print(f"  ※ ラズパイ側が自動で SSH を再起動します")

    reconnect_ok = False
    start        = time.time()
    attempt      = 0

    while time.time() - start < RECONNECT_TIMEOUT:
        attempt += 1
        elapsed = time.time() - start
        print(f"   🔄 再接続試行 {attempt}回目  ({elapsed:.0f}s経過) ...", end="\r")

        # ★ 毎回 新しい SSHClient を生成する (問題4の対処) ★
        ssh2 = ssh_connect(RPI_HOST, RPI_USER, RPI_PASS, timeout=3)
        if ssh2 is not None:
            print(f"\n  [🎉 再接続成功！] {attempt}回目 ({elapsed:.0f}s経過)")
            reconnect_ok = True
            break

        time.sleep(RECONNECT_POLL)

    if not reconnect_ok:
        print(f"\n  [❌ タイムアウト] {RECONNECT_TIMEOUT:.0f}s 待ちましたが SSH が復活しませんでした。")
        print("  ラズパイの /tmp/rpi_blink.log を直接確認してください。")
        return

    # 再接続後の確認情報
    out, _ = run_remote(ssh2, "uptime -p")
    print(f"  ラズパイ稼働時間: {out}")

    # ── ログ確認 ──
    out, _ = run_remote(ssh2, "tail -5 /tmp/rpi_blink.log")
    if out:
        print(f"  [ラズパイ側ログ (末尾5行)]")
        for line in out.splitlines():
            print(f"    {line}")

    # ── LED 点灯確認 ──
    # rpi_blink.py がすでに RECONNECT_SEC 秒の点灯をやっているはず
    # 念のためまだ点灯中か確認
    out, _ = run_remote(ssh2, "pgrep -f rpi_blink.py")
    if out:
        print(f"  [💡 LED] rpi_blink.py が LED を点灯継続中 (PID: {out.strip()})")
        countdown(RECONNECT_SEC, "Step3 (LED再点灯確認中)")
    else:
        print(f"  [💡 LED] rpi_blink.py は完了済み (正常終了)")

    ssh2.close()
    print("\n" + "=" * 60)
    print("  🏁 全シーケンス正常完了")
    print("=" * 60)


# ===========================================================================
if __name__ == "__main__":
    main()
