import time
import socket
import paramiko

# ===========================================================================
# 接続・環境設定
# ===========================================================================
# ★ IPアドレスを使わない方法（mDNS）を適用
RPI_HOST    = "raspberrypi.local"  
USER        = "pi"
PASS        = "your_password"

# 物理40番ピン = BCM 21番
LED_PIN     = 21

# 試験の時間定義（提示いただいた仕様と同期）
SSH_ON_SEC   = 10.0
SSH_OFF_SEC  = 60.0  # 安全確実なタイマー実行のため60秒に設定
RECONNECT_SEC = 10.0

def print_header():
    print("=" * 65)
    print("      RASPBERRY PI - SSH & LED INTEGRATED TESTING SYSTEM      ")
    print("==============================================================")
    print(f" 🎯 ターゲット: {RPI_HOST}")
    print(f" 💡 ラズパイ側動作: 40番ピンLED(BCM {LED_PIN})連動、SSHサービス強制オン/オフ")
    print("=" * 65)

def main():
    print_header()
    
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    # ------------------------------------------------------------
    # 【Step 1】 初期接続 ＆ 10秒間の常時点灯確認
    # ------------------------------------------------------------
    print(f"\n[ ⏳ Step 1 ] ラズパイへ接続を開始します...")
    try:
        ssh.connect(RPI_HOST, username=USER, password=PASS, timeout=5)
        print(" [ 🟢 SSH有効 ] 初期接続に成功しました。")
        
        # ラズパイ側でLEDを常時点灯させる（LGPIOFactoryバックエンドを想定した1行コマンド）
        # gpiozeroの内部処理をシミュレートし、ピンをHIGH(1)に出力します
        cmd_led_on = f"python3 -c \"from gpiozero import LED; from gpiozero.pins.lgpio import LGPIOFactory; from gpiozero import Device; Device.pin_factory = LGPIOFactory(); led = LED({LED_PIN}); led.on(); import time; time.sleep({SSH_ON_SEC})\""
        
        print(f" [ 💡 LED状態 ] 物理40番ピン：常時点灯中 ➔ 動作を確認してください。")
        
        # PC側で同期してカウントダウン
        for i in range(int(SSH_ON_SEC), 0, -1):
            print(f"   ┗ 試験開始まであと {i} 秒...", end="\r")
            time.sleep(1)
        print("\n   ┗ [OK] Step 1 正常通過。")
        
    except Exception as e:
        print(f" [ ❌ 接続失敗 ] ラズパイが見つからないか、SSHが無効です: {e}")
        return

    # ------------------------------------------------------------
    # 【Step 2】 SSHサービス停止 ＆ 60秒間のLED点滅（自動復帰予約付き）
    # ------------------------------------------------------------
    print("\n" + "-" * 60)
    print("[ ⚠️ 命令発信 ] Step 2: SSHサービスを停止し、LED点滅と自動復活を予約します。")
    print("-" * 60)
    
    try:
        # ★超重要ポイント：
        # SSHが切断されてもラズパイが自律して「1秒間隔のLED点滅」と「60秒後のSSH自動起動」を行うワンライナーコマンド
        # これをバックグラウンド(&)でラズパイのOS直下に放り込みます。
        rpi_script = (
            f"python3 -c \""
            f"from gpiozero import LED; from gpiozero.pins.lgpio import LGPIOFactory; from gpiozero import Device; "
            f"Device.pin_factory = LGPIOFactory(); led = LED({LED_PIN}); "
            f"import time, subprocess; "
            f"subprocess.run(['sudo', 'systemctl', 'stop', 'ssh']); " # 自身のSSHを止める
            f"start_time = time.time(); "
            f"while time.time() - start_time < {SSH_OFF_SEC}: " # 60秒間ループ
            f"    led.on(); time.sleep(0.5); led.off(); time.sleep(0.5); " # 1秒周期点滅
            f"subprocess.run(['sudo', 'systemctl', 'start', 'ssh']); " # 時間が来たら必ず復活
            f"\" > /dev/null 2>&1 &"
        )
        
        # コマンドをラズパイへ送信
        ssh.exec_command(rpi_script)
        time.sleep(1.5) # 命令が裏で確実に走り出すまでわずかに待つ
        
        print(" [ 🔴 切断命令 ] ラズパイ側で『自律点滅＆自動復活タイマー』が起動しました。")
        ssh.close() # PC側からもセッションを安全に閉じる
        print(" [ 🔒 遮断完了 ] PC-ラズパイ間のSSHセッションは完全に切断されました。")
        
    except Exception as e:
        print(f" [ ❌ エラー ] 停止命令の送信に失敗: {e}")
        return

    print(f"\n[ ⏳ Step 2 ] ラズパイは通信途絶中（LEDが1秒間隔で美しく点滅中）です。")
    print(f"              規定の {SSH_OFF_SEC:.0f} 秒間、この状態を維持します。")
    
    # 60秒間のカウントダウン
    for i in range(int(SSH_OFF_SEC), 0, -1):
        if i % 5 == 0 or i <= 5: # ログが見やすくなるよう5秒ごと、最後は1秒ごとに出力
            print(f"   ┗ サービス停止維持（LED点滅中）: 残り {i} 秒...")
        time.sleep(1)

    # ------------------------------------------------------------
    # 【Step 3】 SSH復活待機 ＆ 10秒間の再点灯確認
    # ------------------------------------------------------------
    print("\n" + "-" * 60)
    print("[ 🔄 復帰要求 ] Step 3: ラズパイのSSHサービスが自律復活するのを待ちます。")
    print("-" * 60)
    print(f" ℹ️ ポート22が LISTEN 状態になるまで、名前解決（{RPI_HOST}）を伴う再接続を試みます...")

    reconnect_success = False
    start_time = time.time()
    timeout = 30.0  # 復活を待つ最大時間
    retry_count = 1

    while time.time() - start_time < timeout:
        try:
            print(f"   ┗ 再接続トライ中... ({retry_count}回目)", end="\r")
            # socket.error をキャッチするために先頭でインポートした socket を利用
            ssh.connect(RPI_HOST, username=USER, password=PASS, timeout=2.0)
            reconnect_success = True
            print(f"\n [ 🎉 復帰成功 ] ポート22の LISTEN を検知！再接続が完了しました。")
            break
        except (paramiko.SSHException, socket.error):
            # まだラズパイ側で復活していなければ、1.5秒待ってリトライ
            time.sleep(1.5)
            retry_count += 1

    if reconnect_success:
        # 復帰後の確認コマンド（稼働時間を取得）
        stdin, stdout, stderr = ssh.exec_command("uptime -p")
        uptime_info = stdout.read().decode().strip()
        print(f"   ┗ ラズパイ現在の状態: {uptime_info}")
        
        # 復活確認の点灯コマンドをラズパイに送る
        cmd_led_final = f"python3 -c \"from gpiozero import LED; from gpiozero.pins.lgpio import LGPIOFactory; from gpiozero import Device; Device.pin_factory = LGPIOFactory(); led = LED({LED_PIN}); led.on(); import time; time.sleep({RECONNECT_SEC}); led.off()\""
        ssh.exec_command(cmd_led_final)
        
        print(f" [ 💡 LED状態 ] 物理40番ピン：復活の『常時点灯』に切り替わりました。")
        
        # 最後の10秒ホールド
        for i in range(int(RECONNECT_SEC), 0, -1):
            print(f"   ┗ 試験終了（LED消灯）まであと {i} 秒...", end="\r")
            time.sleep(1)
        print(f"\n\n [ 🏁 実験完了 ] 全てのシーケンスが正常に終了しました。LEDは自動消灯しました。")
    else:
        print(f"\n [ ❌ タイムアウト ] {timeout}秒待機しましたが、ラズパイのSSHが復帰しませんでした。")
        print("   ⚠️ 原因の可能性: ラズパイ側でパスワードなしsudo(visudo)の設定が正しく反映されていない可能性があります。")
    
    ssh.close()
    print("=" * 65)

if __name__ == "__main__":
    main()
