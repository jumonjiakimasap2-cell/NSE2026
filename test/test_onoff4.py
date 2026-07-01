import time
import socket
import paramiko

# ===========================================================================
# 接続・環境設定
# ===========================================================================
RPI_HOST    = "raspberrypi.local"  
USER        = "pi"              # ご自身のユーザー名に設定してください
PASS        = "pi"   # ご自身のパスワードに設定してください

# 物理40番ピン = BCM 21番
LED_PIN     = 21

# 試験の時間定義
SSH_ON_SEC   = 10.0
SSH_OFF_SEC  = 60.0  
RECONNECT_SEC = 10.0

def print_header():
    print("=" * 65)
    print("      RASPBERRY PI - SSH & LED INTEGRATED TESTING SYSTEM      ")
    print("==============================================================")
    print(f" 🎯 ターゲット: {RPI_HOST}")
    print(f" 💡 ラズパイ側動作: LED(BCM {LED_PIN})連動、SSHサービス強制オン/オフ")
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
        
        # Windowsのコマンドプロンプト経由でも100%エラーにならないトリプルクォート構造に修正
        cmd_led_on = (
            f"python3 -c \"\n"
            f"from gpiozero import LED\n"
            f"from gpiozero.pins.lgpio import LGPIOFactory\n"
            f"from gpiozero import Device\n"
            f"Device.pin_factory = LGPIOFactory()\n"
            f"led = LED({LED_PIN})\n"
            f"led.on()\n"
            f"import time\n"
            f"time.sleep({SSH_ON_SEC})\n"
            f"\""
        )
        # バックグラウンドで実行して即座に進行させる
        ssh.exec_command(f"{cmd_led_on} > /dev/null 2>&1 &")
        
        print(f" [ 💡 LED状態 ] 物理40番ピン：常時点灯中 ➔ 動作を確認してください。")
        
        for i in range(int(SSH_ON_SEC), 0, -1):
            print(f"   ┗ 試験開始まであと {i} 秒...", end="\r")
            time.sleep(1)
        print("\n   ┗ [OK] Step 1 正常通過。")
        
    except Exception as e:
        print(f" [ ❌ 接続失敗 ] ラズパイが見つからないか、SSHが無効です: {e}")
        return

   # ------------------------------------------------------------
    # 【Step 2】 SSHサービス停止 ＆ 60秒間のLED点滅（2フェーズ安全分離版）
    # ------------------------------------------------------------
    print("\n" + "-" * 60)
    print("[ ⚠️ 命令発信 ] Step 2: SSHサービスを停止し、LED点滅と自動復活を予約します。")
    print("-" * 60)
    
    try:
        # 【1番目の命令】まず「LEDを60秒間点滅させ、最後にSSHを起動する」というPythonを
        # nohupを使ってラズパイのバックグラウンドで完全に切り離して起動します。
        # 単純な1行コマンド（セミコロン区切り）にすることで、Windowsの解釈エラーを完全に防ぎます。
        led_and_recovery_cmd = (
            f"nohup python3 -c \""
            f"from gpiozero import LED; from gpiozero.pins.lgpio import LGPIOFactory; from gpiozero import Device; "
            f"import time, subprocess; "
            f"Device.pin_factory = LGPIOFactory(); led = LED({LED_PIN}); "
            f"start_time = time.time(); "
            f"while time.time() - start_time < {SSH_OFF_SEC}: led.on(); time.sleep(0.5); led.off(); time.sleep(0.5); "
            f"subprocess.run(['sudo', 'systemctl', 'start', 'ssh']);" # 60秒後に自分でSSHを立ち上げる
            f"\" > /dev/null 2>&1 &"
        )
        ssh.exec_command(led_and_recovery_cmd)
        time.sleep(1.0) # ラズパイ側で点滅処理が確実に動き出すのを待つ（ここでチカチカが始まります）

        # 【2番目の命令】点滅が始まったのを確認して、おもむろにSSHサービスを停止する
        ssh.exec_command("sudo systemctl stop ssh")
        time.sleep(0.5)
        
        print(" [ 🔴 切断命令 ] ラズパイ側で『自律点滅＆自動復活タイマー』が起動しました。")
        ssh.close() 
        print(" [ 🔒 遮断完了 ] PC-ラズパイ間のSSHセッションは完全に切断されました。")
        
    except Exception as e:
        print(f" [ ❌ エラー ] 停止命令の送信に失敗: {e}")
        return

    # ------------------------------------------------------------
    # 【Step 3】 SSH復活待機 ＆ 10秒間の再点灯確認
    # ------------------------------------------------------------
    print("\n" + "-" * 60)
    print("[ 🔄 復帰要求 ] Step 3: ラズパイのSSHサービスが自律復活するのを待ちます。")
    print("-" * 60)
    print(f" ℹ️ ポート22が LISTEN 状態になるまで、名前解決（{RPI_HOST}）を伴う再接続を試みます...")

    reconnect_success = False
    start_time = time.time()
    timeout = 30.0  
    retry_count = 1

    while time.time() - start_time < timeout:
        try:
            print(f"   ┗ 再接続トライ中... ({retry_count}回目)", end="\r")
            ssh.connect(RPI_HOST, username=USER, password=PASS, timeout=2.0)
            reconnect_success = True
            print(f"\n [ 🎉 復帰成功 ] ポート22の LISTEN を検知！再接続が完了しました。")
            break
        except (paramiko.SSHException, socket.error):
            time.sleep(1.5)
            retry_count += 1

    if reconnect_success:
        stdin, stdout, stderr = ssh.exec_command("uptime -p")
        uptime_info = stdout.read().decode().strip()
        print(f"   ┗ ラズパイ現在の状態: {uptime_info}")
        
        # 最後の確認用点灯コマンド（こちらも安全な改行形式に統一）
        cmd_led_final = (
            f"python3 -c \"\n"
            f"from gpiozero import LED\n"
            f"from gpiozero.pins.lgpio import LGPIOFactory\n"
            f"from gpiozero import Device\n"
            f"Device.pin_factory = LGPIOFactory()\n"
            f"led = LED({LED_PIN})\n"
            f"led.on()\n"
            f"import time\n"
            f"time.sleep({RECONNECT_SEC})\n"
            f"led.off()\n"
            f"\""
        )
        ssh.exec_command(f"{cmd_led_final} > /dev/null 2>&1 &")
        
        print(f" [ 💡 LED状態 ] ：復活の『常時点灯』に切り替わりました。")
        
        for i in range(int(RECONNECT_SEC), 0, -1):
            print(f"   ┗ 試験終了（LED消灯）まであと {i} 秒...", end="\r")
            time.sleep(1)
        print(f"\n\n [ 🏁 実験完了 ] 全てのシーケンスが正常に終了しました。LEDは自動消灯しました。")
    else:
        print(f"\n [ ❌ タイムアウト ] {timeout}秒待機しましたが、ラズパイのSSHが復帰しませんでした。")
    
    ssh.close()
    print("=" * 65)

if __name__ == "__main__":
    main()
