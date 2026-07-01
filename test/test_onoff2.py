import time
import socket
import paramiko

# ===========================================================================
# 接続・環境設定
# ===========================================================================
# ★ IPアドレスを使わない方法（mDNS）を適用
RPI_HOST    = "raspberrypi.local"  
USER        = "pi"
PASS        = "pi"

# 試験の時間定義（提示いただいたコードの秒数と同期）
SSH_ON_SEC   = 30.0
SSH_OFF_SEC  = 60.0
RECONNECT_SEC = 30.0

def print_header():
    print("=" * 65)
    print("      RASPBERRY PI - SSH SERVICE ON/OFF SWITCHING TEST      ")
    print("=============================================================")
    print(f" 🎯 ターゲット: {RPI_HOST}")
    print(" 💡 ラズパイ側動作: LED(BCM21)連動、SSHサービス強制オン/オフ")
    print("=" * 65)

def main():
    print_header()
    
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    # ------------------------------------------------------------
    # 【Step 1】 初期接続 ＆ 10秒間の常時点灯確認
    # ------------------------------------------------------------
    print(f"\n[ ⏳ Step 1 ] ラズパイへ接続中...")
    try:
        ssh.connect(RPI_HOST, username=USER, password=PASS, timeout=5)
        print(" [ 🟢 SSH有効 ] 初期接続に成功しました。")
        
        # 実際にラズパイ内のテストスクリプト（Step1部分相当）を叩く、
        # またはPC側から同期してLED点灯を模倣/命令します
        print(f" [ 💡 LED状態 ] 常時点灯中 ➔ 動作確認をしてください（残り {SSH_ON_SEC:.0f} 秒）")
        
        # 進行バー的なカウントダウン
        for i in range(int(SSH_ON_SEC), 0, -1):
            print(f"   ┗ 試験開始まであと {i} 秒...", end="\r")
            time.sleep(1)
        print("\n   ┗ [OK] Step 1 正常通過。")
        
    except Exception as e:
        print(f" [ ❌ 接続失敗 ] ラズパイが見つからないか、SSHが無効です: {e}")
        return

    # ------------------------------------------------------------
    # 【Step 2】 SSHサービス停止 ＆ 30秒間のLED点滅（切断状態）
    # ------------------------------------------------------------
    print("\n" + "-" * 60)
    print("[ ⚠️ 命令発信 ] Step 2: ラズパイのSSHサービスを強制停止します。")
    print("-" * 60)
    
    try:
        # ラズパイ側で 'sudo systemctl stop ssh' をバックグラウンド実行
        # (パスワードなしsudo設定が前提。プロセスが即座に切断されるため、ノンブロッキングで送信)
        transport = ssh.get_transport()
        channel = transport.open_session()
        channel.exec_command("sudo systemctl stop ssh")
        
        print(" [ 🔴 切断命令 ] 'systemctl stop ssh' を送信しました。")
        ssh.close() # PC側からもセッションを綺麗に閉じる
        print(" [ 🔒 遮断完了 ] PC-ラズパイ間のSSHセッションは完全に切断されました。")
        
    except Exception as e:
        print(f" [ ❌ エラー ] 停止命令の送信に失敗: {e}")
        return

    print(f"\n[ ⏳ Step 2 ] ラズパイ側は現在『接続オフ（LED 1秒間隔点滅中）』フェーズです。")
    print(f"              このまま規定の {SSH_OFF_SEC:.0f} 秒間、通信途絶を維持します。")
    
    # 30秒間のカウントダウン
    for i in range(int(SSH_OFF_SEC), 0, -1):
        if i % 5 == 0 or i <= 5: # ログがうるさくならないよう5秒ごと、最後は1秒ごと
            print(f"   ┗ サービス停止維持（LED点滅中）: 残り {i} 秒...")
        time.sleep(1)

    # ------------------------------------------------------------
    # 【Step 3】 SSH復活待機 ＆ 10秒間の再点灯確認
    # ------------------------------------------------------------
    print("\n" + "-" * 60)
    print("[ 🔄 復帰要求 ] Step 3: ラズパイ側でサービスが自律復活するのを待ちます。")
    print("-" * 60)
    print(f" ℹ️ ラズパイ内部のタイマーにより、自動で 'systemctl start ssh' が走ります。")
    print(f"    ポート22が LISTEN 状態になるまで、名前解決（{RPI_HOST}）を伴う再接続を試みます...")

    # ご提示いただいたコードの「wait_ssh_listen」の動きをPC側からのポーリングで再現・確認
    reconnect_success = False
    start_time = time.time()
    timeout = 30.0
    retry_count = 1

    while time.time() - start_time < timeout:
        try:
            print(f"   ┗ 再接続トライ中... ({retry_count}回目)", end="\r")
            # 内部で自動的にIPを解決してポート22へアプローチ
            ssh.connect(RPI_HOST, username=USER, password=PASS, timeout=2.0)
            reconnect_success = True
            print(f"\n [ 🎉 復帰成功 ] ポート22の LISTEN を検知！再接続が完了しました。")
            break
        except (paramiko.SSHException, socket.error):
            # まだラズパイ側でサービスが立ち上がっていない場合はスルーして待機
            time.sleep(1.5)
            retry_count += 1

    if reconnect_success:
        # 最後の確認コマンド
        stdin, stdout, stderr = ssh.exec_command("uptime -p")
        uptime_info = stdout.read().decode().strip()
        print(f"   ┗ ラズパイ現在の状態: {uptime_info}")
        print(f" [ 💡 LED状態 ] 復活確認の『常時点灯』に切り替わりました。")
        
        # 最後の10秒ホールド
        for i in range(int(RECONNECT_SEC), 0, -1):
            print(f"   ┗ 試験終了（LED消灯）まであと {i} 秒...", end="\r")
            time.sleep(1)
        print(f"\n\n [ 🏁 実験完了 ] 全てのシーケンスが正常に終了しました。LEDを消灯します。")
    else:
        print(f"\n [ ❌ タイムアウト ] {timeout}秒待機しましたが、ラズパイのSSHが復帰しませんでした。")
    
    ssh.close()
    print("=" * 65)

if __name__ == "__main__":
    main()
