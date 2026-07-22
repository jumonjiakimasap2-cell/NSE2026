"""
wifi_switch_test.py
====================
「別WiFi (SECONDARY_SSID) へ切り替えても問題ないか」の事前テストスクリプト。

wifi_switch.py の本番シーケンス (30秒→1分→30秒) を実行する前に、
実際に別WiFiへ接続して疎通確認だけ行い、最後に必ず元WiFiへ戻す
「お試し接続」を1回だけ実行するスクリプトです。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
チェック内容:
    1. 別WiFi (SECONDARY_SSID) が電波として見えるか (SSID/信号強度/チャンネル)
    2. 実際に接続できるか、IP アドレスを取得できるか
    3. デフォルトゲートウェイに ping が通るか (ローカル疎通)
    4. インターネット (PING_HOST, デフォルト 8.8.8.8) に ping が通るか
    5. 上記を踏まえて「問題なく移行できそうか」を最後にまとめて判定
    → 判定後、成功・失敗にかかわらず必ず元WiFi (ORIGINAL_SSID) に戻す。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
【注意】
- 本番の wifi_switch.py と同様、SSH 越しに実行している場合は
  別WiFiへの接続中に SSH セッションが切れる可能性があります。
  切れても最後まで処理を続けさせたい場合は nohup を使ってください。

      nohup python3 wifi_switch_test.py > wifi_switch_test.log 2>&1 &

- NetworkManager (nmcli) が使える環境が前提です (Raspberry Pi OS
  Bookworm 以降の標準)。`which nmcli` で確認してください。
- SSID/パスワードはこのファイル内に直接書き込む形にしています。
  実際の値に書き換えてから実行してください (wifi_switch.py と同じ値でOK)。
"""

import re
import time
import datetime
import subprocess

# ===========================================================================
# 設定値 ← ここを実地に合わせて変更
# ===========================================================================

ORIGINAL_SSID      = "YOUR_ORIGINAL_SSID"       # 元WiFi の SSID
ORIGINAL_PASSWORD  = "YOUR_ORIGINAL_PASSWORD"   # 元WiFi のパスワード
SECONDARY_SSID     = "YOUR_SECONDARY_SSID"      # テストしたい別WiFi の SSID
SECONDARY_PASSWORD = "YOUR_SECONDARY_PASSWORD"  # 別WiFi のパスワード

WLAN_IFACE = "wlan0"      # 無線LANインターフェース名 (`ip a` で確認可能)

NMCLI_TIMEOUT_SEC = 20.0  # [s] nmcli 接続コマンドのタイムアウト
SETTLE_SEC        = 3.0   # [s] 接続後、DHCPでIPが安定するまでの待ち時間

PING_HOST         = "8.8.8.8"  # インターネット疎通確認先
PING_COUNT        = 4          # ping 送信回数
PING_TIMEOUT_SEC  = 2.0        # ping 1回あたりのタイムアウト

# この割合以上 ping が通れば「疎通OK」とみなす
MIN_PING_SUCCESS_RATIO = 0.5   # 50%以上受信できればOK

# ===========================================================================
# ロガー
# ===========================================================================

def log(msg: str, level: str = "INFO"):
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{ts}][{level}] {msg}", flush=True)


# ===========================================================================
# nmcli ヘルパー
# ===========================================================================

def get_current_connection() -> str:
    """現在 WLAN_IFACE がつながっている接続名を取得 (取れなければ '?')。"""
    try:
        result = subprocess.run(
            ["nmcli", "-t", "-f", "GENERAL.CONNECTION", "device", "show", WLAN_IFACE],
            capture_output=True, text=True, timeout=10.0,
        )
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip().split(":", 1)[-1]
    except Exception as e:
        log(f"現在の接続名取得に失敗: {e}", "WARN")
    return "?"


def scan_for_ssid(target_ssid: str):
    """
    周辺WiFiをスキャンし、target_ssid の信号強度(%)・チャンネルを探して返す。
    見つからなければ (None, None) を返す。
    """
    try:
        subprocess.run(["nmcli", "device", "wifi", "rescan"],
                       capture_output=True, text=True, timeout=15.0)
        time.sleep(2.0)  # スキャン結果が揃うまで少し待つ
        result = subprocess.run(
            ["nmcli", "-t", "-f", "SSID,SIGNAL,CHAN", "device", "wifi", "list"],
            capture_output=True, text=True, timeout=15.0,
        )
        for line in result.stdout.splitlines():
            parts = line.split(":")
            if len(parts) >= 3 and parts[0] == target_ssid:
                signal_pct = parts[1]
                channel = parts[2]
                return signal_pct, channel
    except Exception as e:
        log(f"WiFiスキャンに失敗: {e}", "WARN")
    return None, None


def connect_wifi(ssid: str, password: str) -> bool:
    """nmcli で指定 SSID に接続する。成功すれば True。"""
    log(f"WiFi接続試行: SSID='{ssid}'")
    try:
        result = subprocess.run(
            ["nmcli", "device", "wifi", "connect", ssid, "password", password],
            capture_output=True, text=True, timeout=NMCLI_TIMEOUT_SEC,
        )
        if result.returncode == 0:
            log(f"WiFi接続成功: SSID='{ssid}'")
            return True
        else:
            log(f"WiFi接続失敗: SSID='{ssid}'  stderr={result.stderr.strip()}", "WARN")
            return False
    except FileNotFoundError:
        log("nmcli コマンドが見つかりません。NetworkManager が入っていない可能性があります。", "ERROR")
        return False
    except subprocess.TimeoutExpired:
        log(f"WiFi接続がタイムアウトしました (SSID='{ssid}')", "WARN")
        return False
    except Exception as e:
        log(f"nmcli 実行エラー: {e}", "WARN")
        return False


def get_ip_address() -> str | None:
    """WLAN_IFACE の IPv4 アドレスを取得。"""
    try:
        result = subprocess.run(
            ["nmcli", "-g", "IP4.ADDRESS", "device", "show", WLAN_IFACE],
            capture_output=True, text=True, timeout=10.0,
        )
        ip = result.stdout.strip().split("\n")[0]
        return ip if ip else None
    except Exception:
        return None


def get_default_gateway() -> str | None:
    """デフォルトゲートウェイの IP アドレスを取得。"""
    try:
        result = subprocess.run(["ip", "route", "show", "default"],
                               capture_output=True, text=True, timeout=10.0)
        m = re.search(r"default via (\S+)", result.stdout)
        return m.group(1) if m else None
    except Exception:
        return None


# ===========================================================================
# ping テスト
# ===========================================================================

def ping_test(host: str, count: int = PING_COUNT, timeout_sec: float = PING_TIMEOUT_SEC) -> dict:
    """
    host に ping を打ち、結果を辞書で返す。
    { "success": bool, "loss_pct": float, "avg_rtt_ms": float|None }
    """
    result_info = {"success": False, "loss_pct": 100.0, "avg_rtt_ms": None}
    try:
        proc = subprocess.run(
            ["ping", "-c", str(count), "-W", str(int(timeout_sec)), host],
            capture_output=True, text=True, timeout=timeout_sec * count + 10.0,
        )
        out = proc.stdout

        m_loss = re.search(r"(\d+(?:\.\d+)?)% packet loss", out)
        if m_loss:
            loss_pct = float(m_loss.group(1))
            result_info["loss_pct"] = loss_pct
            result_info["success"] = (1.0 - loss_pct / 100.0) >= MIN_PING_SUCCESS_RATIO

        m_rtt = re.search(r"= [\d.]+/([\d.]+)/", out)
        if m_rtt:
            result_info["avg_rtt_ms"] = float(m_rtt.group(1))

    except Exception as e:
        log(f"ping 実行エラー ({host}): {e}", "WARN")

    return result_info


# ===========================================================================
# メイン
# ===========================================================================

def main():
    log("=" * 60)
    log("別WiFi 事前テスト開始 (1回きり実行)")
    log("=" * 60)

    original_conn_name = get_current_connection()
    log(f"現在の接続: '{original_conn_name}' ({WLAN_IFACE})")

    test_passed = False
    detail_lines = []

    try:
        # ── 1. スキャンして SECONDARY_SSID が見えるか確認 ──
        log(f"[1/5] '{SECONDARY_SSID}' を周辺スキャンで探索中...")
        signal_pct, channel = scan_for_ssid(SECONDARY_SSID)
        if signal_pct is not None:
            log(f"  → 発見: 信号強度={signal_pct}%  チャンネル={channel}")
            detail_lines.append(f"電波検出: 信号強度={signal_pct}%  チャンネル={channel}")
        else:
            log(f"  → '{SECONDARY_SSID}' が周辺スキャンで見つかりませんでした。", "WARN")
            detail_lines.append("電波検出: 見つからず (圏外の可能性)")

        # ── 2. 別WiFiへ接続 ──
        log(f"[2/5] '{SECONDARY_SSID}' へ接続を試みます...")
        connected = connect_wifi(SECONDARY_SSID, SECONDARY_PASSWORD)
        detail_lines.append(f"接続: {'成功' if connected else '失敗'}")

        ip_addr = None
        gw_addr = None
        gw_ping = {"success": False, "loss_pct": 100.0, "avg_rtt_ms": None}
        net_ping = {"success": False, "loss_pct": 100.0, "avg_rtt_ms": None}

        if connected:
            log(f"[3/5] DHCP安定待ち ({SETTLE_SEC:.0f}秒)...")
            time.sleep(SETTLE_SEC)

            ip_addr = get_ip_address()
            log(f"  → 取得したIPアドレス: {ip_addr or '取得できず'}")
            detail_lines.append(f"IPアドレス: {ip_addr or '取得できず'}")

            gw_addr = get_default_gateway()
            log(f"  → デフォルトゲートウェイ: {gw_addr or '不明'}")

            # ── 4. ゲートウェイへの疎通確認 ──
            if gw_addr:
                log(f"[4/5] ゲートウェイ ({gw_addr}) へ ping 中...")
                gw_ping = ping_test(gw_addr)
                log(f"  → 損失率={gw_ping['loss_pct']:.0f}%  "
                    f"平均RTT={gw_ping['avg_rtt_ms']}ms  "
                    f"判定={'OK' if gw_ping['success'] else 'NG'}")
                detail_lines.append(
                    f"ゲートウェイ疎通: 損失率={gw_ping['loss_pct']:.0f}%  "
                    f"平均RTT={gw_ping['avg_rtt_ms']}ms")
            else:
                log("[4/5] ゲートウェイが不明なため疎通確認をスキップします。", "WARN")
                detail_lines.append("ゲートウェイ疎通: 未確認 (ゲートウェイ不明)")

            # ── 5. インターネットへの疎通確認 ──
            log(f"[5/5] インターネット ({PING_HOST}) へ ping 中...")
            net_ping = ping_test(PING_HOST)
            log(f"  → 損失率={net_ping['loss_pct']:.0f}%  "
                f"平均RTT={net_ping['avg_rtt_ms']}ms  "
                f"判定={'OK' if net_ping['success'] else 'NG'}")
            detail_lines.append(
                f"インターネット疎通: 損失率={net_ping['loss_pct']:.0f}%  "
                f"平均RTT={net_ping['avg_rtt_ms']}ms")
        else:
            log("[3-5/5] 接続に失敗したため疎通確認はスキップします。", "WARN")

        # ── 総合判定 ──
        test_passed = connected and (ip_addr is not None) and net_ping["success"]

    finally:
        # ── 必ず元WiFiへ戻す ──
        log(f"元WiFi ('{ORIGINAL_SSID}') へ戻します...")
        restored = connect_wifi(ORIGINAL_SSID, ORIGINAL_PASSWORD)
        if not restored:
            log("元WiFiへの再接続に失敗しました！ 手動での復旧が必要な場合があります。", "ERROR")

        log("=" * 60)
        if test_passed:
            log(f"【結果】'{SECONDARY_SSID}' への切り替えは問題なさそうです ✓")
        else:
            log(f"【結果】'{SECONDARY_SSID}' への切り替えに問題があります ✗", "WARN")
        for line in detail_lines:
            log(f"  - {line}")
        log("=" * 60)


if __name__ == "__main__":
    main()
