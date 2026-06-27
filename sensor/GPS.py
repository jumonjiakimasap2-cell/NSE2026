"""
GPS.py
======
GPS モジュール 全情報ログ表示ドライバ

NSE2026/sensor/GPS.py

micropyGPS で取得できるすべての情報を 1 行にまとめてコマンドウィンドウに表示する。
スタンドアロンで実行可能 (python3 GPS.py) かつ、他モジュールから
  from GPS import GPSReceiver
としてインポートして使える。

表示項目 (横一行に羅列):
    時刻 (JST) | 日付 | 緯度 | 経度 | 速度[kt/km/h] | 進行方位 | 高度[m]
    ジオイド高[m] | Fix種別 | Fix状態 | 使用衛星数 | 視野衛星数
    HDOP | PDOP | VDOP | 解析文数 | CRC失敗数

参照:
    NICS2026/GPS_test.py    ── スレッド / serial / clean_sentences の使い方
    test_finishv.py         ── gps_thread_func の構造 (スレッド / バッファリセット)
    test_GPSrun.py          ── MicropyGPS(local_offset=9, 'ddm') / ddm→dd 変換

シリアル設定:
    PORT     = /dev/ttyAMA0   ← test_finishv.py / test_GPSrun.py と統一
    BAUDRATE = 9600
"""

import sys
import threading
import time
import datetime
from pathlib import Path

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# --- micropyGPS パス解決 (他テストコードと同じ方法) ---
_THIS_DIR = Path(__file__).resolve().parent       # NSE2026/sensor/
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from micropyGPS import MicropyGPS

# ===========================================================================
# 設定
# ===========================================================================

PORT         = "/dev/ttyAMA0"   # test_finishv.py / test_GPSrun.py と統一
BAUDRATE     = 9600
LOCAL_OFFSET = 9                # JST (+9h)
DISPLAY_HZ   = 1.0             # 表示更新レート [Hz]  (1 秒に 1 回)
WAIT_SENTENCES = 20            # NICS2026/GPS_test.py の clean_sentences > 20 と同じ

# Fix 種別テキスト (MicropyGPS.fix_type の値に対応)
FIX_TYPE_STR = {1: "NO_FIX", 2: "2D", 3: "3D"}

# ===========================================================================
# GPS 受信クラス
# ===========================================================================

class GPSReceiver:
    """
    バックグラウンドスレッドで NMEA を読み続け、
    最新の GPS データを属性として保持するクラス。

    他モジュール (test_GPSrun.py 等) からインポートして使う場合:

        gps_rx = GPSReceiver()
        gps_rx.start()
        ...
        lat = gps_rx.lat_dd   # 十進度 [度]
        ...
        gps_rx.stop()
    """

    def __init__(self,
                 port: str = PORT,
                 baudrate: int = BAUDRATE,
                 local_offset: int = LOCAL_OFFSET):

        self._port         = port
        self._baudrate     = baudrate
        self._gps          = MicropyGPS(local_offset=local_offset,
                                        location_formatting='ddm')
        self._thread       = None
        self._running      = False
        self._lock         = threading.Lock()

        # ── 公開プロパティ (スレッドセーフにコピーされる) ──
        self.valid          = False

        # 時刻・日付
        self.timestamp_h    = 0
        self.timestamp_m    = 0
        self.timestamp_s    = 0.0
        self.date_d         = 0
        self.date_mo        = 0
        self.date_y         = 0

        # 位置 (ddm → dd 変換済み)
        self.lat_dd         = 0.0   # 十進度 [度] 北正
        self.lng_dd         = 0.0   # 十進度 [度] 東正
        self.lat_raw        = [0, 0.0, 'N']   # [deg, decimal_min, hemi]
        self.lng_raw        = [0, 0.0, 'E']

        # 速度・方位・高度
        self.speed_kts      = 0.0   # [ノット]
        self.speed_mph      = 0.0   # [mph]
        self.speed_kmh      = 0.0   # [km/h]
        self.course_deg     = 0.0   # [度] 真北基準
        self.altitude_m     = 0.0   # [m]
        self.geoid_height_m = 0.0   # [m]

        # Fix / 精度
        self.fix_type       = 1     # 1=NO_FIX 2=2D 3=3D
        self.fix_stat       = 0     # 0=無効 1=GPS 2=DGPS
        self.satellites_in_use  = 0
        self.satellites_in_view = 0
        self.hdop           = 0.0
        self.pdop           = 0.0
        self.vdop           = 0.0

        # 統計
        self.clean_sentences   = 0
        self.parsed_sentences  = 0
        self.crc_fails         = 0

    # ── スレッド制御 ──────────────────────────────────────────────────────

    def start(self):
        """バックグラウンドスレッドを起動する。"""
        if self._running:
            return
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        """スレッドを停止する。"""
        self._running = False

    # ── NMEA 受信ループ ──────────────────────────────────────────────────

    def _run(self):
        """
        NMEA センテンスを読み続け、内部プロパティを更新する。
        NICS2026/GPS_test.py の rungps() と
        test_finishv.py の gps_thread_func() を合わせた構造。
        """
        if not SERIAL_AVAILABLE:
            print("[GPS] pyserial が見つかりません。GPS は無効です。")
            return

        try:
            with serial.Serial(self._port, self._baudrate, timeout=1.0) as ser:
                print(f"[GPS] Serial open: {self._port} @ {self._baudrate} bps")
                ser.readline()   # 先頭の不完全行を捨てる (NICS2026/GPS_test.py と同じ)

                while self._running:
                    try:
                        # バッファが溜まったらリセット (test_GPSrun.py と同じ)
                        if ser.in_waiting > 128:
                            ser.reset_input_buffer()

                        line = ser.readline().decode("ascii", errors="replace")
                        if not line.startswith("$"):
                            continue

                        for char in line:
                            self._gps.update(char)

                        # ── 内部状態をコピー ──
                        with self._lock:
                            self._copy_gps_state()

                    except Exception as e:
                        print(f"[GPS] 読み取りエラー: {e}")

        except serial.SerialException as e:
            print(f"[GPS] ポートを開けません ({self._port}): {e}")
            print("[GPS] GPS データは 0.0 で表示されます。")

    def _copy_gps_state(self):
        """
        MicropyGPS オブジェクトの状態をスレッドセーフなプロパティへコピーする。
        ddm → dd 変換は test_finishv.py / test_GPSrun.py と同じ手順。
        """
        g = self._gps
        self.valid = g.valid

        # 時刻・日付
        self.timestamp_h  = g.timestamp[0]
        self.timestamp_m  = g.timestamp[1]
        self.timestamp_s  = g.timestamp[2]
        self.date_d       = g.date[0]
        self.date_mo      = g.date[1]
        self.date_y       = g.date[2]

        # 位置 (ddm → dd, test_finishv.py と同じ変換)
        lat_raw = g.latitude    # [deg, decimal_min, 'N'/'S']
        lng_raw = g.longitude   # [deg, decimal_min, 'E'/'W']
        self.lat_raw = lat_raw
        self.lng_raw = lng_raw

        lat_dd = lat_raw[0] + lat_raw[1] / 60.0
        if lat_raw[2] == 'S':
            lat_dd = -lat_dd
        lng_dd = lng_raw[0] + lng_raw[1] / 60.0
        if lng_raw[2] == 'W':
            lng_dd = -lng_dd
        self.lat_dd = lat_dd
        self.lng_dd = lng_dd

        # 速度 (speed[0]=kts, [1]=mph, [2]=km/h)
        self.speed_kts  = g.speed[0]
        self.speed_mph  = g.speed[1]
        self.speed_kmh  = g.speed[2]
        self.course_deg = g.course

        # 高度
        self.altitude_m     = g.altitude
        self.geoid_height_m = g.geoid_height

        # Fix / 精度
        self.fix_type           = g.fix_type
        self.fix_stat           = g.fix_stat
        self.satellites_in_use  = g.satellites_in_use
        self.satellites_in_view = g.satellites_in_view
        self.hdop = g.hdop
        self.pdop = g.pdop
        self.vdop = g.vdop

        # 統計
        self.clean_sentences  = g.clean_sentences
        self.parsed_sentences = g.parsed_sentences
        self.crc_fails        = g.crc_fails

    # ── スナップショット取得 ─────────────────────────────────────────────

    def snapshot(self) -> dict:
        """
        現時点のすべての GPS データを dict で返す。
        他モジュールからのポーリング用。
        """
        with self._lock:
            return {
                "valid"             : self.valid,
                "timestamp"         : f"{self.timestamp_h:02d}:{self.timestamp_m:02d}:{self.timestamp_s:05.2f}",
                "date"              : f"{self.date_y:04d}-{self.date_mo:02d}-{self.date_d:02d}",
                "lat_dd"            : self.lat_dd,
                "lng_dd"            : self.lng_dd,
                "lat_raw"           : self.lat_raw,
                "lng_raw"           : self.lng_raw,
                "speed_kts"         : self.speed_kts,
                "speed_kmh"         : self.speed_kmh,
                "course_deg"        : self.course_deg,
                "altitude_m"        : self.altitude_m,
                "geoid_height_m"    : self.geoid_height_m,
                "fix_type"          : self.fix_type,
                "fix_stat"          : self.fix_stat,
                "satellites_in_use" : self.satellites_in_use,
                "satellites_in_view": self.satellites_in_view,
                "hdop"              : self.hdop,
                "pdop"              : self.pdop,
                "vdop"              : self.vdop,
                "clean_sentences"   : self.clean_sentences,
                "crc_fails"         : self.crc_fails,
            }

# ===========================================================================
# 表示フォーマット (全項目を 1 行に横羅列)
# ===========================================================================

# ヘッダー
_HDR = (
    "{:>8}  "        # Time(JST)
    "{:>10}  "       # Date
    "{:>11}  "       # Lat[dd]
    "{:>12}  "       # Lng[dd]
    "{:>7}  "        # Spd[kt]
    "{:>8}  "        # Spd[km/h]
    "{:>7}  "        # Course[°]
    "{:>8}  "        # Alt[m]
    "{:>9}  "        # Geoid[m]
    "{:>6}  "        # FixType
    "{:>7}  "        # FixStat
    "{:>4}  "        # Use
    "{:>4}  "        # View
    "{:>5}  "        # HDOP
    "{:>5}  "        # PDOP
    "{:>5}  "        # VDOP
    "{:>6}  "        # CleanSent
    "{:>5}"          # CRCFail
)

HEADER = _HDR.format(
    "Time(JST)", "Date",
    "Lat[dd]", "Lng[dd]",
    "Spd[kt]", "Spd[km/h]", "Course[°]",
    "Alt[m]", "Geoid[m]",
    "FixTyp", "FixStat",
    "Use", "View",
    "HDOP", "PDOP", "VDOP",
    "CleanSnt", "CRCFl",
)

_DAT = (
    "{:>8}  "
    "{:>10}  "
    "{:>11.6f}  "
    "{:>12.6f}  "
    "{:>7.3f}  "
    "{:>8.3f}  "
    "{:>7.1f}  "
    "{:>8.2f}  "
    "{:>9.2f}  "
    "{:>6}  "
    "{:>7}  "
    "{:>4d}  "
    "{:>4d}  "
    "{:>5.2f}  "
    "{:>5.2f}  "
    "{:>5.2f}  "
    "{:>6d}  "
    "{:>5d}"
)

SEPARATOR = "-" * len(HEADER)

# ヘッダーを何行おきに再表示するか
HEADER_REPEAT = 30


def format_row(rx: GPSReceiver) -> str:
    """GPSReceiver の現在値を 1 行の文字列にフォーマットして返す。"""
    with rx._lock:
        time_str = (f"{rx.timestamp_h:02d}:"
                    f"{rx.timestamp_m:02d}:"
                    f"{int(rx.timestamp_s):02d}")
        if rx.date_y > 0:
            date_str = f"{rx.date_y:04d}-{rx.date_mo:02d}-{rx.date_d:02d}"
        else:
            date_str = "--/--/----"

        fix_str  = FIX_TYPE_STR.get(rx.fix_type, "?")
        stat_str = {0: "INVALID", 1: "GPS", 2: "DGPS"}.get(rx.fix_stat, "?")

        return _DAT.format(
            time_str, date_str,
            rx.lat_dd, rx.lng_dd,
            rx.speed_kts, rx.speed_kmh, rx.course_deg,
            rx.altitude_m, rx.geoid_height_m,
            fix_str, stat_str,
            rx.satellites_in_use, rx.satellites_in_view,
            rx.hdop, rx.pdop, rx.vdop,
            rx.clean_sentences, rx.crc_fails,
        )

# ===========================================================================
# スタンドアロン実行
# ===========================================================================

def main():
    print("=" * len(HEADER))
    print("  GPS.py  GPS 全情報 ログ表示  (Ctrl+C で終了)")
    print(f"  Port: {PORT}  Baudrate: {BAUDRATE}  LocalOffset: JST+{LOCAL_OFFSET}")
    print("=" * len(HEADER))

    rx = GPSReceiver(port=PORT, baudrate=BAUDRATE, local_offset=LOCAL_OFFSET)
    rx.start()

    # GPS データが溜まるまで待機 (NICS2026/GPS_test.py の clean_sentences > 20 と同じ考え)
    print(f"\n[INFO] データ受信待機中 (clean_sentences > {WAIT_SENTENCES} になるまで)...")
    wait_start = time.time()
    while rx.clean_sentences < WAIT_SENTENCES:
        elapsed = time.time() - wait_start
        print(f"  clean_sentences={rx.clean_sentences:3d}  ({elapsed:.0f} s 経過)", end="\r")
        time.sleep(0.5)
    print(f"\n[INFO] 受信準備完了 (clean_sentences={rx.clean_sentences})\n")

    line_count = 0
    interval   = 1.0 / DISPLAY_HZ

    try:
        while True:
            loop_start = time.time()

            # ヘッダー再表示
            if line_count % HEADER_REPEAT == 0:
                print(SEPARATOR)
                print(HEADER)
                print(SEPARATOR)

            print(format_row(rx))
            line_count += 1

            # ループ待機
            wait = interval - (time.time() - loop_start)
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        print(f"\n\n[INFO] 終了  ({line_count} 行表示)")
    finally:
        rx.stop()
        print(f"[INFO] GPSReceiver 停止")


# ===========================================================================
if __name__ == "__main__":
    main()
