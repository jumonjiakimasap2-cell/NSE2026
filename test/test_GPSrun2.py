"""
test_GPSrun.py
==============
GPS 誘導制御走行 テストプログラム

NSE2026/test/test_GPSrun.py

概要:
    目標地点（TARGET_LAT / TARGET_LNG）を目指して自律走行する。
    BNO055 の地磁気センサで機体の向き（方位角 azimuth）を取得し、
    GPS から算出した目標方位（angle）との差分（diff）でモータを制御する。

フロー:
    1. センサ・モータ初期化
    2. GPS Fix 待機
    3. 制御ループ (LOOP_DT 周期)
       │  BNO055 → 加速度・地磁気取得 → azimuth 計算
       │  GPS共有変数 → lat/lng 取得 → angle・distance 計算
       │  diff = azimuth − angle  →  モータ指令判定
       │  distance < GOAL_RADIUS → ゴール判定 → 停止・終了
       └  TIMEOUT_SEC 超過 → タイムアウト停止
    4. CSV ログ保存

方位制御ロジック (NICS2026/p3_run.py を NSE2026 に移植):
    diff = (azimuth − angle + 360) % 360
    if diff > 180: diff -= 360   # −180〜+180 に正規化
    │  |diff| < ANGLE_DEADBAND  → 前進
    │  diff > ANGLE_TURN_STRONG  → 左旋回 (強)
    │  diff > 0                  → 左寄り前進 (弱)
    │  diff < -ANGLE_TURN_STRONG → 右旋回 (強)
    └  diff < 0                  → 右寄り前進 (弱)

モータ:
    gpiozero + lgpio  ← test_run.py / fall.py と同じ
    BCM: PWMA=13, AIN1=5, AIN2=6, PWMB=24, BIN1=18, BIN2=23

センサ:
    BNO055  (I2C, NDOF モード)  ← test_finishv.py / fall.py と同じ
    micropyGPS (serial スレッド) ← test_finishv.py と同じ

ログ:
    NSE2026/logs/gpsrun_YYYYMMDD_HHMMSS.csv
"""

import sys
import math
import time
import csv
import threading
import datetime
from pathlib import Path

# --- gpiozero (test_run.py / fall.py / test_straight.py と同じバックエンド) ---
from gpiozero import Motor, PWMOutputDevice, Device, OutputDevice  # OutputDevice を追加
from gpiozero.pins.lgpio import LGPIOFactory

# --- シリアル (GPS 用 / test_finishv.py と同じ) ---
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# --- センサモジュールパス (全テストコードと同じ解決方法) ---
SCRIPT_DIR = Path(__file__).resolve().parent      # NSE2026/test/
SENSOR_DIR = SCRIPT_DIR.parent / "sensor"         # .../NSE2026/sensor/
if str(SENSOR_DIR) not in sys.path:
    sys.path.insert(0, str(SENSOR_DIR))

from BNO055 import BNO055
from micropyGPS import MicropyGPS

# ===========================================================================
# 設定 ─── ここを実地に合わせて変更する
# ===========================================================================

# --- 目標座標 (NICS2026/main.py の TARGET_LAT / TARGET_LNG に相当) ---
TARGET_LAT =  38.26052      # 目標緯度  [度]
TARGET_LNG = 140.8544151    # 目標経度  [度]

# --- 地磁気偏角補正 (仙台付近: 約 −8.0°, NICS2026 の MAG_CONST に相当) ---
# 真北と磁北のずれ。正 = 東偏、負 = 西偏
MAG_DECLINATION = -8.0      # [度]

# --- 到達判定半径 ---
GOAL_RADIUS = 3.0           # [m]  この距離以内でゴール

# --- 制御ループ ---
LOOP_DT      = 0.1          # [s]  制御周期
TIMEOUT_SEC  = 10 * 60      # [s]  走行タイムアウト (10分)

# --- 方向制御閾値 (NICS2026/p3_run.py の判定値に相当) ---
ANGLE_DEADBAND    = 10.0    # [度]  この範囲内なら前進
ANGLE_TURN_STRONG = 45.0    # [度]  この角度以上で強旋回

# --- モータ出力 (test_run.py / fall.py と同じ変数名・値) ---
SPEED_FWD   = 0.8           # 前進時 PWM duty (0.0〜1.0)
SPEED_TURN  = 0.8           # 旋回時 外輪 PWM duty
SPEED_WEAK  = 0.4           # 弱旋回時 内輪 PWM duty (弱左/弱右)

# --- GPS ---
GPS_PORT     = "/dev/serial0"
GPS_BAUDRATE = 9600
GPS_FIX_TIMEOUT = 120.0     # [s]  Fix 待機タイムアウト

# --- ログ ---
LOG_DIR = SCRIPT_DIR.parent / "logs"

# --- 地球半径 (NICS2026/main.py の EARTH_RADIUS に相当) ---
EARTH_RADIUS = 6378136.59   # [m]

# ===========================================================================
# 共有グローバル変数
# ===========================================================================

gps_lat    = 0.0
gps_lng    = 0.0
gps_speed  = 0.0
gps_sats   = 0
gps_valid  = False          # Fix 取得フラグ

# ===========================================================================
# GPS スレッド (元のコードの処理フローを完全統合)
# ===========================================================================

def gps_thread_func(gps_obj: MicropyGPS, port: str, baudrate: int):
    """GPS NMEA センテンスをバックグラウンドで読み続ける。"""
    global gps_lat, gps_lng, gps_speed, gps_sats, gps_valid

    if not SERIAL_AVAILABLE:
        print("[GPS] pyserial が見つかりません。GPS は無効です。")
        return

    try:
        with serial.Serial(port, baudrate, timeout=10) as ser:
            print(f"[GPS] Serial open: {port} @ {baudrate} bps")
            
            # 最初の1行目は中途半端なデータである可能性があるため読み飛ばす
            ser.readline()
            
            while True:
                try:
                    # バッファが溜まっていたらリセット (NICS2026/main.py GPS_thread() の手法)
                    if ser.in_waiting > 128:
                        ser.reset_input_buffer()

                    sentence = ser.readline().decode('utf-8')
                    if sentence == "":
                        continue
                    if sentence[0] != '$':  # 先頭が '$' でなければ捨てる
                        continue

                    for char in sentence:
                        gps_obj.update(char)

                    # ちゃんと解析できたデータがある程度たまったらグローバル変数を更新
                    if gps_obj.clean_sentences > 20:
                        lat_raw = gps_obj.latitude   # [deg, hemisphere] (location_formatting='dd')
                        lng_raw = gps_obj.longitude  # [deg, hemisphere] (location_formatting='dd')

                        lat = lat_raw[0]
                        if lat_raw[1] == 'S':
                            lat = -lat
                        lng = lng_raw[0]
                        if lng_raw[1] == 'W':
                            lng = -lng

                        gps_lat    = lat
                        gps_lng    = lng
                        gps_speed = gps_obj.speed[0]
                        gps_sats   = gps_obj.satellites_in_use
                        gps_valid = (lat != 0.0)

                except Exception as e:
                    print(f"[GPS] 読み取りエラー: {e}")

    except serial.SerialException as e:
        print(f"[GPS] ポートを開けません ({port}): {e}")
        print("[GPS] GPS データは 0.0 で表示されます。")

# ===========================================================================
# 航法計算 (NICS2026/main.py の calcAngle / calcdistance / calcAzimuth に相当)
# ===========================================================================

def calc_distance(lat: float, lng: float) -> float:
    """
    現在地 (lat, lng) から目標地点 (TARGET_LAT, TARGET_LNG) までの
    平面近似距離 [m] を返す。
    """
    dx = math.radians(TARGET_LNG - lng) * EARTH_RADIUS * math.cos(math.radians(lat))
    dy = math.radians(TARGET_LAT - lat) * EARTH_RADIUS
    return math.hypot(dx, dy)


def calc_target_bearing(lat: float, lng: float) -> float:
    """
    現在地から目標地点への方位角 [度, 0〜360, 北=0 時計回り] を返す。
    """
    dx = math.radians(TARGET_LNG - lng) * EARTH_RADIUS * math.cos(math.radians(lat))
    dy = math.radians(TARGET_LAT - lat) * EARTH_RADIUS
    angle = 90.0 - math.degrees(math.atan2(dy, dx))
    return angle % 360.0


def calc_azimuth(mag: list) -> float:
    """
    BNO055 地磁気センサから機体の向き（方位角）[度, 0〜360, 北=0] を返す。
    """
    azimuth = 90.0 - math.degrees(math.atan2(mag[1], mag[0]))
    azimuth *= -1
    azimuth += MAG_DECLINATION   # 偏角補正
    return azimuth % 360.0


def calc_direction_diff(azimuth: float, target_bearing: float) -> float:
    """
    機体方位 (azimuth) と目標方位 (target_bearing) の差分を
    −180〜+180 度に正規化して返す。
    """
    diff = (azimuth - target_bearing) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff

# ===========================================================================
# モータコントローラ (fall.py の MotorController を左右独立制御に拡張)
# ===========================================================================

PIN_PWMA = 13
PIN_AIN1 =  5
PIN_AIN2 =  6
PIN_PWMB = 18
PIN_BIN1 = 23
PIN_BIN2 = 24
PIN_STBY = 11  # STBYピン


class MotorController:
    """
    左右モータの独立 PWM 制御。
    gpiozero + LGPIOFactory ── test_run.py / fall.py と統一。
    """

    def __init__(self, speed_fwd: float = SPEED_FWD,
                 speed_turn: float = SPEED_TURN,
                 speed_weak: float = SPEED_WEAK):
        Device.pin_factory = LGPIOFactory()
        self.speed_fwd  = speed_fwd
        self.speed_turn = speed_turn
        self.speed_weak = speed_weak
        self._pwm_a  = PWMOutputDevice(PIN_PWMA)
        self._pwm_b  = PWMOutputDevice(PIN_PWMB)
        self._motor_a = Motor(forward=PIN_AIN1, backward=PIN_AIN2)   # 右
        self._motor_b = Motor(forward=PIN_BIN1, backward=PIN_BIN2)   # 左
        self._stby = OutputDevice(PIN_STBY)
        self.stop()

    def forward(self):
        self._stby.on()
        self._pwm_a.value = self.speed_fwd
        self._pwm_b.value = self.speed_fwd
        self._motor_a.forward()
        self._motor_b.forward()

    def stop(self):
        self._pwm_a.value = 0
        self._pwm_b.value = 0
        self._motor_a.stop()
        self._motor_b.stop()
        self._stby.off()

    def turn_left_strong(self):
        self._stby.on()
        self._pwm_a.value = self.speed_turn
        self._pwm_b.value = 0
        self._motor_a.forward()
        self._motor_b.stop()

    def turn_right_strong(self):
        self._stby.on()
        self._pwm_a.value = 0
        self._pwm_b.value = self.speed_turn
        self._motor_a.stop()
        self._motor_b.forward()

    def turn_left_weak(self):
        self._stby.on()
        self._pwm_a.value = self.speed_fwd
        self._pwm_b.value = self.speed_weak
        self._motor_a.forward()
        self._motor_b.forward()

    def turn_right_weak(self):
        self._stby.on()
        self._pwm_a.value = self.speed_weak
        self._pwm_b.value = self.speed_fwd
        self._motor_a.forward()
        self._motor_b.forward()

    def apply_diff(self, diff: float) -> str:
        abs_diff = abs(diff)

        if abs_diff < ANGLE_DEADBAND:
            self.forward()
            return "FORWARD"
        elif diff > ANGLE_TURN_STRONG:
            self.turn_left_strong()
            return "TURN_L_STRONG"
        elif diff > 0:
            self.turn_left_weak()
            return "TURN_L_WEAK"
        elif diff < -ANGLE_TURN_STRONG:
            self.turn_right_strong()
            return "TURN_R_STRONG"
        else:
            self.turn_right_weak()
            return "TURN_R_WEAK"

    def close(self):
        self.stop()
        self._pwm_a.close()
        self._pwm_b.close()
        self._motor_a.close()
        self._motor_b.close()
        self._stby.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

# ===========================================================================
# コンソール表示フォーマット
# ===========================================================================

HEADER_FMT = (
    "{:>8}  {:>10}  {:>10}  "
    "{:>8}  {:>8}  {:>8}  "
    "{:>7}  {:>7}  {:>7}  "
    "{:>4}  {:>14}"
)
HEADER = HEADER_FMT.format(
    "Time[s]", "Lat", "Lng",
    "Dist[m]", "Target[°]", "Azimuth[°]",
    "Diff[°]", "MagX", "MagY",
    "Sats", "Motor"
)

DATA_FMT = (
    "{:>8.2f}  {:>10.6f}  {:>10.6f}  "
    "{:>8.2f}  {:>8.2f}  {:>8.2f}  "
    "{:>7.2f}  {:>7.3f}  {:>7.3f}  "
    "{:>4d}  {:>14}"
)

# ===========================================================================
# メイン
# ===========================================================================

def main():
    # --- ログ準備 ---
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts_str   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"gpsrun_{ts_str}.csv"

    print("=" * 75)
    print("  test_GPSrun.py  GPS 誘導制御走行 テスト")
    print(f"  {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  目標地点: LAT={TARGET_LAT}  LNG={TARGET_LNG}")
    print(f"  ゴール半径: {GOAL_RADIUS} m  /  タイムアウト: {TIMEOUT_SEC} s")
    print("=" * 75)

    # ── BNO055 初期化 ──
    print("\n[INIT] BNO055 初期化中...")
    bno = BNO055()
    if not bno.setUp(operation_mode=BNO055.OPERATION_MODE_NDOF):
        print("[ERROR] BNO055 の初期化に失敗しました。終了します。")
        sys.exit(1)

    # ── GPS スレッド起動 (元のコードに合わせて location_formatting を 'dd' に設定) ──
    gps_obj    = MicropyGPS(local_offset=9, location_formatting='dd')
    gps_thread = threading.Thread(
        target=gps_thread_func,
        args=(gps_obj, GPS_PORT, GPS_BAUDRATE),
        daemon=True
    )
    gps_thread.start()

    # ── GPS Fix 待機 ──
    print(f"\n[INIT] GPS Fix 待機中 (最大 {GPS_FIX_TIMEOUT:.0f} s)...")
    fix_deadline = time.time() + GPS_FIX_TIMEOUT
    while not gps_valid and time.time() < fix_deadline:
        print(f"  待機中... lat={gps_lat:.6f}  sats={gps_sats}", end="\r")
        time.sleep(1.0)

    if not gps_valid:
        print(f"\n[WARN] GPS Fix 未取得。走行を開始しますが精度が低下します。")
    else:
        print(f"\n[INIT] GPS Fix 取得！ lat={gps_lat:.6f}  lng={gps_lng:.6f}  sats={gps_sats}")

    # ── モータ初期化 ──
    print("[INIT] モータ初期化中...")
    motor = MotorController()
    print("[INIT] 全センサ・モータ 初期化完了\n")

    # ── コンソールヘッダー ──
    separator = "-" * len(HEADER)
    print(separator)
    print(HEADER)
    print(separator)

    # ── CSV ヘッダー ──
    csv_header = [
        "Time_s",
        "Lat", "Lng",
        "Distance_m", "TargetBearing_deg", "Azimuth_deg",
        "Diff_deg",
        "MagX_uT", "MagY_uT", "MagZ_uT",
        "AccX_ms2", "AccY_ms2", "AccZ_ms2",
        "GPS_Speed_kts", "GPS_Sats",
        "Motor_cmd",
    ]
    log_rows   = []
    line_count = 0

    start_time = time.time()
    goal_reached  = False
    timed_out     = False

    try:
        while True:
            loop_start = time.time()
            elapsed    = loop_start - start_time

            # ── タイムアウト判定 ──
            if elapsed > TIMEOUT_SEC:
                print(f"\n[INFO] タイムアウト ({TIMEOUT_SEC} s) — 停止します。")
                timed_out = True
                break

            # ── BNO055: 加速度・地磁気取得 ──
            try:
                acc = bno.getAcc()   # [m/s²]
                mag = bno.getMag()   # [uT]
            except Exception as e:
                print(f"[WARN] BNO055: {e}")
                acc = [0.0, 0.0, 0.0]
                mag = [0.0, 0.0, 0.0]

            # ── 方位角計算 ──
            azimuth = calc_azimuth(mag)

            # ── GPS 取得 (スレッド共有変数) ──
            lat  = gps_lat
            lng  = gps_lng
            sats = gps_sats

            # ── 航法計算 ──
            distance       = calc_distance(lat, lng)
            target_bearing = calc_target_bearing(lat, lng)
            diff           = calc_direction_diff(azimuth, target_bearing)

            # ── ゴール判定 ──
            if gps_valid and distance < GOAL_RADIUS:
                print(f"\n[GOAL] 目標地点に到達！ distance={distance:.2f} m")
                goal_reached = True
                break

            # ── GPS 未取得なら停止して待機 ──
            if not gps_valid:
                motor.stop()
                motor_cmd = "GPS_WAIT"
            else:
                # ── モータ制御 ──
                motor_cmd = motor.apply_diff(diff)

            # ── コンソール表示 ──
            row_str = DATA_FMT.format(
                elapsed,
                lat, lng,
                distance, target_bearing, azimuth,
                diff,
                mag[0], mag[1],
                sats, motor_cmd,
            )
            print(row_str)

            # ── ログ蓄積 ──
            log_rows.append([
                round(elapsed, 3),
                round(lat,  6),  round(lng,  6),
                round(distance,       2),
                round(target_bearing, 2),
                round(azimuth,        2),
                round(diff,           2),
                round(mag[0], 3), round(mag[1], 3), round(mag[2], 3),
                round(acc[0], 4), round(acc[1], 4), round(acc[2], 4),
                round(gps_speed, 3), int(sats),
                motor_cmd,
            ])
            line_count += 1

            # ── ループ待機 ──
            wait = LOOP_DT - (time.time() - loop_start)
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        print("\n\n[INFO] Ctrl+C — 緊急停止します。")

    finally:
        motor.stop()
        motor.close()
        print("[INFO] モータ停止・リソース解放完了")

        # ── 結果サマリ ──
        print(f"\n{'=' * 55}")
        if goal_reached:
            print(f"  [RESULT] ゴール到達！")
        elif timed_out:
            print(f"  [RESULT] タイムアウト終了")
        else:
            print(f"  [RESULT] 中断終了")
        if log_rows:
            final_dist = log_rows[-1][3]
            print(f"  [RESULT] 最終距離: {final_dist:.2f} m")
        print(f"  [RESULT] 走行時間: {time.time() - start_time:.1f} s")
        print(f"  [RESULT] サンプル: {line_count} 点")
        print(f"{'=' * 55}")

        # ── CSV 保存 ──
        print(f"\n[INFO] ログ保存中: {log_path}")
        try:
            with open(log_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(csv_header)
                writer.writerows(log_rows)
            print(f"[INFO] {line_count} 行を保存しました → {log_path}")
        except Exception as e:
            print(f"[ERROR] CSV 保存失敗: {e}")


# ===========================================================================
if __name__ == "__main__":
    main()
