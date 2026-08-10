"""
gps_guidance_run.py
===================
GPS・地磁気センサ（BNO055）併用 誘導制御走行プログラム

概要:
    目標地点（TARGET_LAT / TARGET_LNG）へ向けて自律走行します。
    BNO055から機体の方位角（azimuth）を取得し、GPS位置から計算した
    目標方位（target_bearing）との差分（diff）に基づいて左右モータのPWM出力を制御します。

ハードウェア割り当て:
    - 右モータ (Motor A): PWMA=13, AIN1=5, AIN2=6
    - 左モータ (Motor B): PWMB=18, BIN1=23, BIN2=24
    - モータドライバ有効化: STBY=11
    - 地磁気・加速度センサ: BNO055 (I2C)
    - GPSモジュール: /dev/serial0 (9600 bps)
"""

import sys
import math
import time
import csv
import threading
import datetime
from pathlib import Path

# --- gpiozero & lgpio ピンファクトリ設定 ---
from gpiozero import Motor, PWMOutputDevice, Device, OutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

# --- シリアル通信 (GPS用) ---
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# --- センサモジュールパス設定 ---
SCRIPT_DIR = Path(__file__).resolve().parent
SENSOR_DIR = SCRIPT_DIR.parent / "sensor"
if str(SENSOR_DIR) not in sys.path:
    sys.path.insert(0, str(SENSOR_DIR))

from BNO055 import BNO055
from micropyGPS import MicropyGPS

# ===========================================================================
# 定数・パラメータ設定
# ===========================================================================

# --- モータ制御ピン (BCMピン番号) ---
PIN_PWMA = 13
PIN_AIN1 = 5
PIN_AIN2 = 6
PIN_PWMB = 18
PIN_BIN1 = 23
PIN_BIN2 = 24
PIN_STBY = 11

# --- BNO055 取り付け向き (1:0°, 2:90°, 3:180°, 4:270°) ---
BNO_ORIENTATION = 1

# --- 目標地点・航法パラメータ ---
TARGET_LAT = 38.26052       # 目標緯度 [度]
TARGET_LNG = 140.8544151    # 目標経度 [度]
MAG_DECLINATION = -8.0      # 地磁気偏角補正 (仙台周辺: 約 -8.0度)
GOAL_RADIUS = 3.0           # 到達判定閾値 [m]
EARTH_RADIUS = 6378136.59   # 地球半径 [m]

# --- 走行制御パラメータ ---
LOOP_DT = 0.1               # 制御周期 [s]
TIMEOUT_SEC = 600           # タイムアウト時間 [s] (10分)
ANGLE_DEADBAND = 10.0       # 前進許容角度差 [度] (|diff| < 10度で直進)
ANGLE_TURN_STRONG = 45.0    # 強旋回閾値 [度] (|diff| > 45度で片輪停止旋回)

# --- モータPWM出力 (0.0 〜 1.0) ---
SPEED_FWD = 0.8             # 前進時 duty
SPEED_TURN = 0.8            # 強旋回時 外輪 duty
SPEED_WEAK = 0.4            # 弱旋回時 内輪 duty

# --- GPS設定 ---
GPS_PORT = "/dev/serial0"
GPS_BAUDRATE = 9600
GPS_FIX_TIMEOUT = 120.0     # Fix待機タイムアウト [s]

# --- ログディレクトリ ---
LOG_DIR = SCRIPT_DIR.parent / "logs"

# ===========================================================================
# 共有グローバル変数 (GPSスレッド用)
# ===========================================================================
gps_lat = 0.0
gps_lng = 0.0
gps_speed = 0.0
gps_sats = 0
gps_valid = False

# ===========================================================================
# BNO055 軸再配置 (Axis Remap)
# ===========================================================================
def set_bno_orientation(bno: BNO055, orientation: int):
    """BNO055の配置方向に応じてレジスタ(0x41, 0x42)を操作し軸を再配置する"""
    remap_profiles = {
        1: (0x24, 0x00),  # 0° (正規)
        2: (0x21, 0x04),  # 90° 回転
        3: (0x24, 0x06),  # 180° 回転
        4: (0x21, 0x02),  # 270° 回転
    }

    if orientation not in remap_profiles:
        orientation = 1

    config_val, sign_val = remap_profiles[orientation]

    try:
        if hasattr(bno, 'write_reg'):
            bno.write_reg(0x3D, 0x00)  # CONFIGMODE
        elif hasattr(bno, 'set_mode'):
            bno.set_mode(BNO055.OPERATION_MODE_CONFIG)
        time.sleep(0.05)

        if hasattr(bno, 'write_reg'):
            bno.write_reg(0x41, config_val)
            bno.write_reg(0x42, sign_val)
        elif hasattr(bno, '_i2c_bus'):
            bno._i2c_bus.write_byte_data(bno._address, 0x41, config_val)
            bno._i2c_bus.write_byte_data(bno._address, 0x42, sign_val)
        time.sleep(0.02)

        if hasattr(bno, 'write_reg'):
            bno.write_reg(0x3D, 0x0C)  # NDOF モード復帰
        elif hasattr(bno, 'set_mode'):
            bno.set_mode(BNO055.OPERATION_MODE_NDOF)
        time.sleep(0.05)
        print(f"[BNO055] Axis Remap 完了 (設定パターン: {orientation})")
    except Exception as e:
        print(f"[WARN] BNO055 Axis Remap 失敗: {e}")

# ===========================================================================
# GPS読み取りスレッド
# ===========================================================================
def gps_thread_func(gps_obj: MicropyGPS, port: str, baudrate: int):
    """GPSからのNMEAセンテンスをバックグラウンドで解析・更新する"""
    global gps_lat, gps_lng, gps_speed, gps_sats, gps_valid

    if not SERIAL_AVAILABLE:
        print("[GPS] pyserial が利用できないため、GPSは無効化されます。")
        return

    try:
        with serial.Serial(port, baudrate, timeout=10) as ser:
            ser.readline()  # 1行目破棄
            while True:
                try:
                    if ser.in_waiting > 128:
                        ser.reset_input_buffer()

                    sentence = ser.readline().decode('utf-8', errors='ignore')
                    if not sentence or sentence[0] != '$':
                        continue

                    for char in sentence:
                        gps_obj.update(char)

                    if gps_obj.clean_sentences > 10:
                        lat_raw = gps_obj.latitude
                        lng_raw = gps_obj.longitude

                        lat = lat_raw[0] * (-1 if lat_raw[1] == 'S' else 1)
                        lng = lng_raw[0] * (-1 if lng_raw[1] == 'W' else 1)

                        gps_lat = lat
                        gps_lng = lng
                        gps_speed = gps_obj.speed[0]
                        gps_sats = gps_obj.satellites_in_use
                        gps_valid = (lat != 0.0 and lng != 0.0)

                except Exception as e:
                    print(f"[GPS] 受信エラー: {e}")
    except Exception as e:
        print(f"[GPS] シリアルポートオープンエラー ({port}): {e}")

# ===========================================================================
# 航法演算関数
# ===========================================================================
def calc_distance(lat: float, lng: float) -> float:
    """現在地から目標地点までの平面近似距離 [m] を算出"""
    dx = math.radians(TARGET_LNG - lng) * EARTH_RADIUS * math.cos(math.radians(lat))
    dy = math.radians(TARGET_LAT - lat) * EARTH_RADIUS
    return math.hypot(dx, dy)

def calc_target_bearing(lat: float, lng: float) -> float:
    """現在地から目標地点への目標方位角 [度, 0〜360, 北=0 時計回り] を算出"""
    dx = math.radians(TARGET_LNG - lng) * EARTH_RADIUS * math.cos(math.radians(lat))
    dy = math.radians(TARGET_LAT - lat) * EARTH_RADIUS
    angle = 90.0 - math.degrees(math.atan2(dy, dx))
    return angle % 360.0

def calc_azimuth(mag: list) -> float:
    """BNO055の地磁気値から機体の方位角 [度, 0〜360, 北=0] を算出"""
    azimuth = 90.0 - math.degrees(math.atan2(mag[1], mag[0]))
    azimuth = -azimuth + MAG_DECLINATION
    return azimuth % 360.0

def calc_direction_diff(azimuth: float, target_bearing: float) -> float:
    """機体方位と目標方位の差分 [-180 〜 +180 度] を算出"""
    diff = (azimuth - target_bearing) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff

# ===========================================================================
# モータコントローラ
# ===========================================================================
class MotorController:
    """gpiozero + lgpio による左右独立PWMモータ制御クラス"""
    def __init__(self, speed_fwd=SPEED_FWD, speed_turn=SPEED_TURN, speed_weak=SPEED_WEAK):
        Device.pin_factory = LGPIOFactory()
        self.speed_fwd = speed_fwd
        self.speed_turn = speed_turn
        self.speed_weak = speed_weak

        self._pwm_a = PWMOutputDevice(PIN_PWMA)
        self._pwm_b = PWMOutputDevice(PIN_PWMB)
        self._motor_a = Motor(forward=PIN_AIN1, backward=PIN_AIN2)  # 右モータ
        self._motor_b = Motor(forward=PIN_BIN1, backward=PIN_BIN2)  # 左モータ
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
        """強左旋回 (右前進 / 左停止)"""
        self._stby.on()
        self._pwm_a.value = self.speed_turn
        self._pwm_b.value = 0.0
        self._motor_a.forward()
        self._motor_b.stop()

    def turn_right_strong(self):
        """強右旋回 (右停止 / 左前進)"""
        self._stby.on()
        self._pwm_a.value = 0.0
        self._pwm_b.value = self.speed_turn
        self._motor_a.stop()
        self._motor_b.forward()

    def turn_left_weak(self):
        """弱左旋回 (右高速 / 左低速)"""
        self._stby.on()
        self._pwm_a.value = self.speed_fwd
        self._pwm_b.value = self.speed_weak
        self._motor_a.forward()
        self._motor_b.forward()

    def turn_right_weak(self):
        """弱右旋回 (右低速 / 左高速)"""
        self._stby.on()
        self._pwm_a.value = self.speed_weak
        self._pwm_b.value = self.speed_fwd
        self._motor_a.forward()
        self._motor_b.forward()

    def apply_diff(self, diff: float) -> str:
        """角度差分 diff に基づいて走行状態を自動切り替え"""
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

# ===========================================================================
# メイン処理
# ===========================================================================
def main():
    # --- ログディレクトリ作成 & ファイルパス準備 ---
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"gpsrun_{ts_str}.csv"

    print("=" * 70)
    print(" GPS 誘導走行制御プログラム 開始")
    print(f" 目標地点: Lat={TARGET_LAT}, Lng={TARGET_LNG}")
    print(f" ゴール判定閾値: {GOAL_RADIUS} m")
    print("=" * 70)

    # --- BNO055 初期化 ---
    print("[INIT] BNO055 センサ初期化中...")
    bno = BNO055()
    if not bno.setUp(operation_mode=BNO055.OPERATION_MODE_NDOF):
        print("[ERROR] BNO055 の初期化に失敗しました。プログラムを終了します。")
        sys.exit(1)

    set_bno_orientation(bno, BNO_ORIENTATION)

    # --- GPS スレッド開始 ---
    gps_obj = MicropyGPS(local_offset=9, location_formatting='dd')
    gps_thread = threading.Thread(
        target=gps_thread_func,
        args=(gps_obj, GPS_PORT, GPS_BAUDRATE),
        daemon=True
    )
    gps_thread.start()

    # --- GPS Fix 待機 ---
    print(f"[INIT] GPS Fix 待機中 (最大 {GPS_FIX_TIMEOUT:.0f} 秒)...")
    fix_deadline = time.time() + GPS_FIX_TIMEOUT
    while not gps_valid and time.time() < fix_deadline:
        print(f"  Fix待機中... Lat={gps_lat:.6f}, Sats={gps_sats}", end="\r")
        time.sleep(1.0)

    if not gps_valid:
        print("\n[WARN] GPS Fix未取得のまま制御ループへ移行します。")
    else:
        print(f"\n[INIT] GPS Fix 取得完了: Lat={gps_lat:.6f}, Lng={gps_lng:.6f}, Sats={gps_sats}")

    # --- モータ初期化 ---
    motor = MotorController()
    print("[INIT] モータ制御初期化完了\n")

    # --- CSV ログ設定 ---
    csv_header = [
        "Time_s", "Lat", "Lng", "Distance_m", "TargetBearing_deg",
        "Azimuth_deg", "Diff_deg", "MagX_uT", "MagY_uT", "MagZ_uT",
        "AccX_ms2", "AccY_ms2", "AccZ_ms2", "GPS_Speed_kts", "GPS_Sats", "Motor_cmd"
    ]
    log_rows = []

    start_time = time.time()
    goal_reached = False
    timed_out = False

    try:
        while True:
            loop_start = time.time()
            elapsed = loop_start - start_time

            # タイムアウト判定
            if elapsed > TIMEOUT_SEC:
                print(f"\n[INFO] タイムアウト ({TIMEOUT_SEC}s) に達したため停止します。")
                timed_out = True
                break

            # センサ値取得
            try:
                acc = bno.getAcc()
                mag = bno.getMag()
            except Exception as e:
                print(f"[WARN] センサデータ取得失敗: {e}")
                acc = [0.0, 0.0, 0.0]
                mag = [0.0, 0.0, 0.0]

            # 方位計算
            azimuth = calc_azimuth(mag)

            # GPS 共有データ取得
            lat = gps_lat
            lng = gps_lng
            sats = gps_sats

            # 航法計算
            distance = calc_distance(lat, lng)
            target_bearing = calc_target_bearing(lat, lng)
            diff = calc_direction_diff(azimuth, target_bearing)

            # ゴール到達判定
            if gps_valid and distance < GOAL_RADIUS:
                print(f"\n[GOAL] 判定半径内({distance:.2f}m)に到達しました！")
                goal_reached = True
                break

            # モータ指令実行
            if not gps_valid:
                motor.stop()
                motor_cmd = "GPS_WAIT"
            else:
                motor_cmd = motor.apply_diff(diff)

            # コンソールログ表示
            print(f"[{elapsed:6.1f}s] Dist:{distance:6.2f}m | Target:{target_bearing:5.1f}° | "
                  f"Azim:{azimuth:5.1f}° | Diff:{diff:6.1f}° | Sats:{sats:2d} | Cmd:{motor_cmd}")

            # ログデータ追加
            log_rows.append([
                round(elapsed, 3), round(lat, 6), round(lng, 6),
                round(distance, 2), round(target_bearing, 2), round(azimuth, 2), round(diff, 2),
                round(mag[0], 3), round(mag[1], 3), round(mag[2], 3),
                round(acc[0], 4), round(acc[1], 4), round(acc[2], 4),
                round(gps_speed, 3), int(sats), motor_cmd
            ])

            # 制御周期調整
            wait_time = LOOP_DT - (time.time() - loop_start)
            if wait_time > 0:
                time.sleep(wait_time)

    except KeyboardInterrupt:
        print("\n[INFO] ユーザー操作 (Ctrl+C) により緊急停止します。")

    finally:
        motor.stop()
        motor.close()
        print("[INFO] モータ停止・リソース解放処理完了")

        # ログファイル書き込み
        if log_rows:
            try:
                with open(log_path, "w", newline="", encoding="utf-8") as f:
                    writer = csv.writer(f)
                    writer.writerow(csv_header)
                    writer.writerows(log_rows)
                print(f"[INFO] 走行データを保存しました: {log_path}")
            except Exception as e:
                print(f"[ERROR] ログ書き込み失敗: {e}")

if __name__ == "__main__":
    main()
