"""
test_GPSrun.py
==============
GPS 誘導制御走行 テストプログラム

NSE2026/test/test_GPSrun.py
"""

import sys
import math
import time
import csv
import threading
import datetime
from pathlib import Path

# --- gpiozero (test_run.py / fall.py / test_straight.py と同じバックエンド) ---
from gpiozero import PWMOutputDevice, Device, OutputDevice
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
# 設定 ─── 実機・動作確認に合わせて変更する項目
# ===========================================================================

# --- モータ動作カスタマイズ設定（ハードウェアの繋ぎ方に合わせて調整）---
SWAP_LEFT_RIGHT        = False  # True: モータA/Bの左右割り当てを入れ替える
INVERT_LEFT_DIR        = False  # True: 左モータの正転/逆転を反転させる
INVERT_RIGHT_DIR       = False  # True: 右モータの正転/逆転を反転させる
ALLOW_TEST_WITHOUT_GPS = True   # True: GPS未Fix時でもモータ制御を実行（動作テスト用）

# --- BNO055 の取り付け向き設定 ---
# 1 : 正規の向き (0°回転)
# 2 : Z軸周りに 90°回転
# 3 : Z軸周りに 180°回転
# 4 : Z軸周りに 270°回転 (-90°回転)
BNO_ORIENTATION = 1

# --- 目標座標 ---
TARGET_LAT =  38.263710      # 目標緯度  [度]
TARGET_LNG = 140.860598    # 目標経度  [度]

# --- 地磁気偏角補正 (仙台付近: 約 −8.0°) ---
MAG_DECLINATION = -8.0      # [度]

# --- 到達判定半径 ---
GOAL_RADIUS = 3.0           # [m]  この距離以内でゴール

# --- 制御ループ ---
LOOP_DT      = 0.1          # [s]  制御周期
TIMEOUT_SEC  = 10 * 60      # [s]  走行タイムアウト (10分)

# --- 方向制御閾値 ---
ANGLE_DEADBAND    = 10.0    # [度]  この範囲内なら前進
ANGLE_TURN_STRONG = 45.0    # [度]  この角度以上で強旋回

# --- モータ出力 ---
SPEED_FWD   = 0.8           # 前進時 PWM duty (0.0〜1.0)
SPEED_TURN  = 0.8           # 旋回時 外輪 PWM duty
SPEED_WEAK  = 0.4           # 弱旋回時 内輪 PWM duty

# --- GPS ---
GPS_PORT     = "/dev/serial0"
GPS_BAUDRATE = 9600
GPS_FIX_TIMEOUT = 120.0     # [s]  Fix 待機タイムアウト

# --- ログ ---
LOG_DIR = SCRIPT_DIR.parent / "logs"

# --- 地球半径 ---
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
# BNO055 軸再配置 (Axis Remap) 処理
# ===========================================================================

def set_bno_orientation(bno: BNO055, orientation: int):
    remap_profiles = {
        1: (0x24, 0x00),  # P0: 0°  (正規)
        2: (0x21, 0x04),  # P1: 90°
        3: (0x24, 0x06),  # P2: 180°
        4: (0x21, 0x02),  # P3: 270°
    }

    if orientation not in remap_profiles:
        print(f"[WARN] 無効な BNO_ORIENTATION ({orientation})。デフォルト(1)で動作します。")
        orientation = 1

    config_val, sign_val = remap_profiles[orientation]

    try:
        if hasattr(bno, 'write_reg'):
            bno.write_reg(0x3D, 0x00)
        elif hasattr(bno, 'set_mode'):
            bno.set_mode(BNO055.OPERATION_MODE_CONFIG)
        time.sleep(0.05)

        if hasattr(bno, 'write_reg'):
            bno.write_reg(0x41, config_val)
            bno.write_reg(0x42, sign_val)
        elif hasattr(bno, 'write_bytes'):
            bno.write_bytes(0x41, [config_val])
            bno.write_bytes(0x42, [sign_val])
        elif hasattr(bno, '_i2c_bus'):
            bno._i2c_bus.write_byte_data(bno._address, 0x41, config_val)
            bno._i2c_bus.write_byte_data(bno._address, 0x42, sign_val)
        time.sleep(0.02)

        if hasattr(bno, 'write_reg'):
            bno.write_reg(0x3D, 0x0C)
        elif hasattr(bno, 'set_mode'):
            bno.set_mode(BNO055.OPERATION_MODE_NDOF)
        time.sleep(0.05)

        degrees_map = {1: "0°(正規)", 2: "90°", 3: "180°", 4: "270°"}
        print(f"[BNO055] Axis Remap 完了: 設定 {orientation} ({degrees_map[orientation]})")

    except Exception as e:
        print(f"[WARN] BNO055 Axis Remap 設定時の警告: {e}")

# ===========================================================================
# GPS スレッド
# ===========================================================================

def gps_thread_func(gps_obj: MicropyGPS, port: str, baudrate: int):
    global gps_lat, gps_lng, gps_speed, gps_sats, gps_valid

    if not SERIAL_AVAILABLE:
        print("[GPS] pyserial が見つかりません。GPS は無効です。")
        return

    try:
        with serial.Serial(port, baudrate, timeout=10) as ser:
            print(f"[GPS] Serial open: {port} @ {baudrate} bps")
            ser.readline()
            
            while True:
                try:
                    if ser.in_waiting > 128:
                        ser.reset_input_buffer()

                    sentence = ser.readline().decode('utf-8')
                    if sentence == "" or sentence[0] != '$':
                        continue

                    for char in sentence:
                        gps_obj.update(char)

                    if gps_obj.clean_sentences > 20:
                        lat_raw = gps_obj.latitude
                        lng_raw = gps_obj.longitude

                        lat = lat_raw[0]
                        if lat_raw[1] == 'S':
                            lat = -lat
                        lng = lng_raw[0]
                        if lng_raw[1] == 'W':
                            lng = -lng

                        gps_lat   = lat
                        gps_lng   = lng
                        gps_speed = gps_obj.speed[0]
                        gps_sats  = gps_obj.satellites_in_use
                        gps_valid = (lat != 0.0)

                except Exception as e:
                    print(f"[GPS] 読み取りエラー: {e}")

    except serial.SerialException as e:
        print(f"[GPS] ポートを開けません ({port}): {e}")

# ===========================================================================
# 航法計算
# ===========================================================================

def calc_distance(lat: float, lng: float) -> float:
    dx = math.radians(TARGET_LNG - lng) * EARTH_RADIUS * math.cos(math.radians(lat))
    dy = math.radians(TARGET_LAT - lat) * EARTH_RADIUS
    return math.hypot(dx, dy)

def calc_target_bearing(lat: float, lng: float) -> float:
    dx = math.radians(TARGET_LNG - lng) * EARTH_RADIUS * math.cos(math.radians(lat))
    dy = math.radians(TARGET_LAT - lat) * EARTH_RADIUS
    angle = 90.0 - math.degrees(math.atan2(dy, dx))
    return angle % 360.0

def calc_azimuth(mag: list) -> float:
    azimuth = 90.0 - math.degrees(math.atan2(mag[1], mag[0]))
    azimuth *= -1
    azimuth += MAG_DECLINATION
    return azimuth % 360.0

def calc_direction_diff(azimuth: float, target_bearing: float) -> float:
    diff = (azimuth - target_bearing) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff

# ===========================================================================
# モータコントローラ（確実にGPIO出力を行う高信頼設計）
# ===========================================================================

# BCMピン番号（元コードの標準構成）
PIN_PWMA = 13
PIN_AIN1 = 5
PIN_AIN2 = 6
PIN_PWMB = 18
PIN_BIN1 = 23
PIN_BIN2 = 24
PIN_STBY = 11


class MotorController:
    """
    左右モータの独立 PWM / 方向制御。
    OutputDevice と PWMOutputDevice を使用し、信号競合を完全に防ぎます。
    """

    def __init__(self, speed_fwd: float = SPEED_FWD,
                 speed_turn: float = SPEED_TURN,
                 speed_weak: float = SPEED_WEAK):
        Device.pin_factory = LGPIOFactory()
        self.speed_fwd  = speed_fwd
        self.speed_turn = speed_turn
        self.speed_weak = speed_weak

        # PWM 制御ピン
        self._pwm_a = PWMOutputDevice(PIN_PWMA)
        self._pwm_b = PWMOutputDevice(PIN_PWMB)

        # 方向制御ピン (OutputDevice で手動制御)
        self._ain1 = OutputDevice(PIN_AIN1)
        self._ain2 = OutputDevice(PIN_AIN2)
        self._bin1 = OutputDevice(PIN_BIN1)
        self._bin2 = OutputDevice(PIN_BIN2)

        # スタンバイピン
        self._stby = OutputDevice(PIN_STBY)
        self.stop()

    def _set_channel(self, is_channel_a: bool, speed: float, forward: bool):
        """各モータチャンネルの出力設定"""
        if is_channel_a:
            pwm = self._pwm_a
            in1, in2 = self._ain1, self._ain2
            invert = INVERT_LEFT_DIR if not SWAP_LEFT_RIGHT else INVERT_RIGHT_DIR
        else:
            pwm = self._pwm_b
            in1, in2 = self._bin1, self._bin2
            invert = INVERT_RIGHT_DIR if not SWAP_LEFT_RIGHT else INVERT_LEFT_DIR

        if invert:
            forward = not forward

        if abs(speed) < 0.01:
            pwm.value = 0.0
            in1.off()
            in2.off()
        else:
            pwm.value = min(max(abs(speed), 0.0), 1.0)
            if forward:
                in1.on()
                in2.off()
            else:
                in1.off()
                in2.on()

    def set_motors(self, left_speed: float, right_speed: float):
        """左右モータの出力設定 (正: 前進, 負: 後進, 0: 停止)"""
        self._stby.on()
        left_ch_a = not SWAP_LEFT_RIGHT

        # 左モータ出力
        self._set_channel(
            is_channel_a=left_ch_a,
            speed=abs(left_speed),
            forward=(left_speed >= 0)
        )
        # 右モータ出力
        self._set_channel(
            is_channel_a=not left_ch_a,
            speed=abs(right_speed),
            forward=(right_speed >= 0)
        )

    def stop(self):
        self._pwm_a.value = 0.0
        self._pwm_b.value = 0.0
        self._ain1.off()
        self._ain2.off()
        self._bin1.off()
        self._bin2.off()
        self._stby.off()

    def forward(self):
        self.set_motors(self.speed_fwd, self.speed_fwd)

    def turn_left_strong(self):
        self.set_motors(0.0, self.speed_turn)

    def turn_right_strong(self):
        self.set_motors(self.speed_turn, 0.0)

    def turn_left_weak(self):
        self.set_motors(self.speed_weak, self.speed_fwd)

    def turn_right_weak(self):
        self.set_motors(self.speed_fwd, self.speed_weak)

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
        self._ain1.close()
        self._ain2.close()
        self._bin1.close()
        self._bin2.close()
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
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts_str   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"gpsrun_{ts_str}.csv"

    print("=" * 75)
    print("  test_GPSrun.py  GPS 誘導制御走行 テスト")
    print(f"  {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  BNO055 向き設定: {BNO_ORIENTATION}")
    print(f"  目標地点: LAT={TARGET_LAT}  LNG={TARGET_LNG}")
    print(f"  ゴール半径: {GOAL_RADIUS} m  /  タイムアウト: {TIMEOUT_SEC} s")
    print("=" * 75)

    # ── BNO055 初期化 ──
    print("\n[INIT] BNO055 初期化中...")
    bno = BNO055()
    if not bno.setUp(operation_mode=BNO055.OPERATION_MODE_NDOF):
        print("[ERROR] BNO055 の初期化に失敗しました。終了します。")
        sys.exit(1)

    set_bno_orientation(bno, BNO_ORIENTATION)

    # ── GPS スレッド起動 ──
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
        print(f"\n[WARN] GPS Fix 未取得。テストモード動作可否 = {ALLOW_TEST_WITHOUT_GPS}")
    else:
        print(f"\n[INIT] GPS Fix 取得！ lat={gps_lat:.6f}  lng={gps_lng:.6f}  sats={gps_sats}")

    # ── モータ初期化 ──
    print("[INIT] モータ初期化中...")
    motor = MotorController()
    print("[INIT] 全センサ・モータ 初期化完了\n")

    separator = "-" * len(HEADER)
    print(separator)
    print(HEADER)
    print(separator)

    csv_header = [
        "Time_s", "Lat", "Lng",
        "Distance_m", "TargetBearing_deg", "Azimuth_deg", "Diff_deg",
        "MagX_uT", "MagY_uT", "MagZ_uT",
        "AccX_ms2", "AccY_ms2", "AccZ_ms2",
        "GPS_Speed_kts", "GPS_Sats", "Motor_cmd",
    ]
    log_rows   = []
    line_count = 0

    start_time = time.time()
    goal_reached = False
    timed_out    = False

    try:
        while True:
            loop_start = time.time()
            elapsed    = loop_start - start_time

            if elapsed > TIMEOUT_SEC:
                print(f"\n[INFO] タイムアウト ({TIMEOUT_SEC} s) — 停止します。")
                timed_out = True
                break

            try:
                acc = bno.getAcc()
                mag = bno.getMag()
            except Exception as e:
                print(f"[WARN] BNO055: {e}")
                acc = [0.0, 0.0, 0.0]
                mag = [0.0, 0.0, 0.0]

            azimuth = calc_azimuth(mag)

            lat  = gps_lat
            lng  = gps_lng
            sats = gps_sats

            distance       = calc_distance(lat, lng)
            target_bearing = calc_target_bearing(lat, lng)
            diff           = calc_direction_diff(azimuth, target_bearing)

            if gps_valid and distance < GOAL_RADIUS:
                print(f"\n[GOAL] 目標地点に到達！ distance={distance:.2f} m")
                goal_reached = True
                break

            # ── モータ制御判定 ──
            if not gps_valid and not ALLOW_TEST_WITHOUT_GPS:
                motor.stop()
                motor_cmd = "GPS_WAIT"
            else:
                motor_cmd = motor.apply_diff(diff)

            row_str = DATA_FMT.format(
                elapsed, lat, lng,
                distance, target_bearing, azimuth, diff,
                mag[0], mag[1], sats, motor_cmd,
            )
            print(row_str)

            log_rows.append([
                round(elapsed, 3), round(lat, 6), round(lng, 6),
                round(distance, 2), round(target_bearing, 2), round(azimuth, 2), round(diff, 2),
                round(mag[0], 3), round(mag[1], 3), round(mag[2], 3),
                round(acc[0], 4), round(acc[1], 4), round(acc[2], 4),
                round(gps_speed, 3), int(sats), motor_cmd,
            ])
            line_count += 1

            wait = LOOP_DT - (time.time() - loop_start)
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        print("\n\n[INFO] Ctrl+C — 緊急停止します。")

    finally:
        motor.stop()
        motor.close()
        print("[INFO] モータ停止・リソース解放完了")

        print(f"\n{'=' * 55}")
        if goal_reached:
            print("  [RESULT] ゴール到達！")
        elif timed_out:
            print("  [RESULT] タイムアウト終了")
        else:
            print("  [RESULT] 中断終了")
        if log_rows:
            print(f"  [RESULT] 最終距離: {log_rows[-1][3]:.2f} m")
        print(f"  [RESULT] 走行時間: {time.time() - start_time:.1f} s")
        print(f"  [RESULT] サンプル: {line_count} 点")
        print(f"{'=' * 55}")

        print(f"\n[INFO] ログ保存中: {log_path}")
        try:
            with open(log_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(csv_header)
                writer.writerows(log_rows)
            print(f"[INFO] {line_count} 行を保存しました → {log_path}")
        except Exception as e:
            print(f"[ERROR] CSV 保存失敗: {e}")


if __name__ == "__main__":
    main()
