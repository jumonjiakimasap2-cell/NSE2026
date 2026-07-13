"""
main.py
=======
NSE2026 ミッションシーケンス メインプログラム

NSE2026/main/main.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
フェーズ構成:
    Phase 0 : 初期化 & SSH 切断耐性確立
                各センサ・モータを初期化する。
                nohup で自分自身が動いているため、SSH が切断されても
                ミッションは継続する。SSH が再接続されたらログが表示される。
                ログは /home/pi/NSE2026/logs/mission_YYYYMMDD_HHMMSS.csv に記録。

    Phase 1 : 落下検知
                BNO055 合成加速度ノルムが FALL_THRESHOLD 以下を
                FALL_COUNT_THRESHOLD 回連続 → 落下確定。
                FALL_TIMEOUT_SEC 経過でタイムアウト強制移行。

    Phase 2 : モータ前進 (30 秒)
                サブキャリア脱出 + 初期移動として 30 秒前進する。

    Phase 3 : キャリブレーション & 誘導走行準備
                BNO055 の地磁気・加速度・ジャイロキャリブレーション待機。
                各軸が CALIB_MIN_LEVEL 以上になるまで待つ。
                スタック検知の基準加速度を測定する。

    Phase 4 : GPS 誘導走行 (目標地点 5 m 以内まで)
                GPS + 地磁気で目標座標を追跡し前進。
                スタック検知 (超音波 + 水平加速度) で障害物を回避する。
                ゴール到達 or タイムアウトで終了。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
実行方法 (SSH 切断耐性):
    nohup python3 /home/pi/NSE2026/main/main.py > /home/pi/NSE2026/logs/nohup.log 2>&1 &
    # ↑ これで SSH が切れてもミッションが継続する

モータピン (BCM) ← test_run.py と統一:
    PWMA=13  AIN1=5  AIN2=6
    PWMB=18  BIN1=23 BIN2=24  STBY=11

センサ:
    BNO055   (I2C, NDOF)    ← test_finishv.py / fall.py と統一
    BMP180   (I2C, oss=3)   ← test_finishv.py と統一
    micropyGPS (/dev/serial0, 9600) ← test_GPSrun.py と統一
    HC-SR04  (TRIG=BCM8, ECHO=BCM7) ← sensor/HC-SR04.py と統一
    LED      (BCM21)        ← test_onoff.py と統一
"""

import sys
import os
import math
import time
import csv
import threading
import datetime
import subprocess
from pathlib import Path

# --- gpiozero ---------------------------------------------------------------
from gpiozero import Motor, PWMOutputDevice, OutputDevice, LED, Device
from gpiozero import DigitalOutputDevice, DigitalInputDevice
from gpiozero.pins.lgpio import LGPIOFactory

# --- シリアル ----------------------------------------------------------------
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# --- センサモジュールパス解決 -------------------------------------------------
_MAIN_DIR  = Path(__file__).resolve().parent        # NSE2026/main/
_ROOT_DIR  = _MAIN_DIR.parent                       # NSE2026/
_SENSOR_DIR = _ROOT_DIR / "sensor"
if str(_SENSOR_DIR) not in sys.path:
    sys.path.insert(0, str(_SENSOR_DIR))

from BNO055 import BNO055
from BMP180 import BMP180
from micropyGPS import MicropyGPS

# ===========================================================================
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#   設定値  ← ここを実地に合わせて変更
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# ===========================================================================

# --- 目標座標 ---------------------------------------------------------------
TARGET_LAT =  38.26052      # 目標緯度  [度]
TARGET_LNG = 140.8544151    # 目標経度  [度]
GOAL_RADIUS = 5.0           # [m]  この距離以内でゴール

# --- 地磁気偏角 (仙台付近) --------------------------------------------------
MAG_DECLINATION = -8.0      # [度]  西偏 → 負値

# --- Phase 1: 落下検知 -------------------------------------------------------
FALL_THRESHOLD       = 3.0  # [m/s²]  合成加速度ノルムがこれ以下 → 落下中
FALL_COUNT_THRESHOLD = 8    # 連続カウント数
FALL_TIMEOUT_SEC     = 7 * 60  # [s]  7 分でタイムアウト

# --- Phase 2: 初期前進 -------------------------------------------------------
PHASE2_FWD_SEC = 30.0       # [s]

# --- Phase 3: キャリブレーション ----------------------------------------------
CALIB_MIN_LEVEL   = 2       # 0〜3  各センサのキャリブレーション最低レベル
CALIB_TIMEOUT_SEC = 120.0   # [s]  キャリブレーション待機タイムアウト
# スタック検知用：水平加速度の「動いているとき」の基準を測る秒数
CALIB_ACC_MEASURE_SEC = 3.0

# --- Phase 4: 誘導走行 -------------------------------------------------------
LOOP_DT      = 0.1          # [s]  制御ループ周期
TIMEOUT_SEC  = 15 * 60      # [s]  走行タイムアウト (15 分)

# 方向制御閾値
ANGLE_DEADBAND    = 10.0    # [度]  この範囲内なら前進
ANGLE_TURN_STRONG = 45.0    # [度]  これ以上の角度差で強旋回

# スタック検知
STUCK_HORIZON_ACCEL_THRESH = 0.5   # [m/s²]  水平加速度ノルムがこれ未満 → スタック疑い
STUCK_SONAR_DIST_THRESH    = 0.3   # [m]     超音波距離がこれ未満でも → スタック疑い
STUCK_COUNT_THRESHOLD      = 20    # 連続カウント数 (LOOP_DT × N 秒間継続で確定)
STUCK_RECOVER_TURN_SEC     = 5.0   # [s]  回復: 右旋回時間
STUCK_RECOVER_FWD_SEC      = 30.0  # [s]  回復: 前進時間

# --- モータ -----------------------------------------------------------------
MOTOR_SPEED  = 0.8          # PWM duty (0.0〜1.0)
SPEED_WEAK   = 0.4          # 弱旋回側の duty

# モータピン (BCM) ← test_run.py / test_avoid.py と統一
PIN_PWMA = 13
PIN_AIN1 =  5
PIN_AIN2 =  6
PIN_PWMB = 18
PIN_BIN1 = 23
PIN_BIN2 = 24
PIN_STBY = 11

# LED
LED_PIN  = 21               # BCM21 ← test_onoff.py と統一

# GPS
GPS_PORT     = "/dev/serial0"
GPS_BAUDRATE = 9600

# 超音波センサ ← sensor/HC-SR04.py と統一
PIN_TRIG = 8
PIN_ECHO = 7
SONAR_SETTLE    = 0.01      # [s]
SONAR_PULSE     = 10e-6     # [s]
SONAR_TIMEOUT   = 0.03      # [s]
SOUND_SPEED     = 343.0     # [m/s]

# 地球半径
EARTH_RADIUS = 6378136.59   # [m]

# ログ
LOG_DIR = _ROOT_DIR / "logs"

# ===========================================================================
# グローバル共有変数
# ===========================================================================

phase = 0               # 現在フェーズ番号

# センサ最新値
g_acc        = [0.0, 0.0, 0.0]
g_mag        = [0.0, 0.0, 0.0]
g_gyro       = [0.0, 0.0, 0.0]
g_calib      = (0, 0, 0, 0)    # (sys, gyro, accel, mag)
g_temp       = 0.0
g_pressure   = 0.0
g_altitude   = 0.0
g_gps_lat    = 0.0
g_gps_lng    = 0.0
g_gps_speed  = 0.0
g_gps_sats   = 0
g_gps_valid  = False
g_sonar_m    = None             # None = 測定失敗

# ログバッファ
log_rows  = []
log_lock  = threading.Lock()

# ===========================================================================
# ロガー
# ===========================================================================

def log(msg: str, level: str = "INFO"):
    """タイムスタンプ付きログを標準出力に出す。nohup 経由でファイルにも残る。"""
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    line = f"[{ts}][Phase{phase}][{level}] {msg}"
    print(line, flush=True)

def log_sensor_row(elapsed: float, motor_cmd: str, note: str = ""):
    """センサ全値を 1 行ログとしてバッファに追記する。"""
    with log_lock:
        log_rows.append([
            round(elapsed, 3), phase,
            round(g_acc[0],  4), round(g_acc[1],  4), round(g_acc[2],  4),
            round(g_mag[0],  4), round(g_mag[1],  4), round(g_mag[2],  4),
            round(g_gyro[0], 4), round(g_gyro[1], 4), round(g_gyro[2], 4),
            g_calib[0], g_calib[1], g_calib[2], g_calib[3],
            round(g_temp,     3), round(g_pressure, 2), round(g_altitude, 3),
            round(g_gps_lat,  6), round(g_gps_lng,  6),
            round(g_gps_speed,3), int(g_gps_sats),
            round(g_sonar_m, 4) if g_sonar_m is not None else -1.0,
            motor_cmd, note,
        ])

def save_log(log_path: Path):
    csv_header = [
        "Time_s", "Phase",
        "AccX", "AccY", "AccZ",
        "MagX", "MagY", "MagZ",
        "GyroX", "GyroY", "GyroZ",
        "Cal_Sys", "Cal_Gyro", "Cal_Acc", "Cal_Mag",
        "Temp_C", "Pres_Pa", "Alt_m",
        "Lat", "Lng", "GPS_Speed_kts", "GPS_Sats",
        "Sonar_m",
        "Motor_cmd", "Note",
    ]
    with log_lock:
        rows = list(log_rows)
    try:
        with open(log_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(csv_header)
            writer.writerows(rows)
        log(f"CSV 保存完了: {log_path} ({len(rows)} 行)")
    except Exception as e:
        log(f"CSV 保存失敗: {e}", "ERROR")

# ===========================================================================
# GPS スレッド (test_GPSrun.py / test_finishv.py と同じ構造)
# ===========================================================================

def gps_thread_func(gps_obj: MicropyGPS):
    global g_gps_lat, g_gps_lng, g_gps_speed, g_gps_sats, g_gps_valid
    if not SERIAL_AVAILABLE:
        log("pyserial 未インストール。GPS 無効。", "WARN")
        return
    try:
        with serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=1.0) as ser:
            log(f"GPS Serial open: {GPS_PORT} @ {GPS_BAUDRATE}")
            ser.readline()  # 先頭不完全行を捨てる
            while True:
                try:
                    if ser.in_waiting > 128:
                        ser.reset_input_buffer()
                    line = ser.readline().decode("ascii", errors="replace")
                    if not line.startswith("$"):
                        continue
                    for c in line:
                        gps_obj.update(c)
                    if gps_obj.valid:
                        lr = gps_obj.latitude
                        lo = gps_obj.longitude
                        lat = lr[0] + lr[1] / 60.0
                        if lr[2] == 'S': lat = -lat
                        lng = lo[0] + lo[1] / 60.0
                        if lo[2] == 'W': lng = -lng
                        g_gps_lat   = lat
                        g_gps_lng   = lng
                        g_gps_speed = gps_obj.speed[0]
                        g_gps_sats  = gps_obj.satellites_in_use
                        g_gps_valid = (lat != 0.0)
                except Exception as e:
                    log(f"GPS 読み取りエラー: {e}", "WARN")
    except serial.SerialException as e:
        log(f"GPS ポートエラー: {e}", "WARN")

# ===========================================================================
# 超音波センサ (sensor/HC-SR04.py と同じ原理, デバイス共有のためインライン実装)
# ===========================================================================

class SonarSensor:
    """HC-SR04 超音波距離センサ。測定失敗時は None を返す。"""
    def __init__(self):
        self._trig = DigitalOutputDevice(PIN_TRIG, initial_value=False)
        self._echo = DigitalInputDevice(PIN_ECHO)
        self._available = True
        log(f"超音波センサ初期化完了 TRIG=BCM{PIN_TRIG} ECHO=BCM{PIN_ECHO}")

    def get_distance_m(self) -> float | None:
        if not self._available:
            return None
        try:
            self._trig.off()
            time.sleep(SONAR_SETTLE)
            self._trig.on()
            time.sleep(SONAR_PULSE)
            self._trig.off()

            t0 = time.time()
            while not self._echo.value:
                if time.time() - t0 > SONAR_TIMEOUT:
                    return None
            t_start = time.time()
            while self._echo.value:
                if time.time() - t_start > SONAR_TIMEOUT:
                    return None
            t_end = time.time()
            return (t_end - t_start) * SOUND_SPEED / 2.0
        except Exception as e:
            log(f"超音波センサエラー: {e}", "WARN")
            self._available = False
            return None

    def close(self):
        self._trig.close()
        self._echo.close()

# ===========================================================================
# モータコントローラ (test_run.py / fall.py / test_GPSrun.py と統一)
# ===========================================================================

class MotorController:
    def __init__(self):
        self._pwm_a  = PWMOutputDevice(PIN_PWMA)
        self._pwm_b  = PWMOutputDevice(PIN_PWMB)
        self._mot_a  = Motor(forward=PIN_AIN1, backward=PIN_AIN2)   # 右
        self._mot_b  = Motor(forward=PIN_BIN1, backward=PIN_BIN2)   # 左
        self._stby   = OutputDevice(PIN_STBY)
        self.stop()
        log("モータ初期化完了")

    def forward(self):
        self._stby.on()
        self._pwm_a.value = MOTOR_SPEED
        self._pwm_b.value = MOTOR_SPEED
        self._mot_a.forward()
        self._mot_b.forward()

    def stop(self):
        self._pwm_a.value = 0
        self._pwm_b.value = 0
        self._mot_a.stop()
        self._mot_b.stop()
        self._stby.off()

    def turn_left_strong(self):
        """右モータ前進 / 左モータ停止 → 左旋回"""
        self._stby.on()
        self._pwm_a.value = MOTOR_SPEED
        self._pwm_b.value = 0
        self._mot_a.forward()
        self._mot_b.stop()

    def turn_right_strong(self):
        """右モータ停止 / 左モータ前進 → 右旋回"""
        self._stby.on()
        self._pwm_a.value = 0
        self._pwm_b.value = MOTOR_SPEED
        self._mot_a.stop()
        self._mot_b.forward()

    def turn_left_weak(self):
        self._stby.on()
        self._pwm_a.value = MOTOR_SPEED
        self._pwm_b.value = SPEED_WEAK
        self._mot_a.forward()
        self._mot_b.forward()

    def turn_right_weak(self):
        self._stby.on()
        self._pwm_a.value = SPEED_WEAK
        self._pwm_b.value = MOTOR_SPEED
        self._mot_a.forward()
        self._mot_b.forward()

    def apply_diff(self, diff: float) -> str:
        """
        diff (−180〜+180) でモータ指令を自動選択。
        test_GPSrun.py の MotorController.apply_diff() と同じロジック。
        """
        if abs(diff) < ANGLE_DEADBAND:
            self.forward();           return "FORWARD"
        elif diff > ANGLE_TURN_STRONG:
            self.turn_left_strong();  return "TURN_L_STRONG"
        elif diff > 0:
            self.turn_left_weak();    return "TURN_L_WEAK"
        elif diff < -ANGLE_TURN_STRONG:
            self.turn_right_strong(); return "TURN_R_STRONG"
        else:
            self.turn_right_weak();   return "TURN_R_WEAK"

    def close(self):
        self.stop()
        self._pwm_a.close(); self._pwm_b.close()
        self._mot_a.close(); self._mot_b.close()
        self._stby.close()

# ===========================================================================
# 航法計算 (test_GPSrun.py と同一)
# ===========================================================================

def calc_distance(lat, lng) -> float:
    dx = math.radians(TARGET_LNG - lng) * EARTH_RADIUS * math.cos(math.radians(lat))
    dy = math.radians(TARGET_LAT - lat) * EARTH_RADIUS
    return math.hypot(dx, dy)

def calc_target_bearing(lat, lng) -> float:
    dx = math.radians(TARGET_LNG - lng) * EARTH_RADIUS * math.cos(math.radians(lat))
    dy = math.radians(TARGET_LAT - lat) * EARTH_RADIUS
    return (90.0 - math.degrees(math.atan2(dy, dx))) % 360.0

def calc_azimuth(mag: list) -> float:
    az = 90.0 - math.degrees(math.atan2(mag[1], mag[0]))
    az *= -1
    az += MAG_DECLINATION
    return az % 360.0

def calc_direction_diff(azimuth: float, bearing: float) -> float:
    diff = (azimuth - bearing) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff

# ===========================================================================
# フェーズ関数
# ===========================================================================

# ─── Phase 0: 初期化 & SSH 切断耐性確立 ────────────────────────────────────

def phase0_init() -> tuple:
    """
    全センサ・モータを初期化して返す。
    nohup で起動されている前提で SSH 切断耐性は OS 側が担保する。
    ログディレクトリを作成し、ログパスを返す。
    """
    global phase
    phase = 0
    log("=" * 62)
    log("  NSE2026 ミッションシーケンス 開始")
    log(f"  PID={os.getpid()}  (nohup で SSH 切断耐性あり)")
    log(f"  目標座標: LAT={TARGET_LAT}  LNG={TARGET_LNG}")
    log("=" * 62)

    # gpiozero バックエンド (全テストコードと統一)
    Device.pin_factory = LGPIOFactory()

    # ログ保存先
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"mission_{ts}.csv"
    log(f"ログパス: {log_path}")

    # LED 初期化 (ミッション動作中は点灯)
    led = LED(LED_PIN)
    led.on()
    log(f"LED 点灯 (BCM{LED_PIN})")

    # BNO055
    log("BNO055 初期化中...")
    bno = BNO055()
    if not bno.setUp(operation_mode=BNO055.OPERATION_MODE_NDOF):
        log("BNO055 初期化失敗", "ERROR")
        sys.exit(1)

    # BMP180
    log("BMP180 初期化中...")
    bmp = BMP180(oss=3)
    if not bmp.setUp():
        log("BMP180 初期化失敗", "ERROR")
        sys.exit(1)

    # 基準気圧 (3 回平均)
    log("基準気圧取得中...")
    bp_list = []
    for _ in range(3):
        bmp.getTemperature()
        bp_list.append(bmp.getPressure())
        time.sleep(0.1)
    base_pressure = sum(bp_list) / len(bp_list)
    log(f"基準気圧 = {base_pressure:.2f} Pa")

    # GPS
    gps_obj = MicropyGPS(local_offset=9, location_formatting='ddm')
    gps_t = threading.Thread(target=gps_thread_func, args=(gps_obj,), daemon=True)
    gps_t.start()

    # 超音波センサ
    sonar = SonarSensor()

    # モータ
    motor = MotorController()

    log("[Phase0] 全デバイス初期化完了")
    return bno, bmp, base_pressure, sonar, motor, led, log_path


# ─── Phase 1: 落下検知 ──────────────────────────────────────────────────────

def phase1_fall_detection(bno: BNO055, start_time: float) -> bool:
    global phase, g_acc, g_calib
    phase = 1
    log("─" * 62)
    log(f"[Phase1] 落下検知開始  閾値={FALL_THRESHOLD} m/s²  ×{FALL_COUNT_THRESHOLD}回連続")
    log(f"         タイムアウト={FALL_TIMEOUT_SEC} s")

    fall_count = 0
    p1_start   = time.time()

    # コンソールヘッダー
    print(f"\n{'経過[s]':>8}  {'acc_norm':>10}  {'fall_cnt':>8}  {'Calib(S/G/A/M)':>16}")
    print("-" * 50)

    while True:
        try:
            g_acc   = bno.getAcc()
            g_calib = bno.getCalibrationStatus()
        except Exception as e:
            log(f"BNO055 読み取りエラー: {e}", "WARN")
            time.sleep(0.05)
            continue

        acc_norm = math.sqrt(g_acc[0]**2 + g_acc[1]**2 + g_acc[2]**2)
        elapsed  = time.time() - p1_start

        # 落下判定 (NICS2026/NOA.py phase0() と同じ構造)
        if acc_norm <= FALL_THRESHOLD:
            fall_count += 1
        else:
            fall_count = 0

        calib_str = f"{g_calib[0]}/{g_calib[1]}/{g_calib[2]}/{g_calib[3]}"
        print(f"{elapsed:>8.2f}  {acc_norm:>10.3f}  {fall_count:>8d}  {calib_str:>16}", flush=True)
        log_sensor_row(time.time() - start_time, "WAIT_FALL", f"norm={acc_norm:.3f} cnt={fall_count}")

        if fall_count >= FALL_COUNT_THRESHOLD:
            log(f"[Phase1] 落下検知！ (連続 {fall_count} 回, {elapsed:.2f} s)")
            return True

        if elapsed > FALL_TIMEOUT_SEC:
            log(f"[Phase1] タイムアウト ({FALL_TIMEOUT_SEC} s) → 強制移行")
            return False

        time.sleep(0.05)


# ─── Phase 2: 初期前進 (30 秒) ───────────────────────────────────────────────

def phase2_initial_forward(motor: MotorController, start_time: float):
    global phase
    phase = 2
    log("─" * 62)
    log(f"[Phase2] 初期前進 {PHASE2_FWD_SEC:.0f} 秒")
    motor.forward()
    deadline = time.time() + PHASE2_FWD_SEC
    while time.time() < deadline:
        remaining = deadline - time.time()
        log(f"  前進中... 残り {remaining:.1f} s")
        log_sensor_row(time.time() - start_time, "FORWARD_PHASE2")
        time.sleep(1.0)
    motor.stop()
    log("[Phase2] 前進完了 → 停止")


# ─── Phase 3: キャリブレーション & 誘導準備 ──────────────────────────────────

def phase3_calibration(bno: BNO055, start_time: float) -> float:
    """
    BNO055 キャリブレーション待機。
    完了後、静止時の水平加速度ノルムを基準値として計測して返す。
    """
    global phase, g_acc, g_mag, g_gyro, g_calib
    phase = 3
    log("─" * 62)
    log(f"[Phase3] キャリブレーション待機  最低レベル={CALIB_MIN_LEVEL}  最大={CALIB_TIMEOUT_SEC} s")
    log("         機体を 8 の字に動かして地磁気をキャリブレーションしてください")

    # ヘッダー
    print(f"\n{'経過[s]':>8}  {'Sys':>4}  {'Gyro':>5}  {'Acc':>4}  {'Mag':>4}  {'状態':>12}")
    print("-" * 50)

    p3_start = time.time()
    calib_ok = False

    while True:
        try:
            g_acc   = bno.getAcc()
            g_mag   = bno.getMag()
            g_gyro  = bno.getGyro()
            g_calib = bno.getCalibrationStatus()
        except Exception as e:
            log(f"BNO055 エラー: {e}", "WARN")
            time.sleep(0.1)
            continue

        elapsed = time.time() - p3_start
        s, gy, ac, mg = g_calib

        # キャリブレーション状態の判定
        if s >= CALIB_MIN_LEVEL and gy >= CALIB_MIN_LEVEL and ac >= CALIB_MIN_LEVEL and mg >= CALIB_MIN_LEVEL:
            status = "OK ✓"
        elif mg < CALIB_MIN_LEVEL:
            status = "Mag 不足"
        elif gy < CALIB_MIN_LEVEL:
            status = "Gyro 不足"
        else:
            status = "待機中..."

        print(f"{elapsed:>8.2f}  {s:>4d}  {gy:>5d}  {ac:>4d}  {mg:>4d}  {status:>12}", flush=True)
        log_sensor_row(time.time() - start_time, "CALIBRATING", status)

        if s >= CALIB_MIN_LEVEL and gy >= CALIB_MIN_LEVEL and ac >= CALIB_MIN_LEVEL and mg >= CALIB_MIN_LEVEL:
            log(f"[Phase3] キャリブレーション完了！ Sys={s} Gyro={gy} Acc={ac} Mag={mg}")
            calib_ok = True
            break

        if elapsed > CALIB_TIMEOUT_SEC:
            log(f"[Phase3] キャリブレーションタイムアウト。現状で続行します。 Sys={s} Gyro={gy} Acc={ac} Mag={mg}", "WARN")
            break

        time.sleep(0.2)

    # ── 静止時水平加速度の基準計測 ──
    log(f"[Phase3] 静止基準加速度を {CALIB_ACC_MEASURE_SEC:.0f} 秒計測...")
    samples = []
    t_end = time.time() + CALIB_ACC_MEASURE_SEC
    while time.time() < t_end:
        try:
            acc = bno.getAcc()
            # 水平面 (XY) の加速度ノルム
            horiz = math.sqrt(acc[0]**2 + acc[1]**2)
            samples.append(horiz)
        except Exception:
            pass
        time.sleep(0.05)

    if samples:
        baseline_horiz_acc = sum(samples) / len(samples)
    else:
        baseline_horiz_acc = 0.0
    log(f"[Phase3] 静止基準水平加速度 = {baseline_horiz_acc:.4f} m/s²")
    log("[Phase3] 誘導走行準備完了")
    return baseline_horiz_acc


# ─── Phase 4: GPS 誘導走行 + スタック検知・回避 ──────────────────────────────

def phase4_guided_run(bno: BNO055, sonar: SonarSensor,
                      motor: MotorController, start_time: float,
                      baseline_horiz_acc: float):
    global phase, g_acc, g_mag, g_gyro, g_calib, g_sonar_m

    phase = 4
    log("─" * 62)
    log(f"[Phase4] GPS 誘導走行開始")
    log(f"  目標: ({TARGET_LAT}, {TARGET_LNG})  ゴール半径: {GOAL_RADIUS} m")
    log(f"  スタック閾値: 水平加速度 < {STUCK_HORIZON_ACCEL_THRESH} m/s²  or 超音波 < {STUCK_SONAR_DIST_THRESH} m")
    log(f"  スタック確定: {STUCK_COUNT_THRESHOLD} 回連続 ({STUCK_COUNT_THRESHOLD * LOOP_DT:.1f} s)")

    # コンソールヘッダー (横 1 行ログ)
    HDR = (f"{'T[s]':>7}  {'Lat':>11}  {'Lng':>12}  "
           f"{'Dist[m]':>8}  {'Bear[°]':>7}  {'Az[°]':>6}  "
           f"{'Diff[°]':>7}  {'Sonar[m]':>9}  "
           f"{'HorizAcc':>9}  {'StuckCnt':>8}  {'Motor':>14}  {'Note':>10}")
    DAT = (f"{{:>7.2f}}  {{:>11.6f}}  {{:>12.6f}}  "
           f"{{:>8.2f}}  {{:>7.2f}}  {{:>6.2f}}  "
           f"{{:>7.2f}}  {{:>9}}  "
           f"{{:>9.4f}}  {{:>8d}}  {{:>14}}  {{:>10}}")
    print()
    print("-" * len(HDR))
    print(HDR)
    print("-" * len(HDR))

    p4_start   = time.time()
    stuck_count = 0
    goal_reached = False
    timed_out    = False
    in_recovery  = False   # スタック回復中フラグ

    def do_recovery():
        """スタック回復動作: 右旋回 → 前進"""
        nonlocal stuck_count, in_recovery
        in_recovery = True
        log(f"[Phase4][STUCK] スタック検知！ 回復動作開始")
        log(f"         右旋回 {STUCK_RECOVER_TURN_SEC:.0f} s → 前進 {STUCK_RECOVER_FWD_SEC:.0f} s")

        motor.turn_right_strong()
        t_end = time.time() + STUCK_RECOVER_TURN_SEC
        while time.time() < t_end:
            log_sensor_row(time.time() - start_time, "RECOVER_TURN_R",
                           f"remain={t_end - time.time():.1f}s")
            time.sleep(0.5)

        motor.forward()
        t_end = time.time() + STUCK_RECOVER_FWD_SEC
        while time.time() < t_end:
            log_sensor_row(time.time() - start_time, "RECOVER_FORWARD",
                           f"remain={t_end - time.time():.1f}s")
            time.sleep(0.5)

        motor.stop()
        stuck_count = 0
        in_recovery = False
        log("[Phase4][STUCK] 回復動作完了。誘導走行に戻ります。")

    try:
        while True:
            loop_start = time.time()
            elapsed    = loop_start - p4_start

            # ── タイムアウト判定 ──
            if elapsed > TIMEOUT_SEC:
                log(f"[Phase4] タイムアウト ({TIMEOUT_SEC} s)")
                timed_out = True
                break

            # ── BNO055 取得 ──
            try:
                g_acc   = bno.getAcc()
                g_mag   = bno.getMag()
                g_gyro  = bno.getGyro()
                g_calib = bno.getCalibrationStatus()
            except Exception as e:
                log(f"BNO055 エラー: {e}", "WARN")
                g_acc = [0.0, 0.0, 0.0]
                g_mag = [0.0, 0.0, 0.0]

            # ── 超音波センサ取得 (失敗しても止めない) ──
            g_sonar_m = sonar.get_distance_m()
            sonar_str = f"{g_sonar_m:.3f}" if g_sonar_m is not None else "---"

            # ── 航法計算 ──
            azimuth        = calc_azimuth(g_mag)
            lat, lng       = g_gps_lat, g_gps_lng
            distance       = calc_distance(lat, lng)
            target_bearing = calc_target_bearing(lat, lng)
            diff           = calc_direction_diff(azimuth, target_bearing)

            # ── 水平加速度ノルム (スタック検知用) ──
            horiz_acc = math.sqrt(g_acc[0]**2 + g_acc[1]**2)

            # ── ゴール判定 ──
            if g_gps_valid and distance <= GOAL_RADIUS:
                log(f"[Phase4] ゴール到達！ distance={distance:.2f} m")
                goal_reached = True
                break

            # ── スタック検知 ──────────────────────────────────────────────
            # 判定 A: 水平加速度が基準より大幅に低い (動いていない)
            is_stuck_acc   = horiz_acc < (baseline_horiz_acc + STUCK_HORIZON_ACCEL_THRESH)
            # 判定 B: 超音波が近すぎる (壁に当たっている)
            is_stuck_sonar = (g_sonar_m is not None and g_sonar_m < STUCK_SONAR_DIST_THRESH)
            # 超音波が取れない場合は加速度のみで判定 (仕様通り)
            is_stuck = is_stuck_acc or is_stuck_sonar

            note = ""
            if is_stuck and not in_recovery:
                stuck_count += 1
                if stuck_count >= STUCK_COUNT_THRESHOLD:
                    note = "STUCK!"
            else:
                if not in_recovery:
                    stuck_count = 0

            # ── モータ制御 ──
            if in_recovery:
                motor_cmd = "RECOVERING"
            elif not g_gps_valid:
                motor.stop()
                motor_cmd = "GPS_WAIT"
            elif stuck_count >= STUCK_COUNT_THRESHOLD:
                motor.stop()
                motor_cmd = "STUCK_STOP"
                # 別スレッドで回復動作 (メインループをブロックしないため)
                t_rec = threading.Thread(target=do_recovery, daemon=True)
                t_rec.start()
            else:
                motor_cmd = motor.apply_diff(diff)

            # ── コンソール 1 行表示 ──
            row = DAT.format(
                elapsed, lat, lng,
                distance, target_bearing, azimuth,
                diff, sonar_str,
                horiz_acc, stuck_count, motor_cmd, note,
            )
            print(row, flush=True)

            # ── ログ蓄積 ──
            log_sensor_row(time.time() - start_time, motor_cmd, note)

            # ── ループ待機 ──
            wait = LOOP_DT - (time.time() - loop_start)
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        log("[Phase4] Ctrl+C 受信 → 緊急停止")

    finally:
        motor.stop()
        print()
        log("─" * 62)
        if goal_reached:
            log("[Phase4] 結果: ゴール到達")
        elif timed_out:
            log("[Phase4] 結果: タイムアウト")
        else:
            log("[Phase4] 結果: 中断")
        log(f"[Phase4] 最終距離: {calc_distance(g_gps_lat, g_gps_lng):.2f} m")


# ===========================================================================
# メイン
# ===========================================================================

def main():
    # Phase 0: 初期化
    bno, bmp, base_pressure, sonar, motor, led, log_path = phase0_init()
    start_time = time.time()

    try:
        # Phase 1: 落下検知
        fall_ok = phase1_fall_detection(bno, start_time)
        if fall_ok:
            log("落下確認。次フェーズへ移行します。")
        else:
            log("タイムアウトのため強制移行します。", "WARN")

        # Phase 2: 初期前進 (30 s)
        phase2_initial_forward(motor, start_time)

        # Phase 3: キャリブレーション & 誘導準備
        baseline_horiz_acc = phase3_calibration(bno, start_time)

        # Phase 4: GPS 誘導走行 + スタック検知
        phase4_guided_run(bno, sonar, motor, start_time, baseline_horiz_acc)

        log("=" * 62)
        log("ミッションシーケンス 完了")
        log("=" * 62)

    except KeyboardInterrupt:
        log("Ctrl+C を受信しました。緊急停止します。")
        motor.stop()

    except Exception as e:
        log(f"予期しないエラー: {e}", "ERROR")
        motor.stop()
        raise

    finally:
        motor.close()
        sonar.close()
        led.off()
        led.close()
        save_log(log_path)
        log("全リソース解放完了")


# ===========================================================================
if __name__ == "__main__":
    main()
