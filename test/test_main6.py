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

    Phase 2 : モータ後退 (30 秒) → 停止 (5 秒) → 前進 (10 秒)
                サブキャリア脱出 + 初期移動として 30 秒後退し、
                5 秒停止して切り替えを落ち着かせた後、確実に
                サブキャリアから離脱・前方へ距離を取るために
                10 秒前進する。

    Phase 3 : キャリブレーション & 誘導走行準備
                BNO055 の地磁気・加速度・ジャイロキャリブレーション待機。
                【改良】地上機は姿勢変更が難しいため Acc の要求レベルを 0 に緩和し、
                Gyro 安定のために完全静止時間 (CALIB_STILL_SEC) を 10 秒に延長。
                「静止 CALIB_STILL_SEC 秒 (Acc/Gyro向け) → 左右交互に
                強旋回 FIG8_SPIN_SEC 秒 (Mag向け)」のサイクルを実行。
                キャリブレーション後、静止基準加速度(スタック検知用)を測定し、
                GPS Fix の安定化・平均化を実施。

    Phase 4 : GPS 誘導走行 (目標地点 2m 以内まで)
                GPS + 地磁気で目標座標を追跡し前進。
                【改良】スタック検知の条件を厳しく設定 (基準加速度からの変動閾値を引き下げ、
                判定時間を3秒間に延長して誤検知を防止)。
                目標地点まで 2m 以内になった時点で Phase 4 を終了し Phase 5 へ移行。

    Phase 5 : 最終接近 (目標物まで 0.05m まで)
                【新規】前方オブジェクトとの距離を超音波センサで測定しながら、
                機体を左右に 0.1 秒ずつ振りつつゴール座標に接近。
                距離が 0.05m (5cm) 以下になった時点で完全停止しミッション終了。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
実行方法 (SSH 切断耐性):
    nohup python3 /home/pi/NSE2026/main/main.py > /home/pi/NSE2026/logs/nohup.log 2>&1 &

モータピン (BCM):
    PWMA=13  AIN1=5  AIN2=6
    PWMB=18  BIN1=23 BIN2=24  STBY=11

センサ:
    BNO055   (I2C, NDOF)
    BMP180   (I2C, oss=3)
    micropyGPS (/dev/serial0, 9600)
    HC-SR04  (TRIG=BCM8, ECHO=BCM7)
    LED      (BCM21)
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
_MAIN_DIR   = Path(__file__).resolve().parent        # NSE2026/main/
_ROOT_DIR   = _MAIN_DIR.parent                       # NSE2026/
_SENSOR_DIR = _ROOT_DIR / "sensor"
if str(_SENSOR_DIR) not in sys.path:
    sys.path.insert(0, str(_SENSOR_DIR))

from BNO055 import BNO055
from BMP180 import BMP180
from micropyGPS import MicropyGPS

# ===========================================================================
#  設定値  ← ここを実地に合わせて変更
# ===========================================================================

# --- 目標座標 ---------------------------------------------------------------
TARGET_LAT =  38.26052      # 目標緯度  [度]
TARGET_LNG = 140.8544151    # 目標経度  [度]
GOAL_RADIUS_PHASE4 = 2.0    # [m] Phase 4 終了・Phase 5 移行距離 (2.0 m)
GOAL_DISTANCE_PHASE5 = 0.05 # [m] Phase 5 終了距離 (0.05 m = 5 cm)

# --- 地磁気偏角 (仙台付近) --------------------------------------------------
MAG_DECLINATION = -8.0      # [度]  西偏 → 負値

# --- Phase 1: 落下検知 -------------------------------------------------------
FALL_THRESHOLD       = 3.0  # [m/s²]  合成加速度ノルムがこれ以下 → 落下中
FALL_COUNT_THRESHOLD = 8    # 連続カウント数
FALL_TIMEOUT_SEC     = 7 * 60  # [s]  7 分でタイムアウト

# --- Phase 2: 初期後退 + 停止 + 前進 ------------------------------------------
PHASE2_BACK_SEC      = 30.0 # [s]  後退秒数
PHASE2_PAUSE_SEC     =  5.0 # [s]  後退後の停止秒数
PHASE2_FWD_AFTER_SEC = 10.0 # [s]  後退・停止後に行う前進秒数

# --- Phase 3: キャリブレーション ----------------------------------------------
CALIB_MIN_MAG  = 2          # 地磁気最小要求レベル
CALIB_MIN_GYRO = 2          # ジャイロ最小要求レベル
CALIB_MIN_ACC  = 0          # ★改良: 地上機ではAccのレベルが上がりにくいため0で許容
CALIB_MIN_SYS  = 1          # システム最小要求レベル
CALIB_TIMEOUT_SEC = 180.0   # [s]  キャリブレーション待機タイムアウト
CALIB_ACC_MEASURE_SEC = 3.0 # スタック検知用：水平加速度の「動いているとき」の基準を測る秒数
CALIB_REPORT_INTERVAL_SEC = 5.0
CALIB_STALL_WARN_SEC = 20.0
CALIB_STILL_SEC = 10.0      # ★改良: Gyro安定化のため完全静止時間を10秒に延長

# --- Phase 3: 自律回転動作 (地磁気キャリブレーション促進) --------------------
FIG8_ENABLE   = True   # True: モータで自律的に回転動作を行う
FIG8_SPIN_SEC = 8.0    # [s]  片方向にスピンし続ける秒数

# --- Phase 3: GPS Fix 安定化・平均化 -----------------------------------------
GPS_MIN_SATS          = 4     # [個]  この衛星捕捉数以上で Fix を信頼する
GPS_CALIB_TIMEOUT_SEC = 60.0  # [s]   GPS Fix 安定化待機の最大秒数
GPS_CALIB_SAMPLES     = 10    # [個]  平均化に使う最大サンプル数
GPS_CALIB_SAMPLE_SEC  = 8.0   # [s]   サンプル収集を行う秒数

# --- LED: フェーズ切り替え通知 -----------------------------------------------
LED_BLINK_SEC      = 10.0   # [s]  フェーズ切り替え時の点滅継続時間
LED_BLINK_INTERVAL = 0.3    # [s]  点滅の ON/OFF 半周期

# --- Phase 4: 誘導走行 -------------------------------------------------------
LOOP_DT      = 0.1          # [s]  制御ループ周期
TIMEOUT_SEC  = 15 * 60      # [s]  走行タイムアウト (15 分)

# 方向制御閾値
ANGLE_DEADBAND    = 10.0    # [度]  この範囲内なら前進
ANGLE_TURN_STRONG = 45.0    # [度]  これ以上の角度差で強旋回

# ★改良: スタック検知（条件を厳しく設定して誤検知を防止）
STUCK_HORIZON_ACCEL_THRESH = 0.15   # [m/s²] 基準加速度からの変動がこれ未満 → スタック疑い
STUCK_SONAR_DIST_THRESH    = 0.15   # [m]    超音波距離がこれ未満 → スタック疑い
STUCK_COUNT_THRESHOLD      = 30     # 連続カウント数 (0.1秒 × 30 = 3.0秒間継続で確定)

STUCK_RECOVER_TURN_DEG          = 180.0  # [度] 初回回復: 反対方向へ回転する角度
STUCK_RECOVER_RETRY_TURN_DEG    = 90.0   # [度] 前進中に再スタックした場合の追加回転角度
STUCK_RECOVER_TURN_TIMEOUT_SEC  = 15.0   # [s] 回転動作の安全タイムアウト
STUCK_RECOVER_FWD_SEC           = 10.0   # [s] 回復: 回転後の前進時間
STUCK_RECOVER_MAX_RETRIES       = 5      # [回] 最大リトライ回数

NAV_REPORT_INTERVAL_SEC = 5.0  # [s] 状況報告ログ周期

# --- モータ -----------------------------------------------------------------
MOTOR_SPEED  = 0.8          # PWM duty (0.0〜1.0)
SPEED_WEAK   = 0.4          # 弱旋回側の duty

# モータピン (BCM)
PIN_PWMA = 13
PIN_AIN1 =  6
PIN_AIN2 =  5
PIN_PWMB = 18
PIN_BIN1 = 24
PIN_BIN2 = 23
PIN_STBY = 11

# LED
LED_PIN  = 21

# GPS
GPS_PORT     = "/dev/serial0"
GPS_BAUDRATE = 9600

# 超音波センサ
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

phase = 0                # 現在フェーズ番号

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
    """タイムスタンプ付きログを標準出力に出す。"""
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
# LED 点滅 (フェーズ切り替え通知)
# ===========================================================================

def blink_led(led: LED, duration: float = LED_BLINK_SEC, interval: float = LED_BLINK_INTERVAL):
    log(f"LED 点滅開始 ({duration:.0f} 秒) — フェーズ切り替え通知")
    t_end = time.time() + duration
    while time.time() < t_end:
        led.on()
        time.sleep(interval)
        led.off()
        time.sleep(interval)
    led.on()
    log("LED 点滅終了 → 常時点灯に復帰")

# ===========================================================================
# GPS スレッド
# ===========================================================================

def gps_thread_func(gps_obj: MicropyGPS):
    global g_gps_lat, g_gps_lng, g_gps_speed, g_gps_sats, g_gps_valid
    if not SERIAL_AVAILABLE:
        log("pyserial 未インストール。GPS 無効。", "WARN")
        return
    try:
        with serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=1.0) as ser:
            log(f"GPS Serial open: {GPS_PORT} @ {GPS_BAUDRATE}")
            ser.readline()
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
# 超音波センサ
# ===========================================================================

class SonarSensor:
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
# モータコントローラ
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

    def backward(self):
        self._stby.on()
        self._pwm_a.value = MOTOR_SPEED
        self._pwm_b.value = MOTOR_SPEED
        self._mot_a.backward()
        self._mot_b.backward()

    def stop(self):
        self._pwm_a.value = 0
        self._pwm_b.value = 0
        self._mot_a.stop()
        self._mot_b.stop()
        self._stby.off()

    def turn_left_strong(self):
        self._stby.on()
        self._pwm_a.value = MOTOR_SPEED
        self._pwm_b.value = 0
        self._mot_a.forward()
        self._mot_b.stop()

    def turn_right_strong(self):
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
        if abs(diff) < ANGLE_DEADBAND:
            self.forward();               return "FORWARD"
        elif diff > ANGLE_TURN_STRONG:
            self.turn_left_strong();      return "TURN_L_STRONG"
        elif diff > 0:
            self.turn_left_weak();        return "TURN_L_WEAK"
        elif diff < -ANGLE_TURN_STRONG:
            self.turn_right_strong();     return "TURN_R_STRONG"
        else:
            self.turn_right_weak();       return "TURN_R_WEAK"

    def close(self):
        self.stop()
        self._pwm_a.close(); self._pwm_b.close()
        self._mot_a.close(); self._mot_b.close()
        self._stby.close()

# ===========================================================================
# 航法計算
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

# ─── Phase 0: 初期化 ────────────────────────────────────────────────────────

def phase0_init() -> tuple:
    global phase
    phase = 0
    log("=" * 62)
    log("  NSE2026 ミッションシーケンス 開始")
    log(f"  PID={os.getpid()}  (nohup で SSH 切断耐性あり)")
    log(f"  目標座標: LAT={TARGET_LAT}  LNG={TARGET_LNG}")
    log("=" * 62)

    Device.pin_factory = LGPIOFactory()

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"mission_{ts}.csv"
    log(f"ログパス: {log_path}")

    led = LED(LED_PIN)
    led.on()

    log("BNO055 初期化中...")
    bno = BNO055()
    if not bno.setUp(operation_mode=BNO055.OPERATION_MODE_NDOF):
        log("BNO055 初期化失敗", "ERROR")
        sys.exit(1)

    log("BMP180 初期化中...")
    bmp = BMP180(oss=3)
    if not bmp.setUp():
        log("BMP180 初期化失敗", "ERROR")
        sys.exit(1)

    log("基準気圧取得中...")
    bp_list = []
    for _ in range(3):
        bmp.getTemperature()
        bp_list.append(bmp.getPressure())
        time.sleep(0.1)
    base_pressure = sum(bp_list) / len(bp_list)

    gps_obj = MicropyGPS(local_offset=9, location_formatting='ddm')
    gps_t = threading.Thread(target=gps_thread_func, args=(gps_obj,), daemon=True)
    gps_t.start()

    sonar = SonarSensor()
    motor = MotorController()

    log("[Phase0] 全デバイス初期化完了")
    return bno, bmp, base_pressure, sonar, motor, led, log_path


# ─── Phase 1: 落下検知 ──────────────────────────────────────────────────────

def phase1_fall_detection(bno: BNO055, led: LED, start_time: float) -> bool:
    global phase, g_acc, g_calib
    phase = 1
    blink_led(led)
    log("─" * 62)
    log(f"[Phase1] 落下検知開始  閾値={FALL_THRESHOLD} m/s²  ×{FALL_COUNT_THRESHOLD}回連続")

    fall_count = 0
    p1_start   = time.time()

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

        if acc_norm <= FALL_THRESHOLD:
            fall_count += 1
        else:
            fall_count = 0

        log_sensor_row(time.time() - start_time, "WAIT_FALL", f"norm={acc_norm:.3f} cnt={fall_count}")

        if fall_count >= FALL_COUNT_THRESHOLD:
            log(f"[Phase1] 落下検知！ (連続 {fall_count} 回, {elapsed:.2f} s)")
            return True

        if elapsed > FALL_TIMEOUT_SEC:
            log(f"[Phase1] タイムアウト ({FALL_TIMEOUT_SEC} s) → 強制移行")
            return False

        time.sleep(0.05)


# ─── Phase 2: 初期後退 + 停止 + 前進 ──────────────────────────────────────────

def phase2_initial_backward(motor: MotorController, led: LED, start_time: float):
    global phase
    phase = 2
    blink_led(led)
    log("─" * 62)
    log(f"[Phase2] 初期後退 {PHASE2_BACK_SEC:.0f} 秒")
    motor.backward()
    deadline = time.time() + PHASE2_BACK_SEC
    while time.time() < deadline:
        log_sensor_row(time.time() - start_time, "BACKWARD_PHASE2")
        time.sleep(1.0)
    motor.stop()

    log(f"[Phase2] 停止 {PHASE2_PAUSE_SEC:.0f} 秒 (切り替え待ち)")
    deadline_pause = time.time() + PHASE2_PAUSE_SEC
    while time.time() < deadline_pause:
        log_sensor_row(time.time() - start_time, "STOP_PHASE2")
        time.sleep(1.0)

    log(f"[Phase2] 前進 {PHASE2_FWD_AFTER_SEC:.0f} 秒")
    motor.forward()
    deadline2 = time.time() + PHASE2_FWD_AFTER_SEC
    while time.time() < deadline2:
        log_sensor_row(time.time() - start_time, "FORWARD_PHASE2")
        time.sleep(1.0)
    motor.stop()
    log("[Phase2] 完了")


# ─── Phase 3 補助: GPS Fix 安定化・平均化 ───────────────────────────────────

def phase3_gps_stabilize(start_time: float):
    log("─" * 62)
    log(f"[Phase3] GPS 安定化待機 (衛星数 >= {GPS_MIN_SATS})")

    t_start = time.time()
    while True:
        elapsed = time.time() - t_start
        if g_gps_valid and g_gps_sats >= GPS_MIN_SATS:
            log(f"[Phase3] GPS Fix 取得 (衛星数={g_gps_sats})")
            break
        if elapsed > GPS_CALIB_TIMEOUT_SEC:
            log("[Phase3] GPS 安定化タイムアウト。現状の Fix のまま続行", "WARN")
            break
        log_sensor_row(time.time() - start_time, "GPS_WAIT_CALIB")
        time.sleep(1.0)

    lat_samples, lng_samples = [], []
    t_sample_start = time.time()
    while (time.time() - t_sample_start < GPS_CALIB_SAMPLE_SEC
           and len(lat_samples) < GPS_CALIB_SAMPLES):
        if g_gps_valid:
            lat_samples.append(g_gps_lat)
            lng_samples.append(g_gps_lng)
        log_sensor_row(time.time() - start_time, "GPS_CALIB_SAMPLE")
        time.sleep(0.5)

    if lat_samples:
        avg_lat = sum(lat_samples) / len(lat_samples)
        avg_lng = sum(lng_samples) / len(lng_samples)
        dist    = calc_distance(avg_lat, avg_lng)
        log(f"[Phase3] GPS 平均位置: LAT={avg_lat:.6f} LNG={avg_lng:.6f} 目標距離={dist:.2f}m")


# ─── Phase 3: キャリブレーション & 誘導準備 ──────────────────────────────────

def phase3_calibration(bno: BNO055, motor: MotorController, led: LED, start_time: float) -> float:
    """
    【改良】地上機向けに Acc の要求レベルを 0 とし、Gyro の検知用に静止時間 (CALIB_STILL_SEC)
    を 10 秒に確保。静止とスピンを交互に繰り返します。
    """
    global phase, g_acc, g_mag, g_gyro, g_calib
    phase = 3
    blink_led(led)
    log("─" * 62)
    log(f"[Phase3] キャリブレーション開始")

    p3_start = time.time()
    state = "STILL"
    state_start = p3_start
    next_spin_dir = "L"
    motor.stop()
    motor_cmd = "STOP(Acc/Gyro)"

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

        now = time.time()
        elapsed = now - p3_start
        s, gy, ac, mg = g_calib

        # 静止 ⇔ スピン サイクル切替
        state_elapsed = now - state_start
        if state == "STILL" and state_elapsed >= CALIB_STILL_SEC:
            if FIG8_ENABLE:
                if next_spin_dir == "L":
                    motor.turn_left_strong()
                    motor_cmd = "SPIN_L(Mag)"
                    next_spin_dir = "R"
                else:
                    motor.turn_right_strong()
                    motor_cmd = "SPIN_R(Mag)"
                    next_spin_dir = "L"
                state, state_start = "SPIN", now
                log(f"[Phase3] {motor_cmd}: スピン開始")
        elif state == "SPIN" and state_elapsed >= FIG8_SPIN_SEC:
            motor.stop()
            motor_cmd = "STOP(Acc/Gyro)"
            state, state_start = "STILL", now
            log(f"[Phase3] STILL: 静止 {CALIB_STILL_SEC:.0f}s (Gyro/Acc 安定化)")

        log_sensor_row(time.time() - start_time, motor_cmd, f"Calib={s}/{gy}/{ac}/{mg}")

        # 改良した判定基準 (Accは0以上で即クリア)
        if s >= CALIB_MIN_SYS and gy >= CALIB_MIN_GYRO and mg >= CALIB_MIN_MAG and ac >= CALIB_MIN_ACC:
            log(f"[Phase3] キャリブレーション条件達成！ (Sys:{s} Gyro:{gy} Acc:{ac} Mag:{mg}) 所要={elapsed:.1f}s")
            break

        if elapsed > CALIB_TIMEOUT_SEC:
            log(f"[Phase3] タイムアウト ({CALIB_TIMEOUT_SEC:.0f}s)。現状で続行します。", "WARN")
            break

        time.sleep(0.1)

    motor.stop()

    # 静止時基準水平加速度の測定
    log(f"[Phase3] 静止基準加速度を {CALIB_ACC_MEASURE_SEC:.0f} 秒計測...")
    samples = []
    t_end = time.time() + CALIB_ACC_MEASURE_SEC
    while time.time() < t_end:
        try:
            acc = bno.getAcc()
            horiz = math.sqrt(acc[0]**2 + acc[1]**2)
            samples.append(horiz)
        except Exception:
            pass
        time.sleep(0.05)

    baseline_horiz_acc = (sum(samples) / len(samples)) if samples else 0.0
    log(f"[Phase3] 静止基準水平加速度 = {baseline_horiz_acc:.4f} m/s²")

    phase3_gps_stabilize(start_time)
    return baseline_horiz_acc


# ─── Phase 4: GPS 誘導走行 (目標まで 2m 以内) ─────────────────────────

def phase4_guided_run(bno: BNO055, sonar: SonarSensor,
                      motor: MotorController, led: LED, start_time: float,
                      baseline_horiz_acc: float):
    """
    【改良】スタック判定を「基準加速度からの変動が少ない状態」が3秒間継続した場合に厳格化。
    目標距離が 2m 以内になったら Phase 4 を終了し Phase 5 へ移行。
    """
    global phase, g_acc, g_mag, g_gyro, g_calib, g_sonar_m

    phase = 4
    blink_led(led)
    log("─" * 62)
    log(f"[Phase4] GPS 誘導走行開始 (目標まで {GOAL_RADIUS_PHASE4}m で Phase 5 へ移行)")

    stuck_count = 0
    cmd = "FORWARD"

    while True:
        try:
            g_acc   = bno.getAcc()
            g_mag   = bno.getMag()
            g_gyro  = bno.getGyro()
            g_calib = bno.getCalibrationStatus()
        except Exception as e:
            log(f"BNO055 読み取りエラー: {e}", "WARN")

        g_sonar_m = sonar.get_distance_m()

        dist    = calc_distance(g_gps_lat, g_gps_lng)
        bearing = calc_target_bearing(g_gps_lat, g_gps_lng)
        azimuth = calc_azimuth(g_mag)
        diff    = calc_direction_diff(azimuth, bearing)

        # ── ゴール判定 (2.0 m 以内) ──
        if dist <= GOAL_RADIUS_PHASE4:
            log(f"[Phase4] 目標地点まで {dist:.2f} m (2m以内) 到達！ Phase 5 へ移行します。")
            motor.stop()
            break

        # ── ★改良: スタック検知の厳格化 ──
        horiz = math.sqrt(g_acc[0]**2 + g_acc[1]**2)
        accel_variation = abs(horiz - baseline_horiz_acc)

        # 水平方向の加速度変動がほぼない (タイヤが空転・スタックして動いていない)
        is_stuck_accel = (accel_variation < STUCK_HORIZON_ACCEL_THRESH)
        is_stuck_sonar = (g_sonar_m is not None and g_sonar_m < STUCK_SONAR_DIST_THRESH)

        if is_stuck_accel or is_stuck_sonar:
            stuck_count += 1
        else:
            stuck_count = 0

        # 0.1s × 30 = 3.0 秒間連続検知で確定
        if stuck_count >= STUCK_COUNT_THRESHOLD:
            log(f"[Phase4] スタック検知！ 回避動作を開始します。(変動:{accel_variation:.3f})")
            motor.stop()
            time.sleep(0.5)

            # 回避動作：180度回転
            if "L" in cmd:
                motor.turn_right_strong()
            else:
                motor.turn_left_strong()
            time.sleep(2.0)

            motor.forward()
            time.sleep(STUCK_RECOVER_FWD_SEC)
            motor.stop()
            stuck_count = 0
            continue

        cmd = motor.apply_diff(diff)
        log_sensor_row(time.time() - start_time, cmd, f"Dist={dist:.2f}m Diff={diff:.1f}°")
        time.sleep(LOOP_DT)


# ─── Phase 5: 最終接近 (新規) ────────────────────────────────────────────────

def phase5_final_approach(bno: BNO055, sonar: SonarSensor,
                          motor: MotorController, led: LED, start_time: float):
    """
    【新規】超音波センサで前方障害物との距離を測りつつ、0.05m (5cm) になるまで
    左右に 0.1 秒ずつ機体を振りながらジグザグ前進する最終アプローチ。
    """
    global phase, g_acc, g_mag, g_gyro, g_sonar_m, g_gps_lat, g_gps_lng

    phase = 5
    blink_led(led)
    log("─" * 62)
    log(f"[Phase5] 最終接近フェーズ開始 (目標物体まで {GOAL_DISTANCE_PHASE5}m まで左右に振って接近)")

    while True:
        g_sonar_m = sonar.get_distance_m()

        try:
            g_mag = bno.getMag()
        except Exception:
            pass

        # ── 終了条件: 前方オブジェクトとの距離が 0.05m (5cm) 以下 ──
        if g_sonar_m is not None and g_sonar_m <= GOAL_DISTANCE_PHASE5:
            log(f"[Phase5] ゴールオブジェクト近接完了！ 距離={g_sonar_m:.3f} m (0.05m以下達成)")
            motor.stop()
            break

        # 方位ズレの計算
        bearing = calc_target_bearing(g_gps_lat, g_gps_lng)
        azimuth = calc_azimuth(g_mag)
        diff    = calc_direction_diff(azimuth, bearing)

        # 左右に 0.1 秒ずつ振りながら接近する動作
        if diff > ANGLE_TURN_STRONG:
            # 左側に大きくずれている場合：左振りを強め
            motor.turn_left_strong()
            time.sleep(0.1)
            motor.turn_right_weak()
            time.sleep(0.1)
            cmd = "SWING_LEFT_STRONG"
        elif diff < -ANGLE_TURN_STRONG:
            # 右側に大きくずれている場合：右振りを強め
            motor.turn_left_weak()
            time.sleep(0.1)
            motor.turn_right_strong()
            time.sleep(0.1)
            cmd = "SWING_RIGHT_STRONG"
        else:
            # 通常：左右に0.1秒ずつ均等に振る
            motor.turn_left_weak()
            time.sleep(0.1)
            motor.turn_right_weak()
            time.sleep(0.1)
            cmd = "SWING_FORWARD"

        log_sensor_row(time.time() - start_time, cmd, f"Sonar={g_sonar_m if g_sonar_m else -1:.3f}m")


# ===========================================================================
# メインシーケンス
# ===========================================================================

def main():
    start_time = time.time()

    # Phase 0
    bno, bmp, base_pressure, sonar, motor, led, log_path = phase0_init()

    try:
        # Phase 1: 落下検知
        phase1_fall_detection(bno, led, start_time)

        # Phase 2: 後退 (30s) -> 停止 (5s) -> 前進 (10s)
        phase2_initial_backward(motor, led, start_time)

        # Phase 3: キャリブレーション & 準備
        baseline_horiz_acc = phase3_calibration(bno, motor, led, start_time)

        # Phase 4: GPS 誘導走行 (2m 以内まで)
        phase4_guided_run(bno, sonar, motor, led, start_time, baseline_horiz_acc)

        # Phase 5: 最終接近 (0.05m まで機体を振りながら近づく)
        phase5_final_approach(bno, sonar, motor, led, start_time)

        log("=" * 62)
        log(" 全フェーズが正常に完了しました！ ミッション成功。")
        log("=" * 62)

    except KeyboardInterrupt:
        log("ユーザー割り込み (Ctrl+C) により終了します。", "WARN")
    except Exception as e:
        log(f"メインループ異常終了: {e}", "ERROR")
    finally:
        motor.stop()
        motor.close()
        sonar.close()
        led.off()
        save_log(log_path)


if __name__ == "__main__":
    main()
