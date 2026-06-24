import sys
import os
import time
import csv
import math
import datetime
import threading
from pathlib import Path
 
# --- シリアル (GPS 用) ---
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
 
# --- センサーモジュールのパスを追加 ----------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent          # .../NSE2026/test/
SENSOR_DIR = SCRIPT_DIR.parent / "sensor"             # .../NSE2026/sensor/
if str(SENSOR_DIR) not in sys.path:
    sys.path.insert(0, str(SENSOR_DIR))
 
from BNO055 import BNO055
from BMP180 import BMP180
from micropyGPS import MicropyGPS
 
# ===========================================================================
# 設定
# ===========================================================================
 
LOOP_INTERVAL = 0.02          # メインループ周期 [s]  (約 50 Hz)
GPS_PORT      = "/dev/ttyAMA0"
GPS_BAUDRATE  = 9600
LOG_DIR       = Path(__file__).resolve().parent.parent / "logs"
 
GRAVITY = 9.81                # [m/s²]
 
# ===========================================================================
# 共有グローバル変数
# ===========================================================================
 
acc         = [0.0, 0.0, 0.0]  # [m/s²]
gyro        = [0.0, 0.0, 0.0]  # [dps]
temperature = 0.0               # [°C]
pressure    = 0.0               # [Pa]
altitude    = 0.0               # [m]
 
gps_lat   = 0.0
gps_lng   = 0.0
gps_speed = 0.0                 # [knots]
gps_sats  = 0
 
velocity  = 0.0                 # [m/s] 積分速度
v_lock    = threading.Lock()
 
# ===========================================================================
# GPS スレッド
# ===========================================================================
 
def gps_thread_func(gps_obj: MicropyGPS, port: str, baudrate: int):
    """GPS NMEA センテンスをバックグラウンドで読み続ける。"""
    global gps_lat, gps_lng, gps_speed, gps_sats
 
    if not SERIAL_AVAILABLE:
        print("[GPS] pyserial が見つかりません。GPS は無効です。")
        return
 
    try:
        with serial.Serial(port, baudrate, timeout=1.0) as ser:
            print(f"[GPS] Serial open: {port} @ {baudrate} bps")
            while True:
                try:
                    line = ser.readline().decode("ascii", errors="replace")
                    for char in line:
                        gps_obj.update(char)
                    if gps_obj.valid:
                        lat_raw = gps_obj.latitude
                        lng_raw = gps_obj.longitude
                        # ddm フォーマット: [deg, decimal_minutes, hemisphere]
                        gps_lat = lat_raw[0] + lat_raw[1] / 60.0
                        if lat_raw[2] == 'S':
                            gps_lat = -gps_lat
                        gps_lng = lng_raw[0] + lng_raw[1] / 60.0
                        if lng_raw[2] == 'W':
                            gps_lng = -gps_lng
                        gps_speed = gps_obj.speed[0]      # ノット
                        gps_sats  = gps_obj.satellites_in_use
                except Exception as e:
                    print(f"[GPS] 読み取りエラー: {e}")
    except serial.SerialException as e:
        print(f"[GPS] ポートを開けません ({port}): {e}")
        print("[GPS] GPS データは 0.0 で表示されます。")
 
# ===========================================================================
# 表示フォーマット
# ===========================================================================
 
HEADER_FMT = (
    "{:>8}  "
    "{:>7} {:>7} {:>7}  "
    "{:>10}  "
    "{:>7} {:>7} {:>7}  "
    "{:>7}  {:>11}  {:>8}  "
    "{:>11}  {:>12}  {:>11}  {:>4}"
)
 
HEADER = HEADER_FMT.format(
    "Time[s]",
    "AccX", "AccY", "AccZ",
    "Speed[m/s]",
    "GyroX", "GyroY", "GyroZ",
    "Temp[C]", "Pres[Pa]", "Alt[m]",
    "Lat", "Lng", "GPS_Spd[kt]", "Sats"
)
 
DATA_FMT = (
    "{:>8.3f}  "
    "{:>7.3f} {:>7.3f} {:>7.3f}  "
    "{:>10.4f}  "
    "{:>7.2f} {:>7.2f} {:>7.2f}  "
    "{:>7.2f}  {:>11.2f}  {:>8.2f}  "
    "{:>11.6f}  {:>12.6f}  {:>11.3f}  {:>4d}"
)
 
# ===========================================================================
# メイン
# ===========================================================================
 
def main():
    global acc, gyro, temperature, pressure, altitude, velocity
 
    # --- ログ保存先 ---
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts_str   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"finishv_{ts_str}.csv"
 
    # --- 起動メッセージ ---
    print("=" * 90)
    print("  落下終端速度 測定プログラム  (test_finishv.py)")
    print("=" * 90)
 
    # ---- BNO055 初期化 ----
    bno = BNO055()
    if not bno.setUp(operation_mode=BNO055.OPERATION_MODE_NDOF):
        print("[ERROR] BNO055 の初期化に失敗しました。終了します。")
        sys.exit(1)
 
    # ---- BMP180 初期化 ----
    bmp = BMP180(oss=3)
    if not bmp.setUp():
        print("[ERROR] BMP180 の初期化に失敗しました。終了します。")
        sys.exit(1)
 
    # ---- GPS 初期化 ----
    gps_obj    = MicropyGPS(local_offset=9, location_formatting='ddm')  # JST +9h
    gps_thread = threading.Thread(
        target=gps_thread_func,
        args=(gps_obj, GPS_PORT, GPS_BAUDRATE),
        daemon=True
    )
    gps_thread.start()
    time.sleep(0.3)
 
    # ---- 基準気圧取得 (3 回平均) ----
    print("\n[INFO] 基準気圧を取得中...")
    bp_samples = []
    for _ in range(3):
        bmp.getTemperature()
        bp_samples.append(bmp.getPressure())
        time.sleep(0.1)
    base_pressure = sum(bp_samples) / len(bp_samples)
    print(f"[INFO] 基準気圧 = {base_pressure:.2f} Pa ({base_pressure/100:.2f} hPa)")
 
    # ---- 速度積分 初期化 ----
    with v_lock:
        velocity = 0.0
    prev_time  = time.time()
    start_time = prev_time
 
    # ---- コンソールヘッダー ----
    print()
    separator = "-" * len(HEADER)
    print(separator)
    print(HEADER)
    print(separator)
 
    # ---- CSV ヘッダー ----
    csv_header = [
        "Time_s",
        "AccX_ms2", "AccY_ms2", "AccZ_ms2",
        "Speed_ms",
        "GyroX_dps", "GyroY_dps", "GyroZ_dps",
        "Temp_C", "Pres_Pa", "Alt_m",
        "Lat", "Lng", "GPS_Speed_kts", "GPS_Sats",
    ]
    log_rows   = []
    line_count = 0
 
    try:
        while True:
            loop_start = time.time()
 
            # --- BNO055 ---
            try:
                acc  = bno.getAcc()
                gyro = bno.getGyro()
            except Exception as e:
                print(f"[WARN] BNO055: {e}")
                acc  = [0.0, 0.0, 0.0]
                gyro = [0.0, 0.0, 0.0]
 
            # --- 速度積分 ---
            # 合成加速度ノルムから 1G を差し引き、落下加速度を近似する。
            # 自由落下中: acc_norm ≈ 0  → fall_acc ≈ -9.81  (加速フェーズ)
            # 終端速度時: acc_norm ≈ 9.81 → fall_acc ≈ 0    (等速フェーズ)
            now = time.time()
            dt  = now - prev_time
            prev_time = now
 
            acc_norm = math.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)
            fall_acc = acc_norm - GRAVITY
 
            with v_lock:
                velocity += fall_acc * dt
                cur_vel   = velocity
 
            # --- BMP180 ---
            try:
                temperature = bmp.getTemperature()
                pressure    = bmp.getPressure()
                altitude    = bmp.getAltitude(sea_level_pressure=base_pressure)
            except Exception as e:
                print(f"[WARN] BMP180: {e}")
 
            # --- GPS ---
            lat  = gps_lat
            lng  = gps_lng
            spd  = gps_speed
            sats = gps_sats
 
            elapsed = now - start_time
 
            # --- コンソール表示 ---
            row_str = DATA_FMT.format(
                elapsed,
                acc[0],  acc[1],  acc[2],
                cur_vel,
                gyro[0], gyro[1], gyro[2],
                temperature, pressure, altitude,
                lat, lng, spd, int(sats),
            )
            print(row_str)
 
            # --- ログ蓄積 ---
            log_rows.append([
                round(elapsed,     4),
                round(acc[0],      4), round(acc[1],  4), round(acc[2],  4),
                round(cur_vel,     4),
                round(gyro[0],     4), round(gyro[1], 4), round(gyro[2], 4),
                round(temperature, 3), round(pressure, 2), round(altitude, 3),
                round(lat,  6), round(lng,  6),
                round(spd,  3), int(sats),
            ])
            line_count += 1
 
            # --- 待機 ---
            elapsed_loop = time.time() - loop_start
            wait         = LOOP_INTERVAL - elapsed_loop
            if wait > 0:
                time.sleep(wait)
 
    except KeyboardInterrupt:
        print("\n\n[INFO] Ctrl+C を受信しました。終了処理中...")
 
    finally:
        # ---- 終端速度推定 ----
        if log_rows:
            speeds = [r[4] for r in log_rows]           # Speed_ms 列
            n      = len(speeds)
            tail   = speeds[max(0, n * 3 // 4):]        # 後半 1/4
            if tail:
                terminal_v = sum(tail) / len(tail)
                max_speed  = max(abs(s) for s in speeds)
                print(f"\n{'=' * 60}")
                print(f"  [RESULT] 推定終端速度 (後半 1/4 平均) : {terminal_v:>8.3f} m/s")
                print(f"  [RESULT] 最大推定速度 (絶対値)         : {max_speed:>8.3f} m/s")
                print(f"  [RESULT] 計測時間                       : {log_rows[-1][0]:>8.3f} s")
                print(f"  [RESULT] サンプル数                     : {line_count:>8d} 点")
                print(f"{'=' * 60}")
 
        # ---- CSV 書き出し ----
        print(f"\n[INFO] ログを保存中: {log_path}")
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
 
