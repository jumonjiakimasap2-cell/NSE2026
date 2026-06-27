"""
fall.py
=======
落下検知 → サブキャリア脱出 → 前進 テストプログラム
 
NSE2026/test/fall.py
 
フロー:
    Phase 0 : センサ初期化 & 落下検知待機
               BNO055 の合成加速度ノルムが閾値を下回る状態が
               FALL_COUNT_THRESHOLD 回連続したら「落下中」と判定する。
               タイムアウト (FALL_TIMEOUT_SEC) を超えても検知できなかった
               場合は強制的に Phase1 へ移行する。
 
    Phase 1 : サブキャリア脱出
               パラシュート/サブキャリアからの脱出を想定し、
               ESCAPE_SEC 秒間モータを前進させる。
 
    Phase 2 : 前進走行
               FWD_SEC 秒間だけ前進して停止する。
 
センサ/モータ:
    - IMU  : BNO055  (I2C, smbus)          Locally importable
    - 気圧 : BMP180  (I2C, smbus)          Locally importable
    - モータ: gpiozero + lgpio              ← test_run.py と同じ
              PWMA=BCM13, AIN1=BCM5, AIN2=BCM6
              PWMB=BCM24, BIN1=BCM18, BIN2=BCM23
 
定数:
    FALL_THRESHOLD      合成加速度ノルム [m/s²] がこの値 "以下" のとき落下中と判定
                        (自由落下: ≈ 0 m/s²  /  通常静止: ≈ 9.81 m/s²)
    FALL_COUNT_THRESHOLD 連続して落下判定する必要な回数
    FALL_TIMEOUT_SEC    落下検知のタイムアウト時間 [s]
    ESCAPE_SEC          サブキャリア脱出モータ前進時間 [s]
    FWD_SEC              脱出後の前進時間 [s]
    MOTOR_SPEED          モータ出力 0.0〜1.0
    LOOP_DT              Phase0 ループ周期 [s]
"""

import sys
import time
import math
import datetime
from pathlib import Path
 
# --- gpiozero (test_run.py と同じバックエンド) ---
from gpiozero import Motor, PWMOutputDevice, OutputDevice  # OutputDevice を追加
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
 
# --- センサモジュールパス (test_finishv.py と同じ解決方法) ---
SCRIPT_DIR = Path(__file__).resolve().parent      # NSE2026/test/
SENSOR_DIR = SCRIPT_DIR.parent / "sensor"         # NSE2026/sensor/
if str(SENSOR_DIR) not in sys.path:
    sys.path.insert(0, str(SENSOR_DIR))
 
from BNO055 import BNO055
from BMP180 import BMP180
 
# ===========================================================================
# 定数
# ===========================================================================
 
# --- 落下検知 ---
FALL_THRESHOLD       = 5.0    # [m/s²]  合成加速度ノルムがこれ以下 → 落下とみなす
                               #         自由落下 ≈ 0、空気抵抗あり ≈ 2〜4 程度
FALL_COUNT_THRESHOLD = 8      # 連続カウント数
FALL_TIMEOUT_SEC     = 7 * 60 # [s]  7分でタイムアウト
 
# --- モータ動作時間 ---
ESCAPE_SEC = 10.0   # [s]  Phase1: サブキャリア脱出前進
FWD_SEC    =  5.0   # [s]  Phase2: 本走行前進
 
# --- モータ出力 ---
MOTOR_SPEED = 0.8   # 0.0〜1.0 (test_run.py の speed と同値)
 
# --- ループ周期 ---
LOOP_DT = 0.05      # [s]
 
# --- ピン (BCM)  ← test_run.py と同じ番号 ---
PIN_PWMA = 13
PIN_AIN1 =  5
PIN_AIN2 =  6
PIN_PWMB = 18
PIN_BIN1 = 23
PIN_BIN2 = 24
PIN_STBY = 11  # STBYピンを追加
 
# ===========================================================================
# モータ制御ヘルパー
# ===========================================================================
 
class MotorController:
    """
    test_run.py の forward()/stop() 相当を一クラスにまとめたもの。
    gpiozero の Motor + PWMOutputDevice を使う。
    """
 
    def __init__(self, speed: float = MOTOR_SPEED):
        Device.pin_factory = LGPIOFactory()
        self.speed = speed
        self._pwm_a = PWMOutputDevice(PIN_PWMA)
        self._pwm_b = PWMOutputDevice(PIN_PWMB)
        self._motor_a = Motor(forward=PIN_AIN1, backward=PIN_AIN2)
        self._motor_b = Motor(forward=PIN_BIN1, backward=PIN_BIN2)
        self._stby = OutputDevice(PIN_STBY)  # STBYピンの初期化を追加
        self.stop()
 
    # ---- 基本コマンド (test_run.py の関数と 1対1 対応) ----
 
    def forward(self):
        """両モータ前進 (test_run.py: forward())"""
        self._stby.on()  # モーターを動かす直前にSTBYをHIGHにする
        self._pwm_a.value = self.speed
        self._pwm_b.value = self.speed
        self._motor_a.forward()
        self._motor_b.forward()
 
    def stop(self):
        """全停止 (test_run.py: stop())"""
        self._pwm_a.value = 0
        self._pwm_b.value = 0
        self._motor_a.stop()
        self._motor_b.stop()
        self._stby.off()  # モーターの動きを停止した後にSTBYをLOWにする
 
    def close(self):
        """リソース解放"""
        self.stop()
        self._pwm_a.close()
        self._pwm_b.close()
        self._motor_a.close()
        self._motor_b.close()
        self._stby.close()  # STBYピンのリソース解放を追加
 
    def __enter__(self):
        return self
 
    def __exit__(self, *_):
        self.close()
 
# ===========================================================================
# センサ初期化
# ===========================================================================
 
def init_sensors() -> tuple[BNO055, BMP180]:
    """
    BNO055 と BMP180 を初期化して返す。
    test_finishv.py の main_v2() と同じ手順・同じ引数を使う。
    """
    print("[INIT] BNO055 初期化中...")
    bno = BNO055()
    if not bno.setUp(operation_mode=BNO055.OPERATION_MODE_NDOF):
        print("[ERROR] BNO055 の初期化に失敗しました。終了します。")
        sys.exit(1)
 
    print("[INIT] BMP180 初期化中...")
    bmp = BMP180(oss=3)
    if not bmp.setUp():
        print("[ERROR] BMP180 の初期化に失敗しました。終了します。")
        sys.exit(1)
 
    print("[INIT] センサ初期化 完了")
    return bno, bmp
 
# ===========================================================================
# Phase 0 : 落下検知
# ===========================================================================
 
def phase0_fall_detection(bno: BNO055) -> bool:
    """
    BNO055 の合成加速度ノルムを監視し、落下を検知する。
 
    NICS2026/NOA.py の phase0() を参考に、
    閾値・カウント・タイムアウトのロジックを NSE2026 のセンサ API に合わせた。
 
    Parameters
    ----------
    bno : BNO055
        初期化済み BNO055 インスタンス。
 
    Returns
    -------
    bool
        True  : 落下検知成功
        False : タイムアウト
    """
    print("\n[Phase0] 落下検知 開始")
    print(f"         閾値: acc_norm <= {FALL_THRESHOLD:.1f} m/s²  ×  {FALL_COUNT_THRESHOLD} 回連続")
    print(f"         タイムアウト: {FALL_TIMEOUT_SEC} 秒")
    print("-" * 55)
    print(f"  {'経過[s]':>8}  {'acc_norm[m/s²]':>14}  {'fall_cnt':>8}")
    print("-" * 55)
 
    start_time  = time.time()
    fall_count  = 0
 
    while True:
        # --- BNO055 から加速度取得 (test_finishv.py と同じ呼び出し) ---
        try:
            acc = bno.getAcc()   # [m/s²] X, Y, Z
        except Exception as e:
            print(f"[WARN] BNO055 読み取りエラー: {e}")
            time.sleep(LOOP_DT)
            continue
 
        acc_norm = math.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)
        elapsed  = time.time() - start_time
 
        # --- 落下判定 (NICS2026/NOA.py phase0() の構造を踏襲) ---
        if acc_norm <= FALL_THRESHOLD:
            fall_count += 1
        else:
            fall_count = 0   # 一度閾値を超えたらリセット (誤検知防止)
 
        print(f"  {elapsed:>8.2f}  {acc_norm:>14.3f}  {fall_count:>8d}")
 
        if fall_count >= FALL_COUNT_THRESHOLD:
            print(f"\n[Phase0] 落下検知！ (連続 {fall_count} 回)  経過 {elapsed:.2f} s")
            return True
 
        # --- タイムアウト ---
        if elapsed > FALL_TIMEOUT_SEC:
            print(f"\n[Phase0] タイムアウト ({FALL_TIMEOUT_SEC} s) → Phase1 へ強制移行")
            return False
 
        time.sleep(LOOP_DT)
 
# ===========================================================================
# Phase 1 : サブキャリア脱出 (前進 ESCAPE_SEC 秒)
# ===========================================================================
 
def phase1_escape(motor: MotorController):
    """
    サブキャリアからの脱出を想定したモータ前進。
 
    NICS2026/NOA.py phase2()（escape）を参考に、
    NSE2026 の gpiozero ベースのモータ制御 (test_run.py) に合わせた。
    """
    print(f"\n[Phase1] サブキャリア脱出 — 前進 {ESCAPE_SEC:.1f} 秒")
    motor.forward()
    time.sleep(ESCAPE_SEC)
    motor.stop()
    print("[Phase1] 停止 完了")
 
# ===========================================================================
# Phase 2 : 前進走行 (FWD_SEC 秒)
# ===========================================================================
 
def phase2_forward(motor: MotorController):
    """
    脱出後の本走行。FWD_SEC 秒だけ前進して停止する。
    """
    print(f"\n[Phase2] 前進走行 — {FWD_SEC:.1f} 秒")
    motor.forward()
    time.sleep(FWD_SEC)
    motor.stop()
    print("[Phase2] 停止 完了")
 
# ===========================================================================
# メイン
# ===========================================================================
 
def main():
    print("=" * 60)
    print("  fall.py  落下検知 → 脱出 → 前進 テスト")
    print(f"  {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)
 
    # --- センサ初期化 ---
    bno, bmp = init_sensors()
 
    # --- モータ初期化 ---
    print("[INIT] モータ初期化中...")
    motor = MotorController(speed=MOTOR_SPEED)
    print("[INIT] モータ初期化 完了")
 
    try:
        # ---- Phase 0 ----
        fall_detected = phase0_fall_detection(bno)
 
        if fall_detected:
            print("\n[INFO] 落下を確認しました。")
        else:
            print("\n[INFO] タイムアウトのため強制的に次フェーズへ移行します。")
 
        # ---- Phase 1 ----
        phase1_escape(motor)
 
        # ---- Phase 2 ----
        phase2_forward(motor)
 
        print("\n[INFO] 全フェーズ終了。")
 
    except KeyboardInterrupt:
        print("\n\n[INFO] Ctrl+C を受信。緊急停止します。")
        motor.stop()
 
    finally:
        motor.close()
        print("[INFO] モータリソースを解放しました。")
 
 
# ===========================================================================
if __name__ == "__main__":
    main()
