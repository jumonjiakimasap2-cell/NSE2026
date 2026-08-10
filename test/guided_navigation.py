#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GPS + BNO055 (NDOFモード) 誘導走行（Phase 4）プログラム

【修正要点】
1. モータ反転フラグ (MOTOR_REVERSED) を False に統一し、直進・旋回の反転を防止。
2. BNO055 の NDOF オイラー角 (VECTOR_EULER) から方位 (Heading) を直接取得。
   - 手動の磁気データ計算 (getMag + atan2 + az *= -1) による左右逆転を解消。
   - 磁気偏角 (MAG_DECLINATION) の補正を正しく加算。
3. 前方探索の誤判定ロジックを廃止し、センサ配置に応じた固定オフセット (FRONT_OFFSET_DEG) を採用。
4. 目的地方位角 (Target Bearing) と現在方位角 (Current Heading) の差分 (-180°〜+180°) に応じた
   適切なステアリング制御 (直進 / 右旋回 / 左旋回) を実装。
5. 目標地点到達（閾値半径内）で安全停止。
"""

import math
import time
import sys

# ----------------------------------------------------
# 1. 設定パラメータ
# ----------------------------------------------------
# 目標GPS座標 (緯度, 経度)
TARGET_LAT = 38.258600  # 例: 仙台市青葉区周辺など、目的地の緯度
TARGET_LON = 140.850200 # 例: 目的地の経度

# 到達判定半径 [m]
GOAL_RADIUS_M = 2.0

# 磁気偏角 (仙台の場合 約 -8.5° -> 西偏8.5度 = -8.5 または 地域設定)
MAG_DECLINATION = -8.5

# センサ取り付け角度オフセット [度]
# 機体正対方向に対してBNO055のY軸(またはX軸)が向いているオフセット角度
FRONT_OFFSET_DEG = 0.0

# モータ制御パラメータ
MOTOR_REVERSED = False  # モータ逆転フラグは False (配線通りの回転)
DEFAULT_SPEED = 60      # 基本直進スピード (0-100)
TURN_SPEED = 50         # 旋回スピード (0-100)

# 許容方位誤差角 [度]
# 目標方向との誤差がこの角度以内なら直進、それ以外は旋回修正
HEADING_TOLERANCE_DEG = 15.0

# ----------------------------------------------------
# 2. ハードウェア / ドライバ ダミークラス (実機ライブラリへ差し替え用)
# ----------------------------------------------------
class MotorController:
    """モータドライバ制御クラス"""
    def __init__(self, reversed_flag: bool = False):
        self.reversed = reversed_flag

    def set_reversed(self, reversed_flag: bool):
        self.reversed = reversed_flag

    def forward(self, speed: int = DEFAULT_SPEED):
        # 画面出力等（実機では GPIO / PWM 制御）
        print(f"[MOTOR] 前進 (Speed: {speed})")

    def turn_left(self, speed: int = TURN_SPEED):
        print(f"[MOTOR] 左旋回 (Speed: {speed})")

    def turn_right(self, speed: int = TURN_SPEED):
        print(f"[MOTOR] 右旋回 (Speed: {speed})")

    def stop(self):
        print("[MOTOR] 停止")


class BNO055Sensor:
    """BNO055 9軸センサ制御クラス"""
    def __init__(self):
        # NDOF モードで初期化
        print("[BNO055] Initializing in NDOF mode...")

    def get_euler_heading(self) -> float:
        """
        BNO055のオイラー角からHeading(方位角 0°~360°)を取得する。
        NDOFモードでは内部フュージョンされた高精度なオイラー角が得られる。
        0° = 北, 90° = 東, 180° = 南, 270° = 西
        """
        # ※実機では Adafruit_BNO055 や smbus 等で euler ベクトルを取得
        # heading, roll, pitch = bno.get_vector(BNO055.VECTOR_EULER)
        # ここでは例として仮の値を返す構造
        raw_heading = 45.0  # サンプル値
        return raw_heading


class GPSModule:
    """GPSモジュール受信クラス"""
    def __init__(self):
        print("[GPS] Initializing Serial connection...")

    def get_location(self):
        """
        現在の (緯度, 経度) を返す。
        測位不能時は None を返す。
        """
        # ※実機では NMEA フォーマット ($GPRMC / $GPGGA) の解析処理
        sample_lat = 38.258500
        sample_lon = 140.850100
        return sample_lat, sample_lon

# ----------------------------------------------------
# 3. 幾何・航法計算関数
# ----------------------------------------------------
def calc_distance_and_bearing(lat1: float, lon1: float, lat2: float, lon2: float):
    """
    2点間の距離(m) と 目的地方位角(deg: 0=北, 90=東, 180=南, 270=西) を計算
    (球面三角法 / ヒュベニの公式簡略版)
    """
    R = 6371000.0  # 地球の半径 [m]

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    # 1. 距離計算 (Haversine formula)
    a = math.sin(delta_phi / 2.0)**2 +         math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0)**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    distance = R * c

    # 2. 目的地方位角計算 (Bearing)
    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) -         math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    
    target_bearing = math.degrees(math.atan2(y, x))
    target_bearing = (target_bearing + 360.0) % 360.0  # 0~360度に正規化

    return distance, target_bearing


def get_current_heading(bno: BNO055Sensor) -> float:
    """
    真方位（True Heading）を取得する。
    BNO055のNDOFオイラー角 + 磁気偏角 + センサオフセット
    """
    raw_heading = bno.get_euler_heading()
    
    # 真方位 = 磁気方位 + 偏角 + オイラー角補正
    true_heading = raw_heading + MAG_DECLINATION + FRONT_OFFSET_DEG
    return true_heading % 360.0


def calc_heading_error(target_bearing: float, current_heading: float) -> float:
    """
    目標方位と現在方位の偏差(-180° 〜 +180°)を計算する。
    正の値: 右に旋回すべき
    負の値: 左に旋回すべき
    """
    diff = (target_bearing - current_heading + 540.0) % 360.0 - 180.0
    return diff

# ----------------------------------------------------
# 4. メイン誘導走行ルーチン
# ----------------------------------------------------
def main_guided_navigation():
    print("=========================================")
    print("   Phase 4: GPS + Compass 誘導走行開始   ")
    print("=========================================")

    # ハードウェア初期化
    motor = MotorController(reversed_flag=MOTOR_REVERSED)
    bno = BNO055Sensor()
    gps = GPSModule()

    time.sleep(1.0)

    try:
        while True:
            # 1. 現在のGPS座標を取得
            loc = gps.get_location()
            if loc is None:
                print("[WARN] GPS信号未受信。待機中...")
                motor.stop()
                time.sleep(1.0)
                continue

            current_lat, current_lon = loc

            # 2. 目的地までの距離と方位角を計算
            dist, target_bearing = calc_distance_and_bearing(
                current_lat, current_lon, TARGET_LAT, TARGET_LON
            )

            # 3. 現在の機体方位を取得 (BNO055 NDOFオイラー角ベース)
            current_heading = get_current_heading(bno)

            # 4. 方位偏差を計算
            heading_error = calc_heading_error(target_bearing, current_heading)

            # ログ表示
            print(f"[NAV] 現在地: ({current_lat:.6f}, {current_lon:.6f}) | "
                  f"目標距離: {dist:.2f}m | "
                  f"目標方位: {target_bearing:.1f}° | "
                  f"現在方位: {current_heading:.1f}° | "
                  f"誤差: {heading_error:+.1f}°")

            # 5. 到達判定
            if dist <= GOAL_RADIUS_M:
                print(f"★ [GOAL] 目的地に到達しました！ (距離: {dist:.2f}m <= {GOAL_RADIUS_M}m)")
                motor.stop()
                break

            # 6. 走行制御 (ステアリング判定)
            if abs(heading_error) <= HEADING_TOLERANCE_DEG:
                # 誤差が小範囲内であれば直進
                motor.forward(DEFAULT_SPEED)
            elif heading_error > 0:
                # 目標が右方向にあるため右旋回
                motor.turn_right(TURN_SPEED)
            else:
                # 目標が左方向にあるため左旋回
                motor.turn_left(TURN_SPEED)

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("
[INFO] ユーザー中断が検出されました。停止します。")
    finally:
        motor.stop()
        print("[INFO] 誘導走行プログラム終了")


if __name__ == "__main__":
    main_guided_navigation()
