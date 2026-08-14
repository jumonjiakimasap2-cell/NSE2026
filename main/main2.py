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

    Phase 2 : 超音波分離検知後退 → 停止 (5 秒) → 前進 (10 秒)
                サブキャリア脱出のため後退する。★変更: 固定30秒ではなく、
                前方の超音波センサでサブキャリアとの距離を継続監視し、
                分離した瞬間に生じる距離の急増 (PHASE2_SONAR_JUMP_THRESH_M
                以上) を検知した時点で後退を打ち切る (検知できない場合は
                PHASE2_BACK_TIMEOUT_SEC で安全タイムアウト)。
                後退完了後、5 秒停止して切り替えを落ち着かせた後、確実に
                サブキャリアから離脱・前方へ距離を取るために
                10 秒前進する。

    Phase 3 : キャリブレーション & 誘導走行準備
                BNO055 の地磁気・加速度・ジャイロキャリブレーション待機。
                「静止(セトリング+計測) CALIB_STILL_SEC 秒 (Acc/Gyro向け)
                ⇔ 左右交互に強旋回 FIG8_SPIN_SEC 秒 (Mag向け)」のサイクルを
                最初から回し続けることで、特定軸のキャリブレーションが
                原因で全体が停滞しないようにしている。
                【改良】さらに Acc/Gyro が実地でレベルアップしにくい問題に
                対応するため、軸ごとに現実的な目標レベルを個別設定できる
                ようにした (CALIB_MIN_LEVEL_SYS/GYRO/ACC/MAG。地上機は複数
                姿勢を取れないため Acc は緩め、Mag は比較的到達しやすいので
                従来通り高めに設定)。また、スピン直後は慣性の余韻で機体が
                完全には静止していないため、静止区間の頭に CALIB_SETTLE_SEC
                秒のセトリング(捨て時間)を設け、揺れている間を静止時間として
                誤ってカウントしないようにした。
                CALIB_REPORT_INTERVAL_SEC ごとに詳細な進捗をログへ出力し、
                特定軸が停滞している場合は CALIB_STALL_WARN_SEC ごとに警告する。
                【改良】実機ログ解析の結果、地上機は姿勢を変えられないため
                Acc(加速度)キャリブレーションレベルが構造的に 0 から絶対に
                上がらず、CALIB_TIMEOUT_SEC まで毎回タイムアウトしていたことが
                判明した。そのため CALIB_MIN_LEVEL_ACC を 0 (要求しない) に
                変更し、Sys/Gyro/Mag のみで完了判定するようにした。
                キャリブレーション後、静止基準加速度(スタック検知用)を測定し、
                続けて GPS Fix の安定化・平均化 (衛星数チェック + 複数サンプル
                平均) を行い、誘導走行開始時点での距離・方位を確認する。

    Phase 4 : GPS 誘導走行 (目標地点 PHASE4_TO_5_RADIUS = 2 m 以内まで)
                GPS + 地磁気で目標座標を追跡し前進。
                【改良】実機ログ解析の結果、(1) モータへの高デューティ通電中に
                地磁気センサの値が100ms間隔で数十〜100度以上ジャンプする
                ほどのノイズ(モータ電流による磁気干渉と推定)が発生し、
                機体方位が暴れて目標から遠ざかる事例が確認された。これに
                対応するため、地磁気ベクトルへ EMA(指数移動平均)フィルタ
                (calc_azimuth_with_front 内, MAG_EMA_ALPHA) を適用して
                ノイズを平滑化するようにした。(2) 距離・方位角の計算式を、
                従来の平面近似からより厳密な Haversine 距離公式・大圆初期
                方位角公式に変更し、精度を高めた。
                【安全ロック】Phase4 の開始から終了まで、モータの
                後退方向への回転を絶対に禁止する安全ロックを有効化する。
                backward() が万一 (バグ・将来の改修等で) 呼ばれても、
                MotorController 側で物理的に後退させず停止のみ行う
                (Phase4終了時にロックは解除され、以降のフェーズには影響しない)。
                【改良/修正】方位計算式は test_GPSrun.py と同じ単純な
                固定式 (calc_azimuth) に統一した。以前は「モータ反転
                フラグ (PHASE4_MOTOR_REVERSED)」「方位鏡写しフラグ
                (AZIMUTH_MIRROR_FIX)」など複数の反転系フラグを個別に
                導入していたが、フラグの組み合わせが増えるほど機体の
                動きが把握しづらくなる混乱を招いたため、これらを全て
                撤去し、モータは常に素直に forward=前進/backward=後退、
                方位計算も固定の単純な式で計算するフラットな設計に戻した。
                取り付け向きの補正は BNO055 の Axis Remap 機能
                (BNO_ORIENTATION, Phase0で1回設定) だけに一本化し、
                センサ内部の基準軸を機体の物理的な取り付け向きに合わせて
                再配置する、確実で単純な方式のみを採用する。
                スタック検知 (超音波 + 水平加速度 + GPS速度) で障害物を回避する。
                【改良】実運用でスタックしていないのに誤検知する事例があった
                ため、判定を厳格化した: 水平加速度による判定は「前進」を
                指示している間のみ有効とし (旋回中は元々並進加速度が小さく
                なるため誤検知の原因だった)、さらに GPS 対地速度が十分低い
                ことも AND 条件で要求し (加速度ノイズだけでの誤検知対策)、
                連続確定に必要な時間も 2.0s → 4.0s に延長した。
                スタック回復時は、検知直前に旋回していた方向と逆方向へ
                地磁気(9軸)の方位変化から【ちょうど180°】回転したことを
                確認してから STUCK_RECOVER_FWD_SEC (10秒) 前進する。
                この前進中も引き続きスタック判定を行い、再びスタックを
                検知した場合は 90° 回転してから再度 10 秒前進する
                (STUCK_RECOVER_MAX_RETRIES 回まで繰り返す)。
                走行中は NAV_REPORT_INTERVAL_SEC ごとに、9軸センサから
                算出した目標方位と機体方位のずれ、目標までの距離を
                まとめてログ表示する。
                目標まで PHASE4_TO_5_RADIUS 以内に入ったら Phase4 を終了し
                Phase5 へ移行する (タイムアウト時はそのままミッション終了)。

    Phase 5 : 超音波センサによる最終接近 (★変更: 左右スキャン方式)
                GPS 精度ではこれ以上距離を詰められないため、前方の超音波
                センサでゴールに設置された物体との距離を直接測定しながら
                近づく。★変更: 従来は距離を判断に使わず機械的に左右へ
                振っていたが、これを「左に旋回して距離測定 → 右に旋回して
                距離測定 → 近かった方向へ向き直しながら前進」という、
                超音波センサの値に基づいて能動的に対象物へ機体を向けていく
                方式に変更した (FINAL_SCAN_SEC ごとに1サイクル)。
                前方オブジェクトとの距離が FINAL_STOP_DIST_M (0.05 m) 以下
                になったら「0距離到達」とみなしてモータを停止し、ミッション
                完了とする。FINAL_APPROACH_TIMEOUT_SEC で安全タイムアウト。

    共通 : フェーズが切り替わるたびに LED を LED_BLINK_SEC 秒間点滅させ、
           現在どのフェーズに移行したかを外から視認できるようにする。

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
# グローバル共有変数 (★変更: 全体コードの最上部に配置)
# ===========================================================================
# ★変更: 以前は「設定値」セクションの後 (ファイル中盤) に置いていたが、
#        コード全体を読む際にまず「どんな状態を共有しているか」が
#        ひと目でわかるよう、import 文の直後・設定値セクションの前に
#        移動した。ここに定義される変数は、各フェーズ関数・スレッド間で
#        センサ値や走行状態を共有するために使われる。

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

# ★追加: 地磁気ベクトルの平滑化 (EMA: 指数移動平均) 状態。
#        実機ログ解析により、モータへ高デューティで通電している間、
#        地磁気センサの読み取り値が100ms間隔で数十〜100度以上も
#        ジャンプするほどのノイズ (モータ電流による磁気干渉と推定) が
#        発生し、これが原因で機体方位が暴れて GPS 誘導が発散
#        (目標から遠ざかる) することが確認された。calc_azimuth_with_front()
#        内でこの EMA フィルタを適用し、瞬間的なノイズの影響を抑える。
g_mag_ema = None   # [x, y, z] の平滑化済みベクトル。None = 未初期化 (最初のサンプルで初期化)
MAG_EMA_ALPHA = 0.25   # [-] 0〜1。小さいほど平滑化が強い(応答は遅くなる)。
                       #     新規サンプルの反映比率: new = alpha*sample + (1-alpha)*old

# ログバッファ
log_rows  = []
log_lock  = threading.Lock()

# フェーズ番号 → 日本語名。ログに現在何をしているフェーズかを
# ひと目でわかるように併記するために使う。
PHASE_NAMES = {
    0: "初期化",
    1: "落下検知",
    2: "初期後退・停止・前進",
    3: "キャリブレーション/GPS安定化",
    4: "GPS誘導走行",
    5: "超音波最終接近",
}

# センサ別のデータ取得状況トラッキング ──────────────────────────
# 「直近の読み取りが成功したか」「最後に成功したのはいつか」をセンサごとに
# 記録しておき、ログでひと目に状況がわかるようにする
# (例: GPS だけ Fix していない、Sonar が長時間 NG、等にすぐ気づける)。
SENSOR_NAMES = ("BNO055", "GPS", "Sonar")
g_sensor_status = {
    name: {"ok": False, "last_ok_time": None, "detail": "未取得"}
    for name in SENSOR_NAMES
}
g_sensor_status_lock = threading.Lock()

def mark_sensor_ok(name: str, detail: str = ""):
    """センサ読み取り成功時に状況を更新する。"""
    with g_sensor_status_lock:
        g_sensor_status[name] = {"ok": True, "last_ok_time": time.time(), "detail": detail}

def mark_sensor_fail(name: str, detail: str = ""):
    """センサ読み取り失敗時に状況を更新する (直近成功時刻は保持し、経過時間を追えるようにする)。"""
    with g_sensor_status_lock:
        prev = g_sensor_status.get(name, {})
        g_sensor_status[name] = {
            "ok": False,
            "last_ok_time": prev.get("last_ok_time"),
            "detail": detail,
        }

def format_sensor_status() -> str:
    """全センサの取得状況を 1 行にまとめた文字列を作る (ログ表示用)。"""
    now = time.time()
    parts = []
    with g_sensor_status_lock:
        snapshot = {k: dict(v) for k, v in g_sensor_status.items()}
    for name in SENSOR_NAMES:
        st = snapshot.get(name, {})
        if st.get("ok"):
            detail = st.get("detail", "")
            parts.append(f"{name}=OK" + (f"({detail})" if detail else ""))
        else:
            last = st.get("last_ok_time")
            if last is None:
                parts.append(f"{name}=未取得")
            else:
                parts.append(f"{name}=NG(最終成功 {now - last:.0f}s前)")
    return "  ".join(parts)

# ===========================================================================
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#   設定値  ← ここを実地に合わせて変更
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# ===========================================================================

# --- 目標座標 ---------------------------------------------------------------
TARGET_LAT =  38.26052      # 目標緯度  [度]
TARGET_LNG = 140.8544151    # 目標経度  [度]
# ★変更: この距離以内になったら Phase4 (GPS誘導走行) を終了し、
#        Phase5 (超音波センサによる最終接近) へ移行する。
#        最終的な「0距離」への接近は GPS ではなく Phase5 の超音波センサで行う。
PHASE4_TO_5_RADIUS = 2.0    # [m]

# --- 地磁気偏角 (仙台付近) --------------------------------------------------
MAG_DECLINATION = -8.0      # [度]  西偏 → 負値

# --- Phase 1: 落下検知 -------------------------------------------------------
FALL_THRESHOLD       = 3.0  # [m/s²]  合成加速度ノルムがこれ以下 → 落下中
FALL_COUNT_THRESHOLD = 8    # 連続カウント数
FALL_TIMEOUT_SEC     = 7 * 60  # [s]  7 分でタイムアウト

# --- Phase 2: 初期後退 + 停止 + 前進 ------------------------------------------
# ★変更: 従来は PHASE2_BACK_SEC (固定30秒) で後退していたが、パラシュート/
#        サブキャリアからの分離タイミングは機体の落下姿勢や地面との接触状況で
#        毎回変わるため、固定秒数では「分離できていないのに後退をやめる」
#        「分離済みなのに無駄に後退を続ける」の両方のリスクがあった。
#        そこで前方(進行方向とは反対、後退中は「背面」ではなく後退前に
#        機体前方に設置されている超音波センサがサブキャリアの方を向いている
#        前提)の超音波センサで距離を監視し、サブキャリアから機体が分離した
#        瞬間に生じる距離の急増 (PHASE2_SONAR_JUMP_THRESH_M 以上) を検知して
#        その場で後退を打ち切る方式に変更した。
PHASE2_BACK_TIMEOUT_SEC     = 30.0  # [s]  安全タイムアウト (分離検知できない場合の後退上限)
PHASE2_BACK_MIN_SEC         =  3.0  # [s]  後退開始直後はセンサ値が乱れやすいため、
                                     #      この秒数が経過するまでは分離判定を行わない
PHASE2_SONAR_JUMP_THRESH_M  = 0.5   # [m]  直前の有効値からこれ以上距離が急増したら
                                     #      「サブキャリアから分離した」と判定する
PHASE2_SONAR_LOST_TIMEOUT_SEC = 5.0 # [s]  超音波が有効値を返さない状態がこれだけ続いたら
                                     #      警告ログを出す (判定自体は継続)
PHASE2_LOOP_DT       =  0.1  # [s]  後退中の監視ループ周期
PHASE2_PAUSE_SEC     =  5.0  # [s]  後退後の停止秒数 (後退→前進の切り替え待ち)
PHASE2_FWD_AFTER_SEC = 10.0  # [s]  後退・停止後に行う前進秒数

# --- Phase 3: キャリブレーション ----------------------------------------------
# ★変更: 以前は Sys/Gyro/Acc/Mag すべてに同じ CALIB_MIN_LEVEL(=2) を要求していたが、
#        実地では Acc・Gyro のレベルが上がりにくく、それが原因でタイムアウトしていた。
#        地上機は姿勢を大きく変えられず Acc の複数姿勢キャリブレーションが困難なため、
#        軸ごとに現実的な目標レベルを個別に設定できるようにした。
#        (Mag は自律スピンで比較的到達しやすいので従来通り高めのままにしている)
#
# ★重要な追加変更: 実機ログを分析した結果、CALIB_MIN_LEVEL_ACC=1 でも
#        180秒のタイムアウトまで一度も達成できず、Acc=0 のまま停滞し続けて
#        いたことが判明した。BNO055 の加速度センサキャリブレーションは、
#        内部アルゴリズム上「機体を6方向以上の異なる姿勢 (上下反転を含む)
#        で数秒間静止させる」ことを要求する。しかし本機は地上を走行する
#        平面移動ロボットであり、物理的に姿勢を変える手段が無いため、
#        Acc キャリブレーションレベルが 0 から絶対に上がらない
#        (=どれだけ待っても目標に到達できない) という構造的な限界がある。
#        このため CALIB_MIN_LEVEL_ACC は要求しない (=0) ものとし、
#        Sys/Gyro/Mag のみで完了判定を行うように変更した。
#        なお、Acc が未キャリブレーションであっても、Phase3 完了後に
#        測定する静止基準水平加速度 (baseline_horiz_acc) が実行時の
#        オフセットを実質的に補正するため、落下検知・スタック検知への
#        影響は限定的である。
CALIB_MIN_LEVEL_SYS  = 1    # 0〜3  システム全体 (他軸がある程度揃えば自然に上がることが多い)
CALIB_MIN_LEVEL_GYRO = 1    # 0〜3  ★緩和: 静止だけで上がるはずだが、モータ振動の余韻等で
                            #        上がりにくい場合があるため要求を下げた
CALIB_MIN_LEVEL_ACC  = 0    # 0〜3  ★変更: 地上機は複数姿勢を取れず物理的に到達不可能なため、
                            #        Acc キャリブレーションは要求しない (常に条件を満たす)
CALIB_MIN_LEVEL_MAG  = 2    # 0〜3  スピンで比較的到達しやすいので維持
CALIB_TIMEOUT_SEC = 180.0   # [s]  キャリブレーション待機タイムアウト (地上機なので少し長め)
# スタック検知用：水平加速度の「動いているとき」の基準を測る秒数
CALIB_ACC_MEASURE_SEC = 3.0
# 詳細レポートをログに出す周期 (コンソールの1行表示とは別に、まとめて状況報告する)
CALIB_REPORT_INTERVAL_SEC = 5.0
# あるセンサ軸がこの秒数レベル改善しなければ「停滞」警告を出す
CALIB_STALL_WARN_SEC = 20.0
# 静止フェーズ1回あたりの秒数 (Acc/Gyro向け)。
# スピン(Mag向け)と交互に繰り返すことで、特定軸だけが
# 原因で全体が進まなくなるのを防ぐ。
CALIB_STILL_SEC = 6.0
# ★追加: スピン直後にモータを止めても、慣性で機体がすぐには完全静止しない。
#        「静止」区間の頭からその余韻の分だけ経過を捨てる(セトリング)ことで、
#        実際には揺れている間を静止時間としてカウントしてしまい
#        Gyro/Acc のキャリブレーションが進まなくなる問題を防ぐ。
CALIB_SETTLE_SEC = 1.5

# --- Phase 3: 自律回転動作 (地磁気キャリブレーション促進) --------------------
# 平面移動しかできない地上機では、腕で振る「8の字」の代わりに
# その場でのスピン (弱旋回の弧ではなく強旋回による回転) を左右交互に行うことで
# 全方位の地磁気データを収集し、Magキャリブレーションを促進する。
FIG8_ENABLE   = True   # True: モータで自律的に回転動作を行う
FIG8_SPIN_SEC = 8.0    # [s]  片方向にスピンし続ける秒数。機体が最低1回転できる値に調整すること
                       #      (回転が速すぎる/遅すぎる場合はここを実地に合わせて調整)

# --- Phase 3: GPS Fix 安定化・平均化 (★追加) --------------------------------
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

# ★追加: 真逆 (方位角が目標のちょうど180°反対) 付近での旋回方向拮抗対策。
#        diff (azimuth と target_bearing の差) は -180〜+180 の範囲で
#        表現されるため、機体が目標の真逆を向いているとき diff は
#        +180 付近にも -180 付近にもなり得る (符号はセンサノイズ次第で
#        簡単に反転する)。apply_diff() は diff の符号だけで
#        turn_left_strong / turn_right_strong を選ぶため、この境界付近では
#        ノイズのたびに左右の旋回指令が入れ替わり、機体がその場で
#        左右に拮抗して回転できず、結果的に目標と反対方向へ流れて
#        しまう不具合があった。
#        そこで |diff| がこの閾値以上のとき (=ほぼ真逆を向いているとき)
#        は diff の符号によらず必ず右回転 (turn_right_strong) を選ぶ
#        「不感帯」を設け、旋回方向を一意に固定して拮抗を防ぐ。
#        機体が回頭して |diff| がこの閾値を下回ったら、通常通り
#        diff の符号に応じた最短方向への旋回に自動的に切り替わる。
ANGLE_180_PREFER_RIGHT_DEG = 170.0   # [度]  |diff| がこれ以上なら右回転で統一

# スタック検知
# ★変更: 誤検知(実際はスタックしていないのに検知してしまう)対策として、
#        判定条件を全体的に厳しくした。
#          (1) 水平加速度による判定は「前進を指示している間」のみ有効にする。
#              旋回中はその場スピンにより並進加速度がもともと小さくなるため、
#              以前はこれを誤って「動いていない=スタック」と判定していた。
#          (2) 加速度に加えて GPS 速度が十分低いことも要求する (AND条件)。
#              振動などのノイズだけで加速度が一時的に下がった場合の誤検知を防ぐ。
#          (3) 連続判定に必要な時間を 2.0s → 4.0s に延長し、瞬間的な値の
#              揺れで確定しないようにした。
STUCK_HORIZON_ACCEL_THRESH = 0.5   # [m/s²]  水平加速度ノルムがこれ未満 → スタック疑い (前進中のみ判定)
STUCK_GPS_SPEED_THRESH_MPS = 0.15  # [m/s]   ★追加: GPS対地速度がこれ未満も併せて要求
KNOTS_TO_MPS = 0.514444            # [-]     ノット→m/s 変換係数
STUCK_SONAR_DIST_THRESH    = 0.3   # [m]     超音波距離がこれ未満でも → スタック疑い (常時判定)
STUCK_COUNT_THRESHOLD      = 40    # ★変更: 連続カウント数 (20→40, LOOP_DT × N 秒間継続で確定)

# ★変更: 固定秒数の旋回ではなく、地磁気(9軸)から算出した方位の変化量で
#        「ちょうど指定角度回転した」ことを確認してから前進する方式に変更。
STUCK_RECOVER_TURN_DEG          = 180.0  # [度] 初回回復: 直前と反対方向へ回転する角度
STUCK_RECOVER_RETRY_TURN_DEG    = 90.0   # [度] 前進中に再スタックした場合の追加回転角度
STUCK_RECOVER_TURN_TIMEOUT_SEC  = 15.0   # [s] 回転動作の安全タイムアウト (方位が読めない場合の保険)
STUCK_RECOVER_FWD_SEC           = 10.0   # [s] 回復: 回転後の前進時間 (★変更: 30→10)
STUCK_RECOVER_MAX_RETRIES       = 5      # [回] 前進中の再スタック→90°回転を繰り返す最大回数 (無限ループ防止)

# ★追加: 走行フェーズで目標までの距離・方位ずれをまとめてログ表示する周期
NAV_REPORT_INTERVAL_SEC = 5.0  # [s]

# --- Phase 4: 前方補正 (★変更) -----------------------------------------------
# ★変更: 以前は前方候補を4通り用意しGPS試行で自動探索する方式だったが、
#        欠陥があった (候補を切り替えても実際には機体を回転させておらず、
#        全候補で同じ方向に直進していただけで判別になっていなかった)。
#        BNO055 の Axis Remap 機能 (BNO_ORIENTATION, set_bno_orientation)
#        で Phase0 の初期化時に一度だけ設定する方式に置き換えたため、
#        Phase4 側で前方候補を探索する処理は不要になった。

# --- Phase 5: 超音波センサによる最終接近 (★変更: 左右スキャン方式) ------------
# Phase4 で目標まで PHASE4_TO_5_RADIUS 以内に入ったら開始する最終フェーズ。
# GPS はこれ以上正確に距離を詰められないため、前方の超音波センサで
# 目標物(ゴールに設置された物体)を検知しながら接近する。
# ★変更: 従来は距離を判断に使わず機械的に左右へ振っていたが、これを
#        「左に旋回しながら距離を測定 → 右に旋回しながら距離を測定 →
#        近かった方向へ旋回して進む」という、超音波センサの値に基づいて
#        能動的に対象物へ機体を向けていく方式に変更した。
#        (超音波センサは指向性があるため、旋回してその方向の距離が短く
#        なるほど、その方向に対象物が存在する可能性が高いという前提)
FINAL_STOP_DIST_M      = 0.05   # [m]  この距離以下で最終接近完了 (ゴール)
FINAL_SCAN_SEC          = 0.3   # [s]  左/右それぞれの旋回スキャンにかける時間
FINAL_APPROACH_TIMEOUT_SEC = 5 * 60  # [s]  Phase5 の安全タイムアウト (5分)
FINAL_NAV_REPORT_INTERVAL_SEC = 3.0  # [s]  距離・方位ずれのまとめログ周期
FINAL_SONAR_LOST_WARN_SEC = 15.0     # [s]  超音波が有効値を返さない状態がこの秒数続いたら警告
FINAL_SCAN_LOOP_DT      = 0.02  # [s]  スキャン中の超音波サンプリング周期

# --- モータ: ソフトスタート (デューティ比の段階的引き上げ) (★追加) ---------
# 停止状態から動き出す瞬間にいきなり高いデューティ比をかけると、
# 突入電流やホイールスピン・機体への衝撃が大きくなりやすい。
# そこで「動き始め」は従来と同じ RAMP_START_DUTY (80%) から立ち上がり、
# そのまま動作を継続した場合は RAMP_DURATION_SEC 秒かけてなめらかに
# RAMP_END_DUTY (100%) まで引き上げていく方式にした。
# 一度 stop() で完全停止すると、次に動き出す際は再びこのカーブの
# 最初 (RAMP_START_DUTY) からやり直す。
RAMP_START_DUTY      = 0.8   # [-] 動き始め (t=0) のデューティ比 (=従来の MOTOR_SPEED と同じ)
RAMP_END_DUTY        = 1.0   # [-] 連続動作を続けた末の最終デューティ比 (フル出力)
RAMP_DURATION_SEC    = 3.0   # [s] START → END まで引き上げるのにかける時間
RAMP_UPDATE_INTERVAL_SEC = 0.2  # [s] バックグラウンドでデューティ比を再計算・反映する周期

# 弱旋回側 (turn_left_weak / turn_right_weak) の出力は、従来
# MOTOR_SPEED(0.8) に対して SPEED_WEAK(0.4) = 50% の比率だった。
# ソフトスタートを導入した後も同じ比率を保つよう、強旋回側の
SPEED_WEAK  = 0.4                          # [-] 従来の弱旋回側 duty (比率計算の元値として保持)
WEAK_RATIO  = SPEED_WEAK / RAMP_START_DUTY  # [-] 強旋回側に対する弱旋回側の出力比率 (デフォルト 0.5)

# ★削除: PHASE4_MOTOR_REVERSED (Phase4限定のモータ反転フラグ) は撤去した。
#        フラグを増やして個別補正しようとすると、フラグ同士の組み合わせで
#        逆に機体の動きが把握しづらくなる。モータは常に素直に forward=前進 /
#        backward=後退として動作し、取り付け向きの補正は BNO055 の
#        Axis Remap (BNO_ORIENTATION) だけに一本化する。

# モータピン (BCM) ← test_run.py / test_avoid.py と統一
PIN_PWMA = 13
PIN_AIN1 =  6
PIN_AIN2 =  5
PIN_PWMB = 18
PIN_BIN1 = 24
PIN_BIN2 = 23
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
# ★変更: 従来は赤道半径 (6378136.59m) を使った平面近似 (等長円筒図法) で
#        distance/bearing を計算していたが、より標準的で精度の高い
#        Haversine距離 / 大圆初期方位角の式に変更したため、それに対応する
#        地球の平均半径 (WGS84 authalic mean radius に近い一般的な値) を使用する。
EARTH_RADIUS = 6371000.0   # [m]  Haversine計算用の地球平均半径

# ログ
LOG_DIR = _ROOT_DIR / "logs"

# ===========================================================================
# ロガー
# ===========================================================================

def log(msg: str, level: str = "INFO"):
    """タイムスタンプ付きログを標準出力に出す。nohup 経由でファイルにも残る。
    どのフェーズで何が起きているか一目でわかるよう、フェーズ番号に加えて
    日本語名も併記する。"""
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    phase_name = PHASE_NAMES.get(phase, "?")
    line = f"[{ts}][Phase{phase}:{phase_name}][{level}] {msg}"
    print(line, flush=True)

def log_data_status(context: str = ""):
    """
    ★追加: 現在の全センサのデータ取得状況を 1 行にまとめてログ出力する。
    「今どの演算をしているか」を示す context と併せて呼ぶことで、
    その演算がどのセンサ値に基づいているか・データが正常かがわかりやすくなる。
    """
    prefix = f"{context}: " if context else ""
    log(f"[DataStatus] {prefix}{format_sensor_status()}")

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
    """
    フェーズ切り替えのタイミングで LED を duration 秒間点滅させる。
    ブロッキング実装 (フェーズ開始を明示的に知らせてから本処理に入るため)。
    点滅終了後は常時点灯に戻す。
    """
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
                        if g_gps_valid:
                            mark_sensor_ok("GPS", f"sats={g_gps_sats}")
                        else:
                            mark_sensor_fail("GPS", "Fix未取得(lat=0)")
                except Exception as e:
                    mark_sensor_fail("GPS", f"読み取りエラー: {e}")
                    log(f"GPS 読み取りエラー: {e}", "WARN")
    except serial.SerialException as e:
        mark_sensor_fail("GPS", f"ポートエラー: {e}")
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
            mark_sensor_fail("Sonar", "デバイス無効")
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
                    mark_sensor_fail("Sonar", "ECHO立ち上がりタイムアウト")
                    return None
            t_start = time.time()
            while self._echo.value:
                if time.time() - t_start > SONAR_TIMEOUT:
                    mark_sensor_fail("Sonar", "ECHO立ち下がりタイムアウト")
                    return None
            t_end = time.time()
            dist_m = (t_end - t_start) * SOUND_SPEED / 2.0
            mark_sensor_ok("Sonar", f"{dist_m:.2f}m")
            return dist_m
        except Exception as e:
            log(f"超音波センサエラー: {e}", "WARN")
            mark_sensor_fail("Sonar", f"エラー: {e}")
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
        # ★削除: モータの forward/backward を丸ごと反転させる _reversed
        #        フラグは撤去した。モータは常に素直に forward=前進 /
        #        backward=後退として動作する (test_GPSrun.py と同じ、
        #        余計な補正を挟まないシンプルな設計)。

        # ★追加: 後退方向への回転を絶対に禁止する安全ロック。
        #        True の間は backward() が呼ばれても実際には後退させず、
        #        警告ログを出して停止のみ行う。Phase4 (誘導走行・
        #        スタック回復) のように「いかなる場合も後退してはならない」
        #        フェーズで set_forward_only(True) を呼んで有効化する想定。
        #        コード上のバグや将来の変更で誤って backward() が呼ばれても
        #        物理的に後退できないようにする、最後の防波堤として機能する。
        self._forward_only = False

        # ★追加: ソフトスタート (デューティ比 段階的引き上げ) 用の状態。
        #   バックグラウンドスレッド (_ramp_loop) が RAMP_UPDATE_INTERVAL_SEC
        #   ごとに「動き始めてからの経過時間」に応じたデューティ比を計算して
        #   PWM 出力へ反映し続ける。forward/backward/turn_* のどのメソッドを
        #   呼んでも同じ仕組みが自動的に働くため、各メソッド側は
        #   「今どちらの車輪をどの倍率で回すか (0.0〜1.0)」だけを指定すればよい。
        self._ramp_lock              = threading.Lock()
        self._move_start_time        = None   # 現在の連続動作の開始時刻 (None=停止中)
        self._duty_scale_a           = 1.0    # 右モータ: 現在デューティに対する倍率
        self._duty_scale_b           = 1.0    # 左モータ: 現在デューティに対する倍率
        self._last_duty_pct_reported = None   # ログ間引き用 (5%刻みでのみ報告)
        self._closed                 = False
        self._ramp_thread = threading.Thread(target=self._ramp_loop, daemon=True)
        self._ramp_thread.start()

        self.stop()
        log("モータ初期化完了")
        log(f"[Motor] ソフトスタート設定: {RAMP_START_DUTY*100:.0f}% → {RAMP_END_DUTY*100:.0f}% "
            f"を {RAMP_DURATION_SEC:.1f} 秒かけて引き上げ (更新周期 {RAMP_UPDATE_INTERVAL_SEC:.2f}s)")

    def set_forward_only(self, enabled: bool):
        """
        ★追加: 後退方向への回転を絶対に禁止する安全ロックを有効/無効にする。
        有効化 (True) すると、以降 backward() が呼ばれても実際には
        モータを後退方向へ一切回転させず、警告ログを出して stop() のみ
        行う (＝呼び出し元がミスをしても物理的に後退できない)。
        Phase4 (誘導走行・スタック回復) のように、後退動作が
        絶対に許されないフェーズの開始時に True、終了時に False へ戻す
        使い方を想定している。
        """
        state = "有効(backward()は無効化)" if enabled else "無効(通常通り)"
        log(f"[Motor] 後退禁止モードを{state}に設定")
        self._forward_only = enabled

    # ── ★追加: ソフトスタート (デューティ比 段階的引き上げ) ──────────────
    def _ramp_loop(self):
        """
        バックグラウンドで常時動き続けるループ。
        _move_start_time が設定されている (=モータが動作中) 間、
        RAMP_UPDATE_INTERVAL_SEC ごとに経過時間から現在のデューティ比を
        計算して PWM 出力 (_pwm_a / _pwm_b) へ反映する。
        stop() が呼ばれて _move_start_time が None に戻ると何もせず待機する。
        """
        while not self._closed:
            with self._ramp_lock:
                move_start = self._move_start_time
                scale_a    = self._duty_scale_a
                scale_b    = self._duty_scale_b

            if move_start is not None:
                elapsed = time.time() - move_start
                if elapsed >= RAMP_DURATION_SEC:
                    duty = RAMP_END_DUTY
                else:
                    duty = RAMP_START_DUTY + (RAMP_END_DUTY - RAMP_START_DUTY) * (elapsed / RAMP_DURATION_SEC)

                self._pwm_a.value = max(0.0, min(1.0, duty * scale_a))
                self._pwm_b.value = max(0.0, min(1.0, duty * scale_b))

                # ── 5%刻みでのみログ出力し、ログが埋まらないようにする ──
                pct = round(duty * 100)
                if pct != self._last_duty_pct_reported and pct % 5 == 0:
                    self._last_duty_pct_reported = pct
                    log(f"[Motor] ソフトスタート中: デューティ比 {pct}% "
                        f"(経過 {elapsed:.1f}/{RAMP_DURATION_SEC:.1f} s, "
                        f"右倍率={scale_a:.2f} 左倍率={scale_b:.2f})")

            time.sleep(RAMP_UPDATE_INTERVAL_SEC)

    def _start_move(self, scale_a: float, scale_b: float):
        """
        新しい動作を開始する際に呼ぶ。
        既に停止していた (_move_start_time is None) 場合は「今」を
        動き始めの基準時刻として記録し、ソフトスタートを RAMP_START_DUTY
        から開始する。stop() を挟まずに方向だけ変えた場合 (例: 前進中に
        微調整で弱旋回へ切り替わる等) は経過時間を引き継ぎ、
        デューティ比はそのまま引き上げを継続する。
        scale_a / scale_b は「現在のデューティ比に対する各モータの倍率」
        (0.0〜1.0) で、強旋回側は 1.0、弱旋回側は WEAK_RATIO、
        停止側は 0.0 を指定する。
        """
        with self._ramp_lock:
            if self._move_start_time is None:
                self._move_start_time = time.time()
                self._last_duty_pct_reported = None
                log(f"[Motor] 動作開始 → ソフトスタート {RAMP_START_DUTY*100:.0f}% から立ち上げます")
            self._duty_scale_a = scale_a
            self._duty_scale_b = scale_b
        self._stby.on()

    def forward(self):
        self._start_move(1.0, 1.0)
        self._mot_a.forward()
        self._mot_b.forward()

    def backward(self):
        # ★追加: 後退禁止ロック中は、実際には後退させず安全に停止するだけ。
        #        Phase4 等「絶対に後退してはならない」フェーズ中に
        #        誤って backward() が呼ばれても、物理的に後退できない
        #        ようにするための最終防波堤。
        if self._forward_only:
            log("[Motor][WARN] 後退禁止モード中に backward() が呼び出されました。"
                "安全のため後退はせず停止します。", "WARN")
            self.stop()
            return
        self._start_move(1.0, 1.0)
        self._mot_a.backward()
        self._mot_b.backward()

    def stop(self):
        with self._ramp_lock:
            self._move_start_time        = None
            self._duty_scale_a           = 1.0
            self._duty_scale_b           = 1.0
            self._last_duty_pct_reported = None
        self._pwm_a.value = 0
        self._pwm_b.value = 0
        self._mot_a.stop()
        self._mot_b.stop()
        self._stby.off()

    def turn_left_strong(self):
        """右モータ前進 / 左モータ停止 → 左旋回"""
        self._start_move(1.0, 0.0)
        self._mot_a.forward()
        self._mot_b.stop()

    def turn_right_strong(self):
        """右モータ停止 / 左モータ前進 → 右旋回"""
        self._start_move(0.0, 1.0)
        self._mot_a.stop()
        self._mot_b.forward()

    def turn_left_weak(self):
        self._start_move(1.0, WEAK_RATIO)
        self._mot_a.forward()
        self._mot_b.forward()

    def turn_right_weak(self):
        self._start_move(WEAK_RATIO, 1.0)
        self._mot_a.forward()
        self._mot_b.forward()

    def apply_diff(self, diff: float) -> str:
        """
        diff (−180〜+180) でモータ指令を自動選択。
        test_GPSrun.py の MotorController.apply_diff() をベースに、
        ★追加: 真逆 (|diff| が ANGLE_180_PREFER_RIGHT_DEG 以上、
        ほぼ180°) のときは、diff の符号 (+180寄りか-180寄りか) に
        左右されず必ず右回転 (turn_right_strong) を選ぶよう変更した。
        これにより、真逆を向いた状態でセンサノイズにより diff の符号が
        ±180の境界で頻繁に入れ替わっても、旋回方向が一意に定まり、
        左右への拮抗(その場で行ったり来たりして進めなくなる現象)を防ぐ。
        """
        if abs(diff) < ANGLE_DEADBAND:
            self.forward();           return "FORWARD"
        elif abs(diff) >= ANGLE_180_PREFER_RIGHT_DEG:
            self.turn_right_strong(); return "TURN_R_STRONG(180°反転優先)"
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
        self._closed = True   # ★追加: バックグラウンドの _ramp_loop を終了させる
        self._pwm_a.close(); self._pwm_b.close()
        self._mot_a.close(); self._mot_b.close()
        self._stby.close()

# ===========================================================================
# 航法計算 (★変更: Haversine距離 + 大圆初期方位角の標準公式に変更)
# ===========================================================================
# ★変更: 従来は「経度差×cos(lat)×地球半径」「緯度差×地球半径」による
#        平面近似 (等長円筒図法, 十数m規模ではこれでも大きな誤差は出ないが
#        厳密ではない) で距離・方位を求めていた。実運用ログを分析した結果、
#        この近似式自体の数値は手計算とも一致しており大きなバグは無かったが、
#        「精度をさらに上げてほしい」との要望と、GPS誘導の信頼性を少しでも
#        高める目的で、球面三角法に基づく標準的な Haversine 距離公式・
#        大圆(great-circle)初期方位角公式に置き換えた。緯度によって
#        経度1度あたりの実距離が変わる影響も正確に扱えるため、より高緯度や
#        長距離でも精度が保たれる。

def calc_distance(lat, lng) -> float:
    """
    Haversine の公式による現在地から目標地点までの球面距離 [m]。
    平面近似より厳密に地球の曲率を考慮するため、緯度・経度差が
    大きい場合でも誤差が蓄積しにくい。
    """
    lat1 = math.radians(lat)
    lat2 = math.radians(TARGET_LAT)
    dlat = math.radians(TARGET_LAT - lat)
    dlng = math.radians(TARGET_LNG - lng)
    a = (math.sin(dlat / 2.0) ** 2
         + math.cos(lat1) * math.cos(lat2) * math.sin(dlng / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS * c

def calc_target_bearing(lat, lng) -> float:
    """
    大圆 (great-circle) 初期方位角の公式による、現在地から目標地点への
    真方位 [度, 0〜360, 北を0として時計回り]。
    """
    lat1 = math.radians(lat)
    lat2 = math.radians(TARGET_LAT)
    dlng = math.radians(TARGET_LNG - lng)
    x = math.sin(dlng) * math.cos(lat2)
    y = (math.cos(lat1) * math.sin(lat2)
         - math.sin(lat1) * math.cos(lat2) * math.cos(dlng))
    theta = math.atan2(x, y)
    return (math.degrees(theta) + 360.0) % 360.0

def calc_azimuth(mag: list) -> float:
    """
    生の地磁気ベクトルから磁北基準の方位角 [度, 北=0, 東=90, 南=180, 西=270,
    時計回り] を計算する。

    ★変更: 反転系のフラグ (AZIMUTH_MIRROR_FIX) を廃止し、test_GPSrun.py と
    全く同じ「素の」計算式に統一した。フラグを増やして条件分岐で
    細かく補正しようとすると、フラグ同士の組み合わせが増えて逆に
    機体の動きが把握しづらくなる (今回発生した混乱の原因)。取り付け
    向きの補正は BNO_ORIENTATION (Axis Remap, Phase0で1回設定) だけに
    一本化し、方位計算式自体は常に固定・単純に保つ。
    """
    azimuth = 90.0 - math.degrees(math.atan2(mag[1], mag[0]))
    azimuth *= -1
    azimuth += MAG_DECLINATION
    return azimuth % 360.0

def calc_direction_diff(azimuth: float, bearing: float) -> float:
    diff = (azimuth - bearing) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff

# ===========================================================================
# ★変更: BNO055 軸再配置 (Axis Remap) による前方補正
# ===========================================================================
# BNO055 の地磁気ゼロ点(方位0°の向き)と、機体の物理的な前方(モータで
# forward() させたときに実際に進む向き)は、センサの取り付け角度・配線・
# 個体差等によって必ずしも一致しない。
#
# ★変更の経緯: 以前はこれをソフトウェア側で「前方候補 1〜4 を実際に
# 前進走行させ、GPS距離の変化を見て正しい候補を選ぶ」という前方探索
# (Front Search) で解決しようとしていた。しかし、この実装には重大な
# 欠陥があった: 候補を切り替えても set_front_candidate() が内部の
# オフセット変数を書き換えるだけで、機体を実際に回転させる動作を
# 一切行わずに motor.forward() を呼んでいたため、候補1〜4のすべてで
# 機体は完全に同じ物理的方向へ直進していた。そのため候補間の違いは
# GPS測位誤差程度のノイズでしかなく、前方候補を正しく判別できず、
# 誤った候補が採択されて航法計算全体に恒常的なズレが乗ってしまう
# 危険があった。
#
# そこで test_GPSrun.py で採用されていた、BNO055 の Axis Remap 機能
# (レジスタ 0x41=軸配置, 0x42=符号) を使ってセンサの内部基準軸を
# 物理的な取り付け向きに合わせて再配置する方式に変更した。これは
# BNO055 データシートに定義された正式な機能であり、GPS ノイズに
# 依存する推測ではなく、設定した瞬間から常に一貫して正しい方位が
# 得られる。取り付け向きは実機で 1〜4 を順に試し、目標へ向かって
# 前進するものを BNO_ORIENTATION に設定して固定運用する。

# --- BNO055 の取り付け向き設定 (test_GPSrun.py と同じ Axis Remap 方式) ------
# 1 : 正規の向き (0°回転)
# 2 : Z軸周りに 90°回転
# 3 : Z軸周りに 180°回転
# 4 : Z軸周りに 270°回転 (-90°回転)
# ★実機で 1→2→3→4 の順に試し、Phase4 で目標へ正しく向かって前進する
#   設定値をここに固定すること。
BNO_ORIENTATION = 1

def set_bno_orientation(bno: BNO055, orientation: int):
    """
    ★追加 (test_GPSrun.py より移植): BNO055 の Axis Remap 機能を使って、
    センサ内部の基準軸を機体の物理的な取り付け向きに合わせて再配置する。
    これにより、以降 bno.getMag() 等で得られる値は「機体前方が正しく
    軸に合った」状態になり、ソフトウェア側で前方オフセットを別途
    補正する必要がなくなる。

    設定は一度 CONFIG モードへ切り替えてレジスタ (0x41: 軸配置,
    0x42: 符号) を書き込み、その後 NDOF (フュージョン) モードへ戻す
    ことで反映される。ドライバの実装差異に対応するため、write_reg /
    set_mode / write_bytes / 直接 I2C アクセスのいずれかが使える方を
    試す (test_GPSrun.py と同一のフォールバック構造)。
    """
    remap_profiles = {
        1: (0x24, 0x00),  # P0: 0°  (正規)
        2: (0x21, 0x04),  # P1: 90°
        3: (0x24, 0x06),  # P2: 180°
        4: (0x21, 0x02),  # P3: 270°
    }

    if orientation not in remap_profiles:
        log(f"[BNO055][WARN] 無効な BNO_ORIENTATION ({orientation})。デフォルト(1)で動作します。", "WARN")
        orientation = 1

    config_val, sign_val = remap_profiles[orientation]

    try:
        if hasattr(bno, 'write_reg'):
            bno.write_reg(0x3D, 0x00)   # CONFIG モードへ
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
            bno.write_reg(0x3D, 0x0C)   # NDOF (フュージョン) モードへ復帰
        elif hasattr(bno, 'set_mode'):
            bno.set_mode(BNO055.OPERATION_MODE_NDOF)
        time.sleep(0.05)

        degrees_map = {1: "0°(正規)", 2: "90°", 3: "180°", 4: "270°"}
        log(f"[BNO055] Axis Remap 完了: 設定 {orientation} ({degrees_map[orientation]})")

    except Exception as e:
        log(f"[BNO055][WARN] Axis Remap 設定時の警告: {e}", "WARN")


def calc_azimuth_with_front(mag: list) -> float:
    """
    ★変更: 前方オフセット (g_front_offset_deg) の加算は、BNO055 側の
    Axis Remap (set_bno_orientation, Phase0 で1回設定) に一本化した
    ため廃止した。この関数は calc_azimuth() へ渡す前に地磁気ベクトルへ
    EMA (指数移動平均) フィルタを適用する処理のみを担う。

    実機ログ解析により、モータへ高デューティで通電している間、地磁気
    センサの読み取り値が100ms間隔で数十〜100度以上もジャンプするほどの
    ノイズ (モータ電流による磁気干渉と推定) が発生し、これが原因で
    機体方位が暴れて GPS 誘導が発散 (目標から遠ざかる) することが
    確認された。そのため、まず地磁気ベクトルへ EMA フィルタ (g_mag_ema,
    MAG_EMA_ALPHA) を適用してノイズを平滑化した上で calc_azimuth() へ渡す。
    """
    global g_mag_ema
    if g_mag_ema is None:
        g_mag_ema = [float(mag[0]), float(mag[1]), float(mag[2])]
    else:
        g_mag_ema[0] = MAG_EMA_ALPHA * mag[0] + (1.0 - MAG_EMA_ALPHA) * g_mag_ema[0]
        g_mag_ema[1] = MAG_EMA_ALPHA * mag[1] + (1.0 - MAG_EMA_ALPHA) * g_mag_ema[1]
        g_mag_ema[2] = MAG_EMA_ALPHA * mag[2] + (1.0 - MAG_EMA_ALPHA) * g_mag_ema[2]

    return calc_azimuth(g_mag_ema)

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

    # ★追加: Axis Remap により、機体の物理的な取り付け向きに合わせて
    #        BNO055 内部の基準軸を再配置する (test_GPSrun.py と同じ方式)。
    #        これにより以降 bno.getMag() 等で得られる方位は、ソフトウェア
    #        側で別途オフセットを補正する必要なく、そのまま機体前方基準
    #        として扱える。
    set_bno_orientation(bno, BNO_ORIENTATION)

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

def phase1_fall_detection(bno: BNO055, led: LED, start_time: float) -> bool:
    global phase, g_acc, g_calib
    phase = 1
    blink_led(led)
    log("─" * 62)
    log(f"[Phase1] 落下検知開始  閾値={FALL_THRESHOLD} m/s²  ×{FALL_COUNT_THRESHOLD}回連続")
    log(f"         タイムアウト={FALL_TIMEOUT_SEC} s")
    log_data_status("Phase1開始時点")

    fall_count = 0
    p1_start   = time.time()
    last_status_report = p1_start  # ★追加: データ取得状況の定期報告用

    # コンソールヘッダー
    print(f"\n{'経過[s]':>8}  {'acc_norm':>10}  {'fall_cnt':>8}  {'Calib(S/G/A/M)':>16}")
    print("-" * 50)

    while True:
        try:
            g_acc   = bno.getAcc()
            g_calib = bno.getCalibrationStatus()
            mark_sensor_ok("BNO055", f"acc_norm算出用")
        except Exception as e:
            mark_sensor_fail("BNO055", f"読み取りエラー: {e}")
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

        # ── ★追加: データ取得状況を定期的にログへ報告 ──
        if time.time() - last_status_report >= 5.0:
            last_status_report = time.time()
            log_data_status(f"落下判定演算中 (acc_norm={acc_norm:.3f}, 閾値={FALL_THRESHOLD})")

        if fall_count >= FALL_COUNT_THRESHOLD:
            log(f"[Phase1] 落下検知！ (連続 {fall_count} 回, {elapsed:.2f} s)")
            return True

        if elapsed > FALL_TIMEOUT_SEC:
            log(f"[Phase1] タイムアウト ({FALL_TIMEOUT_SEC} s) → 強制移行")
            return False

        time.sleep(0.05)


# ─── Phase 2: 超音波分離検知後退 → 停止 (5 秒) → 前進 (10 秒) ────────────────

def phase2_initial_backward(motor: MotorController, sonar: SonarSensor,
                            led: LED, start_time: float):
    """
    サブキャリア脱出のため後退する。

    ★変更: 従来は PHASE2_BACK_SEC (固定30秒) 後退していたが、分離タイミングは
    機体の落下姿勢や地面との接触状況で毎回変わるため、固定秒数では
    「まだ分離していないのに後退をやめる」「分離済みなのに無駄に後退を
    続ける」の両方のリスクがあった。

    そこで前方の超音波センサでサブキャリアとの距離を後退中も継続的に
    測定し、直前の有効値から PHASE2_SONAR_JUMP_THRESH_M [m] 以上距離が
    急増した瞬間 (=サブキャリアから機体が実際に分離した瞬間) を検知して、
    その場で後退を打ち切る。後退開始直後の PHASE2_BACK_MIN_SEC 秒間は
    センサ値が安定しない可能性があるため判定を保留し (誤検知防止)、
    超音波が長時間有効値を返さない場合や、分離が検知できないまま
    PHASE2_BACK_TIMEOUT_SEC を超えた場合は、安全のためタイムアウトとして
    後退を打ち切る。

    分離検知(またはタイムアウト)後、PHASE2_PAUSE_SEC 秒停止して切り替えの
    衝撃・振動を落ち着かせ、確実に離脱・前方へ距離を取るために
    PHASE2_FWD_AFTER_SEC 秒前進する。
    """
    global phase
    phase = 2
    blink_led(led)
    log("─" * 62)
    log(f"[Phase2] 超音波分離検知後退を開始します")
    log(f"  判定: 直前値から {PHASE2_SONAR_JUMP_THRESH_M:.2f} m 以上の距離急増で分離とみなす "
        f"(判定開始まで {PHASE2_BACK_MIN_SEC:.0f} s のウォームアップ)")
    log(f"  安全タイムアウト: {PHASE2_BACK_TIMEOUT_SEC:.0f} s")
    log_data_status("Phase2開始時点")

    motor.backward()
    p2_start = time.time()
    prev_valid_dist = None      # 直前の有効な超音波距離 [m]
    last_sonar_valid_time = p2_start
    separated = False
    timed_out = False

    print(f"\n{'経過[s]':>8}  {'Sonar[m]':>9}  {'直前値[m]':>10}  {'差分[m]':>8}  {'判定':>10}")
    print("-" * 55)

    while True:
        now = time.time()
        elapsed = now - p2_start

        dist = sonar.get_distance_m()
        dist_str = f"{dist:.3f}" if dist is not None else "---"

        judge = "計測中"
        if dist is not None:
            last_sonar_valid_time = now
            if elapsed < PHASE2_BACK_MIN_SEC:
                judge = "ウォームアップ中"
            elif prev_valid_dist is not None:
                jump = dist - prev_valid_dist
                if jump >= PHASE2_SONAR_JUMP_THRESH_M:
                    judge = "分離検知!"
                    log(f"[Phase2] 超音波距離が急増 ({prev_valid_dist:.3f}m → {dist:.3f}m, "
                        f"差分={jump:+.3f}m) → サブキャリアから分離したと判定します")
                    separated = True
                else:
                    judge = "後退中"
            prev_valid_dist = dist
        else:
            judge = "センサ無効"
            if now - last_sonar_valid_time >= PHASE2_SONAR_LOST_TIMEOUT_SEC:
                log(f"[Phase2][WARN] 超音波センサが {PHASE2_SONAR_LOST_TIMEOUT_SEC:.0f}s "
                    f"以上有効値を返していません。タイムアウト判定のみで後退を継続します。", "WARN")
                last_sonar_valid_time = now  # 再警告を連発しないよう基準を更新

        diff_str = (f"{dist - prev_valid_dist:+.3f}"
                    if (dist is not None and prev_valid_dist is not None and dist != prev_valid_dist)
                    else "--")
        print(f"{elapsed:>8.2f}  {dist_str:>9}  "
              f"{(f'{prev_valid_dist:.3f}' if prev_valid_dist is not None else '---'):>10}  "
              f"{diff_str:>8}  {judge:>10}", flush=True)
        log_sensor_row(time.time() - start_time, "BACKWARD_PHASE2",
                       f"sonar={dist_str} judge={judge}")

        if separated:
            break

        if elapsed > PHASE2_BACK_TIMEOUT_SEC:
            log(f"[Phase2][WARN] 分離を検知できないまま {PHASE2_BACK_TIMEOUT_SEC:.0f}s "
                f"のタイムアウトに達しました。後退を打ち切ります。", "WARN")
            timed_out = True
            break

        time.sleep(PHASE2_LOOP_DT)

    motor.stop()
    if separated:
        log(f"[Phase2] 後退完了 → 停止 (超音波による分離検知, 所要 {elapsed:.1f} s)")
    elif timed_out:
        log(f"[Phase2] 後退完了 → 停止 (タイムアウト強制終了, 所要 {elapsed:.1f} s)", "WARN")

    # ── ★変更: 後退と前進の間に 5 秒間の停止を挟む ──
    log(f"[Phase2] 停止 {PHASE2_PAUSE_SEC:.0f} 秒 (後退→前進の切り替え待ち)")
    deadline_pause = time.time() + PHASE2_PAUSE_SEC
    while time.time() < deadline_pause:
        remaining = deadline_pause - time.time()
        log(f"  停止中... 残り {remaining:.1f} s")
        log_sensor_row(time.time() - start_time, "STOP_PHASE2")
        time.sleep(1.0)

    # ── 後退・停止後に前進 (サブキャリアから確実に離脱するため) ──
    log(f"[Phase2] 前進 {PHASE2_FWD_AFTER_SEC:.0f} 秒")
    motor.forward()
    deadline2 = time.time() + PHASE2_FWD_AFTER_SEC
    while time.time() < deadline2:
        remaining = deadline2 - time.time()
        log(f"  前進中... 残り {remaining:.1f} s")
        log_sensor_row(time.time() - start_time, "FORWARD_PHASE2")
        time.sleep(1.0)
    motor.stop()
    log("[Phase2] 前進完了 → 停止")


# ─── Phase 3 補助: GPS Fix 安定化・平均化 (★追加) ───────────────────────────

def phase3_gps_stabilize(start_time: float):
    """
    GPS Fix の安定化・平均化。

    有効な Fix が得られ、衛星捕捉数が GPS_MIN_SATS 以上になるまで待機し
    (最大 GPS_CALIB_TIMEOUT_SEC 秒)、その後 GPS_CALIB_SAMPLE_SEC 秒間
    (最大 GPS_CALIB_SAMPLES 点) の緯度経度サンプルを収集して平均を取る。
    これにより単発の測位ノイズ・ふらつきの影響を減らし、誘導走行開始時点
    での目標までの距離・方位をログへ確認用に出力する。
    (誘導走行自体は毎ループ最新の GPS 値を使うため、この平均値は
     Phase4 の航法計算には使わず、あくまで開始前の状況確認・調整用)
    """
    log("─" * 62)
    log(f"[Phase3] GPS 安定化待機 (衛星数 >= {GPS_MIN_SATS}, タイムアウト {GPS_CALIB_TIMEOUT_SEC:.0f}s)")

    t_start = time.time()
    while True:
        elapsed = time.time() - t_start
        if g_gps_valid and g_gps_sats >= GPS_MIN_SATS:
            log(f"[Phase3] GPS Fix 取得 (衛星数={g_gps_sats}) → 位置サンプル収集を開始します")
            break
        if elapsed > GPS_CALIB_TIMEOUT_SEC:
            log("[Phase3] GPS 安定化タイムアウト。現状の Fix のまま続行します。", "WARN")
            break
        log(f"  GPS 待機中... valid={g_gps_valid} sats={g_gps_sats}  経過={elapsed:.1f}s")
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
        bearing = calc_target_bearing(avg_lat, avg_lng)
        log(f"[Phase3] GPS 平均位置: LAT={avg_lat:.6f} LNG={avg_lng:.6f}  ({len(lat_samples)} サンプル)")
        log(f"[Phase3] 目標までの距離={dist:.2f} m  方位={bearing:.1f}°")
    else:
        log("[Phase3] 有効な GPS サンプルが取得できませんでした。生値のまま誘導走行に入ります。", "WARN")


# ─── Phase 3: キャリブレーション & 誘導準備 ──────────────────────────────────

def phase3_calibration(bno: BNO055, motor: MotorController, led: LED, start_time: float) -> float:
    """
    BNO055 キャリブレーション待機。

    【これまでの改良の経緯】
    (1) 以前は Acc/Gyro が揃うまで完全静止し続けてからスピン(Mag促進)に
        入る仕組みで、Acc が上がらずスピンが始まらないままタイムアウト
        していた問題を、静止⇔スピンのサイクル方式に変更して解消。
    (2) それでも Acc・Gyro のレベルが上がりきらずタイムアウトする事例が
        あったため、今回さらに以下の2点を追加改良した。

    【今回の追加改良】
    ① 軸ごとに現実的な目標レベルを個別設定 (CALIB_MIN_LEVEL_*)。
       地上機は複数姿勢を取れないため Acc の高レベル到達は現実的に
       難しく、また静止直後はモータ停止の慣性余韻で完全な静止に
       なっていない場合があり Gyro も上がりにくいことがある。
       これらは Sys/Mag ほど高いレベルを要求せず (デフォルト 1)、
       比較的到達しやすい Mag は従来通り高め (デフォルト 2) のまま
       とすることで、無駄な足踏みで CALIB_TIMEOUT_SEC に達するのを防ぐ。
    ② スピン直後の「静止」区間の頭に CALIB_SETTLE_SEC 秒の
       セトリング(慣性が収まるのを待つだけの捨て時間)を追加。
       モータを止めた直後は機体がまだわずかに揺れており、その揺れの
       間を「静止」としてカウントしてしまうと Gyro/Acc の内部
       キャリブレーションアルゴリズムが正しく収束しないため。

    完了後 (または CALIB_TIMEOUT_SEC タイムアウト後)、モータを停止し、
    静止時の水平加速度ノルムを基準値として計測する。
    続けて GPS Fix の安定化・平均化 (phase3_gps_stabilize) を行い、
    誘導走行の開始位置を確認してから終了する。
    """
    global phase, g_acc, g_mag, g_gyro, g_calib
    phase = 3
    blink_led(led)
    log("─" * 62)

    min_level = {
        "Sys":  CALIB_MIN_LEVEL_SYS,
        "Gyro": CALIB_MIN_LEVEL_GYRO,
        "Acc":  CALIB_MIN_LEVEL_ACC,
        "Mag":  CALIB_MIN_LEVEL_MAG,
    }
    log(f"[Phase3] キャリブレーション開始 目標レベル: "
        f"Sys>={min_level['Sys']} Gyro>={min_level['Gyro']} "
        f"Acc>={min_level['Acc']} Mag>={min_level['Mag']}")
    log("[Phase3] 静止(セトリング+Acc/Gyro) ⇔ スピン(Mag) のサイクルを繰り返します")
    log_data_status("Phase3開始時点")

    p3_start = time.time()
    AXIS_NAMES = ("Sys", "Gyro", "Acc", "Mag")

    last_level        = {name: 0 for name in AXIS_NAMES}
    last_improve_time = {name: p3_start for name in AXIS_NAMES}
    last_report_time  = p3_start

    # ★追加: 静止フェーズの合計時間 = セトリング + 実質静止(計測)時間
    STILL_TOTAL_SEC = CALIB_SETTLE_SEC + CALIB_STILL_SEC

    state         = "STILL"     # "STILL" or "SPIN"
    next_spin_dir = "L"
    state_start   = p3_start
    motor.stop()
    motor_cmd = "STOP(settle)"
    log(f"[Phase3] STILL: セトリング{CALIB_SETTLE_SEC:.1f}s + 静止{CALIB_STILL_SEC:.0f}s (Acc/Gyro 安定化)")

    print(f"\n{'経過[s]':>8}  {'Sys':>4}  {'Gyro':>5}  {'Acc':>4}  {'Mag':>4}  {'状態':>12}  {'Motor':>16}")
    print("-" * 60)

    while True:
        try:
            g_acc   = bno.getAcc()
            g_mag   = bno.getMag()
            g_gyro  = bno.getGyro()
            g_calib = bno.getCalibrationStatus()
            mark_sensor_ok("BNO055", "キャリブレーション監視中")
        except Exception as e:
            mark_sensor_fail("BNO055", f"読み取りエラー: {e}")
            log(f"BNO055 エラー: {e}", "WARN")
            time.sleep(0.1)
            continue

        now     = time.time()
        elapsed = now - p3_start
        s, gy, ac, mg = g_calib
        cur = {"Sys": s, "Gyro": gy, "Acc": ac, "Mag": mg}

        # 停滞検知用: レベルが上がったら更新時刻をリセット
        for name in AXIS_NAMES:
            if cur[name] > last_level[name]:
                last_level[name] = cur[name]
                last_improve_time[name] = now

        # ── 静止(セトリング+計測) ⇔ スピン の状態遷移 ──
        state_elapsed = now - state_start
        if state == "STILL":
            motor_cmd = "STOP(settle)" if state_elapsed < CALIB_SETTLE_SEC else "STOP(Acc/Gyro)"
            if state_elapsed >= STILL_TOTAL_SEC:
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
                    log(f"[Phase3] {motor_cmd}: スピン {FIG8_SPIN_SEC:.0f} s (Mag 安定化)")
                else:
                    # FIG8 無効時は静止のまま (手動で回転させる運用を想定)
                    state_start = now
        elif state == "SPIN" and state_elapsed >= FIG8_SPIN_SEC:
            motor.stop()
            motor_cmd = "STOP(settle)"
            state, state_start = "STILL", now
            log(f"[Phase3] STILL: セトリング{CALIB_SETTLE_SEC:.1f}s + 静止{CALIB_STILL_SEC:.0f}s (Acc/Gyro 安定化)")

        # ── 定期進捗レポート ──
        if now - last_report_time >= CALIB_REPORT_INTERVAL_SEC:
            last_report_time = now
            log(f"[Phase3] 進捗: Sys={s} Gyro={gy} Acc={ac} Mag={mg}  "
                f"state={state}({motor_cmd})  経過={elapsed:.0f}s")
            log_data_status("キャリブレーション進捗判定 (BNO055 calib値使用)")

        # ── 軸別の停滞警告 ──
        for name in AXIS_NAMES:
            if cur[name] < min_level[name] and (now - last_improve_time[name]) >= CALIB_STALL_WARN_SEC:
                log(f"[Phase3][WARN] {name} 軸が {CALIB_STALL_WARN_SEC:.0f}s 以上レベル {cur[name]} "
                    f"(目標{min_level[name]}) のまま停滞しています", "WARN")
                last_improve_time[name] = now  # 連続警告を防ぐため基準時刻を更新

        all_ok = (s >= min_level["Sys"] and gy >= min_level["Gyro"]
                  and ac >= min_level["Acc"] and mg >= min_level["Mag"])

        if all_ok:
            status = "OK ✓"
        elif ac < min_level["Acc"]:
            status = "Acc 不足"
        elif mg < min_level["Mag"]:
            status = "Mag 不足"
        elif gy < min_level["Gyro"]:
            status = "Gyro 不足"
        else:
            status = "待機中..."

        print(f"{elapsed:>8.2f}  {s:>4d}  {gy:>5d}  {ac:>4d}  {mg:>4d}  {status:>12}  {motor_cmd:>16}", flush=True)

        if all_ok:
            log(f"[Phase3] キャリブレーション完了！ (所要 {elapsed:.1f} s)")
            break

        if elapsed > CALIB_TIMEOUT_SEC:
            log(f"[Phase3] タイムアウト ({CALIB_TIMEOUT_SEC:.0f}s)。現状 (Sys={s} Gyro={gy} Acc={ac} Mag={mg}) で続行します。", "WARN")
            break

        time.sleep(0.2)

    motor.stop()

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

    # ── GPS Fix 安定化・平均化 (緯度経度の調整) ──
    phase3_gps_stabilize(start_time)

    log("[Phase3] 誘導走行準備完了")
    return baseline_horiz_acc




# ─── Phase 4: GPS 誘導走行 + スタック検知・回避 ──────────────────────────────

def phase4_guided_run(bno: BNO055, sonar: SonarSensor,
                      motor: MotorController, led: LED, start_time: float,
                      baseline_horiz_acc: float) -> str:
    """
    GPS 誘導走行。目標まで PHASE4_TO_5_RADIUS 以内に入ったら Phase4 を終了し、
    戻り値 "NEAR_GOAL" を返して Phase5 (超音波による最終接近) へ引き継ぐ。
    タイムアウト時は "TIMEOUT"、中断時は "ABORT" を返す。

    ★変更: 機体の物理的な前方とセンサ方位のズレは、Phase0 で一度だけ
    実行する BNO055 の Axis Remap (set_bno_orientation, BNO_ORIENTATION)
    で補正済みであるため、Phase4 側で前方候補を探索する処理は行わない。
    方位計算は全て calc_azimuth_with_front() (EMA平滑化済みの地磁気から
    計算した方位) を使用する。
    """
    global phase, g_acc, g_mag, g_gyro, g_calib, g_sonar_m

    phase = 4
    blink_led(led)
    log("─" * 62)
    log(f"[Phase4] GPS 誘導走行開始")
    log(f"  目標: ({TARGET_LAT}, {TARGET_LNG})  Phase5移行半径: {PHASE4_TO_5_RADIUS} m")
    log(f"  BNO055 Axis Remap 設定: BNO_ORIENTATION={BNO_ORIENTATION} (Phase0 で適用済み)")
    log(f"  スタック閾値(★厳格化): 前進中の水平加速度 < {STUCK_HORIZON_ACCEL_THRESH} m/s² "
        f"かつ GPS速度 < {STUCK_GPS_SPEED_THRESH_MPS} m/s  or 超音波 < {STUCK_SONAR_DIST_THRESH} m")
    log(f"  スタック確定: {STUCK_COUNT_THRESHOLD} 回連続 ({STUCK_COUNT_THRESHOLD * LOOP_DT:.1f} s)")
    log_data_status("Phase4開始時点")

    # ★削除: Phase4限定のモータ反転フラグ (PHASE4_MOTOR_REVERSED) は撤去。
    #        モータは常に素直に forward=前進 / backward=後退として動作する。

    # ★追加: Phase4 (誘導走行・スタック回復) では、いかなる場合も
    #        モータを後退方向へ回転させてはならない。backward() が万一
    #        呼ばれても物理的に後退しないよう、ここで安全ロックを有効化する。
    motor.set_forward_only(True)
    log("[Phase4] ★安全ロック: 後退禁止モードを有効化しました "
        "(以降、backward()を呼んでも実際には後退しません)")

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
    near_goal    = False   # ★変更: GOAL_RADIUS到達→ミッション終了 ではなく Phase5 への移行フラグ
    timed_out    = False
    in_recovery  = False   # スタック回復中フラグ
    last_turn_dir = None   # 直前に旋回していた方向 ("L" or "R")。前進中などは更新しない
    last_nav_report_time = p4_start  # 目標距離・方位ずれのまとめログ用
    motor_cmd = "FORWARD"  # ★追加: スタック判定で「前進中か」を見るため、ループ開始前に初期化しておく

    def rotate_until_degrees(direction: str, target_deg: float, note_prefix: str) -> float:
        """
        機体前方基準の方位 (calc_azimuth_with_front) の変化量を積算し、
        指定方向へ target_deg 度回転するまで強旋回を続ける。
        メインループが継続的に更新しているグローバル g_mag を参照するだけで
        自らは BNO055 に触れない (メインループとの I2C 同時アクセスを回避するため)。
        方位が読めない/回転が進まない等の異常時に無限ループしないよう、
        STUCK_RECOVER_TURN_TIMEOUT_SEC で安全に打ち切る。
        戻り値: 実際に回転できた角度 (概算, 度)
        """
        if direction == "L":
            motor.turn_left_strong()
            note = f"{note_prefix}_L"
        else:
            motor.turn_right_strong()
            note = f"{note_prefix}_R"

        t_start   = time.time()
        prev_az   = calc_azimuth_with_front(g_mag)
        cumulative = 0.0
        while True:
            now = time.time()
            if now - t_start > STUCK_RECOVER_TURN_TIMEOUT_SEC:
                log(f"[Phase4][STUCK] {note}: 回転タイムアウト "
                    f"({STUCK_RECOVER_TURN_TIMEOUT_SEC:.0f}s, 約{cumulative:.1f}度で打ち切り)", "WARN")
                break
            cur_az = calc_azimuth_with_front(g_mag)
            delta  = calc_direction_diff(cur_az, prev_az)
            cumulative += delta
            prev_az = cur_az
            log_sensor_row(time.time() - start_time, note,
                           f"rot={cumulative:.1f}/{target_deg:.0f}deg")
            if abs(cumulative) >= target_deg:
                break
            time.sleep(0.1)

        motor.stop()
        log(f"[Phase4][STUCK] {note}: 回転完了 (目標{target_deg:.0f}度 / 実測約{cumulative:.1f}度)")
        return cumulative

    def forward_with_monitor(duration_sec: float, note_prefix: str) -> bool:
        """
        duration_sec 秒前進しつつ、その間も水平加速度・GPS速度・超音波による
        スタック判定を継続する (★通常走行時と同じ厳格化条件)。
        STUCK_COUNT_THRESHOLD 回連続でスタック条件を満たしたらその場で
        打ち切り True (再スタック) を返す。何も起きなければ False。
        グローバル g_acc / g_gps_speed / g_gps_valid / g_sonar_m
        (メインループが更新) を参照するだけで、自らはセンサに触れない。
        """
        motor.forward()
        t_end = time.time() + duration_sec
        local_stuck_count = 0
        restuck = False
        while time.time() < t_end:
            horiz_acc = math.sqrt(g_acc[0]**2 + g_acc[1]**2)
            gps_speed_mps = g_gps_speed * KNOTS_TO_MPS
            is_slow_gps    = (not g_gps_valid) or (gps_speed_mps < STUCK_GPS_SPEED_THRESH_MPS)
            is_stuck_acc   = is_slow_gps and (horiz_acc < (baseline_horiz_acc + STUCK_HORIZON_ACCEL_THRESH))
            is_stuck_sonar = (g_sonar_m is not None and g_sonar_m < STUCK_SONAR_DIST_THRESH)
            if is_stuck_acc or is_stuck_sonar:
                local_stuck_count += 1
            else:
                local_stuck_count = 0

            log_sensor_row(time.time() - start_time, f"{note_prefix}_FORWARD",
                           f"remain={t_end - time.time():.1f}s stuck_cnt={local_stuck_count}")

            if local_stuck_count >= STUCK_COUNT_THRESHOLD:
                restuck = True
                break
            time.sleep(0.1)

        motor.stop()
        return restuck

    def do_recovery():
        """
        スタック回復動作 (★改良版)。
        スタック検知の直前に旋回していた方向と逆方向へ、地磁気の方位変化から
        【ちょうど180°】回転したことを確認してから STUCK_RECOVER_FWD_SEC (10秒)
        前進する。この前進の最中も引き続きスタック判定を行い、再びスタックを
        検知した場合はさらに90°回転してから再度10秒前進する、という動作を
        STUCK_RECOVER_MAX_RETRIES 回まで繰り返す (無限ループ防止のため上限あり)。
        """
        nonlocal stuck_count, in_recovery
        in_recovery = True

        # 直前の旋回方向の反対を選択 (旋回していなければデフォルトで右旋回)
        if last_turn_dir == "L":
            recover_dir = "R"
        elif last_turn_dir == "R":
            recover_dir = "L"
        else:
            recover_dir = "R"

        log(f"[Phase4][STUCK] スタック検知！ 回復動作開始 (直前の旋回方向: {last_turn_dir or 'なし'})")
        log(f"         {'左' if recover_dir == 'L' else '右'}方向へ約{STUCK_RECOVER_TURN_DEG:.0f}° 回転"
            f" (直前と反対方向) → 前進 {STUCK_RECOVER_FWD_SEC:.0f} s")

        rotate_until_degrees(recover_dir, STUCK_RECOVER_TURN_DEG, "RECOVER180")

        cur_dir = recover_dir
        attempt = 0
        while True:
            attempt += 1
            restuck = forward_with_monitor(STUCK_RECOVER_FWD_SEC, f"RECOVER_ATT{attempt}")
            if not restuck:
                break
            if attempt >= STUCK_RECOVER_MAX_RETRIES:
                log(f"[Phase4][STUCK] 前進中に再スタックを検知しましたが、"
                    f"最大試行回数({STUCK_RECOVER_MAX_RETRIES})に達したため走行を再開します。", "WARN")
                break
            cur_dir = "R" if cur_dir == "L" else "L"
            log(f"[Phase4][STUCK] 前進中に再びスタックを検知 → "
                f"{'左' if cur_dir == 'L' else '右'}方向へ{STUCK_RECOVER_RETRY_TURN_DEG:.0f}° 回転して再度回避します "
                f"(試行 {attempt + 1}/{STUCK_RECOVER_MAX_RETRIES})")
            rotate_until_degrees(cur_dir, STUCK_RECOVER_RETRY_TURN_DEG, f"RECOVER90_{attempt}")

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
                mark_sensor_ok("BNO055", "航法演算用")
            except Exception as e:
                mark_sensor_fail("BNO055", f"読み取りエラー: {e}")
                log(f"BNO055 エラー: {e}", "WARN")
                g_acc = [0.0, 0.0, 0.0]
                g_mag = [0.0, 0.0, 0.0]

            # ── 超音波センサ取得 (失敗しても止めない) ──
            g_sonar_m = sonar.get_distance_m()
            sonar_str = f"{g_sonar_m:.3f}" if g_sonar_m is not None else "---"

            # ── 航法計算 (★変更: calc_azimuth → calc_azimuth_with_front) ──
            azimuth        = calc_azimuth_with_front(g_mag)
            lat, lng       = g_gps_lat, g_gps_lng
            distance       = calc_distance(lat, lng)
            target_bearing = calc_target_bearing(lat, lng)
            diff           = calc_direction_diff(azimuth, target_bearing)

            # ── 水平加速度ノルム (スタック検知用) ──
            horiz_acc = math.sqrt(g_acc[0]**2 + g_acc[1]**2)

            # ── ★追加: 目標までの距離・方位ずれのまとめ表示 ──
            # 9軸センサ(地磁気, 前方補正済み)から算出した機体方位と目標方位の
            # ずれ (diff) を NAV_REPORT_INTERVAL_SEC ごとにログへはっきり表示する。
            if loop_start - last_nav_report_time >= NAV_REPORT_INTERVAL_SEC:
                last_nav_report_time = loop_start
                turn_side = "右" if diff < 0 else ("左" if diff > 0 else "正面")
                log(f"[Phase4] 目標までの距離={distance:.2f} m  "
                    f"機体方位={azimuth:.1f}° (Axis Remap:{BNO_ORIENTATION})  "
                    f"目標方位={target_bearing:.1f}°  "
                    f"ずれ={diff:+.1f}° ({turn_side})")
                log_data_status("GPS誘導走行演算 (航法計算+スタック判定)")

            # ── ゴール判定 (★変更: ここではミッション終了ではなく Phase5 へ移行) ──
            if g_gps_valid and distance <= PHASE4_TO_5_RADIUS:
                log(f"[Phase4] 目標まで {PHASE4_TO_5_RADIUS} m 以内に到達！ distance={distance:.2f} m")
                log("[Phase4] Phase5 (超音波による最終接近) へ移行します。")
                near_goal = True
                break

            # ── スタック検知 (★厳格化) ────────────────────────────────────
            # 判定A: 水平加速度が基準より低い。ただし「前進」を指示している
            #        間のみ有効とする。旋回(その場スピン/弱旋回)中はもともと
            #        並進加速度が小さくなるため、以前はこれを誤って
            #        「動いていない=スタック」と判定していた (実運用で確認された誤検知)。
            is_forward_intent = (motor_cmd == "FORWARD")
            is_stuck_acc_raw  = horiz_acc < (baseline_horiz_acc + STUCK_HORIZON_ACCEL_THRESH)

            # 判定A': 加速度だけでなく GPS 対地速度も十分低いことを AND 条件で要求。
            #         振動などのノイズだけで加速度が一瞬下がっただけの誤検知を防ぐ。
            gps_speed_mps = g_gps_speed * KNOTS_TO_MPS
            is_slow_gps   = (not g_gps_valid) or (gps_speed_mps < STUCK_GPS_SPEED_THRESH_MPS)

            is_stuck_acc = is_forward_intent and is_stuck_acc_raw and is_slow_gps

            # 判定B: 超音波が近すぎる (壁に当たっている。旋回中でも常時有効)
            is_stuck_sonar = (g_sonar_m is not None and g_sonar_m < STUCK_SONAR_DIST_THRESH)

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

            # ── 直前の旋回方向をトラッキング (スタック回復時に反対方向へ回すため) ──
            # ★変更: apply_diff() が返す文字列に "TURN_R_STRONG(180°反転優先)" の
            #        ように末尾補足が付く場合があるため、完全一致 (in) ではなく
            #        接頭辞 (startswith) で判定するように変更した。
            if motor_cmd.startswith("TURN_L"):
                last_turn_dir = "L"
            elif motor_cmd.startswith("TURN_R"):
                last_turn_dir = "R"
            # FORWARD / GPS_WAIT / STUCK_STOP / RECOVERING の場合は前回の値を維持する

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
        # ★追加: Phase4限定の後退禁止ロックを解除 (Phase5などに影響させないため)
        motor.set_forward_only(False)
        log("[Phase4] 安全ロック: 後退禁止モードを解除しました")
        print()
        log("─" * 62)
        if near_goal:
            log("[Phase4] 結果: Phase5 へ移行")
        elif timed_out:
            log("[Phase4] 結果: タイムアウト")
        else:
            log("[Phase4] 結果: 中断")
        log(f"[Phase4] 最終距離: {calc_distance(g_gps_lat, g_gps_lng):.2f} m")

    if near_goal:
        return "NEAR_GOAL"
    elif timed_out:
        return "TIMEOUT"
    else:
        return "ABORT"


# ─── Phase 5: 超音波センサによる最終接近 (★変更: 左右スキャン方式) ───────────

def phase5_final_approach(sonar: SonarSensor, motor: MotorController,
                          led: LED, start_time: float) -> bool:
    """
    Phase4 で目標まで PHASE4_TO_5_RADIUS 以内に入った後の最終接近フェーズ。

    GPS はこれ以上の精度で距離を詰められないため、前方の超音波センサで
    ゴールに設置された物体との距離を直接測りながら近づく。

    ★変更: 従来は超音波の値を判断に使わず、turn_left_weak/turn_right_weak
    を機械的に交互切り替えするだけの蛇行だったが、これを「超音波センサの
    値に基づいて能動的に対象物へ向かう」方式に変更した。1サイクルは
    以下の手順で構成される:

      1. turn_left_strong() で FINAL_SCAN_SEC 秒間左へ旋回しながら、
         その間に得られた超音波距離の最小値を dist_left として記録する
         (指向性のあるセンサが対象物に近い方向を向いたときほど、
          距離が短く読めるという前提)。
      2. turn_right_strong() で FINAL_SCAN_SEC 秒間右へ旋回しながら、
         同様に dist_right を記録する (これで一旦、旋回前の向きに近い
         状態へ戻ることになる)。
      3. dist_left と dist_right を比較し、より近かった方向 (対象物が
         その方向にある可能性が高い) へ turn_left_weak / turn_right_weak
         (両輪前進しつつ片側だけ弱める動き) で FINAL_SCAN_SEC 秒間だけ
         向き直しながら前進する。どちらも無効値ならとりあえず forward()
         で前進する。
      4. 1 に戻って繰り返す。

    スキャン中および接近中、いずれかの時点で超音波距離が
    FINAL_STOP_DIST_M (0.05 m) 以下になったら「ゴールに0距離まで到達」
    とみなして即座に停止・終了する。
    安全のため FINAL_APPROACH_TIMEOUT_SEC で強制終了し、超音波が長時間
    有効値を返さない場合は警告を出す。

    ※ BNO055 の Axis Remap (Phase0 で設定済み) による前方補正は、
       ログ表示用の参考ナビ情報 (calc_azimuth_with_front) にも自動的に
       反映される。

    戻り値: True = 目標到達, False = タイムアウト/中断
    """
    global phase, g_sonar_m
    phase = 5
    blink_led(led)
    log("─" * 62)
    log("[Phase5] 超音波センサによる左右スキャン接近を開始します")
    log(f"  停止距離: {FINAL_STOP_DIST_M:.2f} m 以下で到達  "
        f"スキャン: 左右それぞれ {FINAL_SCAN_SEC:.1f} s  "
        f"タイムアウト: {FINAL_APPROACH_TIMEOUT_SEC:.0f} s")
    log_data_status("Phase5開始時点")

    HDR = (f"{'T[s]':>7}  {'Lat':>11}  {'Lng':>12}  "
           f"{'Dist[m]':>8}  {'Bear[°]':>7}  {'Az[°]':>6}  "
           f"{'Diff[°]':>7}  {'SonL[m]':>8}  {'SonR[m]':>8}  {'Motor':>16}")
    DAT = (f"{{:>7.2f}}  {{:>11.6f}}  {{:>12.6f}}  "
           f"{{:>8.2f}}  {{:>7.2f}}  {{:>6.2f}}  "
           f"{{:>7.2f}}  {{:>8}}  {{:>8}}  {{:>16}}")
    print()
    print("-" * len(HDR))
    print(HDR)
    print("-" * len(HDR))

    p5_start = time.time()
    last_nav_report_time  = p5_start
    last_sonar_valid_time = p5_start
    reached  = False
    timed_out = False

    def check_reached(dist) -> bool:
        """★追加: 距離が FINAL_STOP_DIST_M 以下なら到達とみなす共通判定。"""
        return dist is not None and dist <= FINAL_STOP_DIST_M

    def scan_direction(direction: str, note_prefix: str):
        """
        ★追加: 指定方向へ FINAL_SCAN_SEC 秒間強旋回しながら超音波を
        サンプリングし、その間に得られた最小距離を返す。
        途中で到達判定を満たした場合は即座に停止して "REACHED" を返す
        (呼び出し元はこれを見て全体を打ち切る)。
        戻り値: (最小距離 or None, 到達したか True/False)
        """
        nonlocal last_sonar_valid_time
        global g_sonar_m
        if direction == "L":
            motor.turn_left_strong()
            motor_cmd = f"{note_prefix}_SCAN_L"
        else:
            motor.turn_right_strong()
            motor_cmd = f"{note_prefix}_SCAN_R"

        best = None
        t_end = time.time() + FINAL_SCAN_SEC
        while time.time() < t_end:
            g_sonar_m = sonar.get_distance_m()
            now = time.time()
            if g_sonar_m is not None:
                last_sonar_valid_time = now
                if best is None or g_sonar_m < best:
                    best = g_sonar_m
            log_sensor_row(now - start_time, motor_cmd,
                           f"best={best if best is not None else '---'}")
            if check_reached(g_sonar_m):
                motor.stop()
                log(f"[Phase5] スキャン中に目標到達！ ({direction}方向, "
                    f"前方オブジェクトまで {g_sonar_m:.3f} m)")
                return best, True
            time.sleep(FINAL_SCAN_LOOP_DT)

        return best, False

    try:
        while True:
            now = time.time()
            elapsed = now - p5_start

            if elapsed > FINAL_APPROACH_TIMEOUT_SEC:
                log(f"[Phase5] タイムアウト ({FINAL_APPROACH_TIMEOUT_SEC:.0f} s)", "WARN")
                timed_out = True
                break

            if now - last_sonar_valid_time >= FINAL_SONAR_LOST_WARN_SEC:
                log(f"[Phase5][WARN] 超音波センサが {FINAL_SONAR_LOST_WARN_SEC:.0f}s "
                    f"以上有効値を返していません。前方に物体が無いか、センサ不調の可能性があります。", "WARN")
                last_sonar_valid_time = now  # 再警告を連発しないよう基準を更新

            # ── 1. 左スキャン ──
            dist_left, hit = scan_direction("L", "P5")
            if hit:
                reached = True
                break

            # ── 2. 右スキャン ──
            dist_right, hit = scan_direction("R", "P5")
            if hit:
                reached = True
                break

            # ── 3. 近かった方向を判定し、その方向へ向き直しながら前進 ──
            if dist_left is not None and (dist_right is None or dist_left <= dist_right):
                favored, favored_str = "L", f"{dist_left:.3f}"
                motor.turn_left_weak()
                motor_cmd = "APPROACH_L"
            elif dist_right is not None:
                favored, favored_str = "R", f"{dist_right:.3f}"
                motor.turn_right_weak()
                motor_cmd = "APPROACH_R"
            else:
                favored, favored_str = "-", "---"
                motor.forward()
                motor_cmd = "APPROACH_FWD"

            t_end = time.time() + FINAL_SCAN_SEC
            while time.time() < t_end:
                g_sonar_m = sonar.get_distance_m()
                tnow = time.time()
                if g_sonar_m is not None:
                    last_sonar_valid_time = tnow
                log_sensor_row(tnow - start_time, motor_cmd,
                               f"favored={favored}")
                if check_reached(g_sonar_m):
                    motor.stop()
                    log(f"[Phase5] 接近中に目標到達！ 前方オブジェクトまで "
                        f"{g_sonar_m:.3f} m")
                    reached = True
                    break
                time.sleep(FINAL_SCAN_LOOP_DT)
            if reached:
                break

            # ── 参考表示用の GPS/地磁気ナビ情報 (最終判定には使わない) ──
            lat, lng       = g_gps_lat, g_gps_lng
            distance       = calc_distance(lat, lng)
            azimuth        = calc_azimuth_with_front(g_mag)
            target_bearing = calc_target_bearing(lat, lng)
            diff           = calc_direction_diff(azimuth, target_bearing)

            sonl_str = f"{dist_left:.3f}" if dist_left is not None else "---"
            sonr_str = f"{dist_right:.3f}" if dist_right is not None else "---"
            row = DAT.format(elapsed, lat, lng, distance, target_bearing,
                             azimuth, diff, sonl_str, sonr_str,
                             f"{motor_cmd}(→{favored}:{favored_str})")
            print(row, flush=True)

            if now - last_nav_report_time >= FINAL_NAV_REPORT_INTERVAL_SEC:
                last_nav_report_time = now
                turn_side = "右" if diff < 0 else ("左" if diff > 0 else "正面")
                log(f"[Phase5] 目標までの距離(GPS)={distance:.2f} m  "
                    f"左スキャン距離={sonl_str} m  右スキャン距離={sonr_str} m  "
                    f"→ {favored}方向へ接近  "
                    f"機体方位={azimuth:.1f}°  目標方位={target_bearing:.1f}°  "
                    f"ずれ={diff:+.1f}° ({turn_side})")
                log_data_status("最終接近演算 (左右スキャン超音波距離を主判定に使用)")

    except KeyboardInterrupt:
        log("[Phase5] Ctrl+C 受信 → 緊急停止")

    finally:
        motor.stop()
        print()
        log("─" * 62)
        if reached:
            log("[Phase5] 結果: 目標到達 (ミッション完了)")
        elif timed_out:
            log("[Phase5] 結果: タイムアウト")
        else:
            log("[Phase5] 結果: 中断")

    return reached


# ===========================================================================
# メイン
# ===========================================================================

def main():
    # Phase 0: 初期化
    bno, bmp, base_pressure, sonar, motor, led, log_path = phase0_init()
    start_time = time.time()

    try:
        # Phase 1: 落下検知
        fall_ok = phase1_fall_detection(bno, led, start_time)
        if fall_ok:
            log("落下確認。次フェーズへ移行します。")
        else:
            log("タイムアウトのため強制移行します。", "WARN")

        # Phase 2: 初期後退 (30 s) → 停止 (5 s) → 前進 (10 s)
        phase2_initial_backward(motor, sonar, led, start_time)

        # Phase 3: キャリブレーション (静止⇔スピン サイクル) & GPS 安定化
        baseline_horiz_acc = phase3_calibration(bno, motor, led, start_time)

        # Phase 4: GPS 誘導走行 + スタック検知 (前方はPhase0のAxis Remapで補正済み)
        phase4_result = phase4_guided_run(bno, sonar, motor, led, start_time, baseline_horiz_acc)

        # Phase 5: 目標付近 (PHASE4_TO_5_RADIUS 以内) まで来たら超音波で最終接近
        if phase4_result == "NEAR_GOAL":
            phase5_final_approach(sonar, motor, led, start_time)
        else:
            log(f"[Main] Phase4 の結果が {phase4_result} のため Phase5 は実行しません。", "WARN")

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
