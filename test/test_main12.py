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
                キャリブレーション後、静止基準加速度(スタック検知用)を測定し、
                続けて GPS Fix の安定化・平均化 (衛星数チェック + 複数サンプル
                平均) を行い、誘導走行開始時点での距離・方位を確認する。

    Phase 4 : GPS 誘導走行 (目標地点 PHASE4_TO_5_RADIUS = 2 m 以内まで)
                GPS + 地磁気で目標座標を追跡し前進。
                【安全ロック】Phase4 の開始から終了まで、モータの
                後退方向への回転を絶対に禁止する安全ロックを有効化する。
                backward() が万一 (バグ・将来の改修等で) 呼ばれても、
                MotorController 側で物理的に後退させず停止のみ行う
                (Phase4終了時にロックは解除され、以降のフェーズには影響しない)。
                【改良】Phase4 開始直後に「前方探索 (Front Search)」を行う。
                BNO055 の地磁気ゼロ点(方位0°の向き)と機体の物理的な前方は
                取り付け向き・個体差等で必ずしも一致しないため、前方候補
                1〜4 (センサ方位に対するオフセット 0°/90°/180°/270°) を
                順に短時間前進させて GPS 距離の変化を確認し、実際に目標へ
                近づけた候補を「正しい前方」として採用してから通常の
                誘導走行に入る。
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

    Phase 5 : 超音波センサによる最終接近 (★追加)
                GPS 精度ではこれ以上距離を詰められないため、前方の超音波
                センサでゴールに設置された物体との距離を直接測定しながら
                近づく。turn_left_weak / turn_right_weak (両輪前進しつつ
                片側だけ弱める動き) を FINAL_WIGGLE_SEC (0.1秒) ごとに
                左右交互に切り替えることで、蛇行しながら前進を続ける。
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

# --- Phase 4: 前方探索 (Front Search) (★追加) --------------------------------
# BNO055 の地磁気ゼロ点(方位0°の向き)と機体の物理的な前方が、
# センサの取り付け向き・個体差・配線都合等で必ずしも一致しない場合がある。
# そこで「前方候補」を4つ (センサ方位に対するオフセット 0°/90°/180°/270°)
# グローバルに定義しておき、Phase4 開始直後にそれぞれを短時間前進させて
# GPS 上で実際に目標へ近づいたかどうかを確認することで、正しい前方を
# 自動的に探索・確定する。確定後はそのオフセットを全ての方位計算
# (通常走行 + スタック回復の回転) に適用する。
FRONT_CANDIDATES_DEG = {
    1: 0.0,     # センサの0°方向がそのまま前方
    2: 90.0,    # センサ0°から時計回りに90°回した方向が前方
    3: 180.0,   # センサ0°の真後ろが前方
    4: 270.0,   # センサ0°から反時計回りに90°回した方向が前方
}
FRONT_SEARCH_TRIAL_SEC     = 6.0   # [s] 各候補を試す前進時間
FRONT_SEARCH_SETTLE_SEC    = 0.5   # [s] 前進停止後、GPS距離を測る前の待機 (慣性収まり待ち)
FRONT_SEARCH_RETRY_WAIT_SEC = 1.0  # [s] 次の候補を試す前の待機
FRONT_SEARCH_MIN_IMPROVE_M = 0.5   # [m] この距離以上目標に近づいたら候補確定
FRONT_SEARCH_DEFAULT_CANDIDATE = 1  # 全候補で改善が確認できなかった場合のデフォルト候補

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
# 現在のデューティ比に対する倍率 (WEAK_RATIO) として定義しておく。
SPEED_WEAK  = 0.4                          # [-] 従来の弱旋回側 duty (比率計算の元値として保持)
WEAK_RATIO  = SPEED_WEAK / RAMP_START_DUTY  # [-] 強旋回側に対する弱旋回側の出力比率 (デフォルト 0.5)

# ★変更: 誘導走行フェーズ (Phase4) だけモータの IN/OUT (forward/backward) を
#        一時的に反転させるフラグ。配線都合等で forward 指示が実際には
#        後退になってしまう場合の暫定対応。Phase2/Phase3/Phase5 には影響しない。
#        ★実地確認により、従来の反転設定 (True) は逆だったため False へ変更。
PHASE4_MOTOR_REVERSED = False

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

# ★追加: 現在採用中の前方オフセット [度]。calc_azimuth() の結果へ加算することで
#        「機体の物理的な前方」を基準にした方位を得る。値は set_front_candidate()
#        で 1〜4 の候補から選択して更新する (デフォルトは候補1 = オフセット0°)。
g_front_offset_deg = FRONT_CANDIDATES_DEG[FRONT_SEARCH_DEFAULT_CANDIDATE]
g_front_candidate  = FRONT_SEARCH_DEFAULT_CANDIDATE   # 現在採用中の候補番号 (1〜4)

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

# ★追加: フェーズ番号 → 日本語名。ログに現在何をしているフェーズかを
#        ひと目でわかるように併記するために使う。
PHASE_NAMES = {
    0: "初期化",
    1: "落下検知",
    2: "初期後退・停止・前進",
    3: "キャリブレーション/GPS安定化",
    4: "GPS誘導走行",
    5: "超音波最終接近",
}

# ★追加: センサ別のデータ取得状況トラッキング ──────────────────────────
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
        # ★追加: モータドライバの IN/OUT (forward/backward) を一時的に
        #        反転させるためのフラグ。配線や個体差で forward 指示が
        #        実際には後退になってしまう場合の暫定対応として、
        #        set_reversed() で特定フェーズだけ有効化することを想定。
        self._reversed = False

        # ★追加: 後退方向への回転を絶対に禁止する安全ロック。
        #        True の間は backward() が呼ばれても実際には後退させず、
        #        警告ログを出して停止のみ行う。Phase4 (前方探索・誘導走行・
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

    def set_reversed(self, reversed_: bool):
        """
        ★追加: モータの forward/backward (IN/OUT) をこの呼び出し以降
        一時的に反転させる (True) / 通常に戻す (False)。
        forward/backward/turn_* 系メソッドすべてに一貫して反映される。
        """
        state = "反転" if reversed_ else "通常"
        log(f"モータ方向を{state}モードに設定 (reversed={reversed_})")
        self._reversed = reversed_

    def set_forward_only(self, enabled: bool):
        """
        ★追加: 後退方向への回転を絶対に禁止する安全ロックを有効/無効にする。
        有効化 (True) すると、以降 backward() が呼ばれても実際には
        モータを後退方向へ一切回転させず、警告ログを出して stop() のみ
        行う (＝呼び出し元がミスをしても物理的に後退できない)。
        Phase4 (前方探索・誘導走行・スタック回復) のように、後退動作が
        絶対に許されないフェーズの開始時に True、終了時に False へ戻す
        使い方を想定している。
        """
        state = "有効(backward()は無効化)" if enabled else "無効(通常通り)"
        log(f"[Motor] 後退禁止モードを{state}に設定")
        self._forward_only = enabled

    # ── 反転フラグを考慮した低レベルヘルパー ──
    def _a_fwd(self):
        (self._mot_a.backward if self._reversed else self._mot_a.forward)()

    def _a_back(self):
        (self._mot_a.forward if self._reversed else self._mot_a.backward)()

    def _b_fwd(self):
        (self._mot_b.backward if self._reversed else self._mot_b.forward)()

    def _b_back(self):
        (self._mot_b.forward if self._reversed else self._mot_b.backward)()

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
        self._a_fwd()
        self._b_fwd()

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
        self._a_back()
        self._b_back()

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
        self._a_fwd()
        self._mot_b.stop()

    def turn_right_strong(self):
        """右モータ停止 / 左モータ前進 → 右旋回"""
        self._start_move(0.0, 1.0)
        self._mot_a.stop()
        self._b_fwd()

    def turn_left_weak(self):
        self._start_move(1.0, WEAK_RATIO)
        self._a_fwd()
        self._b_fwd()

    def turn_right_weak(self):
        self._start_move(WEAK_RATIO, 1.0)
        self._a_fwd()
        self._b_fwd()

    def apply_diff(self, diff: float) -> str:
        """
        diff (−180〜+180) でモータ指令を自動選択。
        test_GPSrun.py の MotorController.apply_diff() と同じロジック。
