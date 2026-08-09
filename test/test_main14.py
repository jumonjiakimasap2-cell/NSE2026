# -*- coding: utf-8 -*-
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
    """センサ全値を 1 行ログとしてバッファに追記し、SSHコンソールに出力する。"""
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
        
    # SSHコンソールへのリアルタイムステータス出力 (従来のprintデバッグの強化版)
    try:
        azimuth = calc_azimuth_with_front()
    except Exception:
        azimuth = 0.0
        
    dist_str = "N/A"
    bearing_str = "N/A"
    if g_gps_valid:
        try:
            dist = haversine_distance(g_gps_lat, g_gps_lng, TARGET_LAT, TARGET_LNG)
            bearing = calculate_bearing(g_gps_lat, g_gps_lng, TARGET_LAT, TARGET_LNG)
            dist_str = f"{dist:.2f}m"
            bearing_str = f"{bearing:.1f}°"
        except Exception:
            pass
            
    sonar_str = f"{g_sonar_m:.2f}m" if g_sonar_m is not None else "None"
    
    print(f"[MONITOR] Time:{elapsed:.1f}s | Phase:{phase}({PHASE_NAMES.get(phase, '?')}) | "
          f"GPS:({g_gps_lat:.6f},{g_gps_lng:.6f}) Sats:{g_gps_sats} | "
          f"Dist:{dist_str} | Bear:{bearing_str} | Azim:{azimuth:.1f}° | "
          f"Sonar:{sonar_str} | Motor:{motor_cmd} | Note:{note}", flush=True)


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
        """
        if abs(diff) < ANGLE_DEADBAND:
            self.forward()
            return "FORWARD"
        elif diff > ANGLE_TURN_STRONG:
            self.turn_left_strong()
            return "TURN_L_STRONG"
        elif diff < -ANGLE_TURN_STRONG:
            self.turn_right_strong()
            return "TURN_R_STRONG"
        elif diff > 0:
            self.turn_left_weak()
            return "TURN_L_WEAK"
        else:
            self.turn_right_weak()
            return "TURN_R_WEAK"

    def close(self):
        """GPIO リソースを安全に解放する。"""
        self._closed = True
        self.stop()
        self._pwm_a.close()
        self._pwm_b.close()
        self._mot_a.close()
        self._mot_b.close()
        self._stby.close()
        log("モータコントローラのリソースを解放しました")

# ===========================================================================
# 地理的計算 (Haversine 距離 / 大圏初期方位角)
# ===========================================================================

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """2点の緯度・経度から Haversine 式を用いて距離 [m] を計算する。"""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    
    a = math.sin(dphi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0)**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS * c

def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """2点の緯度・経度から目標方位角 [度] (北=0°, 東=90°) を計算する。"""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    
    bearing = math.degrees(math.atan2(y, x))
    return bearing % 360.0

# ===========================================================================
# センサ取得スレッド & データ取得関数
# ===========================================================================

# グローバル制御フラグ
g_closed = False
baseline_horiz_acc = 0.0

def sensor_thread_func(bno: BNO055, bmp: BMP180, sonar: SonarSensor):
    """BNO055 と BMP180 からデータを定期取得するスレッド。"""
    global g_acc, g_mag, g_gyro, g_calib, g_temp, g_pressure, g_altitude, g_sonar_m
    loop_counter = 0
    while not g_closed:
        # --- BNO055 ---
        try:
            acc = bno.getAcc()
            gyro = bno.getGyro()
            mag = bno.getMag()
            calib = bno.getCalibrationStatus()
            
            if isinstance(acc, list) and len(acc) >= 3:
                g_acc = acc
            if isinstance(gyro, list) and len(gyro) >= 3:
                g_gyro = gyro
            if isinstance(mag, list) and len(mag) >= 3:
                g_mag = mag
            if isinstance(calib, tuple) and len(calib) >= 4:
                g_calib = calib
                
            mark_sensor_ok("BNO055", f"sys={g_calib[0]} gyro={g_calib[1]} acc={g_calib[2]} mag={g_calib[3]}")
        except Exception as e:
            mark_sensor_fail("BNO055", f"エラー: {e}")
            
        # --- BMP180 ---
        try:
            temp = bmp.getTemperature()
            pres = bmp.getPressure()
            if temp is not None:
                g_temp = float(temp)
            if pres is not None:
                g_pressure = float(pres)
                g_altitude = 44330.0 * (1.0 - math.pow(g_pressure / 101325.0, 1.0 / 5.255))
        except Exception as e:
            pass
            
        # --- Sonar (5Hz) ---
        if loop_counter % 4 == 0:
            try:
                dist = sonar.get_distance_m()
                g_sonar_m = dist
            except Exception as e:
                g_sonar_m = None
                
        loop_counter += 1
        time.sleep(0.05)  # 20Hz

def calc_azimuth_with_front() -> float:
    """地磁気 EMA 平滑化 & 前方オフセット反映済みの機体方位角 [度] を計算する。"""
    global g_mag_ema
    current_mag = list(g_mag)
    
    if g_mag_ema is None:
        g_mag_ema = current_mag
    else:
        for i in range(3):
            g_mag_ema[i] = MAG_EMA_ALPHA * current_mag[i] + (1.0 - MAG_EMA_ALPHA) * g_mag_ema[i]
            
    raw_azimuth = math.degrees(math.atan2(g_mag_ema[1], g_mag_ema[0])) - 90.0
    azimuth = (raw_azimuth + MAG_DECLINATION + g_front_offset_deg) % 360.0
    return azimuth

def get_gps_average(samples: int = 5) -> tuple[float, float]:
    """GPSの緯度・経度を数点取得して平均値を返す。"""
    lats, lngs = [], []
    start_time = time.time()
    while len(lats) < samples and time.time() - start_time < 5.0:
        if g_gps_valid:
            lats.append(g_gps_lat)
            lngs.append(g_gps_lng)
        time.sleep(0.2)
    if lats:
        return sum(lats) / len(lats), sum(lngs) / len(lngs)
    return g_gps_lat, g_gps_lng

# ===========================================================================
# Phase 1: 落下検知
# ===========================================================================

def run_phase_1(motor: MotorController, log_path: Path):
    global phase
    phase = 1
    log("Phase 1 (落下検知) 開始")
    
    start_time = time.time()
    fall_count = 0
    last_save_time = time.time()
    
    while True:
        x, y, z = g_acc
        norm = math.sqrt(x*x + y*y + z*z)
        
        if norm <= FALL_THRESHOLD:
            fall_count += 1
        else:
            fall_count = 0
            
        elapsed = time.time() - start_time
        log_sensor_row(elapsed, "STOP", f"norm={norm:.2f} count={fall_count}/{FALL_COUNT_THRESHOLD}")
        
        if time.time() - last_save_time > 5.0:
            save_log(log_path)
            last_save_time = time.time()
            
        if fall_count >= FALL_COUNT_THRESHOLD:
            log(f"落下検知成功: 加速度ノルム {norm:.2f} m/s² が {FALL_COUNT_THRESHOLD} 回連続でしきい値以下を維持")
            break
            
        if elapsed > FALL_TIMEOUT_SEC:
            log("落下検知タイムアウト: 安全のため Phase 2 へ移行します", "WARN")
            break
            
        time.sleep(0.05)

# ===========================================================================
# Phase 2: 超音波分離検知後退 → 停止 → 前進
# ===========================================================================

def run_phase_2(motor: MotorController, log_path: Path):
    global phase
    phase = 2
    log("Phase 2 (分離検知後退) 開始")
    
    motor.backward()
    
    start_time = time.time()
    last_valid_dist = None
    last_sonar_ok_time = time.time()
    last_save_time = time.time()
    jump_detected = False
    
    while True:
        now = time.time()
        elapsed = now - start_time
        
        sonar_val = g_sonar_m
        note = ""
        
        if sonar_val is not None:
            last_sonar_ok_time = now
            if last_valid_dist is not None:
                diff = sonar_val - last_valid_dist
                note = f"sonar={sonar_val:.2f}m diff={diff:.2f}m"
                if elapsed >= PHASE2_BACK_MIN_SEC and diff >= PHASE2_SONAR_JUMP_THRESH_M:
                    log(f"分離検知: 距離急増 (直近={last_valid_dist:.2f}m -> 現在={sonar_val:.2f}m, 差={diff:.2f}m >= しきい値={PHASE2_SONAR_JUMP_THRESH_M}m)")
                    jump_detected = True
            else:
                note = f"sonar={sonar_val:.2f}m first_valid"
            last_valid_dist = sonar_val
        else:
            lost_duration = now - last_sonar_ok_time
            note = f"sonar=None lost={lost_duration:.1f}s"
            if lost_duration > PHASE2_SONAR_LOST_TIMEOUT_SEC:
                log(f"超音波値ロスト警告 (後退中): {lost_duration:.1f}秒間値がありません", "WARN")
                last_sonar_ok_time = now - (PHASE2_SONAR_LOST_TIMEOUT_SEC - 1.0)
                
        log_sensor_row(elapsed, "BACKWARD", note)
        
        if now - last_save_time > 5.0:
            save_log(log_path)
            last_save_time = now
            
        if jump_detected:
            break
            
        if elapsed > PHASE2_BACK_TIMEOUT_SEC:
            log(f"後退タイムアウト: 安全のため {PHASE2_BACK_TIMEOUT_SEC}秒 で後退を打ち切ります", "WARN")
            break
            
        time.sleep(PHASE2_LOOP_DT)
        
    # Stop for 5 seconds
    log(f"後退完了 -> {PHASE2_PAUSE_SEC}秒間停止")
    motor.stop()
    pause_start = time.time()
    while time.time() - pause_start < PHASE2_PAUSE_SEC:
        elapsed = time.time() - start_time
        log_sensor_row(elapsed, "STOP", "Phase2 pause")
        time.sleep(0.1)
        
    # Forward for 10 seconds
    log(f"停止完了 -> サブキャリア離脱のため {PHASE2_FWD_AFTER_SEC}秒間前進")
    motor.forward()
    fwd_start = time.time()
    while time.time() - fwd_start < PHASE2_FWD_AFTER_SEC:
        elapsed = time.time() - start_time
        log_sensor_row(elapsed, "FORWARD", "Phase2 forward after back")
        time.sleep(0.1)
        
    motor.stop()
    log("Phase 2 完了")

# ===========================================================================
# Phase 3: キャリブレーション & 誘導走行準備
# ===========================================================================

def run_phase_3(motor: MotorController, log_path: Path):
    global phase, baseline_horiz_acc
    phase = 3
    log("Phase 3 (キャリブレーション & GPS安定化) 開始")
    
    start_time = time.time()
    last_report_time = start_time
    last_calib_change_time = start_time
    last_calib_levels = (0, 0, 0, 0)
    spin_left = True
    
    while True:
        sys_c, gyro_c, acc_c, mag_c = g_calib
        
        if (sys_c >= CALIB_MIN_LEVEL_SYS and 
            gyro_c >= CALIB_MIN_LEVEL_GYRO and 
            acc_c >= CALIB_MIN_LEVEL_ACC and 
            mag_c >= CALIB_MIN_LEVEL_MAG):
            log(f"キャリブレーション目標達成: Sys={sys_c}, Gyro={gyro_c}, Acc={acc_c}, Mag={mag_c}")
            break
            
        elapsed = time.time() - start_time
        if elapsed > CALIB_TIMEOUT_SEC:
            log("キャリブレーションタイムアウト: 目標未達ですが次に進みます", "WARN")
            break
            
        current_calib = (sys_c, gyro_c, acc_c, mag_c)
        if current_calib != last_calib_levels:
            last_calib_levels = current_calib
            last_calib_change_time = time.time()
        else:
            if time.time() - last_calib_change_time > CALIB_STALL_WARN_SEC:
                stalled = []
                if sys_c < CALIB_MIN_LEVEL_SYS: stalled.append(f"Sys({sys_c}<{CALIB_MIN_LEVEL_SYS})")
                if gyro_c < CALIB_MIN_LEVEL_GYRO: stalled.append(f"Gyro({gyro_c}<{CALIB_MIN_LEVEL_GYRO})")
                if acc_c < CALIB_MIN_LEVEL_ACC: stalled.append(f"Acc({acc_c}<{CALIB_MIN_LEVEL_ACC})")
                if mag_c < CALIB_MIN_LEVEL_MAG: stalled.append(f"Mag({mag_c}<{CALIB_MIN_LEVEL_MAG})")
                log(f"キャリブレーション停滞警告 ({CALIB_STALL_WARN_SEC:.0f}秒変化なし): 未達={', '.join(stalled)}", "WARN")
                last_calib_change_time = time.time()
                
        if time.time() - last_report_time > CALIB_REPORT_INTERVAL_SEC:
            log(f"キャリブレーション進捗: Sys={sys_c}/{CALIB_MIN_LEVEL_SYS}, Gyro={gyro_c}/{CALIB_MIN_LEVEL_GYRO}, Acc={acc_c}/{CALIB_MIN_LEVEL_ACC}, Mag={mag_c}/{CALIB_MIN_LEVEL_MAG}")
            last_report_time = time.time()
            
        # --- SPIN SUB-LOOP ---
        if FIG8_ENABLE:
            if spin_left:
                log(f"スピン動作: 左旋回開始 ({FIG8_SPIN_SEC}秒間)")
                motor.turn_left_strong()
            else:
                log(f"スピン動作: 右旋回開始 ({FIG8_SPIN_SEC}秒間)")
                motor.turn_right_strong()
            spin_left = not spin_left
            
            spin_start = time.time()
            while time.time() - spin_start < FIG8_SPIN_SEC:
                sys_c, gyro_c, acc_c, mag_c = g_calib
                if (sys_c >= CALIB_MIN_LEVEL_SYS and 
                    gyro_c >= CALIB_MIN_LEVEL_GYRO and 
                    acc_c >= CALIB_MIN_LEVEL_ACC and 
                    mag_c >= CALIB_MIN_LEVEL_MAG):
                    break
                if time.time() - start_time > CALIB_TIMEOUT_SEC:
                    break
                log_sensor_row(time.time() - start_time, "SPIN", f"calib={g_calib}")
                time.sleep(0.1)
                
            motor.stop()
            if (g_calib[0] >= CALIB_MIN_LEVEL_SYS and 
                g_calib[1] >= CALIB_MIN_LEVEL_GYRO and 
                g_calib[2] >= CALIB_MIN_LEVEL_ACC and 
                g_calib[3] >= CALIB_MIN_LEVEL_MAG):
                break
            if time.time() - start_time > CALIB_TIMEOUT_SEC:
                break
                
        # --- STILL SUB-LOOP ---
        log("静止動作: セトリング待ち開始")
        motor.stop()
        settle_start = time.time()
        while time.time() - settle_start < CALIB_SETTLE_SEC:
            log_sensor_row(time.time() - start_time, "STOP_SETTLE", f"settling={time.time()-settle_start:.1f}s")
            time.sleep(0.1)
            
        log(f"静止動作: 計測開始 ({CALIB_STILL_SEC}秒間)")
        still_start = time.time()
        while time.time() - still_start < CALIB_STILL_SEC:
            sys_c, gyro_c, acc_c, mag_c = g_calib
            if (sys_c >= CALIB_MIN_LEVEL_SYS and 
                gyro_c >= CALIB_MIN_LEVEL_GYRO and 
                acc_c >= CALIB_MIN_LEVEL_ACC and 
                mag_c >= CALIB_MIN_LEVEL_MAG):
                break
            if time.time() - start_time > CALIB_TIMEOUT_SEC:
                break
            log_sensor_row(time.time() - start_time, "STILL", f"calib={g_calib}")
            time.sleep(0.1)
            
        if (g_calib[0] >= CALIB_MIN_LEVEL_SYS and 
            g_calib[1] >= CALIB_MIN_LEVEL_GYRO and 
            g_calib[2] >= CALIB_MIN_LEVEL_ACC and 
            g_calib[3] >= CALIB_MIN_LEVEL_MAG):
            break
        if time.time() - start_time > CALIB_TIMEOUT_SEC:
            break
            
    # Measure baseline horizontal acceleration
    log(f"静止基準加速度(スタック検知用)測定開始 ({CALIB_ACC_MEASURE_SEC}秒間)")
    acc_measure_start = time.time()
    acc_norms = []
    while time.time() - acc_measure_start < CALIB_ACC_MEASURE_SEC:
        ax, ay, _ = g_acc
        norm = math.sqrt(ax*ax + ay*ay)
        acc_norms.append(norm)
        time.sleep(0.05)
    baseline_horiz_acc = sum(acc_norms) / len(acc_norms) if acc_norms else 0.0
    log(f"静止基準水平加速度確定: baseline_horiz_acc = {baseline_horiz_acc:.4f} m/s²")
    
    # GPS Fix Stabilization
    log("GPS Fix 安定化待ち開始")
    gps_wait_start = time.time()
    while True:
        if g_gps_valid and g_gps_sats >= GPS_MIN_SATS:
            log(f"GPS Fix 成功: 衛星数={g_gps_sats}")
            break
        if time.time() - gps_wait_start > GPS_CALIB_TIMEOUT_SEC:
            log("GPS Fix タイムアウト: 衛星捕捉が不十分ですが誘導へ進みます", "WARN")
            break
        time.sleep(1.0)
        
    # GPS Averaging
    log(f"GPS 平均化サンプル収集開始 ({GPS_CALIB_SAMPLE_SEC}秒間, 最大{GPS_CALIB_SAMPLES}個)")
    gps_samples = []
    sample_start = time.time()
    while time.time() - sample_start < GPS_CALIB_SAMPLE_SEC and len(gps_samples) < GPS_CALIB_SAMPLES:
        if g_gps_valid and g_gps_sats >= GPS_MIN_SATS:
            gps_samples.append((g_gps_lat, g_gps_lng))
            log(f"GPSサンプル追加 [{len(gps_samples)}/{GPS_CALIB_SAMPLES}]: Lat={g_gps_lat:.6f}, Lng={g_gps_lng:.6f}")
        time.sleep(GPS_CALIB_SAMPLE_SEC / GPS_CALIB_SAMPLES)
        
    if gps_samples:
        avg_lat = sum(lat for lat, lng in gps_samples) / len(gps_samples)
        avg_lng = sum(lng for lat, lng in gps_samples) / len(gps_samples)
        log(f"GPS 平均化座標確定: Lat={avg_lat:.6f}, Lng={avg_lng:.6f} (サンプル数={len(gps_samples)})")
    else:
        avg_lat, avg_lng = g_gps_lat, g_gps_lng
        log("GPSサンプル未取得のため最新値を使用します", "WARN")
        
    init_dist = haversine_distance(avg_lat, avg_lng, TARGET_LAT, TARGET_LNG)
    init_bearing = calculate_bearing(avg_lat, avg_lng, TARGET_LAT, TARGET_LNG)
    log(f"誘導走行準備完了: 初期目標距離={init_dist:.2f}m, 初期目標方位={init_bearing:.1f}°")

# ===========================================================================
# Phase 4: GPS 誘導走行 & 前方探索 & スタック回避
# ===========================================================================

def run_front_search(motor: MotorController) -> int:
    """Phase 4 開始直後の前方探索 (Front Search) シーケンス。"""
    global g_front_candidate, g_front_offset_deg
    log("前方探索 (Front Search) 開始")
    
    best_candidate = FRONT_SEARCH_DEFAULT_CANDIDATE
    best_improvement = -9999.0
    
    for i in [1, 2, 3, 4]:
        offset = FRONT_CANDIDATES_DEG[i]
        log(f"前方候補 {i} テスト開始: オフセット = {offset}°")
        
        # Apply candidate offset
        g_front_candidate = i
        g_front_offset_deg = offset
        
        # Measure start GPS position
        lat_start, lng_start = get_gps_average(samples=5)
        dist_start = haversine_distance(lat_start, lng_start, TARGET_LAT, TARGET_LNG)
        log(f"テスト前位置: Lat={lat_start:.6f}, Lng={lng_start:.6f}, 距離={dist_start:.2f}m")
        
        # Drive forward
        log(f"前進テスト ({FRONT_SEARCH_TRIAL_SEC}秒間)")
        motor.forward()
        time.sleep(FRONT_SEARCH_TRIAL_SEC)
        motor.stop()
        
        # Settle
        time.sleep(FRONT_SEARCH_SETTLE_SEC)
        
        # Measure end GPS position
        lat_end, lng_end = get_gps_average(samples=5)
        dist_end = haversine_distance(lat_end, lng_end, TARGET_LAT, TARGET_LNG)
        improvement = dist_start - dist_end
        log(f"テスト後位置: Lat={lat_end:.6f}, Lng={lng_end:.6f}, 距離={dist_end:.2f}m, 改善量={improvement:.2f}m")
        
        if improvement > best_improvement:
            best_improvement = improvement
            best_candidate = i
            
        if improvement >= FRONT_SEARCH_MIN_IMPROVE_M:
            log(f"候補 {i} が基準改善量 {FRONT_SEARCH_MIN_IMPROVE_M}m を満たしたため確定します")
            best_candidate = i
            break
            
        if i < 4:
            log(f"次の候補テストまで待機 ({FRONT_SEARCH_RETRY_WAIT_SEC}秒)")
            time.sleep(FRONT_SEARCH_RETRY_WAIT_SEC)
            
    # Set the determined best front candidate
    g_front_candidate = best_candidate
    g_front_offset_deg = FRONT_CANDIDATES_DEG[best_candidate]
    log(f"前方探索完了: 候補 {best_candidate} (オフセット {g_front_offset_deg}°) を最終採用しました (最善改善={best_improvement:.2f}m)")
    return best_candidate

def check_stuck_condition(motor_cmd: str) -> bool:
    """スタック条件判定を行う。"""
    ax, ay, _ = g_acc
    horiz_acc_norm = math.sqrt(ax*ax + ay*ay)
    
    is_forwarding = (motor_cmd == "FORWARD")
    
    acc_stuck = abs(horiz_acc_norm - baseline_horiz_acc) < STUCK_HORIZON_ACCEL_THRESH
    gps_speed_mps = g_gps_speed * KNOTS_TO_MPS
    
    acc_speed_stuck = is_forwarding and acc_stuck and (not g_gps_valid or (gps_speed_mps < STUCK_GPS_SPEED_THRESH_MPS))
    sonar_stuck = (g_sonar_m is not None and g_sonar_m < STUCK_SONAR_DIST_THRESH)
    
    return acc_speed_stuck or sonar_stuck

def execute_recovery_turn(motor: MotorController, turn_dir: str, target_angle_deg: float) -> bool:
    """地磁気(9軸)の角度変化からちょうど target_angle_deg 旋回したことを確認する。"""
    start_azimuth = calc_azimuth_with_front()
    
    if turn_dir == "LEFT":
        target_azimuth = (start_azimuth - target_angle_deg) % 360.0
        motor.turn_left_strong()
    else:
        target_azimuth = (start_azimuth + target_angle_deg) % 360.0
        motor.turn_right_strong()
        
    log(f"回復旋回開始: 現在={start_azimuth:.1f}°, 目標={target_azimuth:.1f}°, 方向={turn_dir}, 旋回角度={target_angle_deg}°")
    
    turn_start = time.time()
    while time.time() - turn_start < STUCK_RECOVER_TURN_TIMEOUT_SEC:
        current_az = calc_azimuth_with_front()
        diff = (target_azimuth - current_az) % 360.0
        if diff > 180.0:
            diff -= 360.0
            
        log_sensor_row(time.time() - turn_start, f"RECOVER_TURN_{turn_dir}", f"diff={diff:.1f}°")
        
        # Check if close to target azimuth (within 15 degrees)
        if abs(diff) < 15.0:
            motor.stop()
            log(f"目標方位到達: 現在={current_az:.1f}° (誤差={diff:.1f}°)")
            return True
            
        time.sleep(LOOP_DT)
        
    motor.stop()
    log("回復旋回タイムアウト", "WARN")
    return False

def recover_stuck(motor: MotorController, last_turn_dir: str) -> bool:
    """スタック回避・リカバリーシーケンス。"""
    log("スタック回避動作開始")
    
    if "L" in last_turn_dir or last_turn_dir == "LEFT":
        turn_dir = "RIGHT"
    elif "R" in last_turn_dir or last_turn_dir == "RIGHT":
        turn_dir = "LEFT"
    else:
        turn_dir = "RIGHT"
        
    execute_recovery_turn(motor, turn_dir, STUCK_RECOVER_TURN_DEG)
    
    for retry in range(1, STUCK_RECOVER_MAX_RETRIES + 1):
        log(f"回復前進開始 (リトライ {retry}/{STUCK_RECOVER_MAX_RETRIES})")
        motor.forward()
        
        fwd_start = time.time()
        stuck_detected = False
        stuck_count = 0
        
        while time.time() - fwd_start < STUCK_RECOVER_FWD_SEC:
            if check_stuck_condition("FORWARD"):
                stuck_count += 1
            else:
                stuck_count = max(0, stuck_count - 1)
                
            if stuck_count >= STUCK_COUNT_THRESHOLD:
                log("回復前進中に再スタックを検知！")
                stuck_detected = True
                break
                
            elapsed = time.time() - fwd_start
            log_sensor_row(elapsed, "RECOVERY_FORWARD", f"retry={retry} count={stuck_count}")
            time.sleep(LOOP_DT)
            
        motor.stop()
        
        if not stuck_detected:
            log("スタック回避成功")
            return True
            
        log("再スタックのため追加の90度旋回を行います")
        execute_recovery_turn(motor, turn_dir, STUCK_RECOVER_RETRY_TURN_DEG)
        
    log("スタック回避限界オーバー", "WARN")
    return False

def run_phase_4(motor: MotorController, log_path: Path):
    global phase
    phase = 4
    log("Phase 4 (GPS誘導走行) 開始")
    
    motor.set_forward_only(True)
    motor.set_reversed(PHASE4_MOTOR_REVERSED)
    
    run_front_search(motor)
    
    start_time = time.time()
    last_report_time = 0.0
    last_save_time = time.time()
    
    stuck_counter = 0
    last_motor_cmd = "STOP"
    
    while True:
        now = time.time()
        elapsed = now - start_time
        
        if elapsed > TIMEOUT_SEC:
            log("誘導走行タイムアウト強制終了", "WARN")
            break
            
        cur_lat, cur_lng = g_gps_lat, g_gps_lng
        cur_dist = haversine_distance(cur_lat, cur_lng, TARGET_LAT, TARGET_LNG)
        target_bearing = calculate_bearing(cur_lat, cur_lng, TARGET_LAT, TARGET_LNG)
        
        if cur_dist <= PHASE4_TO_5_RADIUS:
            log(f"目標半径内に到達: 距離={cur_dist:.2f}m <= {PHASE4_TO_5_RADIUS}m → Phase 5へ移行")
            break
            
        azimuth = calc_azimuth_with_front()
        diff = target_bearing - azimuth
        diff %= 360.0
        if diff > 180.0:
            diff -= 360.0
            
        motor_cmd = motor.apply_diff(diff)
        
        if check_stuck_condition(motor_cmd):
            stuck_counter += 1
        else:
            stuck_counter = max(0, stuck_counter - 1)
            
        if now - last_report_time > NAV_REPORT_INTERVAL_SEC:
            log(f"GPS走行: 距離={cur_dist:.2f}m, 目標方位={target_bearing:.1f}°, 機体方位={azimuth:.1f}°, 偏差={diff:.1f}°, コマンド={motor_cmd}, stuck={stuck_counter}")
            last_report_time = now
            
        log_sensor_row(elapsed, motor_cmd, f"dist={cur_dist:.2f} bearing={target_bearing:.1f} heading={azimuth:.1f} stuck={stuck_counter}")
        
        if now - last_save_time > 5.0:
            save_log(log_path)
            last_save_time = now
            
        if stuck_counter >= STUCK_COUNT_THRESHOLD:
            log("スタック検知！リカバリー開始します")
            motor.stop()
            recovered = recover_stuck(motor, last_motor_cmd)
            if recovered:
                stuck_counter = 0
            else:
                log("スタック回避に失敗、走行を継続して様子を見ます", "WARN")
                stuck_counter = 0
                
        last_motor_cmd = motor_cmd
        time.sleep(LOOP_DT)
        
    motor.set_forward_only(False)
    motor.set_reversed(False)
    motor.stop()
    log("Phase 4 完了")

# ===========================================================================
# Phase 5: 超音波最終接近 (左右スキャン方式)
# ===========================================================================

def run_phase_5(motor: MotorController, log_path: Path):
    global phase
    phase = 5
    log("Phase 5 (超音波最終接近) 開始")
    
    start_time = time.time()
    last_report_time = 0.0
    last_sonar_ok_time = time.time()
    last_save_time = time.time()
    
    while True:
        now = time.time()
        elapsed = now - start_time
        
        if elapsed > FINAL_APPROACH_TIMEOUT_SEC:
            log("最終接近タイムアウト終了", "WARN")
            break
            
        sonar_val = g_sonar_m
        if sonar_val is not None:
            last_sonar_ok_time = now
            if sonar_val <= FINAL_STOP_DIST_M:
                log(f"目標物到達を検知！ 距離={sonar_val:.2f}m <= {FINAL_STOP_DIST_M}m")
                break
        else:
            lost_duration = now - last_sonar_ok_time
            if lost_duration > FINAL_SONAR_LOST_WARN_SEC:
                log(f"超音波値ロスト警告 (Phase 5): {lost_duration:.1f}秒間値がありません", "WARN")
                last_sonar_ok_time = now - (FINAL_SONAR_LOST_WARN_SEC - 1.0)
                
        # --- SCAN SEQUENCE ---
        # 1. Left Scan
        log("スキャン開始: 左旋回測定中...")
        left_dists = []
        motor.turn_left_strong()
        scan_start = time.time()
        while time.time() - scan_start < FINAL_SCAN_SEC:
            if g_sonar_m is not None:
                left_dists.append(g_sonar_m)
                if g_sonar_m <= FINAL_STOP_DIST_M:
                    break
            time.sleep(FINAL_SCAN_LOOP_DT)
        motor.stop()
        
        if left_dists and min(left_dists) <= FINAL_STOP_DIST_M:
            log(f"目標到達を検知 (左スキャン中): 距離={min(left_dists):.2f}m")
            break
            
        motor.turn_right_strong()
        time.sleep(FINAL_SCAN_SEC)
        motor.stop()
        
        # 2. Right Scan
        log("スキャン開始: 右旋回測定中...")
        right_dists = []
        motor.turn_right_strong()
        scan_start = time.time()
        while time.time() - scan_start < FINAL_SCAN_SEC:
            if g_sonar_m is not None:
                right_dists.append(g_sonar_m)
                if g_sonar_m <= FINAL_STOP_DIST_M:
                    break
            time.sleep(FINAL_SCAN_LOOP_DT)
        motor.stop()
        
        if right_dists and min(right_dists) <= FINAL_STOP_DIST_M:
            log(f"目標到達を検知 (右スキャン中): 距離={min(right_dists):.2f}m")
            break
            
        motor.turn_left_strong()
        time.sleep(FINAL_SCAN_SEC)
        motor.stop()
        
        left_min = min(left_dists) if left_dists else 999.0
        right_min = min(right_dists) if right_dists else 999.0
        
        action = "FORWARD"
        if left_min < 999.0 or right_min < 999.0:
            if left_min < right_min:
                action = "LEFT_WEAK"
                log(f"スキャン結果: 左が近い (左={left_min:.2f}m, 右={right_min:.2f}m) -> 左微旋回前進")
            else:
                action = "RIGHT_WEAK"
                log(f"スキャン結果: 右が近い (左={left_min:.2f}m, 右={right_min:.2f}m) -> 右微旋回前進")
        else:
            log("スキャン結果: どちらも障害物未検知 -> 直進前進")
            
        if action == "LEFT_WEAK":
            motor.turn_left_weak()
        elif action == "RIGHT_WEAK":
            motor.turn_right_weak()
        else:
            motor.forward()
            
        act_start = time.time()
        reached = False
        while time.time() - act_start < 1.0:
            if g_sonar_m is not None:
                last_sonar_ok_time = time.time()
                if g_sonar_m <= FINAL_STOP_DIST_M:
                    reached = True
                    break
            time.sleep(0.05)
            
        motor.stop()
        
        if reached:
            log(f"目標物到達を検知！ 距離={g_sonar_m:.2f}m")
            break
            
        if now - last_report_time > FINAL_NAV_REPORT_INTERVAL_SEC:
            log(f"最終接近中: 最新超音波値={g_sonar_m}m")
            last_report_time = now
            
        log_sensor_row(elapsed, action, f"sonar={g_sonar_m} left={left_min:.2f} right={right_min:.2f}")
        
        if now - last_save_time > 5.0:
            save_log(log_path)
            last_save_time = now
            
    motor.stop()
    log("Phase 5 完了 (ゴール！)")

# ===========================================================================
# メインシーケンス実行エントリーポイント
# ===========================================================================

def main():
    global phase, g_closed
    log("NSE2026 メインシーケンスプログラム始動")
    
    Device.pin_factory = LGPIOFactory()
    
    led = LED(LED_PIN)
    led.on()
    
    sonar = SonarSensor()
    motor = MotorController()
    
    bno = BNO055()
    log("BNO055 接続確認中...")
    if not bno.setUp():
        log("BNO055 の初期化に失敗しました。センサーの接続を確認してください。", "ERROR")
        sys.exit(1)
    log("BNO055 接続成功")
    
    bmp = BMP180(oss=3)
    log("BMP180 接続確認中...")
    if not bmp.setUp():
        log("BMP180 の初期化に失敗しました。警告のみで続行します。", "WARN")
    else:
        log("BMP180 接続成功")
        
    gps_obj = MicropyGPS(9, "dd")
    
    ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"mission_{ts_str}.csv"
    log(f"ミッションログファイルの保存先: {log_path}")
    
    gps_thread = threading.Thread(target=gps_thread_func, args=(gps_obj,), daemon=True)
    gps_thread.start()
    
    sensor_thread = threading.Thread(target=sensor_thread_func, args=(bno, bmp, sonar), daemon=True)
    sensor_thread.start()
    
    phase = 0
    log("Phase 0: 初期化完了。ミッションを開始します。")
    
    try:
        blink_led(led)
        run_phase_1(motor, log_path)
        
        blink_led(led)
        run_phase_2(motor, log_path)
        
        blink_led(led)
        run_phase_3(motor, log_path)
        
        blink_led(led)
        run_phase_4(motor, log_path)
        
        blink_led(led)
        run_phase_5(motor, log_path)
        
        log("ミッション完了！LEDを高速点滅させます。")
        for _ in range(30):
            led.on()
            time.sleep(0.1)
            led.off()
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        log("キーボード割り込みによる強制終了が検知されました", "WARN")
    except Exception as e:
        log(f"エラーによりメインシーケンスが停止しました: {e}", "ERROR")
    finally:
        g_closed = True
        motor.close()
        sonar.close()
        led.close()
        
        save_log(log_path)
        log("NSE2026 メインプログラム終了")

if __name__ == "__main__":
    main()
