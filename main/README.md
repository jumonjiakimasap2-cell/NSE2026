手順書
➀8月13日の午前中に済ませてほしいこと
1.ラズパイイメイジャーを起動しSDカードに書き込みを行う
Ver2.0.10が一番最新
https://www.raspberrypi.com/software/
このURLでやればいける

PCにSDカードを入れて
ダウンロードしたイメイジャーを起動してください

ラズパイの種類を選びます
Raspberry　Pi　02Wを選択してください

OSの種類は1番上の物を選択してください

SDカードを選択してください

Hostname
Localisation（国・キーボード設定）
User
Wi-Fi
Remote access（SSHの有効化、パスワード認証で十分）
Raspberry Pi Connect（オフでよい）

次にこれらの上記の順で設定していくんだけど
ホストネームはraspberrypi
国は日本の東京
ユーザーネームはpi
Wifiは河原のスマホのやつ
次に忘れずにSSH接続の有効化を選択してください（ダブルチェック推奨）
最後にコネクトをオフに設定をして
書き込みを始めてください

書き込み終わったら
SDカードを抜いてラズパイにさして電源を入れてください
（注意：今刺さってるＳＤカードはなくさず必ず区別して持ち歩いてなくさないようにしてね）

PCのコマンドプロンプトを開き
スマホのテザリングのWIFIをつないでください

つないだらしばらく待って
コマンドに
ssh pi@raspberrypi.local
と打ち込んでください！

いろいろ出てきたらYesと打ってください

そしたら順に以下のコマンドを打ち込んで実行していってください
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y git tmux i2c-tools curl
sudo apt install -y python3-gpiozero python3-rpi-lgpio liblgpio1 python3-serial python3-smbus
sudo apt install -y python3-pip python3-setuptools
sudo apt install -y python3-picamera2 python3-libcamera libcamera-apps python3-opencv python3-numpy
sudo apt install -y libgl1
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install --upgrade pip
pip install pynmea2
deactivate

次にラズパイの設定をいじります
sudo raspi-config

中に入れたら
Interface Options → I2C → Yes
と順番に選んで設定してください

次に
Interface Options → Serial Port
に入ったら
順番に
Yes　→　NO
を選んで設定してください

設定し終わったらFinishに入ってそのままrebootするか聞かれると思うのでYesを選択してください
（もしなんもなくて元の画面に戻ってきたら
sudo reboot
を実行してね）

最後に
git clone https://github.com/jumonjiakimsap2-cell/NSE2026
を実行してください
これで準備はばっちり
僕がしてたみたいに
cd NSE2026
ls
などで中のファイルを確認して下さい

プログラムを実行したいときは
source venv/bin/activate
を実効した後に
その階層のところに入り
python3 〇〇〇.pyと打って実行してね
途中で終了したいときは
コントロール＋ｃを押してください
もしファイルの内容を変えたいときは
nano 〇〇〇.pyと打って編集画面に入ってください
編集したら
コントロール＋ｓで保存，
コントロール＋ｘで編集画面から脱出してください
（編集画面の下にコマンドの説明あります）


➁僕がいないとき（鬼頭のＰＣで実行するとき）
リポですが先輩たちの方を使ってください（そっちの方が安定するため）

君たちだけで実行するときは以下の手順で実験を行ってください
まず階層mainに入り
finishv2.pyを実行してください（あえて名前はそのままにしました）
言わずもがな各種センサを動かすコードです
これで右側にある二つの値
緯度，経度を写真を撮りメモしてください

メモした緯度経度を本番用コード
main.py
に編集します（もうすでに先輩が図った本番ゴール周辺の座標が入れられています）

ここで事前準備終了です

最後に
source venv/bin/activate
cd NSE2026
cd main
python3 main.py
をすればコードが実行されます
（マジで僕がいけなくてごめんなさい，もし想定外の動きをしてもごめんなさい、、、プログラマーの僕の責任や…）


➂僕のＰＣ，スマホでやる時
僕がその場にいなければ先輩のリポを使ってね（なくさないでね）

まず僕のスマホのパスワード
226515j
PCのパスワード
8128

僕のスマホのホーム画面になったら
右側の画面を上から下にスワイプし
ｗｉｆｉとかモード設定が見れる画面を出してください
歯車マークを押してください
接続を押してください
そこからテザリングをしてください

ＰＣとラズパイがつながったらあとは同じです
ですが
一回ラズパイとＰＣがつながり
プログラムを実行する画面に来たら
cd NSE2026
git stash
git pull
をしてください（プログラムの更新を適用する魔法です）

その後上記のように実験を遂行してください

もしこれを見てもわからなければもっと詳しいものが下記にあります
https://github.com/YutakaOkutani/NSE2026







