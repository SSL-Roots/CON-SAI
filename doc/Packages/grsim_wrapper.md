# grsim_wrapperについて
このページではgrSim/SSL-Visionとの通信を担う **grsim_wrapper**_nodeについて概要を記述しています。


## ノードの構造
grsim_wrapperには２つのノードが含まれています。

- command_sender # grSimへデータを送信するノード
- detection # grSim/SSL-Visionからデータを受信するノード


## command_senderの説明
command_senderはシミュレータ操作専用のノードです。

grSimのロボットへ動作司令や、
ロボット・ボールの位置/速度パラメータ(Replacement)を送信します。

## detectionの説明
detectionは実機操作、シミュレータ操作の両方で使用するノードです。

grSim/SSL-Visionから送られてくるロボット・ボール情報や、
フィールド情報を受信し、ROSトピックに変換して他のノードに送信します。

ai_core.launchでteam_sideを変更したとき、detectionノードで座標を反転しています。

## 改善点

- detectionノードにあるpose_(friend, enemy)は6台にしか対応していない上に、誰も使用していない。


## ディレクトリ構成(一部省略)
```zsh
    # ここにtree コマンドの出力を記載
    $ tree | xsel -b # ディレクトリ構造をクリップボードへコピー

    .
    ├── CMakeLists.txt
    ├── package.xml
    ├── parameter.txt
    ├── scripts
    │   ├── __init__.pyc
    │   ├── command_sender.py # ROSノード
    │   ├── detection.py # ROSノード
    │   ├── multicast.py # UDP通信用モジュール
    │   └── proto # google protobufファイル格納場所
    │       ├── __init__.py
    │       ├── grSim_Commands.proto
    │       ├── grSim_Commands_pb2.py
    │       ├── grSim_Packet.proto
    │       ├── grSim_Packet_pb2.py
    │       ├── grSim_Replacement.proto
    │       ├── grSim_Replacement_pb2.py
    │       ├── messages_robocup_ssl_detection.proto
    │       ├── messages_robocup_ssl_detection_pb2.py
    │       ├── messages_robocup_ssl_geometry.proto
    │       ├── messages_robocup_ssl_geometry_legacy.proto
    │       ├── messages_robocup_ssl_geometry_pb2.py
    │       ├── messages_robocup_ssl_refbox_log.proto
    │       ├── messages_robocup_ssl_refbox_log_pb2.py
    │       ├── messages_robocup_ssl_wrapper.proto
    │       ├── messages_robocup_ssl_wrapper_pb2.py
    └── test
        ├── test_command_sender.py
        └── test_detection.py
```
