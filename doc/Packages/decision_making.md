# decision_makingについて

このページではCON-SAIの戦術部分**decision_making**_nodeについて概要を記述しています。


## ノードの構造

decision_making_nodeの構造は、
[STP](https://pdfs.semanticscholar.org/5087/460f31babc3fbafafddbb480f216ea72832e.pdf)
に[Behavior Tree](http://michelecolledanchise.com/tro16colledanchise.pdf)
を組み合わせたものです。


## 戦略が切り替わる仕組み

基本的に、Refereeからの信号(HaltやStopなど)をベースに、戦略を切り替えています。

WorldModelクラス内でゲーム状況(situation)を更新しており、
situationに合わせてPlayExecuterがPlayを切り替えています。


## ロボットの役割分担

PlayにはRole[0 - 5]が組み込まれており、
Roleに対して適切なロボットID(0 - 11)を割り当てなければなりません(assignments)。

ここでは、以下のルールで割り当てています。

- Role_0 : *ゴールキーパー*
- Role_1 : *アタッカー*
- Role_2 - 5 : *ディフェンダー*

1. Role_0はゴールキーパのIDで固定する(ai_core内のour_team.yamlで変更可能)
1. IDの若い順にRole_1から埋めていく
1. ロボットが途中で消えたら、空いたRoleをRole_5と交換する(Role_5がいなければ、Role_4)
1. Inplay中は、ボールに１番近いロボットをRole_1に割り当てる

敵ロボットも同様に、役割を与えています(enemy_assignments)。


## 改善点

- 現状のdecision_making_nodeは、線形代数の計算がメインで、
AIと呼べるような学習機能が搭載されていません。

- 移動目標位置や、シュート目標位置は**固定**であることが多いです。
  - *例えば、tactic_inplay_shootで狙うところは常に敵ゴールの中心です。*

- ロボットは単体でボールを蹴ります。パスシュートをしません。


## ディレクトリ構成(一部省略)

```zsh
decision_making
├── CMakeLists.txt
├── package.xml
└── scripts
    ├── command.py # ロボットの動作指令クラス
    ├── constants.py # フィールド、ロボットサイズ等の定数をまとめたもの
    ├── coordinate.py # 動的目標位置生成クラス
    ├── decision_making_node.py # ROSノード(このノードから戦略が始まる)
    ├── observer.py # WorldModelクラスのサポートクラス
    ├── play_executer.py # STPのPlayを動かすもの
    ├── plays # STPのPlayをまとめたディレクトリ
    │   ├── __init__.py 
    │   ├── play_base.py # Playの抽象クラス
    │   ├── play_book.py # Playを集めたもの。Playを作成したらここに加える
    │   ├── play_dummy.py # ダミーPlay。Haltと同じ動作をする。
    │   ├── play_force_start.py # Force StartのPlay
    │   ├── play_halt.py # HaltのPlay
    │   ├── ...
    │   ├── role_base.py # Playが持つRoleの抽象クラス
    │   └── tactics # STPのTacticをまとめたディレクトリ
    │       ├── __init__.py
    │       ├── skills # STPのSkillをまとめたディレクトリ
    │       │   ├── __init__.py
    │       │   ├── adjustments.py # キック、ドリブルなど補助的な機能を使う
    │       │   ├── drive_to_target.py # 静的な目標位置に移動する
    │       │   ├── dynamic_drive.py # 動的な目標位置に移動する
    │       │   ├── observations.py # フィールド状況を判定する
    │       │   └── turn_off.py # 停止する
    │       ├── tactic_clear.py # ボールに向かって移動し、チップキックで弾く
    │       ├── tactic_halt.py # 停止する
    │       ├── tactic_inplay_shoot.py # ボールに向かって移動し、ゴールに向かって蹴る。制度は粗い。
    │       ├── ...
    │       └── tactic_setplay_shoot.py # ボールに向かって移動し、ゴールに向かって蹴る。
    ├── proto # protoファイル、protoc生成物を保管するディレクトリ
    │   ├── __init__.py
    │   ├── referee.proto
    │   └── referee_pb2.py
    ├── tool.py # ２点間の距離を測る、座標系を変換する(Transクラス)、などのツールをまとめたもの
    └── world_model.py # ロボット、ボール、見方ロボットのAssignment、などの情報をもつクラス
```
