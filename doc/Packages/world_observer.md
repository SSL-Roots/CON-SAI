# world_observerについて
このページではロボット・ボールの位置・速度を推定する **world_observer**_nodeについて概要を記述しています。


## ノードの構造
world_observer_nodeはgrSim/SSL-Visionの情報をもとに。ロボット・ボールの状態推定をします。

状態推定には拡張カルマンフィルタを使用しています。


## 味方ロボットのカルマンフィルタ
味方ロボットのカルマンフィルタには動作司令(cmd_vel_world)を組み込んでいます。

cmd_vel_worldは、ロボット座標系の動作司令(cmd_vel)を、フィールド座標系に変換したものです。


## ディレクトリ構成(一部省略)
```zsh
    .
    ├── CMakeLists.txt
    ├── include
    │   └── world_observer
    │       ├── ball_estimator.hpp
    │       ├── enemy_estimator.hpp
    │       └── estimator.hpp
    ├── package.xml
    ├── src
    │   ├── estimator
    │   │   ├── ball_estimator.cpp # ボール用カルマンフィルタ
    │   │   ├── enemy_estimator.cpp # ロボット用カルマンフィルタ
    │   │   └── estimator.cpp
    │   └── world_observer.cpp # ROSノード
    └── test
        └── test_world_observer.py
```
