# velocity_transformerについて
このページではロボット動作司令の座標系を変換する **velocity_transformer**_nodeについて概要を記述しています。


## ノードの構造
velocity_transformer_nodeはロボット位置とロボット動作司令(cmd_vel)を受け取り、
フィールド座標系でのロボット動作司令(cmd_vel_world)と加速度(accel_world)を計算し、Publishします。

## 座標系変換の目的
cmd_velはロボット座標系で定義されています。
ロボットを中心とした移動速度と角速度のパラメータです。

ロボットの動作司令は、world_observerのカルマンフィルタで使用します。

カルマンフィルタはフィールド座標系で演算しています。
そのため、ロボット動作司令をフィールド座標系に変換しなければなりません。

その変換を担うものが、velocity_transformer_nodeです。

## 改善点

- 変換するだけのパッケージのため、cmd_velを生成するパッケージに機能を組み込んだほうが綺麗になる

## ディレクトリ構成(一部省略)
```zsh
    .
    ├── CMakeLists.txt
    ├── package.xml
    ├── src
    │   └── velocity_transformer.py # ROSノード
    └── test
        └── test_velocity_transformer.py
```
