# trapezoidal_controlについて
このページではロボットの台形制御を担う **trapezoidal_control**_nodeについて概要を記述しています。


## ノードの構造
trapezoidal_control_nodeはロボットの移動目標位置/角度を受け取り、
台形制御の演算実行後、cmd_velをPublishします。

また、目標速度を受け取った場合は、一定加速度で速度を調節し、cmd_velをPublishします。


## 台形制御(位置)の説明
台形制御(位置)では以下のアルゴリズムを毎フレーム(60 fps)実行し、目標速度を計算します

1. 現在のロボット位置と移動目標位置をもとに、移動距離を計算する
1. 現在の目標速度をもとに、制動距離を計算する
1. 目標速度を求める
    - 移動距離が制動距離以上であれば、目標速度を加速する
    - 目標速度が制限速度以上であれば、目標速度を維持する
    - 移動距離が制動距離以下であれば、目標速度を減速する

また、現在の移動方向から目標位置が大きく離れる場合は、速度を落とす処理を組み込んでいます。

## 台形制御(角度)の説明
台形制御(角度)では位置の場合と同様のアルゴリズムで、目標角速度を計算します。

また、目標速度と目標角速度が大きい場合、角速度を制限する処理を組み込んでいます。（遠心力リミッター）


## パラメータの調整
ROSの[DynamicReconfigure](http://wiki.ros.org/ja/dynamic_reconfigure/Tutorials)
を使って、パラメータを動的に調整できます。

## 改善点

- 制御パラメータを手動で調整しなければならない


## ディレクトリ構成(一部省略)
```zsh
    .
    ├── CMakeLists.txt
    ├── cfg
    │   └── controller.cfg # DynamicReconfigure用のパラメータファイル
    ├── package.xml
    ├── src
    │   ├── transformation.h # 座標変換ライブラリ
    │   └── trapezoidal_control_node.cpp # ROSノード
    └── test
        └── test_trapezoidal_control.py
```
