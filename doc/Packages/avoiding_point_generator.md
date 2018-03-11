# avoiding_point_generatorについて
このページでは衝突回避位置を生成する **avoiding_point_generator**_nodeについて概要を記述しています。


## ノードの構造

avoiding_point_generator_nodeは味方ロボットの移動目標位置とフィールド情報をもとに、
衝突回避位置を生成します。

## 衝突回避位置の生成アルゴリズムについて

図で説明します。

## avoidingPointの生成1
![target field](https://github.com/SSL-Roots/CON-SAI/blob/Images/Images/avoidingPoint1.png "target field")

## avoidingPointの生成2
![detect nearest obstacle](https://github.com/SSL-Roots/CON-SAI/blob/Images/Images/avoidingPoint2.png "detect nearest obstacle")

## avoidingPointの生成3
![detect overlapping obstalce](https://github.com/SSL-Roots/CON-SAI/blob/Images/Images/avoidingPoint3.png "detect overlapping obstacle")

## avoidingPointの生成4
![generate avoiding point](https://github.com/SSL-Roots/CON-SAI/blob/Images/Images/avoidingPoint4.png "generate avoiding point")


## パラメータの調整
ROSの[DynamicReconfigure](http://wiki.ros.org/ja/dynamic_reconfigure/Tutorials)
を使って、パラメータを動的に調整できます。

# 改善点

- 敵・味方ロボットの移動速度を生成アルゴリズムに組み込んでいない
- 味方ロボットの運動性能を生成アルゴリズムに組み込んでいない(無茶な回避位置を生成する場合がある)


## ディレクトリ構成(一部省略)
```zsh
    .
    ├── CMakeLists.txt
    ├── cfg
    │   └── point_generator.cfg # DynamicReconfigure用パラメータファイル
    ├── package.xml
    ├── src
    │   ├── avoiding_point_generator_node.cpp # ROSノード
    │   └── transformation.h # 座標変換用ライブラリ
    └── test
        └── test_avoiding_point_generator.py
```
