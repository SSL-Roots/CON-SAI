# ai_coreについて
このページではCON-SAIのコアである **ai_core**について概要を記述しています。


## ai_coreの概要
ai_core.launch を起動すれば、試合用のプログラムが動きます。


## 実機/シミュレータ動作の説明
ai_core.launch内でreal.launchとsimulator.launchを書き換えることで、
実機操作、シミュレータ操作を切り替えます。

実機操作はreal.launchを、
シミュレータ操作はsimulator.launchを起動するように切り替えます。

## our_team.yamlの説明
our_team.yamlには、CON-SAIプログラムで使用するパラメータを記載しています。

goalie_idを書き換えることで、味方ロボットのゴールキーパを変更できます。

## 改善点

- our_team.yamlのrobots_numを活用できていない。


## ディレクトリ構成(一部省略)
```zsh
    .
    ├── CMakeLists.txt
    ├── launch
    │   ├── ai_core.launch # メインのlaunchファイル。
    │   ├── enemy.launch # 敵ロボットの観測用launchファイル。
    │   ├── real.launch # 実機操作用launchファイル。ai_core.launchで切り替える。
    │   ├── robot.launch # 味方ロボットの観測・制御用launchファイル。
    │   ├── simulator.launch # シミュレータ操作用launchファイル。ai_core.launchで切り替える。
    │   ├── test_ai_core.launch # rostest用のlaunchファイル。
    │   ├── test_real.launch # rostest用のlaunchファイル。
    │   ├── test_robot.launch # rostest用のlaunchファイル。
    │   └── test_simulator.launch # rostest用のlaunchファイル。
    ├── package.xml
    └── param
        └── our_team.yaml # Parameterファイル
```
