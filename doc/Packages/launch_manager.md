# launch_managerについて
このページでは動的にlaunchファイルを起動する **launch_manager**_nodeについて概要を記述しています。


## ノードの構造
launch_manager_nodeは、フィールド上に敵/味方ロボットが現れると、
対応するIDでai_core/(enemy,robot).launchを起動します。

ロボットが消えた場合は、一定時間後にlaunchのプロセスを削除します。

## 改善点

- CON-SAI起動時に大量にlaunchしてしまうため、処理が重くなる。（最悪の場合、落ちる）

## ディレクトリ構成(一部省略)
```zsh
    .
    ├── CMakeLists.txt
    ├── package.xml
    ├── src
    │   └── launch_manager.py # ROSノード
    └── test
        └── test_launch_manager.py
```
