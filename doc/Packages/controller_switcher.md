# controller_switcherについて
このページではロボットの自動/手動操作を切り替える **controller_switcher**_nodeについて概要を記述しています。

現在、joystick操作をメンテナンスしていないため、このノードは使用していません。


## ノードの構造
パラメータmode (ai / joy) を与えることで、robot_commandsの出力元を切り替えます。
出力元は自動(ai_controller)、手動(ssl_joystick)です。


## 改善点

- joystick操作をメンテしていないため、このノードは使用していない


## ディレクトリ構成(一部省略)
```zsh
    .
    ├── CMakeLists.txt
    ├── package.xml
    └── scripts
        └── controller_switcher.py
```
