# ai_controllerについて
このページではrobot_commands生成機である **ai_controller**_nodeについて概要を記述しています。


## ノードの構造
ai_controller_nodeは、

- cmd_vel
- kick_velocity
- ai_status

をSubscribeし、
robot_commandsに変換してPublishします。


## 改善点

- トピックを変換するだけなので、わざわざパッケージにする必要なし。decision_makingに機能を移したい。

## ディレクトリ構成(一部省略)
```zsh
    .
    ├── CMakeLists.txt
    ├── package.xml
    ├── scripts
    │   └── ai_controller.py # ROSノード
    └── test
        └── test_ai_controller.py
```

