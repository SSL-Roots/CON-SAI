# real_senderについて
このページでは実機ロボットへコマンドを送信する **real_sender**_nodeについて概要を記述しています。


## ノードの構造
real_sender_nodeはrobot_commandsをSubscribeし、シリアル通信でロボットへ司令を送信します。


## 送信機について
送信機はZigBee規格のXbeeを想定しています。

その他の送信機では動作テストしていません。

## 通信プロトコルについて
下記の、10バイトデータを実機ロボットへ送信しています。

チェックサムでエラーを検出できるように、各データの値を調整しています。

エラー検出はできますが、どこでエラーが発生したかわからないため、修正できません。

```cpp
    // serializer.cpp から抜粋
    
    /* Roots Protocol
     * 0: 1111 1111 |HEADER_1 0xFF
     * 1: 1100 0011 |HEADER_2 0xC3
     * 2: 0000 xxxx |x:ID
     * 3: aaaa aaaa |a:vel_norm(0~254)
     * 4: bbbb bbbb |b:vel_theta(0~180)
     * 5: cccc cccc |c:omega(0~254)
     * 6: d01e f110 |d:dribble_flag, e:kick_flag, f:chip_enable
     * 7: gggg hhhh |g:dribble_power, h:kick_power
     * 8: **** **** |XOR([2] ~ [7])
     * 9: **** **** |XOR([8],0xFF)
     *
     */
```

## 改善点

- 送信ポートが/dev/ttyUSB0で固定されている
- 送信ボーレートが57600で固定されている
- kick_power、dribble_powerの値が15 or 0に制限されている


## ディレクトリ構成(一部省略)
```zsh
    .
    ├── CMakeLists.txt
    ├── include
    │   └── real_sender
    │       └── serial.h # serial通信ライブラリ
    ├── package.xml
    ├── src
    │   ├── real_sender_node.cpp # ROS ノード
    │   └── serializer # シリアライザライブラリ
    │       ├── serializer.cpp
    │       ├── serializer.hpp
    │       └── test
    │           └── test.cpp
    └── test
        └── test_real_sender.py
```
