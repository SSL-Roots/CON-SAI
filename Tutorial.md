# CON-SAI Tutorial

## シミュレータ(grSim)のロボットを動かす

![sai_visualizer](https://github.com/SSL-Roots/CON-SAI/blob/Images/Images/sai_visualizer.png "sai_visualizer")

### マルチキャストアドレス/ポート設定

grSimを起動してCommunicationパラメータを確認します。

ここでは以下のパラメータで作業を進めます。

```
* Vision multicast address :224.5.23.2
* Vision multicast port :10006
* Command listen port :20011
```


次にReferee Boxのパラメータを確認します。

Referee Boxディレクトリ内のreferee.confを確認してください。

デフォルトでパラメータは以下のとおりになっています。

```
* ADDRESS :224.5.23.1
* PROTOBUF_PORT :10003
```

これらのパラメータをCON-SAIにセットします。

**ai_core/launch/simulator.launch**
```xml
<!-- 位置座標受信アドレス・ポート設定 -->

<node name="receiver" pkg="grsim_wrapper" type="detection.py">
    <param name="multicast_addr" type="string" value="224.5.23.2" />
    <param name="multicast_port" value="10006" />
</node>
```

```xml
<!-- ロボット動作指令送信アドレス・ポート設定 -->
<!-- server_addressにはローカル・ループバック・アドレス(127.0.0.1) を入力してます -->


<node name="sender" pkg="grsim_wrapper" type="command_sender.py">
    <param name="server_address" type="string" value="127.0.0.1" />
    <param name="server_port" value="20011" />
    <!-- <remap from="/robot_0/robot_commands" to="/ssl_joy/ssl_joystick/robot_commands" /> -->
</node>
```

**ai_core/launch/ai_core.launch**
```xml
<!-- Referee Box 信号受信アドレス・ポート設定 -->

<node name="refbox" pkg="ssl_refbox_wrapper" type="ssl_refbox_wrapper_node.py">
    <param name="multicast_addr" value="224.5.23.1" />
    <param name="multicast_port" value="10003" />
</node>
```

### CON-SAIとビジュアライザの起動

```zsh
# CON-SAIの起動
$ roslaunch ai_core ai_core.launch

# 別のターミナルで
# SAI Visualizerの起動
$ rqt --standalone sai_visualizer

# もしくはrqt単体で起動し、/Plugins/Visualization/SAI-Visualizer から選択することも可能です
$ rqt

```

![open sai_visualizer](https://github.com/SSL-Roots/CON-SAI/blob/Images/Images/rqt_open_sai.png "open SAI-Visualizer")

CON-SAI起動後、grSimとReferee Boxを起動してください。

SAI-Visualizerにロボットとボールが描画されたら通信成功です。

Referee BoxでStop Game -> Kick Off -> Normal Start -> Haltとボタンを押してください。

3台のロボットが動き、1台のロボットがボールを蹴ったら成功です。



### チームカラー/サイドの切り替え
執筆中








