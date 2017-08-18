# CON-SAI Tutorial

## シミュレータ(grSim)のロボットを動かす

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

CON-SAI起動後、grSimとReferee Boxを起動してください。

SAI-Visualizerにロボットとボールが描画されたら通信成功です。

Referee BoxでStop Game -> Kick Off -> Normal Start -> Haltとボタンを押してください。

3台のロボットが動き、1台のロボットがボールを蹴ったら成功です。


### チームカラー/サイドの切り替え

下記ファイルのfriend_colorをyellowに、team_sideをrightにすることで、
右守りのイエローチームとしてAIを動かせます

**ai_core/param/our_team.yaml**
```yaml
friend_color : 'yellow'
team_side    : 'right'
robots_num   : 6
```

out_team.yamlを変更後、先ほどと同じようにロボットを動かしてみてください。

見方サイドは右側になったにもかかわらず、
SAI Visualizer上では黄色ロボットが左側に表示され、右側へボールを蹴ることが確認できます。

これは、SSL-Visionから受信した座標をチームサイドに合わせて変換しているためです。

見方サイドが常に左側(x軸座標でマイナス側)となるため、
右側を攻める戦略プログラムを作成すればコートチェンジにも問題なく対応できます。






