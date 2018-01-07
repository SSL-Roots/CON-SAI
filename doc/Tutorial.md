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

### CON-SAIとSAI-Visualizerの起動

```zsh
# CON-SAIとSAI-Visualizerの起動
$ roslaunch ai_core ai_core.launch
```

CON-SAI起動後、grSimとReferee Boxを起動してください。

SAI-Visualizerにロボットとボールが描画されたら、grSimから位置情報の受信成功です。

Referee BoxでStop Game -> Kick Off -> Normal Start -> Haltとボタンを押してください。

6台のロボットが動き、1台のロボットがボールを蹴ったら成功です。

戦略プログラムの簡単な概要は、[decision_making_nodeについて](AboutDecisionMaking.md)
に記載しています。


### チームカラー/サイドの切り替え

下記のように、ai_core.launch起動時に引数を指定できます。

```zsh
# default (左守りのブルーチーム)
$ roslaunch ai_core ai_core.launch

# 左守りのブルーチーム
$ roslaunch ai_core ai_core.launch color:=blue side:=left

# 右守りのイエローチーム
$ roslaunch ai_core ai_core.launch color:=yellow side:=right
```

それでは、**右守りのイエローチーム**としてCON-SAIを起動し、
先ほどと同じようにロボットを動かしてみてください。

味方サイドは右側になったにもかかわらず、
SAI Visualizer上では黄色ロボットが左側に表示され、右側へボールを蹴ることが確認できます。

これは、SSL-Visionから受信した座標をチームサイドに合わせて変換しているためです。

味方サイドが常に左側(x軸座標でマイナス側)となるため、
右側を攻める戦略プログラムを作成すればコートチェンジにも問題なく対応できます。


### CON-SAI vs CON-SAI
最後に、CON-SAI同士で対戦をします。

ai_core.launch起動時にai_nameを設定することで、
CON-SAIの全ノードをグルーピングできます。

```zsh
# ai_name には "/hoge/" のようにスラッシュで囲われた文字列を入力してください

$ roslaunch ai_core ai_core.launch ai_name:=/ai1/

# NG : Visualizerにロボットが描画されません
# roslaunch ai_core ai_core.launch ai_name:=ai1
```


それでは、ターミナルを2つ用意して、
**/ai1/ 右守りブルーチーム** と **/ai2/ 左守りイエローチーム**を起動しましょう。
```zsh
# Terminal 1 (左守りブルーチーム)
$ roslaunch ai_core ai_core.launch ai_name:=/ai1/ color:=blue side:=left


# Terminal 2 (右守りイエローチーム)
$ roslaunch ai_core ai_core.launch ai_name:=/ai2/ color:=yellow side:=right
```
*注意: CON-SAIを2つ起動すると動作が重たくなります*

SAI-Visualizerも2つ起動されますが、ここでは閉じてください。

Referee Boxを操作して2チームのロボットが動くことを確認してください。

動作が重たい場合は、grSim上でロボットを何台かTurn offしてください。
(ロボットを右クリックして操作できます。)
