# consai_msgsについて
このページではROS msgファイル集である **consai_msgs**について概要を記述しています。


## consai_msgsの概要
CON-SAIで使用するmsgファイルを、このパッケージで生成しています。

## 改善点
- CON-SAI開発初期に作成したmsgで、現在使用してないものがある
  - RobotPoses.msg
  - robot_packet.msg

## ディレクトリ構成(一部省略)
```zsh
    .
    ├── CMakeLists.txt
    ├── msg
    │   ├── AIStatus.msg # decision_making_nodeの出力
    │   ├── FieldCircularArc.msg # SSL_geometry.protoからコピーしたもの
    │   ├── FieldLineSegment.msg # SSL_geometry.protoからコピーしたもの
    │   ├── GeometryFieldSize.msg # SSL_geometry.protoからコピーしたもの
    │   ├── Pose.msg # 位置(x, y), 角度(theta)を格納したもの
    │   ├── RefereeTeamInfo.msg # referee.protoからコピーしたもの
    │   ├── ReplaceBall.msg # grSimコントロール用のmsg。SAI-Visualizerで生成
    │   ├── ReplaceRobot.msg # grSimコントロール用のmsg。SAI-Visualizerで生成
    │   ├── RobotPoses.msg # receiverで生成
    │   ├── VisionObservations.msg # SSL-Visionのロボット・ボールデータをまとめたもの
    │   ├── VisionPacket.msg # SSL-Visionの位置データを定義したもの
    │   ├── VisionRobotPackets.msg # SSL-Visoinのロボット位置データを定義したもの
    │   ├── nodeData.msg # BehaviorTreeのnodeを定義したもの
    │   ├── nodeDataArray.msg # BehaviorTreeのnodeをまとめたもの
    │   ├── robot_commands.msg # Robotへの動作司令を定義したもの。senderが受け取る
    │   └── robot_packet.msg # 未使用
    └── package.xml

```
