[![Build Status](https://travis-ci.org/SSL-Roots/CON-SAI.svg?branch=master)](https://travis-ci.org/SSL-Roots/CON-SAI)

# CON-SAI
CON-SAIは[RoboCup SSL](http://wiki.robocup.org/Small_Size_League)に
初めて参加する人でも開発できるサッカーAIです。

**CON**tribution to **S**occer **AI**

![running CON-SAI](https://github.com/SSL-Roots/CON-SAI/blob/Images/Images/CON-SAI-about.png "running CON-SAI")

## CON-SAIの開発コンセプト
CON-SAIを使う前に[コンセプト](doc/Concept.md)を読んでください。

## Announcements
- RoboCup Japan Open 2018 に新チーム[Scramble](http://nararobocon.sakura.ne.jp/scramble-ssl/)がCON-SAIを使って出場しました。

## Requirements
CON-SAIはUbuntu 16.04、18.04で作成・テストしてます。

下記のPCスペックで問題なく動作します。
- Intel(R) Core(TM) i5-6600K CPU @ 3.50GHz
- 8 GB of RAM
- 有線LANポート（試合会場では有線LANでロボット・ボール位置座標データを受信します)

## Installation

### ROSのインストール
[ROS (Robot Operating System)](http://wiki.ros.org/ja)
は、ロボットソフトウェア開発をサポートする
ライブラリ・ツールが豊富に含まれたオープンソースソフトウェアです。

ROSにはいくつかのDistributionがありますが、**Kinetic**をインストールしてください。

*Melodicでも動作確認済みです。*

フルインストール推奨です。

[**ROS Kinetic の Ubuntu へのインストール**](
http://wiki.ros.org/ja/kinetic/Installation/Ubuntu
)

ROSインストール後はチュートリアルをひと通り実施してください。

(気が早い方は以下を実行)

```zsh
# catkinワークスペースを作成
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace

# ワークスペースの最上位ディレクトリに移動
$ cd ~/catkin_ws

# ワークスペースのビルド
$ catkin_make

# ワークスペースのオーバレイ
# ***shには使用しているshellを入力 (etc: bash, zsh)
$ source devel/setup.***sh 

# ROSのパッケージを追加するたびに入力する必要があるので
# bashrcやzshrcなどに加えておくと楽です

$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```


### Google Protobufをインストール

```zsh
$ sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler
# Python Package Installerのインストール
$ sudo apt-get install python-pip

# Install protobuf for python2
$ sudo pip2 install protobuf
```

### その他インストール
```zsh
# ROS パッケージをインストール
$ sudo apt-get install ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-bfl
    
# グラフ描画ライブラリ(pygraphviz) をインストール
$ sudo apt-get install graphviz libgraphviz-dev pkg-config
$ sudo pip install pygraphviz --install-option="--include-path=/usr/include/graphviz" --install-option="--library-path=/usr/lib/graphviz/"
```

### CON-SAIをインストール
```zsh
# CON-SAIをクローン
$ git clone https://github.com/SSL-Roots/CON-SAI ~/catkin_ws/src/CON-SAI

# サブモジュール化したライブラリをダウンロード
$ cd ~/catkin_ws/src/CON-SAI
$ git submodule init
$ git submodule update

# ワークスペースに移動しCON-SAIをコンパイル
$ cd ~/catkin_ws
$ catkin_make

# エラーがでなければOK
```

CON-SAIの使い方は[チュートリアル](doc/Tutorial.md)に書いてます。

チュートリアルを始める前に下記のgrSimとReferee Boxをインストールしてください。

## RoboCup SSLのAI開発に必要なツールをインストール

RoboCup SSLのAI開発にはシミュレータ(grSim)と審判ソフト(Referee Box)があると便利です。

インストール方法は各ページをご参照ください。

[grSim](https://github.com/RoboCup-SSL/grSim)

![grSim Image](https://github.com/SSL-Roots/CON-SAI/blob/Images/Images/grSim.png "grSim")

[Referee Box](https://robocup-ssl.github.io/ssl-refbox/)

![Referee Box Image](https://github.com/SSL-Roots/CON-SAI/blob/Images/Images/refereeBox.png "Referee Box")

## Tutorial

grSimを使ったシミュレータ上でのロボット操作方法、
実機ロボットの操作方法はこちらに書いてます。

[チュートリアル](doc/Tutorial.md)


## Documents

- [CON-SAIのROSパッケージ概要](./doc/Packages/Main.md)

## Author

CON-SAIはRoboCup SSLに参加している日本人チーム**Roots**が作成しています。

RoboCup SSLへの参加方法、ロボットに必要な機能、開発環境などは
Rootsのホームページに記載してます。


[Roots - Home](https://github.com/SSL-Roots/Roots_home/wiki)
