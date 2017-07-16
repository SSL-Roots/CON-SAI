# CON-SAI
CON-SAI(根菜)はRoboCup SSL 用のサッカーAIです。


## Requirements
Ubuntu 14.04で動作確認してます

ROS IndigoがインストールされるOSはUbuntu 14.04以下となります。

## Installation

### ROSのインストール
**ROS Indigoのみサポートしてるので、ご注意ください**


シェルの.なんちゃらrcファイルに下記を追加
```  
source /opt/ros/indigo/setup.なんちゃらsh
source ~/catkin_ws/devel/setup.なんちゃらsh
```

### いろいろインストール
```
# サブモジュール化したライブラリをダウンロード
$ git submodule init
$ git submodule update

$ sudo apt-get install ros-indigo-navigation
    
# pygraphviz をインストール
$ sudo apt-get install graphviz libgraphviz-dev pkg-config
$ sudo pip install pygraphviz --install-option="--include-path=/usr/include/graphviz" --install-option="--library-path=/usr/lib/graphviz/"
```

### Google Protobufをダウンロード&インストール
https://github.com/google/protobuf/releases/download/v2.5.0/protobuf-2.5.0.zip

```
$ cd {Download Directory}
$ unzip protobuf-2.5.0.zip
$ cd protobuf-2.5.0

# Install protobuf for C++
$ ./configure
$ make
$ make check 
# checkでエラー吐いたらREADME.txtを見て対処して
$ sudo make install
    
# Install protobuf for python
$ cd python
$ python setup.py build
$ python setup.py test
# testでエラーはいたらREADME.txtを見て対処して
$ sudo python setup.py install
```

## How to Control simlator robots

### マルチキャストアドレス/ポート設定
ここにかく

### チームカラー/サイドの切り替え
ここにかく

### grSimとの通信設定
ここにかく

### AI起動方法
端末を起動して

```
roslaunch ai_core ai_core.launch
```

### チュートリアル
grSimとRefere Boxを起動します



## How to Control real robots
### Xbeeの接続設定
1. XBeeをPCにつなげる
2. シリアルポートに接続する権限を取得
 "sudo chmod 666 /dev/ttyUSB0" or "sudo chmod 666 /dev/ttyUSB1"

### launchファイルの変更
ここにかく


## Others


### JoyStick(PS3 コントローラ)接続設定
1. USBケーブルでPS3コンをPCに接続
2. "ls -a /dev/input/js*" で、js0が接続されていることを確認



