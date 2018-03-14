# ssl_refbox_wrapperについて
このページではSSL-RefereeBoxのデータを受信する **ssl_refbox_wrapper**_nodeについて概要を記述しています。


## ノードの構造
ssl_refbox_wrapper_nodeは、SSL-RefereeBoxからReferee情報を取得し、
ROSトピックに変換してPublishします。


## ディレクトリ構成(一部省略)
```zsh
    .
    ├── CMakeLists.txt
    ├── package.xml
    ├── parameters.txt # 使用していないファイル
    ├── src
    │   ├── multicast.py # UDP通信モジュール
    │   ├── proto
    │   │   └── referee.proto # google protobuf ファイル
    │   ├── referee_pb2.py
    │   └── ssl_refbox_wrapper_node.py # ROSノード
    └── test
        └── test_ssl_refbox_wrapper.py
```
