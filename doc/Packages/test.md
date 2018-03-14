# testディレクトリについて
このページではTravis CIテスト用のtestディレクトリについて概要を記述しています。


## 概要
CON-SAIは[Travis CI](https://travis-ci.org/)でビルド&テストしています。

ビルドに使用するスクリプトをtestディレクトリに保存しています。


## ディレクトリ構成(一部省略)
```zsh
    .
    ├── travis_build_consai.bash # CON-SAIビルド用スクリプト
    └── travis_ros_install.bash # ROSインスール用スクリプト
```
