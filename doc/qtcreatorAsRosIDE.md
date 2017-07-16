#QtCreatorをROS IDEとして使う

##1. QtCreator Install
Qtの公式ページからQt とQtCreatorをインストールする。[Qt - Download](https://www.qt.io/download-open-source/)

Qtのインストール場所は`/opt/Qt`とする。

`~/.zshrc`や`~/.bashrc`を読み込ませるために、QtCreatorはターミナルで起動する。

```
$ /opt/Qt/Tools/QtCreator/bin/qtcreator
```

めんどくさいんで`~/.zshrc`を編集

```
alias qtcreator='/opt/Qt/Tools/QtCreator/bin/qtcreator'
```

###*重要！！！QtCreatorはターミナルから起動する！！！*

##2. CMakeLists.txtのシンボリックリンク変更
`~/catkin_ws/src/CMakeLists.txt`はシンボリックリンクなので都合が悪い。
以下のコマンドを実行する。

```
$ mv ~/catkin_ws/src/CMakeLists.txt ~/catkin_ws/src/CMakeLists_symlink.txt 
$ sudo cp /opt/ros/indigo/share/catkin/cmake/toplevel.cmake  ~/catkin_ws/src/CMakeLists.txt
```
これにより、`~/catkin_ws/src`には
シンボリックリンクの`CMakeLists_symlink.txt`と
リンクではない`CMakeLists.txt`が生成される。

##3. CMakeLists.txtの編集
各パッケージの中身をQtCreatorから扱えるように
先ほど生成した`~/catkin_ws/src/CMakeLists.txt`を編集する。

```
$ sudo vim ~/catkin_ws/src/CMakeLists.txt
```

以下のスクリプトを追加する。

```
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
```

どの行に追加しても問題ないとは思う。
最終行の`catkin_workspace()`の前に追加したら成功した。

##4. 確認
QtCreatorを実行する前に、ターミナル上でcatkin_makeが動作するか確認する。
catkin_makeが実行できない場合`/opt/ros/indigo/setup.zsh`を読みこめば解決する*(はず)*。

##5. QtCreator起動
* QtCreatorを**ターミナル**から起動し、「プロジェクトを開く」を押す。
* `~/catkin_ws/src/CMakeLists.txt`を開く。

### プロジェクトを開いた後、CMake設定画面が開く場合
* ビルドディレクトリに`~/catkin_ws/build`を指定。

### プロジェクトを開いた後、Configure Project画面が開く場合
* 使用するキット(Desktop Qt 5.5.1 GCC 64bit等)の「詳細」を開く。
* デフォルト、Debug、Releaseディレクトリに`~/catkin_ws/build`を指定。
* 「プロジェクトの設定」を押す。

##6. CMakeの実行
* CMakeの引数に`../src -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_BUILD_TYPE=Debug`を指定。
(Debug機能を利用しないときは、-DCMAKE_BUILD_TYPE=Debugを書かない)
* 「CMakeの実行」を押す。
* エラーが出ないことを祈る。
* `Build files have been written to: /home/ユーザ名/catkin_ws/build`と表示されたら成功。

##7. ノードの実行
QtCreatorでノードを実行する。
* ウィンドウ左下の三角ボタンを押す。 or
* ツールバーの「ビルド」 -> 「実行」

Projectに複数の実行可能なノードが存在する場合、実行するノードを選択できる。
* ウィンドウ左下、PC画面みたいなタブを開く -> 「実行」の欄からノードを選択する。 or
* ツールバーの「ビルド」 -> 「ビルド/実行キットセレクタを開く」 -> 「実行」の欄からノードを選択。

##8. Packageの新規作成について
QtCreatorではPackageの新規作成ができない。
そのため、ターミナルで`catkin_create_pkg`を実行し、Packageを作る必要がある。

QtCreatorに新規Packageを反映させるためには、
ターミナルでPackageを作成した後、QtCreatorでCMakeを実行する。
* プロジェクトツリーの「Project」を右クリック -> 「CMakeの実行」 or
* ウィンドウ左端の「プロジェクト」タブ ->  「CMakeの実行」 or
* ツールバーの「ビルド」 -> 「CMakeの実行」

CMakeの引数は記憶されているので、「..」と適当に文字を打つだけで候補を出力してくれる。

##References
[IDEs - ROS Wiki](http://wiki.ros.org/IDEs)

[QtCreatorでROSのパッケージをビルド&デバッグ実行する](http://qiita.com/MoriKen/items/ea41e485929e0724d15e)

[ROSのコーディングのためのIDE：Qt Creator編](http://coffeegkgk.hatenablog.com/entry/2015/09/16/021906)
