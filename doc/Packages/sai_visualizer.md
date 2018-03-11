# sai_visualizerについて
このページではCON-SAIのメインビジュアライザである **sai_visualizer**_nodeについて概要を記述しています。


## ノードの構造
sai_visualizerはフィールド情報を描画する[rqt plugin](http://wiki.ros.org/rqt/Plugins)です。

ロボット・ボール位置や、ボール軌道、ロボット移動目標位置を描画できます。

また、grSimのロボット・ボールをsai_visualizerから操作できます。

## 操作方法

- 左クリック&ドラッグ
  - 描画領域 : 描画領域を動かす
  - ロボット(本体) : grSimのロボット位置を変更する(赤い線が描画されます)
  - ロボット(周囲) : grSimのロボット角度を変更する(青い線が描画されます)
  - ボール(本体) : grSimのボール位置を変更する(赤い線が描画されます)
  - ボール(周囲) : grSimのボール速度を変更する(青い線が描画され、カーソル横に速度が表示されます)

- 右クリック
  - 描画領域 : 描画領域の移動とズームをリセットする

- スクロールホイール
  - 描画領域 : 描画領域をズームする


## 改善点

- 描画対象のOn/OffはVisualizer上で変更できず、コードを書き換えなければならない


## ディレクトリ構成(一部省略)
```zsh
	.
	├── CMakeLists.txt
	├── package.xml
	├── plugin.xml
	├── resource
	│   └── visualizer_widget.ui
	├── scripts
	│   └── sai_visualizer
	├── setup.py
	└── src
		└── sai_visualizer
			├── __init__.py
			├── geometry.py # field_geometry格納モジュール
			├── paint_widget.py # 描画モジュール
			├── visualizer.py # pluginのメインモジュール
```
