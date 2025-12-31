# chinorobo_ros2_opencv_03
## パッケージ概要（Package description）
知能ロボットシステムコース5R実習科目「ロボット知能化演習」の画像処理実習パッケージその3

## インストール方法（How to install & setup）
### 本パッケージのダウンロード方法（Download the repository）
```
$cd ros2_ws/src
$git clone https://github.com/tominagalab/chinorobo_ros2_opencv_03.git
```

## ノード仕様（Nodes）
### __*image_proc_node*__
１つの3チャンネル画像をサブスクライブして，画像処理後，結果の画像をパブリッシュするためのテンプレートノードである．  
このノードを基礎として各種画像処理ノードの実装を行う．  
- サブスクライバー（Subscribers）
  - __image_src__ : 3チャンネルの入力画像．
- パブリッシャー（Publishers）
  - __image_dst__: Nチャンネルの出力画像．画像処理後の画像のチャンネル数を考えて実装させる．  
- ノード起動方法（how to excute）  
```
ros2 run chinorobo_ros2_opencv_02 image_proc_node
```  

## 授業内容
### Haar特徴量のカスケード接続による顔検出の実装
1. ***  