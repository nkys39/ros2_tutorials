# MacOSでのROS2Humble環境構築(pixiによる簡略版)



## (オプション)クラッシュレポートを非表示にする
開発時にROS2関連のプログラムやツールを使用していると予期せぬプログラム終了が起こることが多々ある。その際にクラッシュレポートが毎回出るのが邪魔になることがあるため、非表示オプションを記載する。
```sh
# 非表示にしたいとき
defaults write com.apple.CrashReporter DialogType none
# 元に戻したいとき
defaults write com.apple.CrashReporter DialogType prompt
```

## pixiのインストール
```sh
curl -fsSL https://pixi.sh/install.sh | bash
```

## pixiプロジェクトの作成
ホームディレクトリ(~/)で下記を実行し、`pixi_ros2`プロジェクトを作成する
```sh
cd ~/
pixi init pixi_ros2 -c robostack-staging -c conda-forge
```
プロジェクトを作成すると、プロジェクト名のディレクトリが作成され、`pixi.toml`ファイルが作成される。`cat`コマンドで中身を確認する。
```sh
cd pixi_ros2 && cat pixi.toml
```
以下が出力される。
```sh
[project]
channels = ["robostack-staging", "conda-forge"]
description = "Add a short description here"
name = "pixi_ros2"
platforms = ["osx-arm64"]
version = "0.1.0"

[tasks]

[dependencies]
```

ROS2環境用に`pixi.toml`ファイルを編集し、以下のようにする。

```sh
[project]
channels = ["robostack-staging", "conda-forge"]
description = "Add a short description here"
name = "pixi_ros2"
platforms = ["osx-arm64"]
version = "0.1.0"

[tasks]

[dependencies]
ros-humble-desktop = ">=0.10.0,<0.11"
ros-humble-turtlesim = ">=1.4.2,<2"
colcon-common-extensions = ">=0.3.0,<0.4"
setuptools = ">=75.6.0,<76"
ros-humble-ament-cmake-auto = ">=1.3.7,<2"
compilers = ">=1.8.0,<2"
pkg-config = ">=0.29.2,<0.30"
cmake = ">=3.28.3,<4"
ninja = ">=1.12.1,<2"
ros-humble-turtlebot3-gazebo = ">=2.2.5,<3"
ros-humble-nav2-bringup = ">=1.1.13,<2"
protobuf = "4.25.1.*"
glog = "==0.6.0"
graphviz = ">=9.0.0,<10"

[activation.env]
ROS_DOMAIN_ID = "2"
TURTLEBOT3_MODEL = "waffle"
GAZEBO_MODEL_PATH = "$GAZEBO_MODEL_PATH:$PIXI_PROJECT_ROOT/.pixi/envs/default/share/turtlebot3_gazebo/models"

```
上記の`[dependencies]`は以下のコマンドと同義でも追加できる。
```sh
cd ~/pixi_ros2
pixi add ros-humble-desktop ros-humble-turtlesim colcon-common-extensions setuptools ros-humble-ament-cmake-auto compilers pkg-config cmake ninja ros-humble-turtlebot3-gazebo ros-humble-nav2-bringup "protobuf=4.25.1" glog==0.6.0 graphviz
```
`[activation.env]`は手動で追加しておく


