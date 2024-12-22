# Windows11(WSL2)でのROS2Humble環境構築(pixiによる簡略版)



## pixiのインストール
```sh
iwr -useb https://pixi.sh/install.ps1 | iex
```

## pixiプロジェクトの作成
```sh
pixi init ros2 -c robostack-staging -c conda-forge
```
