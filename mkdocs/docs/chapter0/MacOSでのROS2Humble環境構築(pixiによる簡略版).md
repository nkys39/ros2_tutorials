# MacOSでのROS2Humble環境構築(pixiによる簡略版)



## pixiのインストール
```sh
curl -fsSL https://pixi.sh/install.sh | bash
```

## pixiプロジェクトの作成
```sh
pixi init ros2 -c robostack-staging -c conda-forge
```
