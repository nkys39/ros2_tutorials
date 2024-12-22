# Ubuntu22.04でのROS2Humble環境構築(pixiによる簡略版)
## aptを最新の状態にする

```sh title=""
sudo apt update
sudo apt upgrade
```


## 開発時に良く使うものをインストールするをaptでインストールする

```sh title=""
sudo apt -y install curl gnupg lsb-release vim openssh-server net-tools git wget cmake
```


## (オプション)クラッシュレポートを非表示にする
開発時にROS2関連のプログラムやツールを使用していると予期せぬプログラム終了が起こることが多々ある。その際にクラッシュレポートが毎回出るのが邪魔になることがあるため、非表示オプションを記載する。
```sh
sudo sed -i 's/enabled=1/enabled=0/' /etc/default/apport
```

## pixiのインストール
```sh
curl -fsSL https://pixi.sh/install.sh | bash
```

## pixiプロジェクトの作成
```sh
pixi init ros2 -c robostack-staging -c conda-forge
```
