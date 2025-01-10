# ROS2による自律走行のチュートリアル

本資料では、ロボット用ミドルウェアであるROS2 Humbleを用いてロボットの自律走行を体験します。   
  
自律的なロボットのソフトウェアは、ロボットが認識、計画、動作を行う際に、多数のソフトウェアを並列的に実行する必要があります。  
ロボット用ミドルウェアは、分割されたソフトウェアを、相互に通信させるための仕組みを提供するため、このような実行に際し、開発スピードを向上させるための重要なキーテクノロジーとなります。   
そのため、自律走行ロボットの開発には、商用利用可能なオープンソースであるROS2がよく使用されます。     
ROS2は有名アルゴリズムから最新研究までオープンソースで公開されており、その数は増え続けています。  

自律走行ロボットの開発は、アルゴリズムを手元で動かすことで、非常に理解が捗ります。  
現時点でROS2はSLAMや自律走行ロボットを開発するための最短の近道です。  

  

---

## 事前準備
本資料ではOSはubuntu22.04、ROS2のバージョンはHumbleを推奨環境としています。  
資料通りに進めたい場合にはUbuntu22.04環境を用意することをオススメします。  
本資料ではWindows11やMacOSの環境でもお試しができるように`pixi`というRust製のパッケージマネージャを用いた環境構築方法を記載しております。  
様々なOSで高速にROS2環境を構築することができますが、かなり最近出てきた方法であるため、ネット上に情報が少なく、環境構築につまずいた際には自力で解決する必要があります。  
これらのインストールは【環境構築】の章を参考にインストールしてください。

!!! pixiで環境構築した際の注意点について
    `pixi`は`conda`と同じようにローカル環境をプロジェクトとして作成し分離できます。`conda`は`conda activate / deactivate`で環境を切り替えますが、`pixi`は`pixi shell`で行います。`pixi`で環境構築した場合、資料中の新しい端末を開く記述がある場合には、`pixi shell`を毎回行いローカル環境でコマンドを実行するようにしてください。

---


## 資料構成

- [【環境構築】](chapter0/index.md)
- [【Navigation2体験】navigation2を用いた地図構築(SLAM)と自律走行の使い方](chapter3/index.md)
- [【補足資料_ROS2の基礎】ロボット用ミドルウェアROS2の概要とROS2コマンドの使い方](chapter1/index.md)
- [【補足資料_ROS2のツール】シミュレータ(Gazebo)とROS2関連ツールの使い方](chapter2/index.md)
- [【補足資料_Navigation2】自律走行の概要とnavigation2の構成](chapter5/index.md)

---

## 参考サイトや書籍
- 参考サイト
    - [ROS2の公式ドキュメント](https://docs.ros.org/en/humble/index.html)
    - [navigation2の公式ドキュメント](https://navigation.ros.org/)
    - [pixiの日本語での紹介記事](https://zenn.dev/yahooshiken/articles/getting-started-ros2-with-pixish)
    - [pixiの本家サイト](https://pixi.sh/dev/)

- 参考書籍
    - [ROS2ではじめよう 次世代ロボットプログラミング](https://www.youtalk.jp/get-started-ros2/)
        - ROS2で初めて出たの入門書
        - ROS2に関しては一番詳細に書かれている
        - 初版はUbuntu18.04 + Dashingが使われているがFoxyも対応
        - 改訂版は最新のJazzyに対応
        - [サンプルコード](https://github.com/youtalk/get-started-ros2)がgithubで公開されている
        - 改定版が出ており、最新のJazzyに対応している

    - [ScamperとRaspberry Piで学ぶROS2プログラミング入門](https://www.ohmsha.co.jp/book/9784274226809/)
        - Ubuntu20.04 + ROS2 Foxyが使われている
        - ROS2でC++を扱う方法について詳細に書いてある
    - [ROS2とPythonで作って学ぶAIロボット入門](https://bookclub.kodansha.co.jp/product?item=0000368702)
        - Ubuntu20.04 + ROS2 Foxyが使われている
        - ROS2でPythonを扱う方法ついて書いてある
        - Dockerを用いた環境構築方法の記載がある
        - 各種アルゴリズムの説明図が記載されている
        - [サポートページ](https://github.com/AI-Robot-Book)がgithub上に公開されている

---

# お問い合わせ
資料に関するお問い合わせや開発相談はメールか電話で承ります。

## 中村佳雅
## メール：nakamura.yoshimasa@iri-tokyo.jp
## 電話番号は[このリンク先](https://www.iri-tokyo.jp/about/organization/headquarters-dx/robot/)の最下部にございます。中村をお呼び出しください。



