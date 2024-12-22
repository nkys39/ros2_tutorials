# データ保存/再生ツールros2bagについて
開発を行う上でデータの保存と再生ができると非常に便利です。
ROS2のデータ保存/再生ツールros2bagの使い方を学びましょう。

---

## ros2bagの概要
ros2bagは、トピックデータを時系列に保存と再生ができるツールです。  
センサデータなどを保存しておけば、現地実験のデータに対して何回でもアルゴリズム検証などを行えます。

---

## ros2bagの使い方

### データの保存
トピックデータの保存は`ros2 bag record [トピック名]`で行えます。  
`-o [保存先パス]`で任意名のbagファイルを作成できます。(指定なしの場合は日付)  
また、複数のトピックを保存したい場合は、スペース区切りで繋げます。  
トピック名なので/から始まるので注意です。  
全てのトピックを指定せずに保存する場合は`-a`オプションをつけることで保存できますが、多くのトピックが流れている場合は、保存に失敗するケースが出るので注意して使用してください。
```sh
# 任意トピックデータの保存(1個のトピックを保存、ファイル名は実行日時になる)
ros2 bag record [トピック名]
# 名前をつけて保存、任意トピックデータの保存(複数の場合はスペース区切り)
ros2 bag record -o [保存先パス] [トピック名] [トピック名]
# 全てのトピックを保存
ros2 bag record -a
```

例えば、自律走行関連の場合、現地でのデータ収集に時間が取れないが、地図を上手く作成するためにパラメータやアルゴリズムを変更して、実施したい場合などがあります。

その時は一般的なSLAMの入力である、下記などを保存しておけば、現地実験のあとで落ち着いて検証できるのでオススメです。

- `/scan[senseor_msgs/msg/LaserSCan]`:2D点群データ  
- `/pointcloud[sensor_msgs/msg/PointCloud2]`:3D点群データ
- `/image[sensor_msgs/msg/Image]`:画像データ
- `/imu[sensor_msgs/msg/IMU]`:9軸センサデータ
- `/odom[nav_msgs/msg/Odometry]`:オドメトリデータ
- `/tf`:座標関係
- `/tf_static`:静的な座標関係


また、近年では保存方式のプラグインが追加され、デフォルトの`sqlite3`以外に読み書き性能の高い`mcap`が使えます。  
使い方や性能比較に関しては[こちら](https://proc-cpuinfo.fixstars.com/2023/01/rosbag2_storage_mcap_usage/)に詳しい解説があるのでご確認ください  

---

### データの確認
データの保存が上手くできているかを確認するためには`ros2 bag info [bagファイル名]`を使います。

```sh
# ros2 bag info [bagファイル名]
ros2 bag info subset
```
出力例は下記のようになります。保存されているデータ数を確認して保存できているか確認します。
```sh
Files:             subset.db3
Bag size:          228.5 KiB
Storage id:        sqlite3
Duration:          48.47s
Start:             Oct 11 2019 06:09:09.12 (1570799349.12)
End                Oct 11 2019 06:09:57.60 (1570799397.60)
Messages:          3013
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 9 | Serialization Format: cdr
                 Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 3004 | Serialization Format: cdr
```

---

### データの再生
bagファイルの再生には`ros2 bag play [bagファイル名]`を使用します。
```sh
ros2 bag play [bagファイル名]
```


### データの編集
bagデータの編集には`ros2 bag convert`を使用します。

#### 特定トピックの抜き出し(filter処理)
filter.yamlファイル(任意名)を下記のように用意します。
```yaml
output_bags:
- uri: split1                 # (必須)保存したいデータ名
  storage_id: sqlite3         # (必須)元データの形式(デフォルトではsqlite3で、他形式にはmcapなどがある)
  topics: [/topic1, /topic2]  # 抜き出して保存したいトピック名
```
作成したfilter.yamlを使用して特定トピックの抜き出しを行う場合には下記のように実行します。
```sh
ros2 bag convert -i [変換元のbagデータパス] -o filter.yaml
```
上記実行後、同階層に`split1`というディレクトリができ、特定トピックのみが抜き出されます。

## ROS1/2の間のbagデータ変換
ROS1とROS2ではbagファイルのデータ形式が異なります。これの相互変換には便利な`rosbags`というものがあるのでこれを利用します。  
これを使用することでインターネット上に公開されているROS1のbagデータや、ROS1を利用していたときに社内で保存しているbagファイルをROS2形式に変換可能です。(ROS1→ROS2の移行作業にも役立ちます)  

https://github.com/rpng/rosbags

https://pypi.org/project/rosbags/
### インストール方法
```bash
pip install rosbags
```

### 使用方法

`rosbags-convert`コマンドが見つからない場合には、`export PATH=$PATH:~/.local/bin`コマンドを実行します。`~/.bashrc`に記述しておけば、端末を開き直した際に読み込まれるので便利です。
```bash
# ROS1→ROS2(下記の例ではROS2bagデータのtest/ディレクトリが同階層に生成されます)
rosbags-convert test.bag
# --dstオプションで、好きな場所に任意名のフォルダを保存できます
rosbags-convert test.bag  --dst ~/[任意名]/
# ROS2→ROS1(下記の例ではROS1bagデータのtest.bagが同階層に生成されます)
rosbags-convert test/
# --dstオプションで、好きな場所に任意名のbagファイルを保存できます
rosbags-convert test/  --dst ~/[任意名].bag
```

`ros2 bagコマンド`は自前のデータセットなどを作成できるので非常に便利です。  
PythonやC++プログラムでも使用することができ、使いこなすと非常に開発効率が上がるので[本家のREADME](https://github.com/ros2/rosbag2)も読んでおくと良いと思います。