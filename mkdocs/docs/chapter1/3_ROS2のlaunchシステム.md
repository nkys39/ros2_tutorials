# ROS2のlaunchシステム

---

## 1.launchシステムとは
ROS2では、ノードを起動し、複数組み合わせることで機能を実現します。  複数のノードを起動する際、1個の端末につき1個のノードしか起動できない`ros2 run`コマンドでは、非常に不便です。  

これを解決するシステムがlaunchシステムと呼ばれるものです。

launchシステムは`ros2 launch`コマンドを使用し、launchファイルに記述された複数のros2ノードを一括で起動できます。   
launchファイルには、`ros2 run`コマンドのオプションを含めて全ての機能を記述することができます。  
極論として、オープンソースROS2パッケージのソースコードを改変しない場合、launchファイルの記述のみで、ある程度の自律走行ロボットが完成します。  


__<mark>launchシステムは複数のノードを一括で起動するシステム</mark>__  

---

## 2.launchファイルの種類
ROS2のlaunchファイルは、`Python`,`XML`,`YAML`の3種類で記述できます。どれを使うかはユーザの好みで選択して構いませんが、launchシステムはPython言語によって記述されているため、Pythonが最も柔軟な記述ができます。   
機能は限定されているが簡潔に記述可能なXMLやYAML、機能は豊富だが記述が冗長になるPythonという位置づけになります。  
まず覚えるべきは、Pythonでの記述です。理由はオープンソースパッケージの大半がPythonでlaunchを記述しているからです。  
XMLやYAMLは、launchシステムの機能がある程度理解できていれば、簡潔に記述されているため、記述できなくても、何をしているかは見ればわかります。  
Pythonでのlaunchの書き方に慣れた段階で、XMLまたはYAMLにしていくのが良いと思います。  

- 参考(launchシステムのソースコード)
    - ros2/launch(一般的な起動機能)(https://github.com/ros2/launch)
    - ros2/launch_ros(ROS2固有の起動機能)(https://github.com/ros2/launch_ros)

__<mark>launchファイルの書き方は3種類あるが、Pythonでの記述を覚えておくとよい</mark>__

---

## 3.launchファイルの作成と配置
launchファイルの作成は下記コマンドやvscodeエディタ等のファイル作成で行えます。


```bash
touch test_launch.py
code test_launch.py
```

拡張子は、launchファイルの種類により`.py`,`.xml`,`.yaml`,と記述され、拡張子の前に`_launchや.launch`が付いていることが多いです。    
一般的には、launchファイルだと判別できるようにlaunchというディレクトリを作成し、それ以下に配置します。  
また、apt等でインストールしたパッケージ内にもlaunchファイルがサンプルとしてある場合がありますので、作成する際の参考にすると良いかと思います。  
ファイルの配置は、`/opt/ros/[rosバージョン]/share/[パッケージ名]/launch/`以下にあることが多いです。   
例えば、turtlesimパッケージには、`/opt/ros/humble/share/turtlesim/launch/`以下に、`multisim.launch.py`というlaunchファイルがあります。

__<mark>パッケージ内のlaunchファイルの配置はlaunchディレクトリに置く</mark>__

---

## 4.launchファイルの実行
launchファイルの実行は`ros2 launch`コマンドによって行います。  
パッケージ内にlaunchファイルがある場合は、パッケージ名でlaunchファイルのパスを省略できます。  
launchファイル内に変数が定義されている場合、実行時に値を変更できます。  


```bash
ros2 launch [launchファイルパス]
ros2 launch [パッケージ名] [launchファイル名]
ros2 launch [パッケージ名] [launchファイル名] [変数名]:=[設定値]
```

__<mark>launchファイルに変数宣言しておくと実行時に変更できる</mark>__

---

## 5.launchファイル作成手順(簡易版)
ワークスペース作成から`ros2 launch [パッケージ名] [launchファイル名]`コマンドが実行できるようになるまでの手順を下記に記述する。3~5は実行順は自由だが、説明の都合で下記の順で行う。

1. ワークスペースの作成
2. パッケージの作成
3. launchファイルの作成と配置
4. package.xmlの追記(launch依存関係の記述)
5. CMakeLists.txtの追記(launchディレクトリ設定)
6. colconによるパッケージのビルドとlaunch実行


---

### 5-1.ワークスペースの作成
まず、ROS2のワークスペースを作成します。  
既存のワークスペースがある場合は不要です。


```sh
mkdir -p ~/[ワークスペース名]/src/
cd ~/[ワークスペース名]/
colcon build --symlink-install
echo "source ~/[ワークスペース名]/install/setup.bash" >> ~/.bashrc
```

---

### 5-2.パッケージの作成

launch用パッケージの作成では、C++用の設定(ament_cmake)をオススメします。  
詳細理由は後述しますが、Python用設定の場合、launchファイル修正のたびにパッケージのビルドが必要になるからです。


```bash
cd ~/[ワークスペース名]/src/
ros2 pkg create --build-type ament_cmake [パッケージ名]
```

ament_cmakeでパッケージを作成すると、`CMakeLists.txt`,`package.xml`,`includeディレクトリ`,`srcディレクトリ`が自動生成されます。`includeディレクトリ`,`srcディレクトリ`は今回は不要のため削除します。(悪影響はないため消さなくてもよいです)


```sh
cd ~/[ワークスペース名]/src/[パッケージ名]
rm -rf include/ src/
```

---

### 5-3.launchファイルの作成と配置
launchファイルは、一般的にパッケージ直下にlaunchディレクトリを作成し配置します。


```bash
cd ~/[ワークスペース名]/src/[パッケージ名]/
mkdir launch
cd launch
code [launchファイル名].launch.py
```

```python title="[launchファイル名].launch.py" linenums="1"
#!/usr/bin/env python3
# config:utf-8
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
    )
    ld = LaunchDescription()
    ld.add_action(turtlesim_node)
    return ld
```

---

### 5-4.package.xmlの追記(launch依存関係の記述)
launchファイルを含むパッケージの場合は、依存関係の記述として`package.xml`に`<exec_depend>ros2launch</exec_depend>`を追加します。記述位置は、下記になります。


```xml title="package.xml" linenums="1"
<!-- --- -->
  <exec_depend>ros2launch</exec_depend>
<!-- ---以下は自動生成されている-->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
<!-- --- -->
```

---

### 5-5. CMakeLists.txtの追記(launchディレクトリ設定)
launchディレクトリをinstallディレクトリ以下にコピーまたはシンボリックリンクすることで、`ros2 launch [パッケージ名]  [launchファイル名]`コマンドで起動することが可能になります。(`ros2 launch [launchファイルパス]`とすることでも起動可能)  
これをビルド時に自動で行う設定をCMakeLists.txtに記述します。


```bash
cd ~/[ワークスペース名]/src/[パッケージ名]/
code CMakeLists.txt
```
CMakeLists.txtのfind_package()以下にlaunchディレクトリ以下をinstallする記述を追加
```Cmake title="CMakeLists.txt" linenums="1"
#---これは自動生成
# find dependencies
find_package(ament_cmake REQUIRED)
#---下記をこの位置程度に追加
# install directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
#---
```

---

### 5-6. colconによるパッケージのビルドとlaunch実行
`--symlink-install`オプションを付けて、ビルドすると、`~/[ワークスペース名]/install/[パッケージ名]/share/[パッケージ名]/launch/`以下にlaunchファイルのシンボリックリンクが生成される(オプション無しの場合はコピーされる)。    
実行時にはinstall以下のlaunchファイルが使用されます。  


```bash
cd ~/[ワークスペース名]/
colcon build --symlink-install --packages-select [パッケージ名]
# 新規端末を開く、または下記コマンドを実行し、ワークスペースの設定を読み込む
source ~/[ワークスペース名]/install/setup.bash
ros2 launch [パッケージ名] [launchファイル名]
```

新規でlaunchファイルを追加するたびにビルドが必要になります。  
上記のオプション無しの場合は、コピーされているため`~/[ワークスペース名]/src/[パッケージ名]/launch/`以下のlaunchファイルを修正しても、コピー先(install以下)には反映されません。  
この場合再ビルドすることで再度コピーされます。   
オプションを付けてシンボリックリンクにしておくと、修正時にビルドする必要がなくなります。    
PythonではCMakeLists.txtの代わりに、setup.pyでビルド時にinstallにコピーする設定を下記のように記述しますが、現状ではsetup.pyで用いているsetuptoolsがシンボリックリンクを生成できないため、launchファイル修正時にビルドが必要です。  
これについてはいずれ解決するかもしれませんが、現状ではament_cmakeでパッケージを生成した方が混乱しづらいと思います。

---

### ament_pythonでパッケージを生成した場合のsetup.pyへの追記(launchディレクトリ設定)
```bash
cd ~/[ワークスペース名]/src/
ros2 pkg create --build-type ament_python [パッケージ名]
cd ~/[ワークスペース名]/src/[パッケージ名]/
code setup.py
```
```python title="setup.py" linenums="1"
import os
from glob import glob
from setuptools import setup

package_name = 'my_package'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ]
)
```

---

## 6.各種類のlaunchファイルの書き方(抑えておきたい部分のみ)
【minimal】から【中級】までは全種類のlaunchで実現できます。このレベル分けは個人的見解です。  

---


### 【minimal】
1個のノードを起動するのに必要最低限の記述  

__python版launchの書き方__
```python title="minimal.launch.py" linenums="1"
#!/usr/bin/env python3
# config:utf-8
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
    )
    ld = LaunchDescription()
    # turtlesim_nodeの起動
    ld.add_action(turtlesim_node)
    return ld
```


__xml版launchの書き方__
```xml title="minimal.launch.xml" linenums="1"
<?xml version="1.0"?>
<launch>
    <!-- turtlesim_nodeの起動  -->
    <node pkg="turtlesim" exec="turtlesim_node" />
</launch>
```


__yaml版launchの書き方__
```yaml title="minimal.launch.yaml" linenums="1"
launch:
# turtlesim_nodeの起動
- node:
    pkg: "turtlesim"
    exec: "turtlesim_node"
```

---

### 【初級】
- 必須レベル
    - Nodeの起動(ノード名変更)
    - パラメータ値変更(名前指定,yaml読み込み)
    - 【remap】メッセージ通信(トピック,サービス,アクション)名変更
    - パッケージパス検索
    - 変数定義
    - ログレベルの変更


__python版launchの書き方__
```python title="beginner.launch.py" linenums="1"
#!/usr/bin/env python3
# config:utf-8
from os.path import join # パスの検索時に使用するクラス
from ament_index_python.packages import get_package_share_directory # パッケージのinstall/shareディレクトリのパス検索クラス
from launch.actions import DeclareLaunchArgument # 外部から変更可能な変数を宣言するクラス
from launch.substitutions import LaunchConfiguration # 引数にパラメータ名を記述し設定値を参照するクラス
from launch_ros.actions import Node # ノードの実行情報を記述するクラス(ros2 runコマンドに相当)
from launch import LaunchDescription # 引数に渡したノード実行情報を実行するクラス

# launch起動時に使用される関数の宣言(名前変更不可)
def generate_launch_description():
    background_r_launch_arg = DeclareLaunchArgument( # 外部から変更可能な変数を宣言
        'background_r', 
        default_value='255',
        description='背景の赤色の値'
    )
    
    turtlesim_node = Node(  # turtlesim_nodeの起動定義
        package='turtlesim', # パッケージ名
        executable='turtlesim_node', # ノード名
        name='turtlesim', # 起動ノード名(ノード名の変更)
        parameters=[  # パラメータ値の変更(名前指定)
            {'background_r': LaunchConfiguration('background_r')}, # パラメータ値を参照する
            {'background_g': 0},
            {'background_b': 0}
        ],
        # parameters=[ # パラメータ値の変更(yaml読込)
        #     join( # パラメータyamlファイルのパス検索 
        #         get_package_share_directory('launch_samples'),
        #         'config',
        #         'turtlesim.yaml'
        #         # 'wildcard.yaml'
        #     )
        # ],
        remappings=[ # 【remap】メッセージ通信(トピック,サービス,アクション)名変更
            ('/turtle1/cmd_vel', '/cmd_vel'),
        ],
        output={ # ログ出力先の指定
            'stdout': 'log',
            'stderr': 'log'
        },       
        arguments=['--ros-args', '--log-level', 'warn'], # ログレベルは下からDebug,Info,Warn,Error,Fatal
    )
    
    turtle_teleop_key_node = Node( # turtle_teleop_keyノードの起動定義
        package='turtlesim',
        executable='turtle_teleop_key',
        remappings=[
            ('/turtle1/cmd_vel', '/cmd_vel'),
        ],
        prefix='gnome-terminal --', # キーボードインプットを受け付けないため、別ターミナルで実行する
    )
    
    ld = LaunchDescription()  # launch起動時に使用される関数の戻り値(起動するものをここに追加する)
    ld.add_action(background_r_launch_arg) # 外部から変更可能な変数を宣言
    ld.add_action(turtlesim_node) # turtlesim_nodeの起動
    ld.add_action(turtle_teleop_key_node) # turtle_teleop_keyノードの起動
    return ld
```


__xml版launchの書き方__
```xml title="beginner.launch.xml" linenums="1"
<?xml version="1.0"?>
<launch>
    <!-- 外部から変更可能な変数を宣言 -->
    <arg name="background_r" default="255" description="背景の赤色の値"/>

    <!-- turtlesim_nodeの起動 -->
    <!-- <node pkg="パッケージ名" exec="ノード名" name="起動ノード名" namespace="名前空間"> -->
    <!-- warnは起動ノード名:=warnでも可能 -->
    <!-- ログレベルは下からDebug,Info,Warn,Error,Fatal -->
    <node pkg="turtlesim" exec="turtlesim_node" name="turtlesim" args="--ros-args --log-level warn">
        <!-- パラメータ値の変更(名前指定) -->
        <!-- パラメータ値を参照する -->
        <param name="background_r" value="$(var background_r)"/>
        <param name="background_g" value="0"/>
        <param name="background_b" value="0"/>
        <!-- パラメータ値の変更(yaml読込) -->
        <!-- <param from="$(find-pkg-share launch_samples)/config/turtlesim.yaml"/> -->
        <!-- 【remap】メッセージ通信(トピック,サービス,アクション)名変更 -->
        <remap from="/turtle1/cmd_vel" to="/cmd_vel"/>
    </node>

    <!-- turtle_teleop_keyノードの起動 -->
    <node pkg="turtlesim" exec="turtle_teleop_key" launch-prefix="gnome-terminal --">
        <remap from="/turtle1/cmd_vel" to="/cmd_vel"/>
    </node>
</launch>
```


__yaml版launchの書き方__
```yaml title="beginner.launch.yaml" linenums="1"
launch:
# 外部から変更可能な変数を宣言
- arg:
    name: "background_r"
    default: "255"
    description: "背景の赤色の値"

# turtlesim_nodeの起動
- node:
    # パッケージ名
    pkg: "turtlesim"
    # ノード名
    exec: "turtlesim_node"
    # 起動ノード名(ノード名の変更)
    name: "turtlesim"
    # パラメータ値の変更(名前指定)
    param:
    - name: "background_r"
      # パラメータ値を参照する
      value: "$(var background_r)"
    - name: "background_g"
      value: 0
    - name: "background_b"
      value: 0
    # パラメータ値の変更(yaml読込)
    # - from: "$(find-pkg-share launch_samples)/config/turtlesim.yaml"
    # 【remap】メッセージ通信(トピック,サービス,アクション)名変更
    remap:
    - from: "/turtle1/cmd_vel"
      to: "/cmd_vel"
    # warnは起動ノード名:=warnでも可能
    # ログレベルは下からDebug,Info,Warn,Error,Fatal
    args: "--ros-args --log-level warn"

# turtle_teleop_keyノードの起動
- node:
    pkg: "turtlesim"
    exec: "turtle_teleop_key"
    remap:
    - from: "/turtle1/cmd_vel"
      to: "/cmd_vel"
    # キーボードインプットを受け付けないため、別ターミナルで実行する
    launch-prefix: "gnome-terminal --"
    
```

---

### 【中級】
- ノード数が増えてくると必須レベル(個人的には5〜10個以上)
    - include
    - Nodeの起動(名前空間)
    - グループ
    - if文
    - コマンド入力



__python版launchの書き方__
```python title="intermediate.launch.py" linenums="1"
#!/usr/bin/env python3
# config:utf-8
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

# 他のlaunchファイルを再利用する際に使用するクラス
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource,PythonLaunchDescriptionSource

# 他のlaunchファイルを再利用する際に名前空間を追加するクラス
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
# if,unless文を使用するクラス
from launch.conditions import IfCondition,UnlessCondition
# コマンド入力を実行するクラス
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 再利用するlaunchファイルのパスを検索
    launch_file_path = join(get_package_share_directory('launch_samples'),
        'launch','basic','basic.launch.py')
    # 他のlaunchファイルを再利用して使用する
    launch_include = IncludeLaunchDescription(
        # 再利用するlaunchファイルのパスを指定
        AnyLaunchDescriptionSource(launch_file_path),
        # 再利用するlaunchファイル内の変数値を変更する
        launch_arguments={'background_r': '50'}.items(),
    )

    # 再利用するlaunchファイルに名前空間を追加して使用する
    launch_include_with_namespace = GroupAction(
        actions=[
            # 名前空間を追加する(/名前空間/ノード名となる)
            PushRosNamespace('test'),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(launch_file_path),
                launch_arguments={'background_r': '150'}.items(),
                # if文の使用
                condition=IfCondition('false')
            ),
        ]
    )
    
    # コマンド入力(/spawnサービスを実行)
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 1, y: 1, theta: 1.57, name: \'\'}"'
        ]],
        cwd="/home/user",
        name="my_exec",
        shell=True,
        output="screen"
    )

    ld = LaunchDescription()
    ld.add_action(launch_include)
    ld.add_action(launch_include_with_namespace)
    ld.add_action(spawn_turtle)
    return ld
```



__xml版launchの書き方__
```xml title="intermediate.launch.xml" linenums="1"
<?xml version="1.0"?>
<launch>
    
    <!-- 他のlaunchファイルを再利用して使用する-->
    <include file="$(find-pkg-share launch_samples)/launch/basic/basic.launch.xml">
        <!-- 再利用するlaunchファイル内の変数値を変更する -->
        <arg name="background_r" value="50"/>
    </include>
    <!-- 再利用するlaunchファイルに名前空間を追加して使用する-->
    <group>
        <!-- 名前空間を追加する(/名前空間/ノード名となる) -->
        <push-ros-namespace namespace="test"/>
        <!-- if文の使用 -->
        <include file="$(find-pkg-share launch_samples)/launch/basic/basic.launch.xml" if="true">
            <arg name="background_r" value="150"/>
        </include>
    </group>
    
    <!-- コマンド入力(/spawnサービスを実行) -->
    <executable cmd="ros2 service call /spawn turtlesim/srv/Spawn &quot;{x: 1, y: 1, theta: 1.57, name: &apos;&apos;}&quot;" cwd="/home/user" name="my_exec" output="screen" />
    
</launch>
```



__yaml版launchの書き方__
```yaml title="intermediate.launch.yaml" linenums="1"
launch:
# 他のlaunchファイルを再利用して使用する
- include:
    file: "$(find-pkg-share launch_samples)/launch/basic/basic.launch.yaml"
    # 再利用するlaunchファイル内の変数値を変更する
    arg:
    - name: "background_r"
      value: "50"

# 再利用するlaunchファイルに名前空間を追加して使用する
- group:
    # 名前空間を追加する(/名前空間/ノード名となる)
    - push-ros-namespace:
        namespace: "test"
    - include:
        file: "$(find-pkg-share launch_samples)/launch/basic/basic.launch.yaml"
        arg:
        - name: "background_r"
          value: "150"
        # if文の使用
        if: "false"
# コマンド入力(/spawnサービスを実行)
- executable:
    cmd: ros2 service call /spawn turtlesim/srv/Spawn "{x:\ 1, y:\ 1, theta:\ 1.57, name:\ ''}"
    cwd: "/home/user"
    name: "my_exec"
    output: "screen"
    
```

---

### 【上級】python版のみ
- 中級までで実現できない特別な起動や、起動順序を制御したい時に用いる
    - イベントハンドラー
    - タイマー起動


__python版launchの書き方__
```python title="advanced.launch.py" linenums="1"
#!/usr/bin/env python3
# config:utf-8
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable, LaunchConfiguration, LocalSubstitution, PythonExpression)

def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')
    
    turtlesim_ns_launch_arg = DeclareLaunchArgument('turtlesim_ns', default_value='turtlesim1')
    use_provided_red_launch_arg = DeclareLaunchArgument('use_provided_red', default_value='False')
    new_background_r_launch_arg = DeclareLaunchArgument('new_background_r', default_value='200')
    
    turtlesim_node = Node(package='turtlesim', namespace=turtlesim_ns, executable='turtlesim_node', name='sim')
    spawn_turtle = ExecuteProcess(
        cmd=[[FindExecutable(name='ros2'), ' service call ', turtlesim_ns, '/spawn ', 'turtlesim/srv/Spawn ', '"{x: 2, y: 2, theta: 0.2}"']],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[FindExecutable(name='ros2'), ' param set ', turtlesim_ns, '/sim background_r ', '120']],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([new_background_r, ' == 200', ' and ', use_provided_red])
        ),
        cmd=[[FindExecutable(name='ros2'), ' param set ', turtlesim_ns, '/sim background_r ', new_background_r]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'), spawn_turtle]
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawn_turtle,
                on_stdout=lambda event: LogInfo(msg='Spawn request says "{}"'.format(event.text.decode().strip()))
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    change_background_r,
                    TimerAction(period=2.0, actions=[change_background_r_conditioned])
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=turtlesim_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'), ' closed the turtlesim window')),
                    EmitEvent(event=Shutdown(reason='Window closed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(msg=['Launch was asked to shutdown: ', LocalSubstitution('event.reason')])]
            )
        ),
    ])
```

---

## 忘れがちなトラブル
- パラメータのyamlファイルを追加したのに、ビルドしてなくて、installフォルダにyamlファイルがないためパラメータが反映されない。


---