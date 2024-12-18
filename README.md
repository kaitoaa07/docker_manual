# docker_manual

dockerを導入して使用する手順を記します。
今回は主にRaspberry pi 5を用いてdockerを使用する状況を想定します。

## dockerの概要

Dockerを使用するメリットとしてはOS等の環境設定やWebページの作成を手軽に行うことができる点です。\
また、Dockerではコンテナというワークスペースを作成しますが、設定をしない場合、そのワークスペースからhost(今回でいうならRaspberry Pi)にアクセスできないという特徴があります。\
その為Webページ等を公開で作成してもhostを攻撃されないため、安全にWebでの作業を行うことができます。

Dockerはhostのカーネル(CPUの脳のようなもの)を共通で使用します。\
類似の物としてVirtualBoxが挙げられますが、こちらはVirtualBox内にそれぞれカーネルを作成するため動作が重くなります。また、導入もDockerに比べて困難です。\
Dockerを使用する注意点としてはカーネルを共通で使用する仕様上、Linux系のOS上でしか実行できません。その為、Linux系以外でDockerを使用したい場合はVirtualBox上でDocker環境を作成する必要があります。


この説明ではイメージ図を表示するのが難しいためよくわからなかったら聞いてください

## dockerのインストールと初期設定

Raspberry Pi OSの準備
```
sudo apt update && sudo apt upgrade -y
```
dockerインストールコマンド
```
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```
インストールできたかの確認
```
docker --version
```
例：Docker version 24.x.x と表示されれば成功です。



##  ユーザーを docker グループに追加

sudo を毎回使わずに Docker を実行できるようにするには、現在のユーザーを docker グループに追加します。
```
sudo usermod -aG docker $USER
```
変更を反映するために再起動または以下のコマンドを実行します。
```
newgrp docker
```

### Docker Composeのインストール

Docker Compose は複数のコンテナを管理する際に便利です。
```
sudo apt install -y docker-compose
```
インストールの確認
```
docker-compose --version
```

## Dockerコンテナの作成と実行
dockerの基本的な実行方法としてdocker runを使用します。\
docker pullで公開されているimageをインストールできますが、docker runでコンテナの作成とimageのインストールを同時に行うことが出来るのでdocker runを使用することを推奨します。\
使い方として、``docker run -it --name <自分でつけたいコンテナ名> <image名>``という風に使用します。
以下にdockerを使用してUbuntu22.04を使用する方法を示します。
```
docker run -it --name <コンテナ名> ubuntu:22.04
```
一般的に使用頻度が高く、公開されているimage名はdocker hubというサイトに掲載されています。




上記のcodeを実行後は、Ubuntu22.04の環境が準備されたコンテナ内に入ります。\
Raspberry piの環境と分断されるのですべてのコマンドをUbuntuで使用するものに変更する必要があります。\
例えば、sudoコマンドは初期で導入されてないためaptコマンドでsudoをインストールする必要があります。



## コンテナ作成後のマニュアル

dockerコンテナはhost(今回で言うとRaspberry Pi)を再起動した際に自動的に停止されます。\
そのため、hostの再起動後はコンテナも再起動させる必要があります。\
環境設定を行えば、hostの起動時に自動でコンテナの起動も行うことはできますが、後々重くなる可能性も考えられるので、今回の記載は控えます\
作成済みコンテナの表示(起動中のコンテナのみ)
```
docker ps
```
作成済みコンテナの表示（停止中のコンテナも）
```
docker ps -a
```
停止中コンテナの起動
```
docker restart <コンテナ名>
```
すべての停止中コンテの起動（使ったことないのでよくわかりません）
```
docker start $(docker ps -a -q)
```

コンテナ内でシェルを起動して操作する場合
```
docker exec -it <コンテナ名> /bin/bash
```
特定のコマンドをコンテナ内で実行し、結果を取得したい場合（使ったことないのでよくわかりません）
```
docker exec <コンテナ名またはID> <コマンド>
```

### display(GUI)を使用したい場合
```
sudo apt install x11-xserver-utils
```
hostでdockerにdisplayの使用を許可する\
このコマンドはhost再起動時のたびに実行する必要があります。環境設定を行えば自動で実行されます。
```
xhost +local:docker
```
Dockerでディスプレイ(X11と表記される)を使う場合、ホストシステムがコンテナに対してアクセスを許可していないことがあります。以上のコマンドでアクセス許可を与えます。\
環境設定をしていない場合はディスプレイを使用したいたびに実行する必要があります。

docker コンテナの作成
```
docker run -it --net=host --privileged --device=/dev/video0:/dev/vide0 --device=/dev/vchiq -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --name humble-picamera2 ros:humble
```




すべてのデバイスのアクセスを許可する場合--device=/dev/video0:/dev/vide0 --device=/dev/vchiqの部分を-dに変更する \
※危険
```
docker run -it --net=host --privileged -d -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --name humble-picamera2 ros:humble
```
※--rmをつけるとシェルを閉じるときにコンテナごと削除する

### docker imageの作成と使用
imageを自作することもできます。\
メリットとして、自作するとコンテナ作成時の環境を設定することができます。\
例えば、Ubuntuの初期環境では入っていないライブラリを入れた状態でコンテナの作成などが可能です。\
以下にros2のhumbleというディストリビューションが使用できる環境とその他の設定が行えるimage例を示します。

```
nano Dockerfile
```
```
# ROS 2 Humbleの公式イメージをベースに使用
FROM ros:humble

# 必要なパッケージ（byobu、nano）のインストール
RUN apt-get update && apt-get install -y \
    byobu \
    nano \
    && rm -rf /var/lib/apt/lists/*

# ROS 2の環境設定スクリプトをsourceするために必要なコマンド
SHELL ["/bin/bash", "-c"]

# ROS 2 Humbleのセットアップを行うコマンド
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 作業ディレクトリの設定（必要に応じて変更）
WORKDIR /workspace

# コンテナ起動時にbashを実行
CMD ["bash"]
```
```
docker build -t <イメージ名> .
```
```
docker images
```
```
docker run -it <イメージ名>
```
```
docker run -it --net=host --privileged -d --name <コンテナ名> <作成したイメージ名>
```
byobu　使い方    \
shiht + F2 ターミナルを分割（横に）おすすめはこっち    \
ctrl + F2  ターミナルを分割（縦に）    \
shiht + F3 ターミナル移動（右または下に）    \
shiht + F4 ターミナル移動（左または上に）

ros2を使用する人向けですが、byobuというライブラリの仕様を推奨します。\
GUIが使用できる環境であればVScodeからdocker コンテナに接続できるものもあるのでそちらも便利です。\
ros2ではnodeごとにターミナルを作成する必要があるためどちらかを使用する必要があります。

### docker compose

docker composeという複数のコンテナを一括で管理する方法もあります。
使用しようとしましたが上手く使えなかったので記述を控えます。
