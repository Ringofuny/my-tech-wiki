
## ros2の環境構築1(mini PC)
### 環境
- Ubuntu 22.05.5 LTS
- Humble(ros2)

1.  文字コードをUTF-8に設定
~~~bash
sudo locale-gen ja_JP ja_JP.UTF-8  
sudo update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8  
export LANG=ja_JP.UTF-8
~~~

2. ros2の公式APTパッケージのダウンロード先をAPTソースリストに追加
~~~bash
sudo apt update
sudo apt install curl gnupg2 lsb-release
curl http://repo.ros2.org/repos.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/source.list.d/ros2-latest.list'
sudo apt update
~~~

3. ros2パッケージをインストール
~~~bash
export ROS_DISTRO=dashing

sudo apt install ros-$ROS_DISTRO-desktop python3-colcon-common-extensions python3-rosdep python3-argcomplete

sudo rosdep init
rosdep update
~~~

4. 環境設定
~~~bash
source /opt/ros/$ROS_DISTRO/setup.bash
echo "/opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
~~~

[return](/docs/ROS2/ROS2_導入/導入.md)