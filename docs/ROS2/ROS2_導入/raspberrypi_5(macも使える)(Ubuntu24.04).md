
## ros2の環境構築2(raspberrypi_5)
### 環境
- mac book air m3
- Raspberry Pi 5
- OS ubuntu24.04
- jazzy(ros2)

1. [ubuntu 24.04 raspberry pi 5](https://ubuntu.com/download/raspberry-pi)からOSをダウンロードし、解凍
2. [Etcher]([https://etcher.balena.io](https://etcher.balena.io/))でmicroSDにデータを書き込む
3. microSDスロットに挿入し起動
4. アップデートする
~~~bash
sudo apt update  
sudo apt upgrade
~~~
   
5. 文字コードをUTF-8に設定
 ~~~bash
sudo apt update && sudo apt install locales  
sudo locale-gen ja_JP ja_JP.UTF-8  
sudo update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8  
export LANG=ja_JP.UTF-8
~~~

6. APTに必要なパッケージをインストール
~~~bash 
sudo apt install software-properties-common  
sudo add-apt-repository universe
~~~
    
7. ros2リポジトリの暗号鍵を取得、リポジトリの追加
~~~bash
sudo apt update && sudo apt install curl -y  
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg  
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null  
sudo apt update
~~~

8. ros2パッケージのインストール
~~~bash
export ROS_DISTRO=jazzy  
sudo apt install ros-$ROS_DISTRO-desktop ros-dev-tools python3-argcomplete  
sudo rosdep init  
rosdep update
~~~

9. 環境変数の追加
~~~bash
source /opt/ros/$ROS_DISTRO/setup.bash  
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
~~~

#### 環境構築完了
[情報元（参考）](https://inomacreate.com/inobo-raspi5-ros2-jazzy/)


[return](/docs/ROS2/ROS2_導入/導入.md)