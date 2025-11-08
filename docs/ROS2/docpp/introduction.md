# 導入
-  ## まずは導入だ!

- ROS2の入れ方は[ここ](../ROS2_introduction/導入.md)

1. ### ワークスペースを作成
- このコマンドを実行(ディレクトリ作成)
~~~bash
mkdir -p ~/cpp_ws/src
~~~

2. ### パッケージの作成
- srcディレクトリにパッケージを作る(ビルドシステムはament_cmake)
~~~bash
cd ~/cpp_ws_src
ros2 pkg create --build-type ament_cmake cpp_de_yarou
# ros2 pkg create --build-type ament_cmake(ament-cmakeではないことに注意) <ここはパッケージ名>
~~~

3. ### パッケージのビルド
- ここで1つ前のディレクトリに戻り、colcon buildを実行
~~~bash
cd ~/cpp_ws_src && colcon build
~~~
![成功](image.png)

4. ### cppのコードを書く
- cpp_de_yarouのsrcに移動
- ここでcppのコードを書く
#### hello_node.cpp 

~~~cpp　
#include "rclcpp/rclcpp.hpp"

// hello worldを表示するノード
class HelloNode : public rclcpp::Node { // rclcppからNodeクラスを継承
    public: 
        HelloNode() : Node("Hello_node") { // スーパークラスのNodeコンストラクタを呼び出し、ノードの定義をする。
            RCLCPP_INFO(this->get_logger(), "Hello world"); // ターミナルに表示したい文字を出力できる
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // ROS2通信を初期化
    auto node = std::make_shared<HelloNode>(); // ノードを生成 
    rclcpp::shutdown(); // ROS2通信をシャットダウン
    return 0;
}
~~~

5. ### プログラムを実行
- コンパイルするにはCMakelists.txtを書かなければならない

    **CMakelists.txtはコンパイルの手順書みたいなもの**

- これはパッケージを作った時に大半出来上がってる
- 新しく書くとこは以下の部分

~~~CMakelists.txt
add_executable(hello_node src/hello_node.cpp)
ament_target_dependencies(hello_node rclcpp)
.
.
.
install(TARGETS
    hello_node
DESTINATION lib/${PROJECT_NAME})

もし
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
がなければ
find_package(ament_cmake REQUIRED)
の下に追加する
~~~

6. ### Build

~~~bash
colcon build --symlink-install
. install/setup.bash
ros2 run cpp_de_yarou hello_node # 実行
~~~

## 参考
[CPPでROS2のプログラミングをしよう](https://qiita.com/ASAKA-219/items/1053769a4ddc3e8f9c18)

[ros2ではじめよう次世代ロボットプログラミング](https://gihyo.jp/book/2019/978-4-297-10742-0)

