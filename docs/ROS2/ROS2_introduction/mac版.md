## Homebrewをインストール
[homebrew](https://brew.sh/)

## UTMのインストール
~~~bash
brew install --cask utm
~~~

## Ubuntu Serverのインストール
[linux](https://cdimage.ubuntu.com/releases/noble/release/)
- 64-bit ARM (ARMv8/AArch64) server install imageをインストール

## UTM APPを起動~ubuntu serverの起動
1. アプリケーションのUTM.appを起動
2. 新規仮想マシンを作成
3. 仮装化からLinuxをクリック
4. 次へから起動ISOイメージにダウンロードしたUbuntuイメージを指定し続けるをクリック
5. デフォルトでいいなら全部次へでok
6. もし読み込むosみたいな設定が出たら多分１番上（忘れてしまったので出たら調べたほうがいい、参考のサイトに載ってなかった）
7. Linuxをダブルクリックしターミナルみたいなのが起動したら基本doneで進める
8. user name と passwordを設定
9. 次に進みreboot nowを選択	
10. 赤ボタンでウィンドウを消す
11. linuxを右クリックで停止
12. cd/dvdから削除を選択
13. linuxダブルクリックで起動
14. loginしそうな画面が出たらユーザーネームとパスワードを入力

## GUI(Ubuntu desktopのダウンロード)
1. ubuntu serverのプロンプトで以下のコマンドを実行
~~~bash
sudo apt update
sudo apt install ubuntu-desktop
sudo reboot
~~~

## desktopでの設定
1. 日本語化をする
	- settingsを開き検索マーク（虫眼鏡）からregionと検索
	- manage install languageからinstall / remove language...を選択し、japanese(日本語)を選択
	- englishを下にドロップして日本語をトップにする
2. 再起動
~~~bash
reboot
~~~

3. 地域と言語のフォーマットで日本を選択
4. キーボードの入力ソースから日本語(Mozc)を選択

## キーボードの設定
1. キーボードのjapaneseを追加
2. exitする
3. 

**ちょっと中断**

[return](/docs/ROS2/ROS2_導入/導入.md)
