1.[SerialBTのつかいかた](###SerialBTの使い方)

### SerialBTの使い方
~~~c
SerialBT.setName(“Pico_BT); // ペアリングの時のの名前
SerialBT.begin(9600); // 通信速度

SerialBT.available(); // データがあればtrue
SerialBT.read(); // 1文字読み取る
SerialBT.print(“そうしんでーた”); // データを文字列で送信
~~~

[return](raspberry_pi_pico_2w.md)