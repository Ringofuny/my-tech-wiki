[c言語に加えて](Ｃ言語の定数.md)cppにはconstexprがある

### 利点
これは#define同様コンパイル時に値を計算するため実行時の計算速度が速くなる。

### 欠点
クラスに定義ができない（解決策あり）
#### 解決策
静的(static)にすると、inline変数として使える
例
~~~c
class Test { 
   private:
      constexpr int size = 5; //エラー 
      constexpr static int size = 5; 
      //inline constexpr ～と同じ 
};
~~~

##### inlineとは
**変数**
cpp17以降でつかえる
変数をファイル間で共有しやすくする
複数のcppから読み込んでもエラーは出ない

**関数**
関数は普段実装部分にジャンプして実行されている
inline関数にすることでコピーして使うから速くなる関数が大きい場合はコードが大きくなるので避けた方がいい

[return](CPP.md)