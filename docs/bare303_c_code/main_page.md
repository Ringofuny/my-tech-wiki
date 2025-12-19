
# f303k8をc言語で書く
## 1. Lチカ
~~~c
#include <stdint.h>

#define RCC_AHBENR   (*(volatile uint32_t*)0x40021014)
#define GPIOA_MODER (*(volatile uint32_t*)0x48000000)
#define GPIOA_ODR   (*(volatile uint32_t*)0x48000014)

#define RCC_APB1ENR (*(volatile uint32_t*)0x4002101C)

/* enable GPIOB + I2C1 */
RCC_AHBENR  |= (1 << 18);   // GPIOBEN
RCC_APB1ENR |= (1 << 21);   // I2C1EN

void delay(volatile uint32_t t) {
    while (t--) {
        __asm volatile ("nop");
    }
}

int main(void) {
    /* GPIOA クロック ON */
    RCC_AHBENR |= (1 << 17);

    /* PA5 output */
    GPIOA_MODER &= ~(3 << (5 * 2));
    GPIOA_MODER |=  (1 << (5 * 2));

    while (1) {
        GPIOA_ODR ^= (1 << 5);
        delay(500000);
    }
}
~~~
---
- ### 3~5行目
~~~c
#define RCC_AHBENR   (*(volatile uint32_t*)0x40021014)
~~~ 
- 0x40021014 は RCC（Reset and Clock Control）の AHB バス用クロック有効化レジスタ
    - AHBとは... AHBはCPU，メモリ周りの高速用バス,（Advanced High-Performance Bus）[参考](http://www.kumikomi.net/archives/2004/05/11com12.php?page=13)
- (uint32_t*) -> 32bit幅のレジスタとして扱う
- volatile -> コンパイラ最適化を禁止(ハードウェアが勝手に値を書き換えるため必須）
- *(...) -> ポインタの指す先を直接読み書き

~~~c
#define GPIOA_MODER (*(volatile uint32_t*)0x48000000)
~~~
- GPIOAのMODER レジスタを直接操作するための定義
- MODERはピンの入出力モード設定用
- 各ピンにつき 2bit で設定
    1. 00 : 入力
    2. 01 : 出力
    3. 10 : 代替機能
    4. 11 : アナログ
~~~c
#define GPIOA_ODR   (*(volatile uint32_t*)0x48000014)
~~~
- GPIOAのODR（Output Data Register)を操作する定義
- 出力ピンのH/Lを直接制御
---

- ### 7~11行目
~~~c
void delay(volatile uint32_t t) {
    while (t--) {
        __asm volatile ("nop");
    }
}
~~~
- これは簡単な遅延コード
- 何もしないでクロック消費
--- 

- ### main関数内(13行目~)
### 出力ピンに設定(まずはリセット)
~~~c
RCC_AHBENR |= (1 << 17);
~~~
RCC_AHBENRの17bit目を1に(GPIOAを有効化)

---

| MODER ビット   | 対象ピン    |
| ----------- | ------- |
| [1:0]       | PA0     |
| [3:2]       | PA1     |
| [5:4]       | PA2     |
| …           | …       |
| **[11:10]** | **PA5** |
~~~c
GPIOA_MODER &= ~(3 << (5 * 2));
GPIOA_MODER |=  (1 << (5 * 2));
~~~

1. (5 * 2) → 10
2. 3 → 0b11
3. 3 << 10 → 0b11をビット10–11に配置
4. ~(...) → それ以外は1、該当2bitは0
---
### 出力ピンに設定(01→出力に設定)
~~~c
GPIOA_MODER |= (1 << (5 * 2));
~~~
1.  1 << 10 → 0b01 を bit10 にセット
2. bit11 は 0 のまま
3. 効果
4. MODER[11:10] = 01
5. つまり PA5 を汎用出力モードに設定
---

### ledのトグル
~~~c
while (1) {
    GPIOA_ODR ^= (1 << 5);
    delay(500000);
}
~~~
- 50万クロックごとにledをトグル
---

## 2. I2C
~~~c
#include <stdint.h>

/* ================= RCC ================= */
#define RCC_AHBENR   (*(volatile uint32_t*)0x40021014)
#define RCC_APB1ENR  (*(volatile uint32_t*)0x4002101C)

/* ================= GPIO ================= */
#define GPIOA_MODER (*(volatile uint32_t*)0x48000000)
#define GPIOA_ODR   (*(volatile uint32_t*)0x48000014)

#define GPIOB_MODER   (*(volatile uint32_t*)0x48000400)
#define GPIOB_OTYPER (*(volatile uint32_t*)0x48000404)
#define GPIOB_OSPEEDR (*(volatile uint32_t*)0x48000408)
#define GPIOB_PUPDR  (*(volatile uint32_t*)0x4800040C)
#define GPIOB_AFRL   (*(volatile uint32_t*)0x48000420)

/* ================= I2C ================= */
#define I2C1_CR1     (*(volatile uint32_t*)0x40005400)
#define I2C1_CR2     (*(volatile uint32_t*)0x40005404)
#define I2C1_TIMINGR (*(volatile uint32_t*)0x40005410)
#define I2C1_ISR     (*(volatile uint32_t*)0x40005418)
#define I2C1_ICR     (*(volatile uint32_t*)0x4000541C)
#define I2C1_TXDR    (*(volatile uint32_t*)0x40005428)

/* ================= SSD1306 ================= */
#define SSD1306_ADDR 0x3C   // 7bit address

/* ================= delay ================= */
void delay(volatile uint32_t t) {
    while (t--) {
        __asm volatile ("nop");
    }
}

/* ================= I2C init ================= */
void i2c1_init(void) {
    /* GPIOB clock enable */
    RCC_AHBENR |= (1 << 18);   // GPIOBEN

    /* I2C1 clock enable */
    RCC_APB1ENR |= (1 << 21);  // I2C1EN

    /* PB6, PB7 -> AF mode */
    GPIOB_MODER &= ~((3 << (6*2)) | (3 << (7*2)));
    GPIOB_MODER |=  ((2 << (6*2)) | (2 << (7*2)));

    /* Open-drain */
    GPIOB_OTYPER |= (1 << 6) | (1 << 7);

    /* High speed */
    GPIOB_OSPEEDR |= (3 << (6*2)) | (3 << (7*2));

    /* No pull-up/down (external pull-up assumed) */
    GPIOB_PUPDR &= ~((3 << (6*2)) | (3 << (7*2)));

    /* AF4 (I2C1) */
    GPIOB_AFRL &= ~((0xF << (6*4)) | (0xF << (7*4)));
    GPIOB_AFRL |=  ((4 << (6*4)) | (4 << (7*4)));

    /* I2C disable before config */
    I2C1_CR1 &= ~1;

    /* Timing (example: 8MHz PCLK, ~100kHz I2C) */
    I2C1_TIMINGR = 0x00303D5B;

    /* Enable I2C */
    I2C1_CR1 |= 1;
}

/* ================= I2C write ================= */
void i2c_write(uint8_t addr, uint8_t *data, int len) {
    /* Clear STOP/NACK flags */
    I2C1_ICR = (1 << 5) | (1 << 4);

    /* START + write + AUTOEND */
    I2C1_CR2 =
        (addr << 1) |        // SADD
        (len  << 16) |       // NBYTES
        (1 << 25) |          // AUTOEND
        (1 << 13);           // START

    for (int i = 0; i < len; i++) {
        while (!(I2C1_ISR & (1 << 1)));   // TXIS
        I2C1_TXDR = data[i];
    }

    /* Wait STOP */
    while (!(I2C1_ISR & (1 << 5)));       // STOPF
    I2C1_ICR = (1 << 5);                  // clear STOPF
}

/* ================= SSD1306 cmd ================= */
void ssd1306_cmd(uint8_t c) {
    uint8_t buf[2] = {0x00, c};
    i2c_write(SSD1306_ADDR, buf, 2);
}

/* ================= main ================= */
int main(void) {
    i2c1_init();
    delay(100000);

    /* SSD1306 minimal init sequence */
    ssd1306_cmd(0xAE); // display off
    ssd1306_cmd(0xA8); // multiplex
    ssd1306_cmd(0x3F);
    ssd1306_cmd(0xAF); // display on

    while (1) {
        ;
    }
}

~~~
---
### 定義部分
~~~c
#define RCC_AHBENR   (*(volatile uint32_t*)0x40021014)
~~~ 
- 0x40021014 は RCC（Reset and Clock Control）の AHB バス用クロック有効化レジスタ
    - AHBとは... AHBはCPU，メモリ周りの高速用バス,（Advanced High-Performance Bus）
- (uint32_t*) -> 32bit幅のレジスタとして扱う
- volatile -> コンパイラ最適化を禁止(ハードウェアが勝手に値を書き換えるため必須）
- *(...) -> ポインタの指す先を直接読み書き
---

~~~c
#define RCC_APB1ENR (*(volatile uint32_t*)0x4002101C)
~~~
- APB1バスにつながる周辺回路（TIM, USART, I2C 等）のクロックを ON/OFF するためのレジスタ
---

~~~c
#define GPIOB_MODER   (*(volatile uint32_t*)0x48000400)
~~~
- GPIOB 各ピンのモード設定
- I2C を使う場合：
- SCL / SDA ピンを 代替機能（AF）モードにする
---

~~~c
#define GPIOB_OTYPER (*(volatile uint32_t*)0x48000404)
~~~
- 出力の電気的方式を指定
| 値 | 意味           |
| - | ------------ |
| 0 | プッシュプル       |
| 1 | **オープンドレイン** |
- I2C では 必須でオープンドレイン
---

~~~c
#define GPIOB_OSPEEDR (*(volatile uint32_t*)0x48000408)
~~~
- 立ち上がり／立ち下がり速度を指定
- I2C では通常 High または Very High
---

~~~c
#define GPIOB_PUPDR  (*(volatile uint32_t*)0x4800040C)
~~~
- プルアップ／プルダウンの設定

| 値  | 意味    |
| -- | ----- |
| 00 | なし    |
| 01 | プルアップ |
| 10 | プルダウン |

- I2C は通常 外部プルアップ抵抗
--- 

~~~c
#define GPIOB_AFRL   (*(volatile uint32_t*)0x48000420)
~~~
- PB0〜PB7 の AF 番号を指定
- 1 ピンあたり 4bit
- ex) PB6 / PB7 → I2C1
---
~~~c
#define I2C1_CR1     (*(volatile uint32_t*)0x40005400)
~~~
- I2C周辺の基本制御
- 主な役割：
    - I2C 有効化（PE ビット）
    - ACK 制御
    - 割り込み制御
---

~~~c
#define I2C1_CR2     (*(volatile uint32_t*)0x40005404)
~~~
- 通信条件の設定
- 送信先アドレス
- 送信バイト数
- START / STOP 発行
- Read / Write 指定
- 通信開始時に必ず触るレジスタ
---

~~~c
#define I2C1_TIMINGR (*(volatile uint32_t*)0x40005410)
~~~
- I2C クロックのタイミング設定
- 以下を含む：
- SCL 周期
- SCL High / Low
- データセットアップ時間
- ※ デバイスクロック（PCLK）に強く依存
---

~~~c
#define I2C1_ISR     (*(volatile uint32_t*)0x40005418)
~~~
- 状態確認用
- 主なフラグ：
    - TXIS（送信可能）
    - BUSY（バス使用中）
    - STOPF（STOP 検出）
    - NACKF（NACK 受信）
    - ポーリング I2C では最重要
---

~~~c
#define I2C1_TXDR    (*(volatile uint32_t*)0x40005428)
~~~
- 送信データを書き込むレジスタ
- 1バイトずつ書く
--- 

~~~c
#define I2C1_ICR     (*(volatile uint32_t*)0x4000541C)
~~~
- ISR に立ったフラグを クリアする専用レジスタ
- STOPF / NACKF などを消す
---

~~~c
#define SSD1306_ADDR 0x3C
~~~
- ssd1306のアドレス
--- 

~~~c
void delay(volatile uint32_t t) {
    while (t--) {
        __asm volatile ("nop");
    }
}
~~~
- これは簡単な遅延コード
- 何もしないでクロック消費
--- 

### i2c1_init
~~~c
RCC_AHBENR |= (1 << 18);   // GPIOBEN
~~~
- GPIOBを有効化
---

~~~c
RCC_APB1ENR |= (1 << 21);  // I2C1EN
~~~
- I2Cを有効化
---

~~~c
GPIOB_MODER &= ~((3 << (6*2)) | (3 << (7*2)));
GPIOB_MODER |=  ((2 << (6*2)) | (2 << (7*2)));
~~~
- PB6,7をAFモードで起動
---

~~~c
GPIOB_OTYPER |= (1 << 6) | (1 << 7);
~~~
- オープンドレインに設定
---
 
~~~c
GPIOB_OSPEEDR |= (3 << (6*2)) | (3 << (7*2));
~~~
- high speedに設定
---

~~~c
GPIOB_PUPDR &= ~((3 << (6*2)) | (3 << (7*2)));
~~~
- 外部プルアップを使用
---

~~~c
GPIOB_AFRL &= ~((0xF << (6*4)) | (0xF << (7*4)));
GPIOB_AFRL |=  ((4 << (6*4)) | (4 << (7*4)));
~~~
- AF番号をI2C1に設定
---

~~~c
I2C1_CR1 &= ~1;
~~~
- レジスタ書き換えのため１度無効化
- CR1.PE（Peripheral Enable）ビットを 0 にする
- I2C ハードウェアを 完全停止
- 分解
    - 1 → bit0
    - ~1 → bit0 だけ 0、他は 1
    - &= → 他の設定を壊さず PE だけ OFF
---

~~~c
I2C1_TIMINGR = 0x00303D5B;
~~~
-I2Cのクロック波形（SCL）を決定する
- この値で：
    - SCL 周波数
    - データセットアップ／ホールド時間
    - フィルタ動作が決まる

TIMINGR の構造（STM32F3）

| ビット     | 名前     | 役割         |
| ------- | ------ | ---------- |
| [31:28] | PRESC  | プリスケーラ     |
| [23:20] | SCLDEL | SDA セットアップ |
| [19:16] | SDADEL | SDA ホールド   |
| [15:8]  | SCLH   | SCL High   |
| [7:0]   | SCLL   | SCL Low    |

0x00303D5B を分解すると：

| フィールド  | 値    |
| ------ | ---- |
| PRESC  | 0    |
| SCLDEL | 3    |
| SDADEL | 0    |
| SCLH   | 0x3D |
| SCLL   | 0x5B |

PCLK=8MHz、I2C ≈100kHz の一例

---

~~~c
I2C1_CR1 |= 1;
~~~
- I2C再度有効化
- CR1.PE を 1
- I2C ハードウェアが動作開始
- この瞬間から：
    - START 生成
    - TXDR 書き込み
    - BUSY 判定
    - がすべて有効になる
---

~~~c
i2c_write(uint8_t addr, uint8_t *data, int len)
~~~
- addr : 7bit の I2C スレーブアドレス
- data : 送信するデータ配列
- len : 送信バイト数
- START → アドレス送信 → データ送信 → STOP を自動で行う。
---

- ### フラグのクリア
~~~c
I2C1_ICR = (1 << 5) | (1 << 4);
~~~
- STOPF (bit5) : STOP 検出フラグ
- NACKF (bit4) : NACK 受信フラグ
- 前回通信のゴミ状態を消すためにクリア
---

- ### CR2 設定（通信条件の指定）
~~~c
I2C1_CR2 =
        (addr << 1) |        // SADD
        (len  << 16) |       // NBYTES
        (1 << 25) |          // AUTOEND
        (1 << 13);           // START
~~~
- (addr << 1) : SADD[7:1] 
    - スレーブアドレスを設定
    - LSB（bit0）は R/W ビットなので 0（write）
- (len << 16) : NBYTES
    - 送信する データバイト数
    - この回数だけ TXDR に書き込むと自動終了
- (1 << 25) : AUTOEND
    - NBYTES 分送信したら自動で STOP を出す
    - 手動 STOP が不要になる
- (1 << 13) : START
    - START コンディションを生成
    - この瞬間に通信が始まる
---

~~~c
for (int i = 0; i < len; i++) {
    while (!(I2C1_ISR & (1 << 1)));   // TXIS
    I2C1_TXDR = data[i];
}
~~~
- TXIS = 1 → TXDR に次のデータを書いてよい
- 処理内容
    1. TXIS が立つまで待つ
    2. TXDR に 1 バイト書く
    3. ハードウェアが自動で送信
---

~~~c
while (!(I2C1_ISR & (1 << 5)));       // STOPF
I2C1_ICR = (1 << 5);                  // clear STOPF
~~~
- STOPF : STOP コンディションが発生したことを示す
- AUTOEND が有効なので 自動で STOP が出る
- STOP を検出してからフラグをクリア
---

~~~c
void ssd1306_cmd(uint8_t c)
~~~
- 引数 c → SSD1306 に送る コマンド 1 バイト
---

~~~c
uint8_t buf[2] = {0x00, c};
~~~
- buf[0] = 0x00（制御バイト）
- SD1306 の I2C プロトコル仕様：

| 制御バイト | 意味                  |
| ----- | ------------------- |
| 0x00  | **次はコマンド**          |
| 0x40  | **次は表示データ（GDDRAM）** |
- この後に送る 1 バイトはコマンドです,という宣言。
---

~~~c
buf[1] = c
~~~
- 実際の SSD1306 コマンド本体
- 例：
    - 0xAE : Display OFF
    - 0xAF : Display ON
    - 0xA4 : Entire Display ON Resume
---

~~~c
i2c_write(SSD1306_ADDR, buf, 2);
~~~
- I2C送信
---