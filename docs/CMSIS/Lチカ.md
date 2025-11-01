## includeするもの
~~~cpp
#include "stm32f303x8.h"
~~~

## ピンの設定
~~~cpp
// GPIO(B)クロックを有効化
RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

// PB7を出力モードに
GPIOB->MODER &= ~(3 << (7*2)); // ビットクリア
GPIOB->MODER |= (1 << (7*2)); // 出力モードに設定
~~~

## Lチカ
~~~cpp
#include "stm32f303x8.h"

int main() {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER &= ~(3 << (7*2));
	GPIOB->MODER |= (1 << (7*2));
	for (;;) {
		// ビット反転操作
		GPIOB->ODR ^= (1 << 7);　// PB7を表す
		for (volatile int i = 0; i < 1000000; i++);　//　目に見える程度待つ
	}
}
~~~

[return](CMSIS.md)