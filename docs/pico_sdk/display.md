## lcdの初期設定
~~~c
LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;
LCD_Init(Lcd_ScanDir, 200);
~~~

## lcdへの文字の表示
~~~c
GUI_Dis_String_EN(0, 0, “abc”, &Font16, FONT_BACKGROUND, WHITE);
~~~

## lcd塗りつぶし
~~~c
~~~

[return](pico_sdk.md)