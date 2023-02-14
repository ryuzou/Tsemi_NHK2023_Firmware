# RRER共通バッテリー監視ファームウェア
本プログラムはRRER共通バッテリー監視用のプロトコルです。
# 機能
## CAN インターフェイス
| CAN ID        | Tx/Rx | byte長 | Data                                                                                   |
|---------------|-------|-------|-------------------------------------------------------------------------------------|
| 0x011 | Tx    | 1    | [0:バッテリー電流超過、非常停止要求(0xff):1] |
| 0x001 | Rx    | 1    | [0:非常停止命令(0xff):1] |
| 0x201 | Tx    | 4     | (double)[0:電流値(A):3]                                                                   |


## 制御
>**Warning**
>マイコンから基板のオンオフを制御する場合，基板のJP2を外すこと（短絡マンを外す）

やってること

|ペリフェラル| 内容                                                | 
|----|---------------------------------------------------|
|ADC| 電流センサと電圧センサ，V<sub>refint</sub>の値を読み取る             |
|DMA| ADCの値を取得                                          |
|TIM3| 1msごとにADCの値からそれぞれ電流値，電圧値を計算＆それぞれフィルター処理           |
|TIM2| 500msごとにフィルター処理後の電流，電圧値をプリント                      |
|PA8| マイコンから電源のオンオフ                                     |
|PA9| GPIO割り込みで電源のオンオフの変化を読み取り(HAL_GPIO_EXTI_Interrupt) |

周期とかは適当に変えて大丈夫なはず

Analog WatchdogでADCを監視して過放電や過電流時に割り込み処理したかったが，チャンネルごとの監視が難しそうだったのとノイズとか考えて諦め  
その代わりタイマー割込みで電流電圧計算する時に値が範囲内か判定するようにした

```c
    if (V < VOLTAGE_NOTIFICATION) {
    //      do when battery voltage falls below VOLTAGE_NOTIFICATION
      printf("BATTERY VOLTAGE UNDER %d\r\n", (int)VOLTAGE_NOTIFICATION);
    }
    if (A > CURRENT_NOTIFICATION) {
    //      do when current flows over CURRENT_NOTIFICATION
      printf("CURRENT over %d\r\n", (int)CURRENT_NOTIFICATION);
    }
```

この部分を別タイマーに分けてもいいかも

`c_values.h` ： 計算に使う値（基板上のテストポイント） 