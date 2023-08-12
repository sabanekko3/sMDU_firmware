# sMDUレジスタマップ  

## 通信規則  

### CAN通信の場合  

1Mbpsで標準IDを使用する。  
値を書きこむ場合、IDは (CAN ID<<6 | レジスタID) とし、標準データフレームで値を送信する。  
値を読み出す場合、リモートフレームでIDを送信する。  

### UART通信の場合  

2Mbpsで通信する。
値を書きこむ場合、1バイト目にID、続けてデータを送信。終端文字として'\n'を送信。  
値を読み出す場合、1バイト目に ID|0x8、2バイト目に終端文字として'\n'を送信。  

### レジスタマップ概要

| ID | 名称 | IO | 機能 |
|:--:|:--:|:--:|:--:|
| 0x0 | CAN_ID | r/(w via UART) | CAN通信用ID |
| 0x1 | MOTOR_TYPE | r/w | 使用するモーター |
| 0x2 | ENC_TYPE | r/w | 使用するエンコーダ/分解能/モーター極数 | NO_ENC |
| 0x3 | BATT_V | r | パワー系電源電圧 |
| 0x4 | FOC_GAIN_PI | r/w | 電流制御用ゲイン |
| 0x5 | FOC_GAIN_D | r/w | 電流制御用ゲイン |
| 0x6 | DQ_TARGET | r/w | dq電流目標値 |
|0x7|DQ|r|現在のdq電流|
| 0x8 | SPD_GAIN_PI | r/w | 速度制御ゲイン |
| 0x9 | SPD_GAIN_D | r/w | 速度制御ゲイン |
| 0xA | SPD_TARGET | r/w | 速度目標値(電気角rad/s) |
|0xB| SPD |r|現在の電気角速度(電気角rad/s)|
|0xC|POS|r|現在の電気角|
|0xD|POS_SUM|r/w|電気角の移動量の総和|
|0xE|I_LIMIT|r/w|電流の最大値 $\sqrt{i_d^2 + i_q^2}$ をもとに算出|

### 各レジスタ詳細  

### CAN_ID(0x0)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~4|CAN_ID|unsinged int| UARTからのみ書き換え可能|

### MOTOR_TYPE(0x1)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~7|MOTOR|motor_type| |

### ENC_TYPE(0x2)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~7|ENC|ENC_type| |
|8~23|RSL|int16_t|エンコーダー分解能。逆向きなら負数|
|24~31|POLE|uint8_t|モーター極数|

### BATT_V(0x3)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~31|BATT_V|float| |

### FOC_GAIN_PI(0x4)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~31|FOC_P|float| |
|32~63|FOC_I|float| |

### FOC_GAIN_D(0x5)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~31|FOC_D|float| |

### DQ_TARGET(0x6)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~31|D_TARGET|float| |
|32~63|Q_TARGET|float| |

### DQ(0x7)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~31|D|float| 現在のd軸電流 |
|32~63|Q|float| 現在のq軸電流 |

### SPD_GAIN_PI(0x8)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~31|SPD_P|float| |
|32~63|SPD_I|float| |

### SPD_GAID_D(0x9)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~31|SPD_D|float| |

### SPD_TARGET(0xA)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~31|SPD_TARGET|float| 目標速度(rad/s) |

### SPD(0xB)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~31|SPD|float| 現在の速度(rad/s) |

### POS(0xC)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~31|POS|float| |

### POS_SUM(0xD)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~31|POS_SUM|float| 任意の値を書きこんで初期化可能 |

### I_LIMIT(0xE)  

|bits|名称|データ型|備考|
|:----|:----|:----|:----|
|0~31|I_LIMIT|float| |

## enum classしてあるやつら  

### motor_type

| 名称 | 値 | 備考 |
|:--:|:--:|:--:|
| NOP | 0 | なにもしない |
| DC | 1 | ブラシ付きモーター |
| BLDC_FOC | 2 | センサ付きベクトル制御 |
| BLDC_FORCED_COMM | 3 | 正弦波強制転流 |
| BLDC_FOC_SENSORLESS|4|センサレスベクトル制御|

### ENC_type

| 名称 | 値 | 備考 |
|:--:|:--:|:--:|
| NO_ENC | 0 | エンコーダー無 |
| ABX | 1 | AMT102みたいなやつ |
| UVW_HALL |2|普通のホールセンサ|
| AB_LINER_HALL |3|ロボマスモーターとか|
| AS5600 | 4 | I2C |
| AS5048 | 5 | SPI |
