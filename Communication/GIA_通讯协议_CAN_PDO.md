# GIA\_通讯协议\_CAN_PDO

8位数据，数据头55，数据尾0d0a

## Bit[0]: 数据头0x55

## Bit[1]: 关节号Joint Number

| 取值 | 说明                    |
| ---- | ----------------------- |
| 0xd0 | 关节1                   |
| 0xd1 | 关节2                   |
| 0xd2 | 关节3                   |
| 0xd3 | 关节4                   |
| 0xd4 | 关节5                   |
| 0xd5 | 关节6                   |
| 0xd6 | 关节7                   |
| else | Error, returnBit[0] = 1 |

## Bit[2~3]: PDO值

### 1. ActualPosition==(ADDRESS = 0x3082)==

```
Bit[2] = _get_high_byte_uint(oPdo->actual_position)
Bit[3] = _get_low_byte_uint(oPdo->actual_position)
```

actual_position 取值范围 0~4096，直接由函数`as5600_get_raw_angle(oEncoder, &actual_position)`采集得到
$$
q=(p-p_0)\frac{2\pi}{4096}
$$

$$
p=\frac{4096}{2\pi}q+p_0
$$



==未实现的：==

```
#define _get_high_byte_uint(data) (data & 0xff00) //取无符号整型数据的高8位
#define _get_low_byte_uint(data) (data & 0x00ff) //取无符号整型数据的低8位
```

### 2. ActualVelocity==(ADDRESS = 0x3085)==

```
Bit[2] = _get_high_byte_int(oPdo->actual_velocity)
Bit[3] = _get_low_byte_int(oPdo->actual_velocity)
```

actual_velocity取值范围±20480（电机额定转速300rpm），由函数`get_velocity`计算得到
$$
\dot q=(p-32768)\frac{2\pi}{4096}
$$

$$
p=\frac{4096}{2\pi}q+32768
$$







==未实现的：==

```
#define _get_high_byte_int(data) ((data + 0x8000) & 0xff00)
#define _get_low_byte_int(data) ((data + 0x8000) & 0x00ff)
```

==未实现的：==

```
get_velocity 
```

### 3. Actual Current-Q==(ADDRESS = 0x330E)==

```
Bit[2] = _get_high_byte_uint(oPdo->actual_current_q)
Bit[3] = _get_low_byte_uint(oPdo->actual_current_q)
```

actual_current_q 取值范围0~4096，由函数`convert_current_to_pulse`计算得到

计算方法，已知电流采样时：
$$
I=\frac{p-2048}{K*R}*\frac{3.3}{4096}
$$
那么交换顺序：
$$
p=K*R*I*\frac{4096}{3.3}+2048
$$
其中，$K$为放大系数50，$R$为采样电阻0.01Ω，4096为ADC分辨率

==未实现的：==

```
convert_current_to_pulse
```

### 4. Actual Current-D==(ADDRESS = 0x330D)==

```
Bit[2] = _get_high_byte_uint(oPdo->actual_current_d)
Bit[3] = _get_low_byte_uint(oPdo->actual_current_d)
```

和3.完全一样

### 5. Mode Of Operation Display==(ADDRESS=0x3081)==

```
Bit[2] = 0x00;
Bit[3] = oPdo->mode;
```

mode取值范围0x00,0x08,0x09,0x0a

### 6. Following Error==(ADDRESS=0x3084)==

```
switch (oPdo->mode){
	case MODE_CSP:
	oPdo->following_error = oPdo->target_position - oPdo->actual_position;
	
	break;
	case MODE_CSV:
	oPdo->following_error = oPdo->target_velocity - oPdo->actual_position;
	
	break;
	case MODE_CST:
	oPdo->following_error = oPdo->target_current_q - oPdo->actual_current_q;
	
	break;
	otherwise:
	oPdo->following_error = 0;
	break;
}
// 上面的计算也可以放在三环里算

Bit[2] = _get_high_byte_int(oPdo->following_error)
Bit[3] = _get_low_byte_int(oPdo->following_error)
```









## Bit[4]~Bit[5]: 目标值TargetValue = Bit[4] * 0x100 + Bit[5]

### 1. Bit[2] == 0x08||0x09||0x0a

| Bit[2] Value         | Bit[4] 含义  | 单位    | 取值范围       | 转换关系                   |
| -------------------- | ------------ | ------- | -------------- | -------------------------- |
| 0x08                 | 目标位置     | count   | 0~4096         | TargetPos = Bit[2]         |
| 0x09                 | 目标速度     | count/s | 0~40960        | TargetVel = Bit[2] - 20480 |
| 0x0a                 | 目标电流     | count_i | 0~4096         | TargetCur = Bit[2] - 2048  |
| 0x08\|\|0x09\|\|0x0a | 超出取值范围 |         | returnBit[2]=1 |                            |

示例：

Bit[2] = 0x08, TargetValue = 2048 ==> 目标位置为2048count

Bit[2] = 0x09, TargetValue  = 0 ==> 目标速度为-20480rad/s

Bit[2] = 0x0a, TargetValue  = 0 ==> 目标电流为-2048count_i

提供两个常数：

Const_count_to_pos = (2*PI)/4096 rad/count

Const_count_to_cur = 6.6/4096 A/count (INA240A2_50V/V + 0.01R)

### 2. Bit[2] == 0x10 || 0x20

上报16Bit配置参数，上报数据根据TargetValue的取值不同，输出格式如下

| Target Value                   | 数据位 | 说明                      | 单位     | 取值范围   |
| ------------------------------ | ------ | ------------------------- | -------- | ---------- |
| /                              | 0      | 数据头:0x55               |          |            |
|                                | 1      | Joint Number:0xd0~0xd6    |          |            |
| 0x3000:Actual State            | 2.3    | Actual Position           |          |            |
|                                | 4.5    | Actual Velocity           |          |            |
|                                | 6.7    | Actual Current-Q          |          |            |
|                                | 8.9    | Actual Current-D          |          |            |
|                                | 10.11  | Mode of Operation Display | Next Ver |            |
|                                | 12.13  | Following Error           | Next Ver |            |
| 0x3080:Target State            | 2.3    | 最近一次Target Position   |          |            |
|                                | 4.5    | 最近一次Target Velocity   |          |            |
|                                | 6.7    | 最近一次Target Current-Q  |          |            |
|                                | 8.9    | 最近一次Velocity Offset   | Next Ver |            |
|                                | 10.11  | 最近一次Current Offset    | Next Ver |            |
|                                | 12.13  | 最近一次Mode Of Operation | Next Ver |            |
| 0x3300:Pos Demand              | 2.3    | Position Demand Value     |          |            |
|                                | 4.5    | Limited Position          |          |            |
|                                | 6.7    | Position Loop Effort      | Next Ver |            |
| 0x3303:Vel Demand              | 2.3    | Velocity Demand Value     |          |            |
|                                | 4.5    | Limited Velocity          |          |            |
|                                | 6.7    | Velocity Loop Effort      | Next Ver |            |
| 0x3309:Cur-Phase Actual        | 2.3    | Actual Current A-Phase    |          |            |
|                                | 4.5    | Actual Current B-Phase    |          |            |
| 0x330d:Cur-Q Demand            | 2.3    | Current-Q Demand Value    |          |            |
|                                | 4.5    | Limited Current-D         |          |            |
|                                | 6.7    | Current-Q Loop Effort     | Next Ver |            |
| 0x330e:Cur-D Demand            | 2.3    | Current-D Demand Value    | Next Ver |            |
|                                | 4.5    | \                         |          |            |
|                                | 6.7    | Current-D Loop Effort     |          |            |
| **0x3506:Pos Loop Para Std**   | 6.7    | KP_P_STD                  | Next Ver |            |
|                                | 8.9    | KI_P_STD                  |          |            |
|                                | 10.11  | KD_P_STD                  |          |            |
| **0x3507:Vel Loop Para Std**   | 6.7    | KP_V_STD                  | Next Ver |            |
|                                | 8.9    | KI_V_STD                  |          |            |
|                                | 10.11  | KD_V_STD                  |          |            |
| **0x3508:Cur-Q Loop Para Std** | 6.7    | KP_Q_STD                  | Next Ver |            |
|                                | 8.9    | KI_Q_STD                  |          |            |
|                                | 10.11  | KD_Q_STD                  |          |            |
| **0x3509:Cur-D Loop Para Std** | 6.7    | KP_D_STD                  | Next Ver |            |
|                                | 8.9    | KI_D_STD                  |          |            |
|                                | 10.11  | KD_D_STD                  |          |            |
| **0x3516:Pos Loop Para Per**   | 6.7    | KP_P_PER                  | 0~65535  | 0.001      |
|                                | 8.9    | KI_P_PER                  |          |            |
|                                | 10.11  | KD_P_PER                  |          |            |
| **0x3517:Vel Loop Para Per**   | 6.7    | KP_V_PER                  |          |            |
|                                | 8.9    | KI_V_PER                  |          |            |
|                                | 10.11  | KD_V_PER                  |          |            |
| **0x3518:Cur-Q Loop Para Per** | 6.7    | KP_Q_PER                  |          |            |
|                                | 8.9    | KI_Q_PER                  |          |            |
|                                | 10.11  | KD_Q_PER                  |          |            |
| **0x3519:Cur-D Loop Para Per** | 6.7    | KP_D_PER                  |          |            |
|                                | 8.9    | KI_D_PER                  |          |            |
|                                | 10.11  | KD_D_PER                  |          |            |
| **0x3510:Pos Limit**           | 6.7    | 位置环上限幅              | count    |            |
|                                | 8.9    | 位置环下限幅              | count    |            |
| **0x3512:Vel Limit**           | 6.7    | 速度环上限幅              | count/s  |            |
|                                | 8.9    | 速度环下限幅              | count/s  | 上限取反   |
| **0x3514:Cur-Q Limit**         | 6.7    | 电流环d轴上限幅           | count_i  |            |
|                                | 8.9    | 电流环d轴下限幅           | count_i  | 上限取反   |
| **0x3515:Cur-D Limit**         | 6.7    | 电流环q轴上限幅           | count_i  |            |
|                                | 8.9    | 电流环q轴下限幅           | count_i  | 上限取反   |
| **0x3500:Motor Const**         | 6.7    | 极对数：14                |          |            |
|                                | 8.9    | 供电电压：24              | V        |            |
| **0x3501:Encoder Const**       | 6.7    | 线数：4096                |          | 0~65535    |
|                                | 8.9    | 编码器偏置                |          | Next Ver   |
|                                | 10.11  | 电角度偏置                |          | 待完成     |
| **0x3502:ADC Const**           | 6.7    | 线数：4096                |          | 0~65535    |
|                                | 8.9    | ADC_0偏置                 | count_i  | -2048~2048 |
|                                | 10.11  | ADC_1偏置                 | count_i  | -2048~2048 |
| **0x3503:Mechanical Const**    | 6.7    | 电流和位置的方向：0(neg)  |          | 0~1        |
| **0x3505:Cur-Sample Const**    | 6.7    | 采样电阻：10              | mΩ       | 0~65535    |
|                                | 8.9    | 增益系数：50              | V/V      | 0~65535    |
|                                | 10.11  | MCU电压：3300             | mV       | 0~65535    |
| /                              | 14     | 数据尾:0x0d               |          |            |
| /                              | 15     | 数据尾:0x0a               |          |            |

若Bit[2] = 0x20，则为写入模式，其取值同上表，只取到**加粗**的数值。

示例：

Bit[2] = 0x10, TargetValue = 0x3502：读ADC常数，按上表格式输出

Bit[2] = 0x20, TargetValue = 0x3502：写ADC常数，写入格式为Bit[6\~7]计算线数，Bit[8\~9]计算ADC_0偏置，Bit[10\~11]计算ADC_1偏置，以此类推。

+ 滤波参数当前版本暂未写入

### 4. Bit[2] == 0x12 || 0x13 || 0x14

Reserve.

## Bit [6~11]: 配置值

仅当Bit[2] == 0x20 时生效，取值意义参照上表。

## Bit [12~13]: Reserve

## Bit [14~15]: 0x0d 0x0a 数据尾