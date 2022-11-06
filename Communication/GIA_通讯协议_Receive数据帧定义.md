# GIA\_通讯协议\_Receive数据帧定义

16位数据，数据头55，数据尾0d0a

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

## Bit[2]: 控制类型Type

| 取值 | 模式                          | 说明                     |
| ---- | ----------------------------- | ------------------------ |
| 0x00 | No Mode                       |                          |
| 0x01 | PP Mode                       | Next Ver                 |
| 0x08 | CSP Mode                      |                          |
| 0x09 | CSV Mode                      |                          |
| 0x0a | CST Mode                      |                          |
| 0x10 | Read Mode                     | 读取配置参数             |
| 0x12 | Internal CSP Mode             | Next Ver                 |
| 0x13 | Internal CSV Mode             | Next Ver                 |
| 0x14 | Internal CST Mode             | Next Ver                 |
| 0x20 | Write Mode                    | 写入配置参数             |
| 0x21 | Electric Angle Calibrate Mode | 电角度校正模式, 待完成   |
| 0x22 | Encoder Calibrate Mode        | 编码器校正模式, Next Ver |
| 0x23 | ADC Calibrate Mode            | ADC校正模式, 待完成      |
| 0xff | Initial Mode                  | 初始化模式, 待完成       |
| else | Error, returnBit[1] = 1       |                          |

## Bit[3]: Reserve

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