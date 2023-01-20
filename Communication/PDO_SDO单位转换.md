# PDO_SDO单位转换

## PDO

### 1.PDO_MODE_OF_OPERATION

| ADDRESS | RANGE |
| ------- | ----- |
| 0x3001  | \     |

Formula:
$$
PDO=In_l
$$

```
PDO_MODE_OF_OPERATION = Rxdata[3];
```

Example:

```
Rxdata = {0x55, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x0d, 0x0a};
PDO_MODE_OF_OPERATION = 0x0a;
```

### 2.PDO_TARGET_POSITION

| ADDRESS | RANGE  |
| ------- | ------ |
| 0x3002  | 0~4096 |

Formula:
$$
PDO=In_h * 256 + In_l
$$

```
temp = _convert_8bit_to_16bit(Rxdata[2], Rxdata[3]);
if(_check_range(temp, oSdo->SDO_TARGET_POSITION_LB, oSdo->SDO_TARGET_POSITION_UB) == 1)
{
	oPdo->PDO_TARGET_POSITION = temp;
}
```

### 3.PDO_TARGET_VELOCITY

| ADDRESS | RANGE        |
| ------- | ------------ |
| 0x3003  | -32768~32767 |

Formula:
$$
PDO=In_h*256+In_l
$$

### 4.PDO_TARGET_CURRENT_Q



## SDO

