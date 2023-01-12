
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROLL_H__
#define __CONTROLL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "IQmathLib.h"
#include <stdint.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define _check_data(data, std) ((data == std)?1:0)
#define _convert_8bit_to_16bit(byte_h, byte_l) (byte_h * 0x100 + byte_l)
#define _get_high_byte_uint(data) ((data & 0xff00) >> 8) //取无符号整型数据的高8位
#define _get_low_byte_uint(data) (data & 0x00ff) //取无符号整型数据的低8位
    
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */

// typedef
typedef struct {
	/* Controller gains */
	_iq15 Kp;
	_iq15 Ki;
	_iq15 Kd;

	/* Derivative low-pass filter time constant */
	_iq15 tau;

	/* Output limits */
	_iq15 limMin;
	_iq15 limMax;
	
	/* Integrator limits */
	_iq15 limMinInt;
	_iq15 limMaxInt;

	/* Sample time (in seconds) */
	_iq T;

	/* Controller "memory" */
    _iq15 proportional;
	_iq15 integrator;
	_iq15 prevError;			/* Required for integrator */
	_iq15 differentiator;
	_iq15 prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	_iq15 out;

} pid_typedef;

typedef struct{
    
    _iq k[2];
    _iq output[2];
    
} filter_typedef;

typedef struct{
    
    // PWM自动重装值
    _iq CONST_PWM_PERIOD;

    // 极对数
    _iq CONST_POLAR_PAIRS;
    // 运动学零位的编码器值
    _iq15 CONST_ZERO_POSITION;
    // ADC位数
    _iq15 CONST_ADC_RESOLUTION;
    // 编码器线数
    _iq15 CONST_ENC_RESOLUTION;
    
    // ADC0偏置值，静态电流的ADC采样值比2048大时，这一值为正
    int CONST_ADC0_OFFSET;
    // ADC1偏置值
    int CONST_ADC1_OFFSET;
    // MCU供电电压
    _iq CONST_MCU_VOLTAGE;
    // 采样电阻增益
    _iq CONST_CUR_SAMP_GAIN;
    // 采样电阻阻值
    _iq CONST_CUR_SAMP_RESISTANCE;
    
    // 电流脉冲和安培互换的系数
    _iq CONST_PULSE_TO_CUR_SLOPE;
//    _iq5 CONST_PULSE_TO_CUR_SLOPE;
    _iq CONST_PULSE_TO_CUR_OFFSET;
    _iq CONST_CUR_TO_PULSE_SLOPE;
    _iq CONST_CUR_TO_PULSE_OFFSET;
    
    // 编码器采样时间
    _iq CONST_POSITION_SAMP_TIME;
    // 电流采样时间
    _iq CONST_CURRENT_SAMP_TIME;
    
    // 电流环控制时间
    _iq CONST_CURRENT_CONTROL_TIME;
    // 位置速度环控制时间
    _iq CONST_POSITION_CONTROL_TIME;
    
    // PID参数
    _iq15 PID_POS_KP;
    _iq15 PID_POS_KI;
    _iq15 PID_POS_MAX;
    _iq15 PID_POS_INTMAX;
    
    _iq15 PID_VEL_KP;
    _iq15 PID_VEL_KI;
    _iq15 PID_VEL_MAX;
    _iq15 PID_VEL_INTMAX;
    
    _iq15 PID_CUR_Q_KP;
    _iq15 PID_CUR_Q_KI;
    _iq15 PID_CUR_Q_MAX;
    _iq15 PID_CUR_Q_INTMAX;
    _iq15 PID_CUR_D_KP;
    _iq15 PID_CUR_D_KI;
    _iq15 PID_CUR_D_MAX;
    _iq15 PID_CUR_D_INTMAX;
    
    // 滤波器截止频率
    _iq FILT_VEL_CUTOFF_FREQ;
    _iq FILT_CUR_Q_CUTOFF_FREQ;
    _iq FILT_CUR_D_CUTOFF_FREQ;
    
} sdo_typedef;

typedef struct{
    
    _iq iqPos;
    _iq iqPosElec;
    _iq iqVel;
    _iq iqCurA;
    _iq iqCurB;
    _iq iqCurQ;
    _iq iqCurD;
    
    _iq iqPosPrev;
    
    _iq iqTargQ;
    _iq iqTargD;
    _iq iqTargV;
    _iq iqTargP;
    
    
    uint16_t actual_position;
    int32_t actual_velocity;
    uint16_t actual_current_q;
    uint16_t actual_current_d;
    uint16_t following_error;
    uint16_t mode;
    
    uint16_t target_position;
    uint32_t target_velocity;
    uint16_t target_current_q;
    uint16_t target_current_d;
    
    uint16_t actual_position_prev;
    
    
    
} pdo_typedef;

// 声明及初始化函数
pid_typedef* pid_new(void);
filter_typedef* filter_new(void);
sdo_typedef* config_new(void);
pdo_typedef* pdo_new(void);

void pid_init(pid_typedef *handle, _iq15 Kp, _iq15 Ki, _iq T, _iq15 Max, _iq15 MaxInt);
void filter_init(filter_typedef* handle, _iq cutoff_freq, _iq sample_time);
void config_init(sdo_typedef* handle);
void pdo_init(pdo_typedef* handle);

// 功能函数
_iq pid_update(pid_typedef* handle, _iq setpoint, _iq measurement);
_iq filter_update(filter_typedef* handle, _iq input);
void compute_svpwm(_iq Uq, _iq Ud, _iq angle_elec);
uint16_t compute_following_error(pdo_typedef* handle);
_iq compute_position_elec(_iq pos);


// 单位转换
_iq convert_pulse_to_position(uint16_t actual_position);
_iq convert_pulse_to_velocity(int32_t actual_velocity);
_iq convert_pulse_to_current(uint16_t actual_current);

uint16_t convert_position_to_pulse(_iq pos);
uint32_t convert_velocity_to_pulse(_iq vel);
uint16_t convert_current_to_pulse(_iq cur);






/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */



    
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CONTROLL_H__ */
