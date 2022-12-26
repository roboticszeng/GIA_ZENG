
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
	_iq Kp;
	_iq Ki;
	_iq Kd;

	/* Derivative low-pass filter time constant */
	_iq tau;

	/* Output limits */
	_iq limMin;
	_iq limMax;
	
	/* Integrator limits */
	_iq limMinInt;
	_iq limMaxInt;

	/* Sample time (in seconds) */
	_iq T;

	/* Controller "memory" */
    _iq proportional;
	_iq integrator;
	_iq prevError;			/* Required for integrator */
	_iq differentiator;
	_iq prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	_iq out;

} pid_typedef;

typedef struct{
    
    _iq k[2];
    _iq output[2];
    
} filter_typedef;

typedef struct{
    
    // 常数
    _iq CONST_PWM_PERIOD;
    _iq CONST_ENC_RESOLUTION;
    _iq CONST_POLAR_PAIRS;
    _iq CONST_ZERO_POSITION;
    
    _iq CONST_ADC_RESOLUTION;
    _iq CONST_MCU_VOLTAGE;
    _iq CONST_CUR_SAMP_GAIN;
    _iq CONST_CUR_SAMP_RESISTANCE;
    
    _iq CONST_PULSE_TO_CUR_SLOPE;
    _iq CONST_PULSE_TO_CUR_OFFSET;
    _iq CONST_CUR_TO_PULSE_SLOPE;
    _iq CONST_CUR_TO_PULSE_OFFSET;
    
    _iq CONST_POSITION_SAMP_TIME;
    
    
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
    
    
    
    
    
    
    
    
    uint16_t actual_position;
    uint16_t actual_velocity;
    uint16_t actual_current_q;
    uint16_t actual_current_d;
    uint16_t following_error;
    uint16_t mode;
    
    uint16_t target_position;
    uint16_t target_velocity;
    uint16_t target_current_q;
    
    uint16_t actual_position_prev;
    
    
    
} pdo_typedef;

// 声明及初始化函数
pid_typedef* pid_new(void);
filter_typedef* filter_new(void);
sdo_typedef* config_new(void);
pdo_typedef* pdo_new(void);

void pid_init(pid_typedef* pid, float Kp, float Ki, float T, float Max, float MaxInt);
void filter_init(filter_typedef* handle, float freq, float sample_time);
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
_iq convert_pulse_to_velocity(uint16_t actual_velocity);
_iq convert_pulse_to_current(uint16_t actual_current);

uint16_t convert_position_to_pulse(_iq pos);
uint16_t convert_velocity_to_pulse(_iq vel);
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
