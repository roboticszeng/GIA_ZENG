
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROLL_H__
#define __CONTROLL_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "IQmathLib.h"
#include <stdint.h>

    /* USER CODE BEGIN Includes */

    /* USER CODE END Includes */

    /* USER CODE BEGIN Private defines */

#define _check_data(data, std) ((data == std) ? 1 : 0)
#define _convert_8bit_to_16bit(byte_h, byte_l) (byte_h * 0x100 + byte_l)
#define _convert_16bit_to_32bit(byte_h, byte_l) (byte_h * 0x10000 + byte_l)
#define _check_range(data, lb, ub) (((data > lb) && (data < ub)) ? 1 : 0)
#define _get_high_byte_uint(data) ((data & 0xff00) >> 8) // 取无符号整型数据的高8位
#define _get_low_byte_uint(data) (data & 0x00ff)         // 取无符号整型数据的低8位

#define _convert_uint16_intdec_to_float(integer, decimal) (integer + decimal / BIT_16)
#define _convert_uint16_intdec_to_iq(integer, decimal) _IQ(_convert_uint16_intdec_to_float(integer, decimal))
#define _convert_uint16_intdec_to_iq15(integer, decimal) _IQ15(_convert_uint16_intdec_to_float(integer, decimal))

#define _convert_uint16_to_float(data) (_get_high_byte_uint(data) + _get_low_byte_uint(data) / BIT_8)
#define _convert_uint16_to_iq(data) _IQ(_convert_uint16_to_float(data))
#define _convert_uint16_to_iq15(data) _IQ15(_convert_uint16_to_float(data))

#define _get_addr_std(addr) (addr - oConfig->ID * 0x1000) // 转化到0x3000~0x3fff
#define _get_addr_joint(addr) (addr + oConfig->ID * 0x1000)

#define _get_addr_flash(addr) (addr - 0x3000 + 0x0800f000)
    // #define _get_addr_flash(addr) (0x0800f000)

    // uint32_t _get_addr_flash(uint32_t addr);

    /* USER CODE END Private defines */

    /* USER CODE BEGIN Prototypes */

    // typedef
    typedef struct
    {
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
        _iq15 prevError; /* Required for integrator */
        _iq15 differentiator;
        _iq15 prevMeasurement; /* Required for differentiator */

        /* Controller output */
        _iq15 out;

    } pid_typedef;

    typedef struct
    {

        _iq k[2];
        _iq output[2];

    } filter_typedef;

    typedef struct
    {

        // PWM自动重装值
        _iq CONST_PWM_PERIOD;

        // 极对数
        _iq CONST_POLAR_PAIRS;

        // ADC位数
        _iq15 CONST_ADC_RESOLUTION;
        // 编码器线数
        _iq15 CONST_ENC_RESOLUTION;

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
        _iq15 CONST_CUR_TO_PULSE_SLOPE;
        _iq15 CONST_CUR_TO_PULSE_OFFSET;

        // 编码器采样时间
        _iq CONST_POSITION_SAMP_TIME;
        // 电流采样时间
        _iq CONST_CURRENT_SAMP_TIME;

        // 电流环控制时间
        _iq CONST_CURRENT_CONTROL_TIME;
        // 位置速度环控制时间
        _iq CONST_POSITION_CONTROL_TIME;

        // 运动学零位的编码器值
        _iq15 ENC_ZERO_POSITION;

        _iq15 CONST_MAX_VELOCITY;

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

        uint16_t ID;

        // ADC0偏置值，静态电流的ADC采样值比2048大时，这一值为正
        uint16_t CONST_ADC0_OFFSET;
        // ADC1偏置值
        uint16_t CONST_ADC1_OFFSET;
        // 电角度0度对应的编码器值
        uint16_t ELEC_ZERO_POSITION;

    } config_typedef;

    typedef struct
    {

        _iq iqPos;
        _iq iqPosElec;
        _iq iqVel;
        _iq iqCurA;
        _iq iqCurB;
        _iq iqCurQ;
        _iq iqCurD;
        _iq iqVoltQ;
        _iq iqVoltD;

        _iq iqPosPrev;

        _iq iqTargQ;
        _iq iqTargD;
        _iq iqTargV;
        _iq iqTargP;

        uint16_t actual_position;
        int32_t actual_velocity;
        uint16_t actual_position_prev;

    } state_typedef;

    typedef struct
    {
        // 0~256
        uint16_t PDO_MODE_OF_OPERATION;

        // 0~65535
        uint16_t PDO_TARGET_POSITION;

        // -32768~32768
        uint16_t PDO_TARGET_VELOCITY;

        //
        uint16_t PDO_TARGET_CURRENT_Q;
        uint16_t PDO_TARGET_CURRENT_D;

        uint16_t PDO_MODE_OF_OPERATION_DISPLAY;
        uint16_t PDO_ACTUAL_POSITION;
        uint16_t PDO_FOLLOWING_ERROR;
        uint16_t PDO_ACTUAL_VELOCITY;
        uint16_t PDO_ACTUAL_CURRENT_Q;
        uint16_t PDO_ACTUAL_CURRENT_D;

        uint16_t PDO_VELOCITY_OFFSET;
        uint16_t PDO_TORQUE_OFFSET;

    } pdo_typedef;

    typedef struct
    {

        uint16_t SDO_ID;

        uint16_t SDO_ENC_ZERO_POSITION_H;
        uint16_t SDO_ENC_ZERO_POSITION_L;

        uint16_t SDO_PID_POS_KP_INT;
        uint16_t SDO_PID_POS_KP_DEC;
        uint16_t SDO_PID_POS_KI_INT;
        uint16_t SDO_PID_POS_KI_DEC;
        uint16_t SDO_PID_POS_MAX;
        uint16_t SDO_PID_POS_INTMAX;

        uint16_t SDO_PID_VEL_KP_INT;
        uint16_t SDO_PID_VEL_KP_DEC;
        uint16_t SDO_PID_VEL_KI_INT;
        uint16_t SDO_PID_VEL_KI_DEC;
        uint16_t SDO_PID_VEL_MAX;
        uint16_t SDO_PID_VEL_INTMAX;

        uint16_t SDO_PID_CUR_Q_KP_INT;
        uint16_t SDO_PID_CUR_Q_KP_DEC;
        uint16_t SDO_PID_CUR_Q_KI_INT;
        uint16_t SDO_PID_CUR_Q_KI_DEC;
        uint16_t SDO_PID_CUR_Q_MAX;
        uint16_t SDO_PID_CUR_Q_INTMAX;

        uint16_t SDO_PID_CUR_D_KP_INT;
        uint16_t SDO_PID_CUR_D_KP_DEC;
        uint16_t SDO_PID_CUR_D_KI_INT;
        uint16_t SDO_PID_CUR_D_KI_DEC;
        uint16_t SDO_PID_CUR_D_MAX;
        uint16_t SDO_PID_CUR_D_INTMAX;

        uint16_t SDO_FILT_VEL_CUTOFF_FREQ;
        uint16_t SDO_FILT_CUR_Q_CUTOFF_FREQ;
        uint16_t SDO_FILT_CUR_D_CUTOFF_FREQ;

        uint16_t SDO_POSITION_LOWER_LIMIT;
        uint16_t SDO_POSITION_UPPER_LIMIT;
        uint16_t SDO_VELOCITY_LOWER_LIMIT;
        uint16_t SDO_VELOCITY_UPPER_LIMIT;
        uint16_t SDO_CURRENT_Q_LOWER_LIMIT;
        uint16_t SDO_CURRENT_Q_UPPER_LIMIT;
        uint16_t SDO_CURRENT_D_LOWER_LIMIT;
        uint16_t SDO_CURRENT_D_UPPER_LIMIT;

        uint16_t SDO_ELEC_ZERO_POSITION;
        uint16_t SDO_ADC0_OFFSET;
        uint16_t SDO_ADC1_OFFSET;

    } sdo_typedef;

    // 声明及初始化函数
    pid_typedef *pid_new(void);
    filter_typedef *filter_new(void);
    config_typedef *config_new(void);
    state_typedef *state_new(void);
    pdo_typedef *pdo_new(void);
    sdo_typedef *sdo_new(void);

    void pid_init(pid_typedef *handle, config_typedef *config, uint8_t pid_index);
    void filter_init(filter_typedef *handle, _iq cutoff_freq, _iq sample_time);
    void config_init(sdo_typedef *sdo, config_typedef *handle);
    void state_init(state_typedef *handle);
    void pdo_init(pdo_typedef *handle);
    void sdo_init(sdo_typedef *handle, config_typedef *config);
    void para_init(sdo_typedef *sdo, config_typedef *config);

    // 功能函数
    _iq pid_update(pid_typedef *handle, _iq setpoint, _iq measurement);
    _iq filter_update(filter_typedef *handle, _iq input);
    void compute_svpwm(_iq Uq, _iq Ud, _iq angle_elec);
    uint16_t compute_following_error(state_typedef *handle);
    _iq compute_position_elec(_iq pos);
    void calib_elec_angle(state_typedef* state, sdo_typedef* sdo);
    void calib_adc_offset(pdo_typedef* pdo, sdo_typedef* sdo);

    // 单位转换
    _iq convert_pulse_to_position(uint16_t actual_position);
    _iq convert_pulse_to_velocity(int32_t actual_velocity);
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
