#include "controll.h"
#include "tim.h"
#include <math.h>
#include <AS5600.h>
#include "IQmathLib.h"
#include "adc.h"

extern encoder_typedef *oEncoder;

extern pid_typedef *oPidPosition;
extern pid_typedef *oPidVelocity;
extern pid_typedef *oPidCurrentD;
extern pid_typedef *oPidCurrentQ;

extern filter_typedef *oFilterVelocity;
extern filter_typedef *oFilterCurrentD;
extern filter_typedef *oFilterCurrentQ;

extern config_typedef *oConfig;


/***********************************************************************************************************************************************/
/*** PID ***/

pid_typedef *pid_new(void)
{

    return (pid_typedef *)calloc(1, sizeof(pid_typedef));
}

void pid_init(pid_typedef *handle, config_typedef *config, uint8_t pid_index)
{
    /* Clear controller variables */

    switch (pid_index)
    {
    case PID_INDEX_POS:
        handle->Kp = config->PID_POS_KP;
        handle->Ki = config->PID_POS_KI;
        handle->T = config->CONST_POSITION_CONTROL_TIME;
        handle->limMax = config->PID_POS_MAX;
        handle->limMaxInt = config->PID_POS_INTMAX;
        break;
    case PID_INDEX_VEL:
        handle->Kp = config->PID_VEL_KP;
        handle->Ki = config->PID_VEL_KI;
        // 速度环和位置环共用CONST_POSITION_CONTROL_TIME
        handle->T = config->CONST_POSITION_CONTROL_TIME;
        handle->limMax = config->PID_VEL_MAX;
        handle->limMaxInt = config->PID_VEL_INTMAX;
        break;
    case PID_INDEX_CUR_Q:
        handle->Kp = config->PID_CUR_Q_KP;
        handle->Ki = config->PID_CUR_Q_KI;
        handle->T = config->CONST_CURRENT_CONTROL_TIME;
        handle->limMax = config->PID_CUR_Q_MAX;
        handle->limMaxInt = config->PID_CUR_Q_INTMAX;
        break;
    case PID_INDEX_CUR_D:
        handle->Kp = config->PID_CUR_D_KP;
        handle->Ki = config->PID_CUR_D_KI;
        // 电流环共用CONST_CURRENT_CONTROL_TIME
        handle->T = config->CONST_CURRENT_CONTROL_TIME;
        handle->limMax = config->PID_CUR_D_MAX;
        handle->limMaxInt = config->PID_CUR_D_INTMAX;
        break;
    }

    handle->limMin = -handle->limMax;
    handle->limMinInt = -handle->limMaxInt;
    // 暂时不用Kd
    handle->Kd = _IQ15(0.0);
    handle->tau = _IQ15(0.0);
    handle->integrator = _IQ15(0.0);
    handle->prevError = _IQ15(0.0);
    handle->differentiator = _IQ15(0.0);
    handle->prevMeasurement = _IQ15(0.0);
    handle->out = _IQ15(0.0);
}

_iq pid_update(pid_typedef *pid, _iq setpoint, _iq measurement)
{

    static _iq15 error;
    error = _IQtoIQ15(setpoint) - _IQtoIQ15(measurement);

    pid->proportional = _IQ15mpy(pid->Kp, error);

    pid->integrator = pid->integrator + _IQ15mpyIQX(_IQmpy(_IQmpy(_IQ(0.5), _IQ15toIQ(pid->Ki)), pid->T), 20, (error + pid->prevError), 15);

    if (pid->integrator > pid->limMaxInt)
    {
        pid->integrator = pid->limMaxInt;
    }
    else if (pid->integrator < pid->limMinInt)
    {
        pid->integrator = pid->limMinInt;
    }
    //    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
    //                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
    //                        / (2.0f * pid->tau + pid->T);
    // 暂时不用KD
    pid->differentiator = _IQ15(0.0);
    pid->out = pid->proportional + pid->integrator + pid->differentiator;
    if (pid->out > pid->limMax)
    {
        pid->out = pid->limMax;
    }
    else if (pid->out < pid->limMin)
    {
        pid->out = pid->limMin;
    }
    pid->prevError = error;
    pid->prevMeasurement = _IQtoIQ15(measurement);
    return _IQ15toIQ(pid->out);
}

/*** PID ***/
/***********************************************************************************************************************************************/

/***********************************************************************************************************************************************/
/*** lowpass filter ***/

filter_typedef *filter_new(void)
{
    return (filter_typedef *)calloc(1, sizeof(filter_typedef));
}

void filter_init(filter_typedef *handle, _iq cutoff_freq, _iq sample_time)
{

    _iq rc = _IQdiv(_IQ(1.0), _IQmpy(_IQ(_2PI), cutoff_freq));
    handle->k[0] = _IQdiv(sample_time, sample_time + rc);
    handle->k[1] = _IQdiv(rc, sample_time + rc);

    //    handle->k[0] = _IQ(0.1);
    //    handle->k[1] = _IQ(0.9);

    handle->output[0] = _IQ(0.0);
    handle->output[1] = _IQ(0.0);
}

_iq filter_update(filter_typedef *handle, _iq input)
{

    handle->output[1] = handle->output[0];
    handle->output[0] = _IQmpy(handle->k[0], input) + _IQmpy(handle->k[1], handle->output[1]);

    return handle->output[0];
}

/*** lowpass filter ***/
/***********************************************************************************************************************************************/

/***********************************************************************************************************************************************/
/*** state & config ***/

config_typedef *config_new(void)
{

    return (config_typedef *)calloc(1, sizeof(config_typedef));
}

void config_init(sdo_typedef *sdo, config_typedef *handle)
{

    handle->CONST_PWM_PERIOD = _IQ(1500 - 1);
    handle->CONST_POLAR_PAIRS = _IQ(14);
    handle->CONST_ADC_RESOLUTION = _IQ15(BIT_12);
    handle->CONST_ENC_RESOLUTION = _IQ15(BIT_12);
    handle->CONST_MCU_VOLTAGE = _IQ(3.3);
    handle->CONST_CUR_SAMP_GAIN = _IQ(50);
    handle->CONST_CUR_SAMP_RESISTANCE = _IQ(0.01);
    handle->CONST_PULSE_TO_CUR_SLOPE = _IQdiv(_IQdiv(handle->CONST_MCU_VOLTAGE, handle->CONST_CUR_SAMP_GAIN),
                                              _IQ15toIQ(_IQ15mpyIQX(handle->CONST_CUR_SAMP_RESISTANCE, 20, handle->CONST_ADC_RESOLUTION, 15)));
    handle->CONST_PULSE_TO_CUR_OFFSET = -_IQdiv(handle->CONST_MCU_VOLTAGE,
                                                _IQmpy(_IQmpy(handle->CONST_CUR_SAMP_GAIN, handle->CONST_CUR_SAMP_RESISTANCE), _IQ(2.0)));
    handle->CONST_CUR_TO_PULSE_SLOPE = _IQ15div(_IQ15mpyIQX(_IQmpy(handle->CONST_CUR_SAMP_GAIN, handle->CONST_CUR_SAMP_RESISTANCE), 20,
                                                            handle->CONST_ADC_RESOLUTION, 15),
                                                _IQtoIQ15(handle->CONST_MCU_VOLTAGE));
    handle->CONST_CUR_TO_PULSE_OFFSET = _IQ15div(handle->CONST_ADC_RESOLUTION, _IQ15(2.0));
    handle->CONST_POSITION_SAMP_TIME = _IQ(1e-3);
    handle->CONST_CURRENT_SAMP_TIME = _IQ(41.66666667e-6);
    handle->CONST_CURRENT_CONTROL_TIME = _IQ(160e-6);
    handle->CONST_POSITION_CONTROL_TIME = _IQ(320e-6);
    handle->CONST_MAX_VELOCITY = _IQ15(20000);

    // 从这里往下需要写入flash，上面的暂不写入
    // 这里写进config怎么搞，再考虑，将来做成自动的
    // handle->CONST_ADC0_OFFSET = 1985;
    // handle->CONST_ADC1_OFFSET = 1985;
    // handle->ENC_ZERO_POSITION = 0;

    // handle->PID_POS_KP = _IQ15(20.0);
    // handle->PID_POS_KI = _IQ15(0.0);
    // handle->PID_POS_MAX = _IQ15(15.0);
    // handle->PID_POS_INTMAX = _IQ15(15.0);
    // handle->PID_VEL_KP = _IQ15(0.2);
    // handle->PID_VEL_KI = _IQ15(1.5);
    // handle->PID_VEL_MAX = _IQ15(2.0);
    // handle->PID_VEL_INTMAX = _IQ15(1.5);
    // handle->PID_CUR_Q_KP = _IQ15(0.8);
    // handle->PID_CUR_Q_KI = _IQ15(40.0);
    // handle->PID_CUR_Q_MAX = _IQ15(0.9);
    // handle->PID_CUR_Q_INTMAX = _IQ15(0.5);
    // handle->PID_CUR_D_KP = _IQ15(0.2);
    // handle->PID_CUR_D_KI = _IQ15(0.0);
    // handle->PID_CUR_D_MAX = _IQ15(0.2);
    // handle->PID_CUR_D_INTMAX = _IQ15(0.1);
    // handle->FILT_VEL_CUTOFF_FREQ = _IQ(20.0);
    // handle->FILT_CUR_Q_CUTOFF_FREQ = _IQ(20.0);
    // handle->FILT_CUR_D_CUTOFF_FREQ = _IQ(20.0);
    // handle->ID = 1;

    handle->ENC_ZERO_POSITION = _IQ15(sdo->SDO_ENC_ZERO_POSITION_L);


    handle->PID_POS_KP = _convert_uint16_intdec_to_iq15(sdo->SDO_PID_POS_KP_INT, sdo->SDO_PID_POS_KP_DEC);
    handle->PID_POS_KI = _convert_uint16_intdec_to_iq15(sdo->SDO_PID_POS_KI_INT, sdo->SDO_PID_POS_KI_DEC);
    handle->PID_POS_MAX = _convert_uint16_to_iq15(sdo->SDO_PID_POS_MAX);
    handle->PID_POS_INTMAX = _convert_uint16_to_iq15(sdo->SDO_PID_POS_INTMAX);

    handle->PID_VEL_KP = _convert_uint16_intdec_to_iq15(sdo->SDO_PID_VEL_KP_INT, sdo->SDO_PID_VEL_KP_DEC);
    handle->PID_VEL_KI = _convert_uint16_intdec_to_iq15(sdo->SDO_PID_VEL_KI_INT, sdo->SDO_PID_VEL_KI_DEC);
    handle->PID_VEL_MAX = _convert_uint16_to_iq15(sdo->SDO_PID_VEL_MAX);
    handle->PID_VEL_INTMAX = _convert_uint16_to_iq15(sdo->SDO_PID_VEL_INTMAX);

    handle->PID_CUR_Q_KP = _convert_uint16_intdec_to_iq15(sdo->SDO_PID_CUR_Q_KP_INT, sdo->SDO_PID_CUR_Q_KP_DEC);
    handle->PID_CUR_Q_KI = _convert_uint16_intdec_to_iq15(sdo->SDO_PID_CUR_Q_KI_INT, sdo->SDO_PID_CUR_Q_KI_DEC);
    handle->PID_CUR_Q_MAX = _convert_uint16_to_iq15(sdo->SDO_PID_CUR_Q_MAX);
    handle->PID_CUR_Q_INTMAX = _convert_uint16_to_iq15(sdo->SDO_PID_CUR_Q_INTMAX);

    handle->PID_CUR_D_KP = _convert_uint16_intdec_to_iq15(sdo->SDO_PID_CUR_D_KP_INT, sdo->SDO_PID_CUR_D_KP_DEC);
    handle->PID_CUR_D_KI = _convert_uint16_intdec_to_iq15(sdo->SDO_PID_CUR_D_KI_INT, sdo->SDO_PID_CUR_D_KI_DEC);
    handle->PID_CUR_D_MAX = _convert_uint16_to_iq15(sdo->SDO_PID_CUR_D_MAX);
    handle->PID_CUR_D_INTMAX = _convert_uint16_to_iq15(sdo->SDO_PID_CUR_D_INTMAX);

    handle->FILT_VEL_CUTOFF_FREQ = _IQ(sdo->SDO_FILT_VEL_CUTOFF_FREQ);
    handle->FILT_CUR_Q_CUTOFF_FREQ = _IQ(sdo->SDO_FILT_CUR_Q_CUTOFF_FREQ);
    handle->FILT_CUR_D_CUTOFF_FREQ = _IQ(sdo->SDO_FILT_CUR_D_CUTOFF_FREQ);

    handle->ID = sdo->SDO_ID;
    
    handle->ELEC_ZERO_POSITION = sdo->SDO_ELEC_ZERO_POSITION;
    handle->CONST_ADC0_OFFSET = sdo->SDO_ADC0_OFFSET;
    handle->CONST_ADC1_OFFSET = sdo->SDO_ADC1_OFFSET;
}

state_typedef *state_new(void)
{

    return (state_typedef *)calloc(1, sizeof(state_typedef));
}

void state_init(state_typedef *handle)
{

    handle->iqTargP = _IQ(_PI);
    handle->iqTargV = _IQ(0.0);
    handle->iqTargQ = _IQ(0.0);
    handle->iqTargD = _IQ(0.0);
}

/*** state & config ***/
/***********************************************************************************************************************************************/

/***********************************************************************************************************************************************/
/*** PDO & SDO ***/

pdo_typedef *pdo_new(void)
{

    return (pdo_typedef *)calloc(1, sizeof(pdo_typedef));
}

sdo_typedef *sdo_new(void)
{

    return (sdo_typedef *)calloc(1, sizeof(sdo_typedef));
}

void pdo_init(pdo_typedef *handle)
{
    handle->PDO_TARGET_POSITION = BIT_11;
    handle->PDO_TARGET_VELOCITY = BIT_15;
    handle->PDO_TARGET_CURRENT_Q = BIT_11;
    handle->PDO_MODE_OF_OPERATION = 0;
}

void sdo_init(sdo_typedef *handle, config_typedef *config)
{

    handle->SDO_PID_POS_KP_INT = 0;
    handle->SDO_PID_POS_KP_DEC = 0;

    handle->SDO_POSITION_LOWER_LIMIT = 0;
    handle->SDO_POSITION_UPPER_LIMIT = BIT_12;
    handle->SDO_VELOCITY_LOWER_LIMIT = 24000;
    handle->SDO_VELOCITY_UPPER_LIMIT = 41000;
    handle->SDO_CURRENT_Q_LOWER_LIMIT = 800;
    handle->SDO_CURRENT_Q_UPPER_LIMIT = 3300;

    handle->SDO_ID = config->ID;
}

/*** PDO & SDO ***/
/***********************************************************************************************************************************************/

/***********************************************************************************************************************************************/
/*** foc ***/

// FOC核心函数：输入Ud、Uq和电角度，输出PWM

void compute_svpwm(_iq Uq, _iq Ud, _iq angle_elec)
{
    static _iq Uref;
    static _iq T0, T1, T2;
    static _iq Ta, Tb, Tc;
    static uint8_t sector;
    static int Pa, Pb, Pc;
    static _iq U_alpha, U_beta;

    if (Uq > 0)
    {
        angle_elec = angle_elec + _IQ(_PI_2);
    }
    else
    {
        angle_elec = angle_elec - _IQ(_PI_2);
    }

    U_alpha = _IQmpy(Ud, _IQcos(angle_elec)) - _IQmpy(Uq, _IQsin(angle_elec)); // 反park变换
    U_beta = _IQmpy(Ud, _IQsin(angle_elec)) + _IQmpy(Uq, _IQcos(angle_elec));

    Uref = _IQsqrt(_IQmpy(U_alpha, U_alpha) + _IQmpy(U_beta, U_beta));

    if (Uref > _IQ(_1_SQRT3))
    {
        Uref = _IQ(_1_SQRT3); // 六边形的内切圆(SVPWM最大不失真旋转电压矢量赋值)根号3/3
    }
    if (Uref < _IQ(-_1_SQRT3))
    {
        Uref = _IQ(-_1_SQRT3);
    }

    sector = _IQint(_IQdiv(angle_elec, _IQ(_PI_3))); // 根据角度判断参考电压所在扇区
    sector = sector % 6 + 1;
    T1 = _IQmpy(_IQmpy(_IQ(_SQRT3), Uref), _IQsin(_IQ(sector * _PI_3) - angle_elec)); // 计算两个相邻电压矢量作用时间
    T2 = _IQmpy(_IQmpy(_IQ(_SQRT3), Uref), _IQsin(angle_elec - _IQmpy(_IQ(sector - 1), _IQ(_PI_3))));
    T0 = _IQ(1.0) - T1 - T2; // 零矢量作用时间

    switch (sector)
    {
    case 1:
        Ta = T1 + T2 + _IQmpy(_IQ(0.5), T0);
        Tb = T2 + _IQmpy(_IQ(0.5), T0);
        Tc = _IQmpy(_IQ(0.5), T0);
        break;
    case 2:
        Ta = T1 + _IQmpy(_IQ(0.5), T0);
        Tb = T1 + T2 + _IQmpy(_IQ(0.5), T0);
        Tc = _IQmpy(_IQ(0.5), T0);
        break;
    case 3:
        Ta = _IQmpy(_IQ(0.5), T0);
        Tb = T1 + T2 + _IQmpy(_IQ(0.5), T0);
        Tc = T2 + _IQmpy(_IQ(0.5), T0);
        break;
    case 4:
        Ta = _IQmpy(_IQ(0.5), T0);
        Tb = T1 + _IQmpy(_IQ(0.5), T0);
        Tc = T1 + T2 + _IQmpy(_IQ(0.5), T0);
        break;
    case 5:
        Ta = T2 + _IQmpy(_IQ(0.5), T0);
        Tb = _IQmpy(_IQ(0.5), T0);
        Tc = T1 + T2 + _IQmpy(_IQ(0.5), T0);
        break;
    case 6:
        Ta = T1 + T2 + _IQmpy(_IQ(0.5), T0);
        Tb = _IQmpy(_IQ(0.5), T0);
        Tc = T1 + _IQmpy(_IQ(0.5), T0);
        break;
    default: // possible error state
        Ta = 0.0;
        Tb = 0.0;
        Tc = 0.0;
    }

    Pa = _IQint(_IQmpy(Ta, _IQ(PWM_PERIOD)));
    Pb = _IQint(_IQmpy(Tb, _IQ(PWM_PERIOD)));
    Pc = _IQint(_IQmpy(Tc, _IQ(PWM_PERIOD)));

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Pa);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Pb);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Pc);
}

void calib_elec_angle(state_typedef* state, sdo_typedef* sdo)
{
    // 自动电角度对正
    static uint16_t calib_position[100];
    static long int pos_mean;
    for (int i = 0; i < 100; i++)
    {
        compute_svpwm(_IQ(0.5), _IQ(0.0), _IQ(0.0));
        as5600_get_angle(oEncoder, state);
        calib_position[i] = state->actual_position;
        pos_mean += calib_position[i];
        HAL_Delay(10);
    }
    compute_svpwm(_IQ(0.0), _IQ(0.0), _IQ(0.0));
    // 标定失败的判断(峰峰值过大)
    
    sdo->SDO_ELEC_ZERO_POSITION = pos_mean / 100;
    // 暂时用打印肉眼观察
    printf("elec_zero_pos = %d, actual_pos = %d", sdo->SDO_ELEC_ZERO_POSITION, state->actual_position);
}

void calib_adc_offset(pdo_typedef* pdo, sdo_typedef* sdo)
{
    static uint16_t calib_adc_0[100], calib_adc_1[100];
    static long int adc_0_mean, adc_1_mean;
    pdo->PDO_MODE_OF_OPERATION = MODE_NO;
    for (int i = 0; i < 100; i++)
    {
        calib_adc_0[i] = ADC_DMA_AVERAGE(0);
        calib_adc_1[i] = ADC_DMA_AVERAGE(1);
        adc_0_mean += calib_adc_0[i];
        adc_1_mean += calib_adc_1[i];
        HAL_Delay(100);
    }
    // 标定失败的判断(峰峰值过大或平均值离2048太远)
    sdo->SDO_ADC0_OFFSET = adc_0_mean / 100;
    sdo->SDO_ADC1_OFFSET = adc_1_mean / 100;
    printf("adc offset = %d, %d", sdo->SDO_ADC0_OFFSET, sdo->SDO_ADC1_OFFSET);
}

/*** foc ***/
/***********************************************************************************************************************************************/

// uint16_t compute_following_error(state_typedef* handle){
//     switch (handle->mode){
//         case MODE_CSP:
//         handle->following_error = handle->target_position - handle->actual_position + BIT_15;
//
//         break;
//         case MODE_CSV:
//         handle->following_error = handle->target_velocity - handle->actual_position + BIT_15;
//
//         break;
//         case MODE_CST:
//         handle->following_error = handle->target_current_q - handle->actual_current_q + BIT_15;
//
//         break;
//         otherwise:
//         handle->following_error = 0;
//         break;
//
//     return handle->following_error;
//     }
//
// }

void para_init(sdo_typedef* sdo, config_typedef* config)
{
    config_init(sdo, config);
    pid_init(oPidPosition, config, PID_INDEX_POS);
    pid_init(oPidVelocity, config, PID_INDEX_VEL);
    pid_init(oPidCurrentQ, config, PID_INDEX_CUR_Q);
    pid_init(oPidCurrentD, config, PID_INDEX_CUR_D);

    filter_init(oFilterVelocity, config->FILT_VEL_CUTOFF_FREQ, config->CONST_POSITION_SAMP_TIME);
    filter_init(oFilterCurrentQ, config->FILT_CUR_Q_CUTOFF_FREQ, config->CONST_CURRENT_SAMP_TIME);
    filter_init(oFilterCurrentD, config->FILT_CUR_D_CUTOFF_FREQ, config->CONST_CURRENT_SAMP_TIME);
}

_iq compute_position_elec(_iq pos)
{
    //    return _IQmpy(pos, oConfig->CONST_POLAR_PAIRS);
    _iq pos_elec_0 = convert_pulse_to_position(_IQint(oConfig->ELEC_ZERO_POSITION));
    return _IQmpy(pos - pos_elec_0, oConfig->CONST_POLAR_PAIRS);
}

_iq convert_pulse_to_position(uint16_t actual_position)
{
    return _IQ15toIQ(_IQ15div(_IQ15mpy(_IQ15(actual_position) - oConfig->ENC_ZERO_POSITION, _IQ15(_2PI)), oConfig->CONST_ENC_RESOLUTION));
}

_iq convert_pulse_to_velocity(int32_t actual_velocity)
{

    return _IQ((float)(actual_velocity - BIT_15) / _IQ15toF(oConfig->CONST_ENC_RESOLUTION) * _2PI);
}

_iq convert_pulse_to_current(uint16_t actual_current)
{
    return _IQmpyI32(oConfig->CONST_PULSE_TO_CUR_SLOPE, actual_current) + oConfig->CONST_PULSE_TO_CUR_OFFSET;
}

uint16_t convert_position_to_pulse(_iq pos)
{
    return _IQ15int(_IQ15div(_IQ15mpyIQX(oConfig->CONST_ENC_RESOLUTION, 15, pos, 20), _IQ15(_2PI)) + oConfig->ENC_ZERO_POSITION);
}

uint16_t convert_velocity_to_pulse(_iq vel)
{
    return _IQ15int(_IQ15div(_IQ15mpyIQX(oConfig->CONST_ENC_RESOLUTION, 15, vel, 20), _IQ15(_2PI)) + _IQ15(BIT_15));
}

uint16_t convert_current_to_pulse(_iq cur)
{
    return _IQ15int(_IQ15mpy(_IQtoIQ15(cur), oConfig->CONST_CUR_TO_PULSE_SLOPE) + oConfig->CONST_CUR_TO_PULSE_OFFSET);
}
