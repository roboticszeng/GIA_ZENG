#include "controll.h"
#include "tim.h"
#include <math.h>
#include "IQmathLib.h"

float temp[10];

/***********************************************************************************************************************************************/
/*** PID ***/

pid_typedef* pid_new(void) { 
    
    return (pid_typedef*)calloc(1, sizeof(pid_typedef)); 
}

void pid_init(pid_typedef *handle, float Kp, float Ki, float T, float Max, float MaxInt) {

	/* Clear controller variables */
    // 暂时不用Kd
    
    handle->Kp = _IQ15(Kp);
    handle->Ki = _IQ15(Ki);
    handle->T = _IQ(T);
    handle->limMax = _IQ15(Max);
    handle->limMin = -_IQ15(Max);
    handle->limMaxInt = _IQ15(MaxInt);
    handle->limMinInt = -_IQ15(MaxInt);
    
    handle->Kd = _IQ15(0.0);
    handle->tau = _IQ15(0.0);
	handle->integrator = _IQ15(0.0);
	handle->prevError  = _IQ15(0.0);
	handle->differentiator  = _IQ15(0.0);
	handle->prevMeasurement = _IQ15(0.0);
	handle->out = _IQ15(0.0);

}

_iq pid_update(pid_typedef *pid, _iq setpoint, _iq measurement) {

    static _iq15 error;
    error = _IQtoIQ15(setpoint) - _IQtoIQ15(measurement);
    
//    printf("%f\r\n", _IQ15toF(error));
    
    pid->proportional = _IQ15mpy(pid->Kp, error);
    
    pid->integrator = pid->integrator + _IQ15mpyIQX(_IQmpy(_IQmpy(_IQ(0.5), _IQ15toIQ(pid->Ki)), pid->T), 20, (error + pid->prevError), 15);
//    pid->integrator = _IQ15mpyIQX(_IQmpy(_IQmpy(_IQ(0.5), _IQ15toIQ(pid->Ki)), pid->T), 20, (error + pid->prevError), 15);
    
    if (pid->integrator > pid->limMaxInt) {
        pid->integrator = pid->limMaxInt;
    } else if (pid->integrator < pid->limMinInt) {
        pid->integrator = pid->limMinInt;
    }
//    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
//                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
//                        / (2.0f * pid->tau + pid->T);
    // 暂时不用KD
    pid->differentiator = _IQ15(0.0);
    pid->out = pid->proportional + pid->integrator + pid->differentiator;
    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }
    pid->prevError       = error;
    pid->prevMeasurement = _IQtoIQ15(measurement);
    return _IQ15toIQ(pid->out);
}

/*** PID ***/
/***********************************************************************************************************************************************/




/***********************************************************************************************************************************************/
/*** lowpass filter ***/

filter_typedef* filter_new(void){
    return (filter_typedef*)calloc(1, sizeof(filter_typedef)); 
}


void filter_init(filter_typedef* handle, float cutoff_freq, float sample_time){
    
    float rc = 1.0 / (_2PI * cutoff_freq);
    handle->k[0] = _IQ(sample_time / (sample_time + rc));
    handle->k[1] = _IQ(rc / (sample_time + rc));
//    handle->k[0] = _IQ(0.1);
//    handle->k[1] = _IQ(0.9);
    
    handle->output[0] = _IQ(0.0);
    handle->output[1] = _IQ(0.0);
    
}

_iq filter_update(filter_typedef* handle, _iq input){
    
    handle->output[1] = handle->output[0];
    handle->output[0] = _IQmpy(handle->k[0], input) + _IQmpy(handle->k[1], handle->output[1]);
    
    return handle->output[0];
    
}

/*** lowpass filter ***/
/***********************************************************************************************************************************************/





/***********************************************************************************************************************************************/
/*** pdo & sdo ***/


sdo_typedef* config_new(void) { 
    
    return (sdo_typedef*)calloc(1, sizeof(sdo_typedef)); 
}

void config_init(sdo_typedef *handle){
    
//    handle->CONST_PWM_PERIOD = _IQ(__HAL_TIM_GET_AUTORELOAD(&htim1) + 1);
    handle->CONST_PWM_PERIOD = _IQ(1500);
    // check 怎么get prescaler?
    handle->CONST_POSITION_SAMP_TIME = _IQ(1e-3);
    
    handle->CONST_POLAR_PAIRS = _IQ(14);
    
    handle->CONST_ZERO_POSITION = _IQ15(0);
    handle->CONST_ENC_RESOLUTION = _IQ15(4096);
    handle->CONST_ADC_RESOLUTION = _IQ15(4096);
    
    handle->CONST_MCU_VOLTAGE = _IQ(3.3);
    handle->CONST_CUR_SAMP_GAIN = _IQ(50);
    handle->CONST_CUR_SAMP_RESISTANCE = _IQ(0.01);
    
    handle->CONST_ADC0_OFFSET = -63;
    handle->CONST_ADC1_OFFSET = -63;
    
    handle->CONST_CURRENT_SAMP_TIME = _IQ(62.5e-6);
    
//    handle->CONST_PULSE_TO_CUR_SLOPE = _IQdiv(handle->CONST_MCU_VOLTAGE, \
//        _IQ15mpyIQX(_IQmpy(handle->CONST_CUR_SAMP_GAIN, handle->CONST_CUR_SAMP_RESISTANCE), 20, handle->CONST_ADC_RESOLUTION, 15));
    handle->CONST_PULSE_TO_CUR_SLOPE = _IQ(0.001611328);

    handle->CONST_PULSE_TO_CUR_OFFSET = -_IQdiv(handle->CONST_MCU_VOLTAGE, \
        _IQmpy(_IQmpy(handle->CONST_CUR_SAMP_GAIN, handle->CONST_CUR_SAMP_RESISTANCE), _IQ(2.0)));

//    handle->CONST_PULSE_TO_CUR_SLOPE = _IQ5(0.001611328);
//    handle->CONST_PULSE_TO_CUR_OFFSET = _IQ(-3.3);

    handle->CONST_CUR_TO_PULSE_SLOPE = _IQdiv(_IQmpy(_IQmpy(handle->CONST_CUR_SAMP_GAIN, handle->CONST_CUR_SAMP_RESISTANCE), handle->CONST_ADC_RESOLUTION), \
        handle->CONST_MCU_VOLTAGE);
    handle->CONST_CUR_TO_PULSE_OFFSET = _IQdiv(handle->CONST_ADC_RESOLUTION, _IQ(2.0));
    
    handle->CONST_CURRENT_CONTROL_TIME = 160e-6;
    handle->CONST_POSITION_CONTROL_TIME = 320e-6;
    
}

pdo_typedef* pdo_new(void) { 
    
    return (pdo_typedef*)calloc(1, sizeof(pdo_typedef)); 
}

void pdo_init(pdo_typedef *handle){
    
    handle->mode = MODE_CSP;
    handle->target_position = 0;
    handle->target_velocity = BIT_15;
    handle->target_current_q = BIT_15;
    
    handle->iqTargQ = _IQ(0.0);
    handle->iqTargD = _IQ(0.0);
    
}


/*** pdo & sdo ***/
/***********************************************************************************************************************************************/





/***********************************************************************************************************************************************/
/*** foc ***/

//FOC核心函数：输入Ud、Uq和电角度，输出PWM
extern sdo_typedef* oConfig;
void compute_svpwm(_iq Uq, _iq Ud, _iq angle_elec)
{
	static _iq Uref;
	static _iq T0,T1,T2;
    static _iq Ta,Tb,Tc;
    static uint8_t sector;
    static int Pa, Pb, Pc;
	static _iq U_alpha,U_beta;
    
    if(Uq > 0){
        angle_elec = angle_elec + _IQ(_PI_2);
    }
	else{
        angle_elec = angle_elec - _IQ(_PI_2);
    }

	U_alpha = _IQmpy(Ud, _IQcos(angle_elec)) - _IQmpy(Uq, _IQsin(angle_elec));            //反park变换
	U_beta = _IQmpy(Ud, _IQsin(angle_elec)) + _IQmpy(Uq, _IQcos(angle_elec));
	
    Uref = _IQsqrt(_IQmpy(U_alpha, U_alpha) + _IQmpy(U_beta, U_beta));
	
	if(Uref > _IQ(_1_SQRT3)){
        Uref = _IQ(_1_SQRT3);                     			//六边形的内切圆(SVPWM最大不失真旋转电压矢量赋值)根号3/3
    }
	if(Uref < _IQ(-_1_SQRT3)){
        Uref = _IQ(-_1_SQRT3);
    }
	
	sector = _IQint(_IQdiv(angle_elec, _IQ(_PI_3)));                			//根据角度判断参考电压所在扇区                        
    sector = sector % 6 + 1;
	T1 = _IQmpy(_IQmpy(_IQ(_SQRT3), Uref), _IQsin(_IQ(sector * _PI_3) - angle_elec));           //计算两个相邻电压矢量作用时间
	T2 = _IQmpy(_IQmpy(_IQ(_SQRT3), Uref), _IQsin(angle_elec - _IQmpy(_IQ(sector - 1), _IQ(_PI_3))));
	T0 = _IQ(1.0) - T1 - T2;                                          //零矢量作用时间
	
	switch(sector) 
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
			Tb = T1+ _IQmpy(_IQ(0.5), T0);
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
		default:  // possible error state
			Ta = 0.0;
			Tb = 0.0;
			Tc = 0.0;
	}
    
    Pa = _IQint(_IQmpy(Ta, oConfig->CONST_PWM_PERIOD));
    Pb = _IQint(_IQmpy(Tb, oConfig->CONST_PWM_PERIOD));
    Pc = _IQint(_IQmpy(Tc, oConfig->CONST_PWM_PERIOD));
    
    
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, Pa);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, Pb);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, Pc);

}

/*** foc ***/
/***********************************************************************************************************************************************/

uint16_t compute_following_error(pdo_typedef* handle){
    switch (handle->mode){
        case MODE_CSP:
        handle->following_error = handle->target_position - handle->actual_position + BIT_15;
        
        break;
        case MODE_CSV:
        handle->following_error = handle->target_velocity - handle->actual_position + BIT_15;
        
        break;
        case MODE_CST:
        handle->following_error = handle->target_current_q - handle->actual_current_q + BIT_15;
        
        break;
        otherwise:
        handle->following_error = 0;
        break;
    
    return handle->following_error;
    }
    
}

_iq compute_position_elec(_iq pos){
    return _IQmpy(pos, oConfig->CONST_POLAR_PAIRS);
}

_iq convert_pulse_to_position(uint16_t actual_position){
    
//    return _IQmpyIQX(_IQ15div(_IQ15(actual_position) - oConfig->CONST_ZERO_POSITION, oConfig->CONST_ENC_RESOLUTION), 15, _IQ(_2PI), 20);
    return _IQ15toIQ(_IQ15div(_IQ15mpy(_IQ15(actual_position) - oConfig->CONST_ZERO_POSITION, _IQ15(_2PI)), oConfig->CONST_ENC_RESOLUTION));
}

_iq convert_pulse_to_velocity(uint32_t actual_velocity){
    
    
    return _IQ((float)(actual_velocity - BIT_15)  / 4096.0 * _2PI);
    
//    return _IQ15toIQ(_IQ15div(_IQ15mpyIQX(_IQ15(actual_velocity - BIT_15), 15, _IQ(_2PI), 20), oConfig->CONST_ENC_RESOLUTION));
    
}

_iq convert_pulse_to_current(uint16_t actual_current){
    return _IQmpyI32(oConfig->CONST_PULSE_TO_CUR_SLOPE, actual_current) + oConfig->CONST_PULSE_TO_CUR_OFFSET;
//    return _IQ(actual_current * 0.001611328 - 3.3);
}

uint16_t convert_position_to_pulse(_iq pos){
    return _IQint(_IQdiv(_IQmpy(oConfig->CONST_ENC_RESOLUTION, pos), _IQ(_2PI)) + oConfig->CONST_ZERO_POSITION);
}
    
uint32_t convert_velocity_to_pulse(_iq vel){
    return _IQint(_IQdiv(_IQmpy(oConfig->CONST_ENC_RESOLUTION, vel), _IQ(_2PI)) + _IQ(BIT_15));
}

uint16_t convert_current_to_pulse(_iq cur){
    return _IQint(_IQmpy(cur, oConfig->CONST_CUR_TO_PULSE_SLOPE) + oConfig->CONST_CUR_TO_PULSE_OFFSET);
}

