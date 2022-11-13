#include "tim.h"
#include <math.h>
#include "foc_kernal.h"
#include <simple_math.h>
#include "IQmathLib.h"

extern int PWM_PERIOD;
extern uint16_t CONST_POLAR_PAIRS;
extern uint16_t CONST_ENC_RESOLUTION;
extern uint16_t actualPos;

float angle_elec = 0.0;



float _normalizeAngle(float angle)
{
    float a = fmod(angle, _2PI);                     //fmod()对浮点数取模
    return a >= 0 ? a : (a + _2PI);
}



void get_angle_elec(void){
    // angle_elec = _IQ10toF(_IQ10div( _IQ10mpy(_IQ10(actualPos * CONST_POLAR_PAIRS), _IQ10(_2PI)), _IQ10(CONST_ENC_RESOLUTION)));
    angle_elec = actualPos * CONST_POLAR_PAIRS * _2PI / CONST_ENC_RESOLUTION;
    angle_elec = _normalizeAngle(angle_elec); // 电角度标准化在[0, _2PI]
}


/***************************************************************************/

/******************************************************************************/
int Pa, Pb, Pc;
float Pf[8];

int Pa_old, Pb_old, Pc_old;
float Pf_old[8];
//FOC核心函数：输入Ud、Uq和电角度，输出PWM
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	_iq Uref;
	// _iq sector;
	_iq T0,T1,T2;
    _iq Ta,Tb,Tc;
    uint8_t sector;
	
	_iq U_alpha,U_beta;
    
    _iq angle_eiq;
	
    if(Uq > 0){
//        angle_el += _PI_2;
        angle_el = _normalizeAngle(angle_el + _PI_2);            //加90度后是参考电压矢量的位置
    }
	else{
//        angle_el -= _PI_2;
        angle_el = _normalizeAngle(angle_el - _PI_2);
    }
    
    angle_eiq = _IQ(angle_elec);
	                    

	U_alpha = _IQmpy(_IQ(Ud), _IQcos(angle_eiq)) - _IQmpy(_IQ(Uq), _IQsin(angle_eiq));            //反park变换
	U_beta = _IQmpy(_IQ(Ud), _IQsin(angle_eiq)) + _IQmpy(_IQ(Uq), _IQcos(angle_eiq));
	
    Uref = _IQsqrt(_IQmpy(U_alpha, U_alpha) + _IQmpy(U_beta, U_beta));
	
	if(Uref > _IQ(_1_SQRT3)){
        Uref = _IQ(_1_SQRT3);                     			//六边形的内切圆(SVPWM最大不失真旋转电压矢量赋值)根号3/3
    }
	if(Uref < _IQ(-_1_SQRT3)){
        Uref = _IQ(-_1_SQRT3);
    }
	

		
	sector = _IQint(_IQdiv(angle_eiq, _IQ(_PI_3))) + 1;                			//根据角度判断参考电压所在扇区                        

	T1 = _IQmpy(_IQmpy(_IQ(_SQRT3), Uref), _IQsin(_IQ(sector * _PI_3 - angle_el)));           //计算两个相邻电压矢量作用时间
	T2 = _IQmpy(_IQmpy(_IQ(_SQRT3), Uref), _IQsin(_IQ(angle_el - (sector - 1) * _PI_3)));
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
    
    Pa = _IQint(_IQmpy(Ta, _IQ(PWM_PERIOD)));
    Pb = _IQint(_IQmpy(Tb, _IQ(PWM_PERIOD)));
    Pc = _IQint(_IQmpy(Tc, _IQ(PWM_PERIOD)));
    
    
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, Pa);      //输出PWM的函数
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, Pb);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, Pc);
    
}

