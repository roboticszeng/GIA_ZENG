#include "tim.h"
#include <math.h>
#include "foc_kernal.h"
#include <simple_math.h>

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
    angle_elec = actualPos * CONST_POLAR_PAIRS * _2PI / CONST_ENC_RESOLUTION;
    angle_elec = _normalizeAngle(angle_elec); // 电角度标准化在[0, _2PI]
}


/***************************************************************************/

/******************************************************************************/
//FOC核心函数：输入Ud、Uq和电角度，输出PWM
void setPhaseVoltage(float Uq, float Ud)
{
	float Uref;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;
	float U_alpha,U_beta;
	
	                    

	U_alpha = Ud * _cos(angle_elec) - Uq * _sin(angle_elec);            //反park变换
	U_beta = Ud * _sin(angle_elec) + Uq * _cos(angle_elec);
	
	//Uref = _sqrtApprox(U_alpha * U_alpha + U_beta * U_beta) / 24;
    Uref = _sqrtApprox(U_alpha * U_alpha + U_beta * U_beta);
	
	if(Uref > 0.577){
        Uref= 0.577;                     			//六边形的内切圆(SVPWM最大不失真旋转电压矢量赋值)根号3/3
    }
	if(Uref < -0.577){
        Uref=-0.577;
    }
	
	if(Uq > 0){
        angle_elec = _normalizeAngle(angle_elec + _PI_2);            //加90度后是参考电压矢量的位置
    }
	else{
        angle_elec = _normalizeAngle(angle_elec - _PI_2);
    }
		
	sector = (angle_elec / _PI_3) + 1;                			//根据角度判断参考电压所在扇区                        

	T1 = _SQRT3 * _sin(sector * _PI_3 - angle_elec) * Uref;           //计算两个相邻电压矢量作用时间
	T2 = _SQRT3 * _sin(angle_elec - (sector - 1.0) * _PI_3) * Uref;
	T0 = 1 - T1 - T2;                                          //零矢量作用时间
	
	// calculate the duty cycles(times)
	switch(sector) 
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
    
    
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Ta * PWM_PERIOD);      //输出PWM的函数
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Tb * PWM_PERIOD);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Tc * PWM_PERIOD);
}
