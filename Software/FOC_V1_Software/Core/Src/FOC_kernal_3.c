#include "tim.h"
#include <math.h>
#include "FOC_kernal_3.h"

// 中心对齐PWM会不会改善效果？
#define PWM_Period 1200
/******************************************************************************/
float voltage_power_supply = 24;              //24V
float sensor_offset = 0;
float zero_electric_angle = 0;
/******************************************************************************/
// 标准化角度 [0,2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, _2PI);                     //fmod()对浮点数取模
  return a >= 0 ? a : (a + _2PI);
}

const int sine_array[200] = {0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000};

/***************************************************************************/
// function approximating the sine calculation by using fixed size array
float _sin(float a){
  if(a < _PI_2){
    //return sine_array[(int)(199.0*( a / (_PI/2.0)))];
    //return sine_array[(int)(126.6873* a)];           // float array optimized
    return 0.0001*sine_array[_round(126.6873* a)];      // int array optimized
  }else if(a < _PI){
    // return sine_array[(int)(199.0*(1.0 - (a-_PI/2.0) / (_PI/2.0)))];
    //return sine_array[398 - (int)(126.6873*a)];          // float array optimized
    return 0.0001*sine_array[398 - _round(126.6873*a)];     // int array optimized
  }else if(a < _3PI_2){
    // return -sine_array[(int)(199.0*((a - _PI) / (_PI/2.0)))];
    //return -sine_array[-398 + (int)(126.6873*a)];           // float array optimized
    return -0.0001*sine_array[-398 + _round(126.6873*a)];      // int array optimized
  } else {
    // return -sine_array[(int)(199.0*(1.0 - (a - 3*_PI/2) / (_PI/2.0)))];
    //return -sine_array[796 - (int)(126.6873*a)];           // float array optimized
    return -0.0001*sine_array[796 - _round(126.6873*a)];      // int array optimized
  }
}
/***************************************************************************/
// function approximating cosine calculation by using fixed size array
float _cos(float a){
  float a_sin = a + _PI_2;
  a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
  return _sin(a_sin);
}
/***************************************************************************/
//近似开根号函数
float _sqrtApprox(float number) {//low in fat
  long i;
  float y;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i; 
  return number * y;
}
/***************************************************************************/

/******************************************************************************/
//FOC核心函数：输入Ud、Uq和电角度，输出PWM
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	float Uref;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;
	float U_alpha,U_beta;
	
	angle_el =_normalizeAngle(angle_el);                    //电角度标准化在【0,2pi】

	U_alpha=Ud*_cos(angle_el)-Uq*_sin(angle_el);            //反park变换
	U_beta=Ud*_sin(angle_el)+Uq*_cos(angle_el);
	
	Uref=_sqrtApprox(U_alpha*U_alpha + U_beta*U_beta) / voltage_power_supply;
	
	if(Uref> 0.577)Uref= 0.577;                     			//六边形的内切圆(SVPWM最大不失真旋转电压矢量赋值)根号3/3
	if(Uref<-0.577)Uref=-0.577; 
	
	if(Uq>0)
	  angle_el =_normalizeAngle(angle_el+_PI_2);            //加90度后是参考电压矢量的位置
	else
		angle_el =_normalizeAngle(angle_el-_PI_2);
	sector = (angle_el / _PI_3) + 1;                			//根据角度判断参考电压所在扇区                        

	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uref;           //计算两个相邻电压矢量作用时间
	T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uref;
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
    
    
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Ta*PWM_Period);      //输出PWM的函数
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Tb*PWM_Period);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Tc*PWM_Period);
}
