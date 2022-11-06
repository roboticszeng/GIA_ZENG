#ifndef _constrain

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#endif

/******************************************************************************/
float pid_Id_P, pid_Iq_P, pid_vel_P, pid_ang_P;
float pid_Id_I, pid_Iq_I, pid_vel_I, pid_ang_I;
float pid_Id_D, pid_Iq_D, pid_vel_D, pid_ang_D; 
float integral_Id_prev, integral_Iq_prev, integral_vel_prev, integral_ang_prev;
float error_Id_prev, 		error_Iq_prev, 		error_vel_prev, 	 error_ang_prev;

float current_limit;
float velosity_limit;
//extern float Ts;
/******************************************************************************/
void PID_init(void)
{
	current_limit=0.75;
	velosity_limit=25;
    
	
//    pid_vel_P = 10;
//    pid_vel_I = 0;
	pid_vel_P = 0.008;  //0.1
	pid_vel_I = 0.5;    //1
	pid_vel_D = 0;    //0
	integral_vel_prev=0;
	error_vel_prev=0;
	
	pid_ang_P = 20;
	pid_ang_I = 0;
	pid_ang_D = 0;
	integral_ang_prev=0;
	error_ang_prev=0;
	

	
	pid_Iq_P = 10;                    
	pid_Iq_I = 2000;
	pid_Iq_D = 0;     
	integral_Iq_prev=0;
	error_Iq_prev=0;
    
    pid_Id_P = 5;
	pid_Id_I = 0;
	pid_Id_D = 0;
	integral_Id_prev=0;
	error_Id_prev=0;
    
}
/******************************************************************************/
float PID_current_D(float error)
{
	float proportional,integral,derivative,output;
    float Ts_i = 0.002;

	proportional = pid_Id_P * error;
	integral = integral_Id_prev + pid_Id_I*Ts_i*0.5*(error + error_Id_prev);
	derivative = pid_Id_D*(error - error_Id_prev)/Ts_i;
	
	output = proportional + integral + derivative;
	
	// saving for the next pass
	integral_Id_prev = integral;
	error_Id_prev = error;
	
	return output;
}
/******************************************************************************/
float PID_current_Q(float error)
{
	float proportional,integral,derivative,output;
    float Ts_i = 0.002;

	proportional = pid_Iq_P * error;
	integral = integral_Iq_prev + pid_Iq_I*Ts_i*0.5*(error + error_Iq_prev);
	derivative = pid_Iq_D*(error - error_Iq_prev)/Ts_i;
	
	output = proportional + integral + derivative;
	
	// saving for the next pass
	integral_Iq_prev = integral;
	error_Iq_prev = error;
	
	return output;
}
/******************************************************************************/
float PID_velocity(float error)
{
	float proportional,integral,derivative,output;
    float Ts_p = 0.004;

	proportional = pid_vel_P * error;
	integral = integral_vel_prev + pid_vel_I*Ts_p*0.5*(error + error_vel_prev);
	integral = _constrain(integral, -current_limit * 0.5, current_limit * 0.5);
	derivative = pid_vel_D*(error - error_vel_prev)/Ts_p;
	
	output = proportional + integral + derivative;
	output = _constrain(output, -current_limit, current_limit);
	
	// saving for the next pass
	integral_vel_prev = integral;
	error_vel_prev = error;
	
	return output;
}
/******************************************************************************/
float PID_angle(float error)
{
	float proportional,integral,derivative,output;
    float Ts_p = 0.004;

	proportional = pid_ang_P * error;
	integral = integral_ang_prev + pid_ang_I*Ts_p*0.5* + error_ang_prev);
	//integral = _constrain(output, -velosity_limit, velosity_limit);
	derivative = pid_vel_D*(error - error_ang_prev)/Ts_p;
	
	output = proportional + integral + derivative;
	output = _constrain(output, -velosity_limit, velosity_limit);
	
	// saving for the next pass
	integral_ang_prev = integral;
	error_ang_prev = error;
	
	return output;
}
/******************************************************************************/


