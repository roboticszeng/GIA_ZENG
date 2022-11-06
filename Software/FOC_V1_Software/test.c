#include <stdio.h>
//#include <stdlib.h>

#define BIT_8 256
#define BIT_11 2048
#define BIT_12 4096
#define BIT_15 32768
#define BIT_16 65536

#define DATA_HEAD 0x55

#define TYPE_NO_MODE 0x00
#define TYPE_PP_MODE 0x01
#define TYPE_CSP_MODE 0x08
#define TYPE_CSV_MODE 0x09
#define TYPE_CST_MODE 0x0a
#define TYPE_READ_MODE 0x10
#define TYPE_INTERNAL_CSP_MODE 0x12
#define TYPE_INTERNAL_CSV_MODE 0x13
#define TYPE_INTERNAL_CST_MODE 0x14
#define TYPE_WRITE_MODE 0x20
#define TYPE_ELE_CALIB_MODE 0x21
#define TYPE_ENC_CALIB_MODE 0x22
#define TYPE_ADC_CALIB_MODE 0x23
#define TYPE_INIT_MODE 0xff

#define CONFIG_ACTUAL_STATE 0x3000
#define CONFIG_TARGET_STATE 0x3080
#define CONFIG_POS_DEMAND 0x3300
#define CONFIG_VEL_DEMAND 0x3303
#define CONFIG_CUR_PHASE_ACTUAL 0x3309
#define CONFIG_CUR_Q_DEMAND 0x330d
#define CONFIG_CUR_D_DEMAND 0x330e
#define CONFIG_POS_LOOP_PARA_STD 0x3506
#define CONFIG_VEL_LOOP_PARA_STD 0x3507
#define CONFIG_CUR_Q_LOOP_PARA_STD 0x3508
#define CONFIG_CUR_D_LOOP_PARA_STD 0x3509
#define CONFIG_POS_LOOP_PARA_PER 0x3516
#define CONFIG_VEL_LOOP_PARA_PER 0x3517
#define CONFIG_CUR_Q_LOOP_PARA_PER 0X3518
#define CONFIG_CUR_D_LOOP_PARA_PER 0X3519
#define CONFIG_POS_LIMIT 0x3510
#define CONFIG_VEL_LIMIT 0x3512
#define CONFIG_CUR_Q_LIMIT 0x3514
#define CONFIG_CUR_D_LIMIT 0x3515
#define CONFIG_MOTOR_CONST 0x3500
#define CONFIG_ENC_CONST 0x3501
#define CONFIG_ADC_CONST 0x3502
#define CONFIG_MECHANICAL_CONST 0x3503
#define CONFIG_CUR_SAMPLE_CONST 0x3505

#define DATA_PENULT 0x0d
#define DATA_TAIL 0x0a

#define _convert_bit_to_byte(bith, bitl) (bith * 0x100 + bitl)
#define _get_lim(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

typedef struct
{
    double Ta;
    double Tb;
    double Tc;
    double Ia;
    double Ib;
    double Ic;
    double Uq;
    double Ud;
    double Iq;
    double Id;
} StructSvpwm;
StructSvpwm *svpwm;

typedef struct
{
    double Kp;
    double Ki;
    double Kd;
} StructPid;
StructPid *pid_pos;
StructPid *pid_vel;
StructPid *pid_cur_q;
StructPid *pid_cur_d;
StructPid *pid_pos_std;
StructPid *pid_vel_std;
StructPid *pid_cur_q_std;
StructPid *pid_cur_d_std;

typedef struct
{
    int Kp;
    int Ki;
    int Kd;
} StructPidPer;
StructPidPer *pid_pos_per;
StructPidPer *pid_vel_per;
StructPidPer *pid_cur_q_per;
StructPidPer *pid_cur_d_per;

unsigned int CONST_JOINT_NUMBER = 0xd3;
unsigned int CONST_POLAR_PAIRS = 14;
unsigned int CONST_SUPPLY_VOLTAGE = 24;
unsigned int CONST_ENC_RESOLUTION = 4096;
unsigned int CONST_ENC_OFFSET = 0;
unsigned int CONST_ELEC_ANG_OFFSET = 0;
unsigned int CONST_ADC_RESOLUTION = 4096;
unsigned int CONST_ADC_OFFSET_0 = 0;
unsigned int CONST_ADC_OFFSET_1 = 0;
unsigned int CONST_DIRECT_CUR_POS = 1;
unsigned int CONST_SAMPLE_RESISTANCE = 10;
unsigned int CONST_SAMPLE_GAIN = 50;
unsigned int CONST_MCU_VOLTAGE = 3300;
unsigned int CONST_POS_HIGHER_BOUND = 4096;
unsigned int CONST_POS_LOWER_BOUND = 0;
unsigned int CONST_VEL_HIGHER_BOUND = 40960;
unsigned int CONST_VEL_LOWER_BOUND = 0;
unsigned int CONST_CUR_Q_HIGHER_BOUND = 4096;
unsigned int CONST_CUR_Q_LOWER_BOUND = 0;
unsigned int CONST_CUR_D_HIGHER_BOUND = 4096;
unsigned int CONST_CUR_D_LOWER_BOUND = 0;

unsigned int targetPos;
unsigned int targetVel;
unsigned int targetCurQ;
unsigned int actualPos;
unsigned int actualVel;
unsigned int actualCurA;
unsigned int actualCurB;
unsigned int actualCurQ;
unsigned int actualCurD;
unsigned int demandPos;
unsigned int demandVel;
unsigned int demandCurQ;
unsigned int limitedPos;
unsigned int limitedVel;
unsigned int limitedCurQ;

double tsPos;
double tsVel;
double tsCur;
int elecAng;
unsigned int configType;

int protocol_analysis(int data[]);

int main()
{

    int a = 1;
    printf("%d", a);

    system("pause");
    return 0;
}

int protocol_analysis(int data[])
{

    int resflag = 0;
    int targetValue;
    int configValue[3];
    int i, j;
    int *readData;

    if (data[0] != DATA_HEAD)
    {
        resflag += 0b0001;
        return resflag;
    }

    if (data[1] != CONST_JOINT_NUMBER)
    {
        resflag += 0b0010;
        return resflag;
    }

    targetValue = _get_2bit_value(data[4], data[5]);
    for (i = 0; i < 3; i++)
    {
        configValue[i] = get2bitValue(data[2 * i + 6], data[2 * i + 7]);
    }

    switch (data[2])
    {
    case TYPE_NO_MODE:
        // pwm output 0
        svpwm->Ta = 0;
        svpwm->Tb = 0;
        svpwm->Tc = 0;
        break;
    case TYPE_PP_MODE:
        // reserve.
        break;
    case TYPE_CSP_MODE:
        targetPos = targetValue;
        // set_pos() 写在it中断函数里
        break;
    case TYPE_CSV_MODE:
        targetVel = targetValue;
        break;
    case TYPE_CST_MODE:
        targetCurQ = targetValue;
        break;
    case TYPE_READ_MODE:
        configType = targetValue;
        readData = read_config(configType);
        break;
    case TYPE_INTERNAL_CSP_MODE:
        // reserve.
        break;
    case TYPE_INTERNAL_CSV_MODE:
        // reserve.
        break;
    case TYPE_INTERNAL_CST_MODE:
        // reserve.
        break;
    case TYPE_WRITE_MODE:
        configType = targetValue;
        write_config(configType, configValue);
        break;
    }
}

int *read_config(int type)
{
    static int data[16];
    data[0] = DATA_HEAD;
    data[1] = CONST_JOINT_NUMBER;
    switch (configType)
    {
    case CONFIG_ACTUAL_STATE:
        data[2] = actualPos / BIT_8;
        data[3] = actualPos % BIT_8;
        data[4] = actualVel / BIT_8;
        data[5] = actualVel % BIT_8;
        data[6] = actualCurQ / BIT_8;
        data[7] = actualCurQ % BIT_8;
        data[8] = actualCurD / BIT_8;
        data[9] = actualCurD % BIT_8;
        // Mode of Operation Display & Following Error 下一版本再实现
        // data[10] = ModeOfOperationDisplay / BIT_8;
        // data[11] = ModeOfOperationDisplay % BIT_8;
        // data[12] = FollowingError / BIT_8;
        // data[13] = FollowingError % BIT_8;
        break;
    case CONFIG_TARGET_STATE:
        data[2] = targetPos / BIT_8;
        data[3] = targetPos % BIT_8;
        data[4] = targetVel / BIT_8;
        data[5] = targetVel % BIT_8;
        data[6] = targetCurQ / BIT_8;
        data[7] = targetCurQ % BIT_8;
        // Velocity Offset & Current Offset & Mode Of Operation 下一版本再实现
        // data[8] = velocityOffset / BIT_8;
        // data[9] = velocityOffset % BIT_8;
        // data[10] = currentOffset / BIT_8;
        // data[11] = currentOffset % BIT_8;
        // data[12] = ModeOfOperation / BIT_8;
        // data[13] = ModeOfOperation % BIT_8;
        break;
    case CONFIG_POS_DEMAND:
        data[2] = demandPos / BIT_8;
        data[3] = demandPos & BIT_8;
        data[4] = limitedPos / BIT_8;
        data[5] = limitedPos & BIT_8;
        // Loop Effort 下一版本再实现
        // data[6] = PositionLoopEffort / BIT_8;
        // data[7] = PositionLoopEffort & BIT_8;
        break;
    case CONFIG_VEL_DEMAND:
        data[2] = demandVel / BIT_8;
        data[3] = demandVel & BIT_8;
        data[4] = limitedVel / BIT_8;
        data[5] = limitedVel & BIT_8;
        // Loop Effort 下一版本再实现
        // data[6] = PositionLoopEffort / BIT_8;
        // data[7] = PositionLoopEffort & BIT_8;
        break;
    case CONFIG_CUR_PHASE_ACTUAL:
        data[2] = actualCurA / BIT_8;
        data[3] = actualCurA % BIT_8;
        data[4] = actualCurB / BIT_8;
        data[5] = actualCurB % BIT_8;
        break;
    case CONFIG_CUR_Q_DEMAND:
        data[2] = demandCurQ / BIT_8;
        data[3] = demandCurQ & BIT_8;
        data[4] = limitedCurQ / BIT_8;
        data[5] = limitedCurQ & BIT_8;
        // Loop Effort 下一版本再实现
        // data[6] = PositionLoopEffort / BIT_8;
        // data[7] = PositionLoopEffort & BIT_8;
        break;

    case CONFIG_CUR_D_DEMAND:
        // reserve.
        break;
    case CONFIG_POS_LOOP_PARA_STD:
        // reserve.
        break;
    case CONFIG_VEL_LOOP_PARA_STD:
        // reserve.
        break;
    case CONFIG_CUR_Q_LOOP_PARA_STD:
        // reserve.
        break;
    case CONFIG_CUR_D_LOOP_PARA_STD:
        // reserve.
        break;

    case CONFIG_POS_LOOP_PARA_PER:
        data[6] = pid_pos_per->Kp / BIT_8;
        data[7] = pid_pos_per->Kp % BIT_8;
        data[8] = pid_pos_per->Ki / BIT_8;
        data[9] = pid_pos_per->Ki % BIT_8;
        data[10] = pid_pos_per->Kd / BIT_8;
        data[11] = pid_pos_per->Kd % BIT_8;
        break;
    case CONFIG_VEL_LOOP_PARA_PER:
        data[6] = pid_vel_per->Kp / BIT_8;
        data[7] = pid_vel_per->Kp % BIT_8;
        data[8] = pid_vel_per->Ki / BIT_8;
        data[9] = pid_vel_per->Ki % BIT_8;
        data[10] = pid_vel_per->Kd / BIT_8;
        data[11] = pid_vel_per->Kd % BIT_8;
        break;
    case CONFIG_CUR_Q_LOOP_PARA_PER:
        data[6] = pid_cur_q_per->Kp / BIT_8;
        data[7] = pid_cur_q_per->Kp % BIT_8;
        data[8] = pid_cur_q_per->Ki / BIT_8;
        data[9] = pid_cur_q_per->Ki % BIT_8;
        data[10] = pid_cur_q_per->Kd / BIT_8;
        data[11] = pid_cur_q_per->Kd % BIT_8;
        break;
    case CONFIG_CUR_D_LOOP_PARA_PER:
        data[6] = pid_cur_d_per->Kp / BIT_8;
        data[7] = pid_cur_d_per->Kp % BIT_8;
        data[8] = pid_cur_d_per->Ki / BIT_8;
        data[9] = pid_cur_d_per->Ki % BIT_8;
        data[10] = pid_cur_d_per->Kd / BIT_8;
        data[11] = pid_cur_d_per->Kd % BIT_8;
        break;
    case CONFIG_POS_LIMIT:
        data[6] = CONST_POS_HIGHER_BOUND / BIT_8;
        data[7] = CONST_POS_HIGHER_BOUND % BIT_8;
        data[8] = CONST_POS_LOWER_BOUND / BIT_8;
        data[9] = CONST_POS_LOWER_BOUND % BIT_8;
        break;
    case CONFIG_VEL_LIMIT:
        data[6] = CONST_VEL_HIGHER_BOUND / BIT_8;
        data[7] = CONST_VEL_HIGHER_BOUND % BIT_8;
        data[8] = CONST_VEL_LOWER_BOUND / BIT_8;
        data[9] = CONST_VEL_LOWER_BOUND % BIT_8;
        break;
    case CONFIG_CUR_Q_LIMIT:
        data[6] = CONST_CUR_Q_HIGHER_BOUND / BIT_8;
        data[7] = CONST_CUR_Q_HIGHER_BOUND % BIT_8;
        data[8] = CONST_CUR_Q_LOWER_BOUND / BIT_8;
        data[9] = CONST_CUR_Q_LOWER_BOUND % BIT_8;
        break;
    case CONFIG_CUR_D_LIMIT:
        data[6] = CONST_CUR_D_HIGHER_BOUND / BIT_8;
        data[7] = CONST_CUR_D_HIGHER_BOUND % BIT_8;
        data[8] = CONST_CUR_D_LOWER_BOUND / BIT_8;
        data[9] = CONST_CUR_D_LOWER_BOUND % BIT_8;
        break;
    case CONFIG_MOTOR_CONST:
        data[6] = CONST_POLAR_PAIRS / BIT_8;
        data[7] = CONST_POLAR_PAIRS % BIT_8;
        data[8] = CONST_SUPPLY_VOLTAGE / BIT_8;
        data[9] = CONST_SUPPLY_VOLTAGE % BIT_8;
        break;
    case CONFIG_ENC_CONST:
        data[6] = CONST_ENC_RESOLUTION / BIT_8;
        data[7] = CONST_ENC_RESOLUTION % BIT_8;
        data[8] = CONST_ENC_OFFSET / BIT_8;
        data[9] = CONST_ENC_OFFSET % BIT_8;
        data[10] = CONST_ELEC_ANG_OFFSET / BIT_8;
        data[11] = CONST_ELEC_ANG_OFFSET % BIT_8;
        break;
    case CONFIG_ADC_CONST:
        data[6] = CONST_ADC_RESOLUTION / BIT_8;
        data[7] = CONST_ADC_RESOLUTION % BIT_8;
        data[8] = CONST_ADC_OFFSET_0 / BIT_8;
        data[9] = CONST_ADC_OFFSET_0 % BIT_8;
        data[10] = CONST_ADC_OFFSET_1 / BIT_8;
        data[11] = CONST_ADC_OFFSET_1 % BIT_8;
        break;
    case CONFIG_MECHANICAL_CONST:
        data[6] = CONST_DIRECT_CUR_POS / BIT_8;
        data[7] = CONST_DIRECT_CUR_POS % BIT_8;
        break;
    case CONFIG_CUR_SAMPLE_CONST:
        data[6] = CONST_SAMPLE_RESISTANCE / BIT_8;
        data[7] = CONST_SAMPLE_RESISTANCE % BIT_8;
        data[8] = CONST_SAMPLE_GAIN / BIT_8;
        data[9] = CONST_SAMPLE_GAIN % BIT_8;
        data[10] = CONST_MCU_VOLTAGE / BIT_8;
        data[11] = CONST_MCU_VOLTAGE % BIT_8;
        break;
    }
    data[14] = DATA_PENULT;
    data[15] = DATA_TAIL;
    return data;
}

void write_config(int configType, int configValue[])
{
    switch (configType)
    {
    case CONFIG_POS_LOOP_PARA_STD:
        // reserve.
        break;
    case CONFIG_VEL_LOOP_PARA_STD:
        // reserve.
        break;
    case CONFIG_CUR_Q_LOOP_PARA_STD:
        // reserve.
        break;
    case CONFIG_CUR_D_LOOP_PARA_STD:
        // reserve.
        break;

    case CONFIG_POS_LOOP_PARA_PER:
        pid_pos_per->Kp = configValue[0];
        pid_pos_per->Ki = configValue[1];
        pid_pos_per->Kd = configValue[2];
        break;
    case CONFIG_VEL_LOOP_PARA_PER:
        pid_vel_per->Kp = configValue[0];
        pid_vel_per->Ki = configValue[1];
        pid_vel_per->Kd = configValue[2];
        break;
    case CONFIG_CUR_Q_LOOP_PARA_PER:
        pid_cur_q_per->Kp = configValue[0];
        pid_cur_q_per->Ki = configValue[1];
        pid_cur_q_per->Kd = configValue[2];
        break;
    case CONFIG_CUR_D_LOOP_PARA_PER:
        pid_cur_d_per->Kp = configValue[0];
        pid_cur_d_per->Ki = configValue[1];
        pid_cur_d_per->Kd = configValue[2];
        break;
    case CONFIG_POS_LIMIT:
        CONST_POS_HIGHER_BOUND = configValue[0];
        CONST_POS_LOWER_BOUND = configValue[1];
        break;
    case CONFIG_VEL_LIMIT:
        CONST_VEL_HIGHER_BOUND = configValue[0];
        CONST_VEL_LOWER_BOUND = configValue[1];
        break;
    case CONFIG_CUR_Q_LIMIT:
        CONST_CUR_Q_HIGHER_BOUND = configValue[0];
        CONST_CUR_Q_LOWER_BOUND = configValue[1];
        break;
    case CONFIG_CUR_D_LIMIT:
        CONST_CUR_D_HIGHER_BOUND = configValue[0];
        CONST_CUR_D_LOWER_BOUND = configValue[1];
        break;
    case CONFIG_MOTOR_CONST:
        CONST_POLAR_PAIRS = configValue[0];
        CONST_SUPPLY_VOLTAGE = configValue[1];
        break;
    case CONFIG_ENC_CONST:
        CONST_ENC_RESOLUTION = configValue[0];
        CONST_ENC_OFFSET = configValue[1];
        CONST_ELEC_ANG_OFFSET = configValue[2];
        break;
    case CONFIG_ADC_CONST:
        CONST_ADC_RESOLUTION = configValue[0];
        CONST_ADC_OFFSET_0 = configValue[1];
        CONST_ADC_OFFSET_1 = configValue[2];
        break;
    case CONFIG_MECHANICAL_CONST:
        CONST_DIRECT_CUR_POS = configValue[0];
        break;
    case CONFIG_CUR_SAMPLE_CONST:
        CONST_SAMPLE_RESISTANCE = configValue[0];
        CONST_SAMPLE_GAIN = configValue[1];
        CONST_MCU_VOLTAGE = configValue[2];
        break;
    }
}

void set_pos()
{
    limitedPos = _get_lim(targetPos, CONST_POS_LOWER_BOUND, CONST_POS_HIGHER_BOUND);
    demandVel = (int)get_pid_pos();
    limitedVel = _get_lim(demandVel, CONST_VEL_LOWER_BOUND, CONST_VEL_HIGHER_BOUND);
    demandCurQ = (int)get_pid_vel();
    limitedCurQ = _get_lim(demandCurQ, CONST_CUR_Q_LOWER_BOUND, CONST_CUR_Q_HIGHER_BOUND);
    svpwm->Uq = (int)get_pid_cur_q();
    svpwm->Ud = (int)get_pid_cur_d();
    // 这里要加限幅吗？
    set_svpwm(svpwm->Uq, svpwm->Ud, elecAng);
}

void set_vel()
{
    limitedVel = _get_lim(targetVel, CONST_VEL_LOWER_BOUND, CONST_VEL_HIGHER_BOUND);
    demandCurQ = (int)get_pid_vel();
    limitedCurQ = _get_lim(demandCurQ, CONST_CUR_Q_LOWER_BOUND, CONST_CUR_Q_HIGHER_BOUND);
    svpwm->Uq = (int)get_pid_cur_q();
    svpwm->Ud = (int)get_pid_cur_d();
    set_svpwm(svpwm->Uq, svpwm->Ud, elecAng);
}

void set_cur_q()
{
    limitedCurQ = _get_lim(targetCurQ, CONST_CUR_Q_LOWER_BOUND, CONST_CUR_Q_HIGHER_BOUND);
    svpwm->Uq = (int)get_pid_cur_q();
    svpwm->Ud = (int)get_pid_cur_d();
    set_svpwm(svpwm->Uq, svpwm->Ud, elecAng);
}

double get_pid_pos()
{
    double demandValue;
    int thisErr = targetPos - actualPos;
    static int integralErr, lastErr;

    integralErr += (int)((thisErr + lastErr) / 2);
    // 积分限幅
    integralErr = _get_lim(integralErr, CONST_VEL_LOWER_BOUND, CONST_VEL_HIGHER_BOUND);

    demandValue = (int)(pid_pos->Kp * thisErr + pid_pos->Ki * integralErr * tsPos +
                        pid_pos->Kd * (thisErr - lastErr) / tsPos);
    lastErr = thisErr;
    return demandValue;
}

double get_pid_vel()
{
    double demandValue;
    int thisErr = targetVel - actualVel;
    static int integralErr, lastErr;

    integralErr += (int)((thisErr + lastErr) / 2);
    // 积分限幅
    integralErr = _get_lim(integralErr, CONST_CUR_Q_LOWER_BOUND, CONST_CUR_Q_HIGHER_BOUND);

    demandValue = (int)(pid_vel->Kp * thisErr + pid_vel->Ki * integralErr * tsVel +
                        pid_vel->Kd * (thisErr - lastErr) / tsVel);
    lastErr = thisErr;
    return demandValue;
}

double get_pid_cur_q()
{
    double demandValue;
    int thisErr = targetCurQ - actualCurQ;
    static int integralErr, lastErr;

    integralErr += (int)((thisErr + lastErr) / 2);
    // 积分限幅
    // integralErr = _get_lim(integralErr, lbCurQ, hbCurQ);

    demandValue = (int)(pid_cur_q->Kp * thisErr + pid_cur_q->Ki * integralErr * tsCur +
                        pid_cur_q->Kd * (thisErr - lastErr) / tsCur);
    lastErr = thisErr;
    return demandValue;
}

double get_pid_cur_d()
{
    double demandValue;
    int thisErr = -actualCurD;
    static int integralErr, lastErr;

    integralErr += (int)((thisErr + lastErr) / 2);
    // 积分限幅
    // integralErr = _get_lim(integralErr, lbCurD, hbCurD);

    demandValue = (int)(pid_cur_d->Kp * thisErr + pid_cur_d->Ki * integralErr * tsCur +
                        pid_cur_d->Kd * (thisErr - lastErr) / tsCur);
    lastErr = thisErr;
    return demandValue;
}

void set_svpwm()
{
    
}
