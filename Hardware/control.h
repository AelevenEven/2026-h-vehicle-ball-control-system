/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29

 
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/
#ifndef __CONTROL_H
#define __CONTROL_H
#include "board.h"

extern int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
#define EncoderMultiples  2				//编码器倍频数，取决于编码器初始化设置
#define CONTROL_FREQUENCY 100			//TIMER_0周期为10ms，即100Hz
#define	Black_WheelDiameter   0.065f	//轮胎直径
#define Perimeter	0.204203519			//轮子周长(单位:m)
#define MOTOR_GEAR_RATIO       30.0f    // 电机减速比
#define ENCODER_RESOLUTION     13.0f    // 编码器线数
#define Wheelspacing 0.1610f		//主动轮轴距(单位:m)
#define PI 3.1415926
/*

 *
 * ENCODER_A_SIGN / ENCODER_B_SIGN 是左右编码器方向修正系数：
 * 小车向前推时，对应轮的累计距离应增加；若某一路反而减少，就把该路的符号翻转。

 */
 /* 编码器方向参数 */
#define ENCODER_A_SIGN  -1.0f
#define ENCODER_B_SIGN 1.0f

//电机速度控制相关参数结构体
typedef struct  
{
	float Current_Encoder;     	//编码器数值，读取电机实时速度
	float Motor_Pwm;     		//电机PWM数值，控制电机实时速度
	float Target_Encoder;  		//电机目标编码器速度值，控制电机目标速度
	float Velocity; 	 		//电机速度值
}Motor_parameter;

//编码器结构体
typedef struct  
{
  int A;      
  int B;  
}Encoder;
extern float Move_X,Move_Z;						//目标速度和目标转向速度
extern Encoder OriginalEncoder; 					//编码器原始数据   
extern Motor_parameter MotorA,MotorB;				//左右电机相关变量
extern float Voltage_Count,Voltage_All,Voltage;
/* 左右轮独立速度 PI 参数，单位与 control.c 中的增量式 PI 一致。 */
extern float Velocity_KP_Left, Velocity_KI_Left;
extern float Velocity_KP_Right, Velocity_KI_Right;
extern volatile int MotorA_Runtime_Pwm_Bias;
extern volatile int MotorB_Runtime_Pwm_Bias;
extern int Run_Mode;//小车运行模式
extern volatile int Flag_Stop;
/* 累计左右轮里程，供无标线路段按距离结束，并允许每段路线重新清零。 */
extern volatile float EncoderA_Distance;
extern volatile float EncoderB_Distance;
/* 左右轮实测/目标速度，lap_test 安全监测使用 (mm/s) */
extern float left_speed_meas;
extern float right_speed_meas;
extern float left_speed_ref;
extern float right_speed_ref;
void Control_Init(void);
void Control_ResetSpeedControllers(void);
void TIM6_Init(void);
void Get_Velocity_From_Encoder(int Encoder1,int Encoder2);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
void Get_Target_Encoder(float Vx,float Vz);
int Incremental_PI_Left (float Encoder,float Target);
int Incremental_PI_Right (float Encoder,float Target);
void Get_Motor_PWM(void);
void Set_Pwm(int motor_a,int motor_b);
int Turn_Off(void);
int myabs(int a);
void Get_RC(void);
void Key(void);
void Control_ResetOdometry(void);
float Control_GetAverageDistance(void);
#endif
