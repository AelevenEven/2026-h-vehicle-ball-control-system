/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29

***********************************************/
#include "control.h"

u8 CCD_count,ELE_count;
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;

Encoder OriginalEncoder; 					//编码器原始数据   
Motor_parameter MotorA,MotorB;				//左右电机相关变量
/*
 * PID 调试起始值。
 * 当前速度误差的单位是 m/s，若 Kp=1、Ki=0，0.2 m/s 的误差只产生 0.2 PWM，
 * 转成电机整数 PWM 后为 0，电机不会起转。先恢复为可起转的起始值，再实车调节。
 */
/*
 * 左右电机分别使用独立的增量式 PI 参数。
 *
 * 调参时只修改对应一侧：
 *   MotorA（左轮）：Velocity_KP_Left、Velocity_KI_Left
 *   MotorB（右轮）：Velocity_KP_Right、Velocity_KI_Right
 *
 * 原来的共用参数保留在注释中，便于对照，不再参与计算。
 */
float Velocity_KP_Left  = 500.0f;
float Velocity_KI_Left  = 700.0f;
float Velocity_KP_Right = 800.0f;
float Velocity_KI_Right = 1600.0f;  /* 右侧重物，右转极度需要扭矩 */
// float Velocity_KP = 395.0f, Velocity_KI = 380.0f;
// float Velocity_KP = 400.0f, Velocity_KI = 390.0f;

/*
 * 电机静态 PWM 补偿，默认关闭。
 * 只有当两侧“实际速度已经接近”，但仍希望补偿电机/轮胎的固定差异时才使用。
 * 正值表示增加该侧 PWM；调试时建议一次只改 20~50。
 */
#define MOTOR_A_PWM_BIAS  0
#define MOTOR_B_PWM_BIAS  1000  /* 右侧步进电机加重 */
#define MOTOR_PWM_LIMIT   7800

/*
 * 路线模块运行时补偿：只在无标线直行阶段由 route.c 赋值，
 * 巡线、找线和转向时恢复为 0，避免改变红外巡线的左右差速。
 */
volatile int MotorA_Runtime_Pwm_Bias;
volatile int MotorB_Runtime_Pwm_Bias;

static int apply_pwm_bias(int pwm, int bias)
{
	if (pwm > 0) {
		pwm += bias;
	} else if (pwm < 0) {
		pwm -= bias;
	}

	if (pwm > MOTOR_PWM_LIMIT) pwm = MOTOR_PWM_LIMIT;
	if (pwm < -MOTOR_PWM_LIMIT) pwm = -MOTOR_PWM_LIMIT;
	return pwm;
}
/* 上电默认选择最简单的 A->B 模式，先便于检查循迹、编码器和电机方向。 */
int Run_Mode=0;//默认选择第(1)项：A到B
volatile int Flag_Stop=1;//小车启动标志位
volatile float EncoderA_Distance;
volatile float EncoderB_Distance;

/* 左右轮实测/目标速度，lap_test 安全监测使用 (mm/s) */
float left_speed_meas;
float right_speed_meas;
float left_speed_ref;
float right_speed_ref;

/* 增量式 PI 控制器静态变量移至文件作用域，便于 Control_ResetSpeedControllers 复位 */
static float PI_Left_Bias, PI_Left_Pwm, PI_Left_Last_bias;
static float PI_Right_Bias, PI_Right_Pwm, PI_Right_Last_bias;

void TIMER_0_INST_IRQHandler(void)
{
	/*
	 * 每 10 ms 原子地取走编码器增量，然后完成按键、速度换算和双轮 PI 控制。
	 */
	if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
			int32_t count_a = Get_Encoder_countA;
			int32_t count_b = Get_Encoder_countB;
			Get_Encoder_countA = 0;
			Get_Encoder_countB = 0;

			Key();
			/* PID 调试期间关闭状态灯闪烁，只保留按键、编码器、速度环和 PWM。 */
//			LED_Flash(100);
			/* 编码器端口对调：A口→右编码器, B口→左编码器，交换配对 */
			Get_Velocity_From_Encoder(count_a, count_b);

			/* 更新 lap_test 安全监测使用的速度变量 (mm/s) */
			left_speed_meas  = MotorA.Current_Encoder * 1000.0f;
			right_speed_meas = MotorB.Current_Encoder * 1000.0f;
			left_speed_ref   = MotorA.Target_Encoder * 1000.0f;
			right_speed_ref  = MotorB.Target_Encoder * 1000.0f;

//			//计算左右电机对应的PWM
			MotorA.Motor_Pwm = apply_pwm_bias(
				Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder),
				MOTOR_A_PWM_BIAS + MotorA_Runtime_Pwm_Bias);
			MotorB.Motor_Pwm = apply_pwm_bias(
				Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder),
				MOTOR_B_PWM_BIAS + MotorB_Runtime_Pwm_Bias);
			if(!Flag_Stop)
			{
				Set_PWM(MotorA.Motor_Pwm,MotorB.Motor_Pwm);
			}else Set_PWM(0,0);
	}
}

/**************************************************************************
Function: Get_Velocity_From_Encoder
Input   : none
Output  : none
函数功能：读取编码器和转换成速度
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Get_Velocity_From_Encoder(int Encoder1,int Encoder2)
{
	/* 将“每控制周期脉冲数”统一换算为 m/s，并同时积分成左右轮里程。 */
	const float meters_per_count = Perimeter /
		(EncoderMultiples * ENCODER_RESOLUTION * MOTOR_GEAR_RATIO);

	OriginalEncoder.A = Encoder1;
	OriginalEncoder.B = Encoder2;
	MotorA.Current_Encoder = ENCODER_A_SIGN * Encoder1 *
		CONTROL_FREQUENCY * meters_per_count;
	MotorB.Current_Encoder = ENCODER_B_SIGN * Encoder2 *
		CONTROL_FREQUENCY * meters_per_count;
	EncoderA_Distance += ENCODER_A_SIGN * Encoder1 * meters_per_count;
	EncoderB_Distance += ENCODER_B_SIGN * Encoder2 * meters_per_count;
}

void Control_ResetOdometry(void)
{
	/* 每次开始新路线时从零计算路段距离，避免沿用上一次运行的累计值。 */
	EncoderA_Distance = 0.0f;
	EncoderB_Distance = 0.0f;
}

float Control_GetAverageDistance(void)
{
	float left = EncoderA_Distance;
	float right = EncoderB_Distance;
	if (left < 0.0f) left = -left;
	if (right < 0.0f) right = -right;
	return 0.5f * (left + right);
}
//运动学逆解，由x和y的速度得到编码器的速度,Vx是m/s,Vz单位是度/s(角度制)
void Get_Target_Encoder(float Vx,float Vz)
{
	/* 修改原因：运动学公式需要 rad/s，调用接口保留较直观的 deg/s，因此在此转换。 */
	float angular_velocity = Vz * (PI / 180.0f);
	if(Vx<0) angular_velocity=-angular_velocity;
	//Inverse kinematics //运动学逆解
	 MotorA.Target_Encoder = Vx - angular_velocity * Wheelspacing / 2.0f;
	 MotorB.Target_Encoder = Vx + angular_velocity * Wheelspacing / 2.0f;
}


/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：需要计算绝对值的数
返回  值：无符号整型
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0)  temp=-a;
	else temp=a;
	return temp;
}

int Turn_Off(void)
{
	u8 temp = 0;
//	if(Voltage>700&&EN==0)//电压高于7V且使能开关打开
//	{
//		temp = 1;
//	}
	return temp;			
}
/**************************************************************************
Function: PWM_Limit
Input   : IN;max;min
Output  : OUT
函数功能：限制PWM赋值
入口参数: IN：输入参数  max：限幅最大值  min：限幅最小值 
返回  值：限幅后的值
**************************************************************************/	 	
float PWM_Limit(float IN,float max,float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_Left (float Encoder,float Target)
{
	 PI_Left_Bias=Target-Encoder;                			//计算偏差
	 PI_Left_Pwm+=Velocity_KP_Left*(PI_Left_Bias-PI_Left_Last_bias)+Velocity_KI_Left*PI_Left_Bias;
	 /* 左轮独立 PI：只使用左轮的 Kp、Ki，避免被右轮调参相互影响。 */
	if(Flag_Stop) PI_Left_Pwm=0;
	 if(PI_Left_Pwm>7800)PI_Left_Pwm=7800;
	 if(PI_Left_Pwm<-7800)PI_Left_Pwm=-7800;
	 PI_Left_Last_bias=PI_Left_Bias;	               		//保存上一次偏差
	 return PI_Left_Pwm;                         			//增量输出
}


int Incremental_PI_Right (float Encoder,float Target)
{
	 PI_Right_Bias=Target-Encoder;                			//计算偏差
	 PI_Right_Pwm+=Velocity_KP_Right*(PI_Right_Bias-PI_Right_Last_bias)+Velocity_KI_Right*PI_Right_Bias;
	 /* 右轮独立 PI：只使用右轮的 Kp、Ki，便于单独调右轮响应。 */
	if(Flag_Stop) PI_Right_Pwm=0;
	 if(PI_Right_Pwm>7800)PI_Right_Pwm=7800;
	 if(PI_Right_Pwm<-7800)PI_Right_Pwm=-7800;
	 PI_Right_Last_bias=PI_Right_Bias;	               		//保存上一次偏差
	 return PI_Right_Pwm;                         			//增量输出
}

/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
函数功能：按键修改小车运行状态
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
	u8 tmp;
	tmp=key_scan(CONTROL_FREQUENCY);
	/*
	 * 单击和长按都用于启停。
	 * 原程序忽略长按事件，按住超过约 0.5 秒时不会改变 Flag_Stop。
	 */
	if((tmp==USEKEY_single_click) || (tmp==USEKEY_long_click))
	{
		Flag_Stop=!Flag_Stop;
	}		//单击控制小车的启停
	else if(tmp==USEKEY_double_click)
	{
		/* 仅在停车时允许双击切换路线，防止行驶中途突然更改状态机。 */
		if (Flag_Stop) {
			Run_Mode = (Run_Mode + 1) % 4;
		}
	}
}

/**************************************************************************
Function: Control_Init
Input   : none
Output  : none
函数功能：控制模块初始化
入口参数：无
返回  值：无
**************************************************************************/
void Control_Init(void)
{
	Flag_Stop = 1;
	MotorA.Target_Encoder = 0.0f;
	MotorB.Target_Encoder = 0.0f;
	MotorA.Motor_Pwm = 0.0f;
	MotorB.Motor_Pwm = 0.0f;
	MotorA_Runtime_Pwm_Bias = 0;
	MotorB_Runtime_Pwm_Bias = 0;
	EncoderA_Distance = 0.0f;
	EncoderB_Distance = 0.0f;
	left_speed_meas  = 0.0f;
	right_speed_meas = 0.0f;
	left_speed_ref   = 0.0f;
	right_speed_ref  = 0.0f;
	Control_ResetSpeedControllers();
}

/**************************************************************************
Function: Control_ResetSpeedControllers
Input   : none
Output  : none
函数功能：复位左右轮增量式 PI 控制器历史状态
入口参数：无
返回  值：无
**************************************************************************/
void Control_ResetSpeedControllers(void)
{
	PI_Left_Pwm       = 0.0f;
	PI_Left_Last_bias = 0.0f;
	PI_Left_Bias      = 0.0f;
	PI_Right_Pwm      = 0.0f;
	PI_Right_Last_bias = 0.0f;
	PI_Right_Bias     = 0.0f;
	MotorA.Motor_Pwm  = 0.0f;
	MotorB.Motor_Pwm  = 0.0f;
}
