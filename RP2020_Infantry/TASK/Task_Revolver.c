/**
 * @file        Task_Revolver.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        10-October-2019
 * @brief       This file includes the Revolver(拨盘电机) external functions 
 */
 
/**
 *	# m2006 P36 拨盘电机
 *	反馈的最大转速大概在20800左右
 */
 
/* Includes ------------------------------------------------------------------*/
#include "Task_Revolver.h"

#include "can.h"
#include "my_app.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define REVOLVER_SPEED_PID_IOUT_MAX		250
#define REVOLVER_SPEED_PID_OUT_MAX		9999
#define REVOLVER_ANGLE_PID_IOUT_MAX		2000
#define REVOLVER_ANGLE_PID_OUT_MAX		6000

#define REVOLVER_STUCK_PID_OUT			4000
#define REVOLVER_STUCK_SPEED			60
#define REVOLVER_STUCK_TIME				100
#define REVOLVER_TURNBACK_TIME			100

#define	TRIPLE_SHOOT_PRESSED_TIME		TIME_STAMP_250MS

#define LEVEL_COUNT			3
#define LEVEL_1				0
#define LEVEL_2				1
#define LEVEL_3				2


/* ## Global variables ## ----------------------------------------------------*/
Revolver_PID_t Revolver_PID = {
	/* 速度环 */
	.Speed.kp = 9.2,		
	.Speed.ki = 0,
	.Speed.kd = 0,	
	.Speed.target = 0,
	.Speed.feedback = 0,
	.Speed.erro = 0,
	.Speed.last_erro = 0,
	.Speed.integrate = 0,
	.Speed.pout = 0,
	.Speed.iout = 0,
	.Speed.dout = 0,
	.Speed.out = 0,
	/* 位置环 */
	.Angle.kp = 0.26,		
	.Angle.ki = 0,
	.Angle.kd = 0,
	.Angle.target = 0,	
	.Angle.feedback = 0,
	.Angle.erro = 0,
	.Angle.last_erro = 0,
	.Angle.integrate = 0,
	.Angle.pout = 0,
	.Angle.iout = 0,
	.Angle.dout = 0,
	.Angle.out = 0,		
};

/* # Revolver # */
Revolver_Info_t Revolver = {
	.RemoteMode = RC,
	
	.State = REVO_STATE_OFF,
	.PidMode = REVO_POSI_MODE,
	.Action = SHOOT_NORMAL,
	
	.Shoot.freq = 8,	// 默认射频8
	.Shoot.num = 0,	// 允许射弹量
	.Shoot.heat_cooling_rate = 0,	// 17mm枪管热量冷却值
	.Shoot.heat_real = 0, // 17mm枪管初始热量
	.Shoot.heat_limit = 240,	// 默认枪口热量上限为240
	.Shoot.stuck_count = 0,// 卡弹计数
	
	.Buff.lost_time = 0,
	.Buff.lost_time_boundary = TIME_STAMP_70MS,
	.Buff.lock_time = 0,
	.Buff.lock_time_boundary = TIME_STAMP_80MS,
	.Buff.change_armor_delay = 0,
	.Buff.change_armor_delay_boundary = TIME_STAMP_50MS,
	.Buff.respond_interval = TIME_STAMP_800MS,	// TIME_STAMP_800MS 调试的时候需要
	.Buff.change_armor = false,
	.Buff.fire = false,
};

/* 枪口热量上限 */
uint16_t REVO_HEAT_LIMIT[LEVEL_COUNT] = {180, 240, 300};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	拨盘电机PID参数初始化
 */
void REVOLVER_PidParamsInit(Revolver_PID_t *pid)
{
	/* 速度环参数重置 */
	pid->Speed.target = 0;
	pid->Speed.feedback = 0,
	pid->Speed.erro = 0;
	pid->Speed.last_erro = 0;
	pid->Speed.integrate = 0;
	pid->Speed.pout = 0;
	pid->Speed.dout = 0;
	pid->Speed.iout = 0;
	pid->Speed.out = 0;

	/* 位置环参数重置 */
	pid->Angle.target = 0;
	pid->Angle.feedback = 0,
	pid->Angle.erro = 0;
	pid->Angle.last_erro = 0;
	pid->Angle.integrate = 0;
	pid->Angle.pout = 0;
	pid->Angle.dout = 0;
	pid->Angle.iout = 0;
	pid->Angle.out = 0;
	
	pid->AngleRampBuffer = 0;
	pid->Out = 0;
}

/**
 *	@brief	拨盘电机停止输出
 */
void REVOLVER_Stop(Revolver_PID_t *pid)
{
	int16_t pidOut[4] = {0, 0, 0, 0};
	
	/* 内环速度环最终输出 */
	Revolver_PID.Speed.out = 0;
	Revolver_PID.Out = 0;
	
#if REVOLVER_ID > 0x204	
	CAN2_Send(0x1FF, pidOut);
#else
	CAN2_Send(0x200, pidOut);
#endif
}

/**
 *	@brief	拨盘电机速度环
 */
void REVOLVER_Speed_PidCalc(Revolver_PID_t *pid)
{
	pid->Speed.erro = pid->Speed.target - pid->Speed.feedback;
	pid->Speed.integrate += pid->Speed.erro;
	/* # 调试操作 */
	pid->Speed.integrate = constrain(pid->Speed.integrate, -REVOLVER_SPEED_PID_IOUT_MAX, REVOLVER_SPEED_PID_IOUT_MAX);
		
	/* Pout */
	pid->Speed.pout = pid->Speed.kp * pid->Speed.erro;
	/* Iout */
	pid->Speed.iout = pid->Speed.ki * pid->Speed.integrate * 0.002f; // 模仿19步兵代码(0.002f是2ms的意思)
	/* Iout Limits */
	pid->Speed.iout = constrain(pid->Speed.iout, -REVOLVER_SPEED_PID_IOUT_MAX, REVOLVER_SPEED_PID_IOUT_MAX);
	/* Dout*/
	pid->Speed.dout = pid->Speed.kd * (pid->Speed.erro - pid->Speed.last_erro)/0.002f;
	/* Record Last Error */
	pid->Speed.last_erro = pid->Speed.erro;		
	
	/* Total PID Output*/
	pid->Speed.out = pid->Speed.pout + pid->Speed.iout + pid->Speed.dout;
	/* Total PID Output Limits */
	pid->Speed.out = constrain(pid->Speed.out, -REVOLVER_SPEED_PID_OUT_MAX, REVOLVER_SPEED_PID_OUT_MAX);
	/* 内环速度环最终输出 */
	pid->Out = pid->Speed.out;
}

/**
 *	@brief	拨盘电机位置环
 */
void REVOLVER_Angle_PidCalc(Revolver_PID_t *pid)
{	
	pid->Angle.erro = pid->Angle.target - pid->Angle.feedback;
	/* 对误差进行卡尔曼滤波，消除低频地幅抖动 */
	//pid->Angle.erro = KalmanFilter(&Gimbal_kalmanError[Flag.Gimbal.FLAG_pidMode][MOTORx], pid->Angle.erro);
	pid->Angle.integrate += pid->Angle.erro;

	/* Pout */
	pid->Angle.pout = pid->Angle.kp * pid->Angle.erro;
	/* Iout */
	pid->Angle.iout = pid->Angle.ki * pid->Angle.integrate * 0.002f; // 模仿19步兵代码(0.002f是2ms的意思)
	/* Iout Limits */
	//pid->Angle.iout = constrain(pid->Angle.iout, -REVOLVER_ANGLE_PID_IOUT_MAX, REVOLVER_ANGLE_PID_IOUT_MAX);
	/* Dout*/
	pid->Angle.dout = pid->Angle.kd * (pid->Angle.erro - pid->Angle.last_erro)/0.002f;
	/* Record Last Error */
	pid->Angle.last_erro = pid->Angle.erro;

	/* Total PID Output*/
	pid->Angle.out = pid->Angle.pout + pid->Angle.iout + pid->Angle.dout;
	/* Total PID Output Limits */
	//pid->Angle.out = constrain(pid->Angle.out, -REVOLVER_ANGLE_PID_OUT_MAX, REVOLVER_ANGLE_PID_OUT_MAX);
}

/**
 *	@brief	拨盘电机PID的最终输出
 *	@note		# 电调ID号更换的时候需要修改标识符和发送的位置(代码实现)
 *					逆时针旋转机械角度会增大
 *	@direction
 *					逆时针 -> 机械角度↑,转速+
 *					顺时针 -> 机械角度↓,转速-
 */
void REVOLVER_PidOut(Revolver_PID_t *pid)
{
	int16_t pidOut[4] = {0, 0, 0, 0};
	uint8_t pidSn = ((REVOLVER_ID - 0x201) % 4);	// ((0x207-0x201)%4)
	
	/* CAN发送电流值 */
	if(BitMask.Revolver.BM_rxReport & REVOLVER_BM_RX_REPORT) {
		pidOut[pidSn] = (int16_t)pid->Out;
	} else {
		pidOut[pidSn] = 0;	// 失联后拨盘卸力
	}
	
#if REVOLVER_ID > 0x204
	CAN2_QueueSend(0x1FF, pidOut);
#else
	CAN2_QueueSend(0x200, pidOut);
#endif	
}

/**
 *	@brief	速度环卡弹处理方式(计时判断+反转尝试)
 */
void REVOLVER_Speed_BulletStuck(Revolver_PID_t *pid, Revolver_Info_t *revo)
{
	static uint16_t stuck_time = 0;
	static uint16_t turnback_time = 0;
	static uint8_t	stuck_recover_flag = 0;
	
	if(stuck_recover_flag == false) {	// 没有启动卡弹恢复尝试
		if(pid->Speed.out > REVOLVER_STUCK_PID_OUT
		   && pid->Speed.feedback < REVOLVER_STUCK_SPEED) {	// PID输出过大而速度反馈较小则认为卡弹
			stuck_time++;	
		} else {
			stuck_time = 0;
		}
		if(stuck_time > REVOLVER_STUCK_TIME) {	// 连续卡弹超过60ms
			stuck_time = 0;
			revo->Shoot.stuck_count++;
			stuck_recover_flag = true;
			revo->Shoot.num = 0;	// 屏蔽打弹指令
		}
	} else {	// 卡弹恢复
		pid->Speed.target = -4000;
		turnback_time++;
		if(turnback_time > REVOLVER_TURNBACK_TIME) {	// 倒转完成
			turnback_time = 0;
			stuck_recover_flag = false;
		}
	}
}

/**
 *	@brief	位置环卡弹处理方式(计时判断+反转尝试)
 */
void REVOLVER_Angle_BulletStuck(Revolver_PID_t *pid, Revolver_Info_t *revo)
{
	static uint16_t stuck_time = 0;
	static uint16_t turnback_time = 0;
	static uint8_t	stuck_recover_flag = 0;
	
	if(stuck_recover_flag == false) {	// 没有启动卡弹恢复尝试
		if(pid->Speed.out > REVOLVER_STUCK_PID_OUT
		   && pid->Speed.feedback < REVOLVER_STUCK_SPEED) {	// PID输出过大而速度反馈较小则认为卡弹
			stuck_time++;	
		} else {
			stuck_time = 0;
		}
		if(stuck_time > REVOLVER_STUCK_TIME) {	// 连续卡弹超过60ms
			stuck_time = 0;
			revo->Shoot.stuck_count++;
			stuck_recover_flag = true;
			revo->Shoot.num = 0;	// 屏蔽打弹指令
			//pid->Angle.target = ANGLE_AN_BULLET;	// 反转1格
			pid->AngleRampBuffer = pid->Angle.target - ANGLE_AN_BULLET;
		}
	} else {	// 卡弹恢复
		turnback_time++;
		if(turnback_time > REVOLVER_TURNBACK_TIME) {	// 倒转完成
			pid->Angle.target = pid->Angle.feedback;
			pid->AngleRampBuffer = pid->Angle.target;
			turnback_time = 0;
			stuck_recover_flag = false;
		}
	}
}

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief 设置拨盘的控制行为
 */
void REVOLVER_SetAction(Revolver_Action_t action)
{
	Revolver.Action = action;
}

/**
 *	@brief	发弹量+1
 */
void REVOLVER_AddShootCount(void)
{
	Revolver.Shoot.total_count++;
}

/**
 *	@brief	返回实时射击时间
 */
uint32_t REVOLVER_GetRealShootTime(void)
{
	return Revolver.Shoot.real_time;
}

/**
 *	@brief	拨盘获取系统信息
 */
void REVOLVER_GetSysInfo(System_t *sys, Revolver_Info_t *revo)
{
	/*----控制方式----*/
	/* 控制方式 - 遥控器 */
	if(sys->RemoteMode == RC) {
		revo->RemoteMode = RC;
	} 
	/* 控制方式 - 键鼠 */
	else if(sys->RemoteMode == KEY) {
		revo->RemoteMode = KEY;
	}

	/*----模式修改----*/
	switch(sys->Action)
	{
		case SYS_ACT_NORMAL: 
			{
				revo->Action = SHOOT_NORMAL;// 常规
			}break;
		case SYS_ACT_AUTO: 
			{
				revo->Action = SHOOT_AUTO;	// 自瞄
			}break;
		case SYS_ACT_BUFF:
			{
				revo->Action = SHOOT_BUFF;	// 常闭
			}break;
		case SYS_ACT_PARK:
			{
				revo->Action = SHOOT_BAN;// 常规
			}break;
	}	
}

/**
 *	@brief	拨盘读取裁判系统信息
 */
void REVOLVER_GetJudgeInfo(Judge_Info_t *judge, Revolver_Info_t *revo)
{
	if(judge->data_valid == true) {
		revo->Shoot.heat_real = JUDGE_usGetShooterRealHeat17();//judge->PowerHeatData.shooter_heat0;
		revo->Shoot.heat_limit = JUDGE_usGetShooterLimitHeat17();//judge->GameRobotStatus.shooter_heat0_cooling_limit;
		revo->Shoot.heat_cooling_rate = JUDGE_usGetShooterHeatCoolingRate17();//judge->GameRobotStatus.shooter_heat0_cooling_rate;	// 步兵<20%HP,冷却速率翻倍	
		
		//revo->Shoot.num_buffer = 
	} else {
		//..暂不处理
	}
	
	switch(judge->GameRobotStatus.robot_level)
	{
		// 1级步兵
		case 1:
			revo->Shoot.freq = 8;
			break;
		// 2级步兵
		case 2:
			revo->Shoot.freq = 10;
			break;
		// 3级步兵
		case 3:
			revo->Shoot.freq = 12;
			break;
		// 防止出bug
		default:
			revo->Shoot.freq = 8;
			break;
	}
}

/**
 *	@brief	拨盘获取遥控信息
 *	@note	防止出错
 */
static uint8_t prev_sw1 = RC_SW_DOWN;
static uint16_t MouseLockTime_L = 0;

void REVOLVER_GetRemoteInfo(System_t *sys, RC_Ctl_t *remote, Revolver_Info_t *revo)
{
	/* 系统正常 */
	if(sys->State == SYSTEM_STATE_NORMAL)
	{
		if(sys->RemoteMode == RC) {
			MouseLockTime_L = 0;
		}
		else if(sys->RemoteMode == KEY) {
			prev_sw1 = RC_SW1_VALUE;
		}
	}
	/* 系统异常 */
	else
	{
		// 复位成初始值
		prev_sw1 = RC_SW_DOWN;
		MouseLockTime_L = 0;
	}	
}

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #遥控# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	遥控设置拨盘电机转动(完成射击动作)
 *	@note	SW1_DOWN(前提是摩擦轮开启)
 */
void REMOTE_SetRevolverShoot(RC_Ctl_t *remote)
{
	uint8_t	sw1, sw2;
	sw1 = RC_SW1_VALUE;
	sw2 = RC_SW2_VALUE;
	
	if(FRICTION_IfReady()) {
		if(Revolver.PidMode == REVO_POSI_MODE) {	// 点射模式
			if(sw1 == RC_SW_DOWN && prev_sw1 != RC_SW_DOWN) 
			{
				Revolver.Shoot.freq = 8;
				Revolver.Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
			}
		} else if(Revolver.PidMode == REVO_SPEED_MODE) {	// 连射模式
			if(sw1 == RC_SW_DOWN) {
				Revolver.State = REVO_STATE_ON;
				Revolver.Shoot.freq = 12;
				Revolver_PID.Speed.target = SPEED_AN_BULLET_PER_SECOND * Revolver.Shoot.freq;	// 发/s
			} else {
				Revolver.State = REVO_STATE_OFF;
				Revolver_PID.Speed.target = 0;
			}
		}
	}
	prev_sw1 = sw1;
	
	if(sw2 == RC_SW_MID) {
		/* 速度环->位置环*/
		if(Revolver.PidMode == REVO_SPEED_MODE) {
			Revolver.PidMode = REVO_POSI_MODE;
			Revolver.State = REVO_STATE_OFF;
			Revolver_PID.Angle.target = Revolver_PID.Angle.feedback;	// 防止切换模式的时候动一下
			Revolver_PID.AngleRampBuffer = Revolver_PID.Angle.target;
		}
	} else if(sw2 == RC_SW_DOWN) {
		/* 位置环->速度环 */
		if(Revolver.PidMode == REVO_POSI_MODE) {
			Revolver.PidMode = REVO_SPEED_MODE;
			Revolver.State = REVO_STATE_OFF;
			Revolver_PID.Speed.target = 0;	// s1_down,s2_down -> s2_mid -> s1_mid -> s2_down => 拨盘会正转一小步(小于一格)
		}
	}
}

/* #键盘鼠标# -------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	按键设置拨盘电机转动(完成射击动作)
 *	@note	    左键：点射
 *			长按左键：三连发(相当于快速三次点射)
 *			
 *			# Loop Time:5ms
 *			timePressed最大正值为 +32767(32767*5 = 163.835s)
 */

/*
	常规		- 不发弹
	单击左键 - 点射
	长按左键 - 连射
	打符		- 自动发弹
	自瞄		- 手动发弹
*/

void KEY_SetRevolverShoot(RC_Ctl_t *remote)
{	
//	/* 摩擦轮开启的时候才响应拨盘 */
//	if(FRICTION_IfReady()) 
//	{		
//		/* 按下鼠标左键时累计时间 */
//		if(IF_MOUSE_PRESSED_LEFT)
//		{
//			if(MouseLockTime_L < TIME_STAMP_6000MS)	// 防止长时间按变量溢出
//					MouseLockTime_L++;
//		}
//		
//		if(IF_KEY_PRESSED_B && !IF_MOUSE_PRESSED_LEFT & !IF_KEY_PRESSED_B)
//		{
//			/* 推家模式 */
//			Revolver.PidMode = REVO_POSI_MODE;
//			Revolver.Action = SHOOT_HIGH_F_LOW_S;
//		}
//		else if(GIMBAL_IfBuffMode())
//		{
//			/* 打符模式 */
//			Revolver.PidMode = REVO_POSI_MODE;
//			Revolver.Action = SHOOT_BUFF;
//			/* 切换模式时清除左键响应时间 */
//			MouseLockTime_L = 0;
//		}
//		else if(GIMBAL_IfAutoMode())
//		{
//			/* 自瞄模式 */
//			Revolver.PidMode = REVO_POSI_MODE;
//			Revolver.Action = SHOOT_AUTO;
//			/* 切换模式时清除左键响应时间 */
//			MouseLockTime_L = 0;
//		}
//		else if(IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z) 
//		{
//			/* 三连发模式 */
//			if(MouseLockTime_L >= TRIPLE_SHOOT_PRESSED_TIME) 
//			{	// 首次启动需要长按左键250ms
//				//MouseLockTime_L = 0;
//				Revolver.PidMode = REVO_SPEED_MODE;	// 速度环
//				Revolver.Action = SHOOT_TRIPLE;		// 三连发
//				Revolver.State = REVO_STATE_ON;		// 开启拨盘旋转
//			} 
//		} 
//		else if((MouseLockTime_L > TIME_STAMP_10MS && MouseLockTime_L < TRIPLE_SHOOT_PRESSED_TIME)
//				&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z)
//		{
//			/* 点射模式 */
//			MouseLockTime_L = 0;
//			Revolver.PidMode = REVO_POSI_MODE;
//			Revolver.Action = SHOOT_SINGLE;
//		}
//		else 
//		{
//			/* 常规控制模式 */
//			/* 速度环切位置环 */
//			if(Revolver.PidMode == REVO_SPEED_MODE) {
//				Revolver_PID.Angle.target = Revolver_PID.Angle.feedback;	// 防止切换模式的时候动一下
//				Revolver_PID.AngleRampBuffer = Revolver_PID.Angle.target;
//			}
//			MouseLockTime_L = 0;
//			Revolver.PidMode = REVO_POSI_MODE;
//			Revolver.Action = SHOOT_NORMAL;
//			Revolver.State = REVO_STATE_OFF;		// 停止拨盘旋转			
//		}
//	}
//	/* 未开启摩擦轮屏蔽指令*/
//	else	
//	{
//		MouseLockTime_L = 0;
//		Revolver.PidMode = REVO_POSI_MODE;
//		Revolver.Action = SHOOT_NORMAL;		
//	}
}

/* #拨盘# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief 	结合当前热量情况计算最大发弹量(0<=allow_num<=target_num)
 *	@return allow_num( 0 ~ target_num)
 */
uint8_t REVOLVER_HeatLimit(Revolver_Info_t *revo, uint8_t target_num)
{
	uint16_t remain_heat;	// 剩余热量
	uint8_t  allow_num;		// 允许的发弹数量
	
	if(target_num == 0)	// 防止出错
		return 0;
	
	if(revo->Shoot.suicide_mode == true)	// 热量超限扣血换取射频
		return target_num;
	
	remain_heat = revo->Shoot.heat_limit - revo->Shoot.heat_real;
	allow_num = remain_heat / HEAT_AN_BULLET;	// 当前热量下允许的最大发弹量
	if(allow_num > 0) {
		while(target_num) {
			if(allow_num >= target_num) {
				allow_num = target_num;
				break;
			}
			target_num--;
		}
	}
	return allow_num;
}

/**
 *	@brief	拨盘电机常规控制
 *	@note	默认不打弹
 */
void REVOLVER_NormalCtrl(Revolver_Info_t *revo)
{
	/* 按下鼠标左键时累计时间 */
	if(IF_MOUSE_PRESSED_LEFT)
	{
		if(MouseLockTime_L < TIME_STAMP_6000MS)	// 防止长时间按变量溢出
			MouseLockTime_L++;
	}

	/* 短按 */
	if( !IF_MOUSE_PRESSED_LEFT
			&& (MouseLockTime_L > TIME_STAMP_10MS)
			&& (MouseLockTime_L < TRIPLE_SHOOT_PRESSED_TIME) ) 
	{
		/* 点射 */
		REVOLVER_SingleShootCtrl(revo);
	}
	/* 长按 */
	else if( 
			(MouseLockTime_L >= TRIPLE_SHOOT_PRESSED_TIME) )
	{
		/* 连射 */
		REVOLVER_TripleShootCtrl(revo);
	}
	/* 松开 */
	else
	{
		MouseLockTime_L = 0;
		/* 防止出错 */
		/* 速度环切位置环 */
		if(revo->PidMode == REVO_SPEED_MODE) {
			Revolver_PID.Angle.target = Revolver_PID.Angle.feedback;	// 防止切换模式的时候动一下
			Revolver_PID.AngleRampBuffer = Revolver_PID.Angle.target;
		}
		revo->PidMode = REVO_POSI_MODE;
		revo->State = REVO_STATE_OFF;		// 停止拨盘旋转
	}	
}

/**
 *	@brief	拨盘电机单发控制
 *	@note	点射
 */
void REVOLVER_SingleShootCtrl(Revolver_Info_t *revo)
{
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	//响应间隔计时
	
	current_time = xTaskGetTickCount();

	revo->PidMode = REVO_POSI_MODE;	// 位置环
	revo->State = REVO_STATE_OFF;	// 关拨盘旋转
	
	/* # 怎么根据性能点调整射频? */
	revo->Shoot.interval = TIME_STAMP_1000MS / revo->Shoot.freq;
	
	if((respond_time < current_time)
			&& (revo->Shoot.num == 0))
	{
		respond_time = current_time + revo->Shoot.interval;
		revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
	}

	/* 松开左键后返回常规控制 */
	if(!IF_MOUSE_PRESSED_LEFT) {
		MouseLockTime_L = 0;
	}
}

/**
 *	@brief	拨盘电机三连发控制
 *	@note	三连发
 */
void REVOLVER_TripleShootCtrl(Revolver_Info_t *revo)
{
//	/* # 怎么根据性能点调整射频? */
//	if(revo->Shoot.heat_real >= 160) {
//		revo->Shoot.freq = 14;
//	} else {
//		revo->Shoot.freq = 8;
//	}	
	
	#if	0
	/* 真-连发 */
	revo->PidMode = REVO_SPEED_MODE;	// 速度环
	revo->State = REVO_STATE_ON;		// 开拨盘旋转
	
	/* 松开左键后返回常规控制 */
	if(!IF_MOUSE_PRESSED_LEFT) {	
		MouseLockTime_L = 0;
	}
	
	#else
	/* 连发设置 */
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	//响应间隔计时
		
	current_time = xTaskGetTickCount();
	
	revo->PidMode = REVO_POSI_MODE;	// 位置环
	revo->State = REVO_STATE_OFF;	// 关拨盘旋转
	revo->Shoot.interval = TIME_STAMP_1000MS / revo->Shoot.freq;
	
	if(respond_time < current_time
		&& (revo->Shoot.num == 0)) 
	{
		respond_time = current_time + revo->Shoot.interval;
		revo->Shoot.num += REVOLVER_HeatLimit(revo, 1);
	}
	
	/* 松开左键后返回常规控制 */
	if(!IF_MOUSE_PRESSED_LEFT) {	
		MouseLockTime_L = 0;
	}	
	#endif
}

/**
 *	@brief	秘技之暴雨梨花
 *	@note		# RM2020对抗赛规则里面枪口热量与子弹初速度无关（固定一颗为10）！
 */
void REVOLVER_ComboShootCtrl(Revolver_Info_t *revo)
{
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	//响应间隔计时	
	
	current_time = xTaskGetTickCount();
	
	/* # 怎么根据性能点调整射频? */
	revo->Shoot.interval = TIME_STAMP_1000MS/15;	//最快一秒15发
	
	if((respond_time < current_time) 
			&& (revo->Shoot.num == 0))
	{
		respond_time = current_time + revo->Shoot.interval;
		revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
	}	
	
	/* 松开B键后返回常规控制 */
	if(!IF_KEY_PRESSED_B) {	
		MouseLockTime_L = 0;
		revo->Action = SHOOT_NORMAL;
	}
}

/**
 *	@brief	拨盘电机打符控制
 *	@note	打符
 *			0.5s的时间切换装甲板
 */
uint8_t test_buff_bullet = 0;
portTickType first_into_armor_4 = 0;
uint32_t	 respond_into_armor_4 = 0;
void REVOLVER_BuffShootCtrl(Revolver_Info_t *revo)
{
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	// 响应间隔计时
	static uint16_t manual_pressed_left = 0;	// 手动打弹左键响应

	/* #位置环 */
	revo->PidMode = REVO_POSI_MODE;
	revo->State = REVO_STATE_OFF;
	
	current_time = xTaskGetTickCount();

	/* 是否更换装甲板 */
	if(VISION_GetFlagStatus(VISION_FLAG_CHANGE_ARMOR) == true) {
		revo->Buff.change_armor = true;
	}
	
	/* 是否切换到第四块装甲板 */
	revo->Buff.change_armor_4 = VISION_GetFlagStatus(VISION_FLAG_CHANGE_ARMOR_4);
	
	if(test_buff_bullet == 0)
	{
		/* 松开右键自动打弹 */
		if(!IF_MOUSE_PRESSED_RIGH)
		{
			manual_pressed_left = 0;
			/* 未识别到符 */
			if(VISION_GetFlagStatus(VISION_FLAG_LOCK_BUFF) == false)
			{
				revo->Buff.lost_time++;
				if(revo->Buff.lost_time > revo->Buff.lost_time_boundary) {
					revo->Buff.fire = false;
					revo->Buff.lock_time = 0;
				}
				
	//			if(revo->Buff.change_armor == true)
	//			{
	//				revo->Buff.lock_time = 0;
	//				revo->Buff.lost_time = 0;
	//			}
			}
			/* 识别到符 */
			else
			{
				revo->Buff.lost_time = 0;
				
				/* 刚切换了装甲板稳定一段时间不打弹 */
				if(revo->Buff.change_armor_delay > 0)
				{
					revo->Buff.change_armor_delay--;
				}

				if(revo->Buff.change_armor_delay == 0)		
				{					
					/* 刚切换了装甲板 */
					if(revo->Buff.change_armor == true)
					{
						revo->Buff.change_armor = false;
						revo->Buff.fire = false;	// 禁止射击
						revo->Buff.lock_time = 0;	// 跟踪装甲板时间清零
						respond_time = current_time-1;// 刷新射击时间
						revo->Buff.change_armor_delay = revo->Buff.change_armor_delay_boundary;	// 切换后先稳定一段时间
					} 
					/* 未切换装甲板 */
					else if(revo->Buff.change_armor == false)
					{
						/* 云台已跟上目标 */
						if(GIMBAL_BUFF_IfChaseReady()) 
						{
							revo->Buff.lock_time++;
							/* 云台稳定锁定装甲板 */
							if(revo->Buff.lock_time > revo->Buff.lock_time_boundary)
							{
								/* 可以开炮 */
								revo->Buff.fire = true;
							}
						}
						/* 云台落后目标 */
						else
						{
							/* 禁止开炮 */
							revo->Buff.lock_time = 0;
							revo->Buff.fire = false;
						}
					}
					
					/* 响应时间已到 & 可以开炮 & 非刚切换装甲板 & 上一颗子弹已打出去 */
					if((respond_time < current_time) &&
						revo->Buff.fire == true && 
						revo->Buff.change_armor == false &&
						revo->Shoot.num == 0) 
					{
						respond_time = current_time + revo->Buff.respond_interval;
						// 打符时由于有冷却加成，一般不会热量超限。这里是剔除热量影响打符打弹
						revo->Shoot.num += 1;	//revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
					}
				}
			}
			
			/* # 测试的时候默认这里拨一个就打一颗出去(之后可以利用裁判系统的射速反馈来判断是否射出) */
			if(revo->Buff.change_armor_4 == true) {
				if(revo->Shoot.num != 0) {
					if(first_into_armor_4 == 0) {
						first_into_armor_4 = xTaskGetTickCount();
					}
				}
				if(first_into_armor_4 != 0) {
					respond_into_armor_4 = xTaskGetTickCount();
					if(respond_into_armor_4 > first_into_armor_4 + TIME_STAMP_100MS) {	// 适当延时模拟发弹延迟
						VISION_SetFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
					}
				}
			} else {
				VISION_ClearFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
				first_into_armor_4 = 0;
			}
		}
		/* 按住右键接管打弹 */
		else
		{
			/* 防止手动切换成自动的时候马上打弹 */
			revo->Buff.fire = false;	// 禁止自动射击
			revo->Buff.lock_time = 0;	// 跟踪装甲板时间清零
			if(IF_MOUSE_PRESSED_LEFT) 
			{
				if(manual_pressed_left < TIME_STAMP_10S)	// 防止长时间按变量溢出
					manual_pressed_left++;
			}
			else
			{
				if(manual_pressed_left > TIME_STAMP_10MS) {	// 左键抬起后才打出
					// 打符时由于有冷却加成，一般不会有热量超限。这里是剔除热量影响打符打弹
					revo->Shoot.num += 1;	//revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
				}
				
				/* # 测试的时候默认这里拨一个就打一颗出去(之后可以利用裁判系统的射速反馈来判断是否射出) */
				if(revo->Buff.change_armor_4 == true) {
					if(revo->Shoot.num != 0) {
						if(first_into_armor_4 == 0) {
							first_into_armor_4 = xTaskGetTickCount();
						}
					}
					if(first_into_armor_4 != 0) {
						respond_into_armor_4 = xTaskGetTickCount();
						if(respond_into_armor_4 > first_into_armor_4 + TIME_STAMP_100MS) {// 适当延时模拟发弹延迟
							VISION_SetFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
						}
					}
				} else {
					VISION_ClearFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
					first_into_armor_4 = 0;
				}
				
				manual_pressed_left = 0;
			}
		}
	}
	else
	{
		/* 防止手动切换成自动的时候马上打弹 */
		revo->Buff.fire = false;	// 禁止自动射击
		revo->Buff.lock_time = 0;	// 跟踪装甲板时间清零
		if(IF_MOUSE_PRESSED_LEFT) 
		{
			if(manual_pressed_left < TIME_STAMP_10S)	// 防止长时间按变量溢出
				manual_pressed_left++;
		}
		else
		{
			if(manual_pressed_left > TIME_STAMP_10MS) {	// 左键抬起后才打出
				revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
			}
			manual_pressed_left = 0;
		}		
	}
}

/**
 *	@brief	拨盘电机自瞄控制
 *	@note		自瞄预测
 */
void REVOLVER_AutoShootCtrl(Revolver_Info_t *revo)
{
	static portTickType current_time     = 0;
	static uint32_t respond_time_Stop    = 0;//响应间隔计时，静止
	static uint32_t respond_time_MobiPre = 0;//响应间隔计时，移动预测	

	/* #位置环 */
	revo->PidMode = REVO_POSI_MODE;
	revo->State = REVO_STATE_OFF;
	
	current_time = xTaskGetTickCount();
	
	/* 自爆模式(紧急情况下以热量超限扣血换取射频 */
	if(IF_KEY_PRESSED_B) {
		revo->Shoot.freq = 20;
		revo->Shoot.suicide_mode = true;
	} else {
		revo->Shoot.suicide_mode = false;
	}

	/* 根据射频计算响应间隔 */
	revo->Shoot.interval = TIME_STAMP_1000MS / revo->Shoot.freq;
	
	/* 自瞄移动预测已开启 */
	if( GIMBAL_IfAutoMobiPre() == true) {
		//revo->Shoot.interval = TIME_STAMP_1000MS/10;
		/* 自瞄开火条件 */
		if(GIMBAL_AUTO_IfChaseReady() == true			// 预测到位
				&& respond_time_MobiPre < current_time	// 射击响应
					&& revo->Shoot.num == 0				// 上一颗已打出
						&& IF_MOUSE_PRESSED_LEFT)		// 左键按下
		{
			respond_time_MobiPre = current_time + revo->Shoot.interval;
			revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
		}
		else{
			revo->Shoot.num = 0;	// 预测未到位，禁止打弹
		}
	}
	/* 自瞄移动预测未开启 */
	else if( GIMBAL_IfAutoMobiPre() == false) {
		//revo->Shoot.interval = TIME_STAMP_1000MS/5;
		/* 自瞄开火条件 */
		if(GIMBAL_AUTO_IfChaseReady() == true			// 预测到位
				&& respond_time_Stop < current_time		// 射击响应
					&& revo->Shoot.num == 0				// 上一颗已打出
						&& IF_MOUSE_PRESSED_LEFT)		// 左键按下
		{
			respond_time_Stop = current_time + revo->Shoot.interval;
			revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
		}
		else{
			revo->Shoot.num = 0;	// 预测未到位，禁止打弹
		}		
	}
}

/**
 *	@brief	拨盘电机禁止射击控制
 */
void REVOLVER_BanShootCtrl(Revolver_Info_t *revo)
{
	revo->PidMode = REVO_POSI_MODE;	// 位置环
	revo->State = REVO_STATE_OFF;	// 关拨盘旋转
	
	revo->Shoot.num = 0;	// 屏蔽打弹指令
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	根据云台模式调整拨盘模式
 *	@note		# 与原有的控制逻辑存在冲突，暂时不用
 */
void REVOLVER_GetInfo(void)
{
	// 获取系统信息
	REVOLVER_GetSysInfo(&System, &Revolver);
	// 获取遥控信息
	REVOLVER_GetRemoteInfo(&System, &Remote, &Revolver);
	// 获取裁判系统信息
	REVOLVER_GetJudgeInfo(&Judge, &Revolver);	
}

/**
 *	@brief	拨盘电机PID控制器
 *	@note	(前提是摩擦轮开启)
 */
uint16_t js_shoot_num = 0;
void REVOLVER_PidCtrlTask(void)
{
	/* 未开摩擦轮 */
	if(FRICTION_IfOpen() == false) {
		g_Revolver_Motor_Info.angle_sum = 0;	// 累加角度值清零
		REVOLVER_PidParamsInit(&Revolver_PID);
		REVOLVER_Stop(&Revolver_PID);
	}
	else {
		if(Revolver.PidMode ==  REVO_SPEED_MODE) // 连射模式
		{	
			if(Revolver.State == REVO_STATE_ON) {	// 开启拨盘旋转
				if(REVOLVER_HeatLimit(&Revolver, 1))
					Revolver_PID.Speed.target = SPEED_AN_BULLET_PER_SECOND * Revolver.Shoot.freq;	// 每秒一颗的转速乘以射频
				else
					Revolver_PID.Speed.target = 0;
			} else if(Revolver.State == REVO_STATE_OFF) {	// 关闭拨盘旋转
				Revolver_PID.Speed.target = 0;
			}
			/* 卡弹判断 */
			REVOLVER_Speed_BulletStuck(&Revolver_PID, &Revolver);
			/* 速度环计算 */
			REVOLVER_Speed_PidCalc(&Revolver_PID);
		} 
		else if(Revolver.PidMode ==  REVO_POSI_MODE) // 点射模式
		{	
			/* 卡弹判断 */
			REVOLVER_Angle_BulletStuck(&Revolver_PID, &Revolver);	
			if(Revolver.Shoot.num > 0) {	// 需要打弹数量
				js_shoot_num++;
				Revolver.Shoot.num--;
				Revolver_PID.AngleRampBuffer += ANGLE_AN_BULLET;
				Revolver.Shoot.real_time = xTaskGetTickCount();	// 
			}
			/* 位置环计算(让目标缓慢逼近最终目标) */
			Revolver_PID.Angle.target = RampFloat(Revolver_PID.AngleRampBuffer, Revolver_PID.Angle.target, ANGLE_AN_BULLET_RAMP);
			REVOLVER_Angle_PidCalc(&Revolver_PID);
			Revolver_PID.Speed.target = Revolver_PID.Angle.out;
			REVOLVER_Speed_PidCalc(&Revolver_PID);
		}
		/*----最终输出----*/
		REVOLVER_PidOut(&Revolver_PID);	
	} 
}

/**
 *	@brief	拨盘电机遥控控制任务
 */
void REVOLVER_RcCtrlTask(void)
{
	REMOTE_SetRevolverShoot(&Remote);
}

/**
 *	@brief	拨盘电机键盘控制任务
 */
void REVOLVER_KeyCtrlTask(void)
{
	/* 摩擦轮没有就绪屏蔽打弹指令 */
	if(FRICTION_IfReady() == false) {
		MouseLockTime_L = 0;
		Revolver.Shoot.num = 0;
		Revolver.State = REVO_STATE_OFF;
		Revolver.PidMode = REVO_POSI_MODE;
		return;
	}

	switch(Revolver.Action)
	{
		case SHOOT_NORMAL:
			REVOLVER_NormalCtrl(&Revolver);
			break;
		case SHOOT_BUFF:
			REVOLVER_BuffShootCtrl(&Revolver);
			break;
		case SHOOT_AUTO:
			REVOLVER_AutoShootCtrl(&Revolver);
			break;
		case SHOOT_BAN:
			REVOLVER_BanShootCtrl(&Revolver);
		default:
			break;
	}	
}

/**
 *	@brief	拨盘电机失控保护
 */
void REVOLVER_SelfProtect(void)
{
	g_Revolver_Motor_Info.angle_sum = 0;	// 累加角度值清零
	REVOLVER_Stop(&Revolver_PID);
	REVOLVER_PidParamsInit(&Revolver_PID);	
}

/**
 *	@brief	拨盘电机控制
 */
void REVOLVER_Ctrl(void)
{
	/*----信息读入----*/
	REVOLVER_GetInfo();
	/*----期望修改----*/
	if(Revolver.RemoteMode == RC) {
		REVOLVER_RcCtrlTask();
	} else if(Revolver.RemoteMode == KEY) {
		REVOLVER_KeyCtrlTask();
	}	
	
	/*----最终输出----*/
	REVOLVER_PidCtrlTask();
}

