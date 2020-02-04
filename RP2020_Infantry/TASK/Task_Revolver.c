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
#define REVOLVER_STUCK_SPEED				60
#define REVOLVER_STUCK_TIME					100
#define REVOLVER_TURNBACK_TIME			100

#define	TRIPLE_SHOOT_PRESSED_TIME		TIME_STAMP_250MS

#define LEVEL_COUNT		3
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
	.state = REVO_STATE_OFF,
	.pidMode = REVO_POSI_MODE,
	.action = SHOOT_NORMAL,
	.Shoot.freq = 8,	// 默认射频8
	.Shoot.num = 0,	// 允许射弹量
	.Shoot.heat_cooling_rate = 0,	// 17mm枪管热量冷却值
	.Shoot.heat_real = 0, // 17mm枪管初始热量
	.Shoot.heat_limit = 240,	// 默认枪口热量上限为240
	.Shoot.stuck_count = 0,// 卡弹计数
	
	.buff.lost_time = 0,
	.buff.lost_time_boundary = TIME_STAMP_70MS,
	.buff.lock_time = 0,
	.buff.lock_time_boundary = TIME_STAMP_80MS,
	.buff.change_armor_delay = 0,
	.buff.change_armor_delay_boundary = TIME_STAMP_50MS,
	.buff.respond_interval = TIME_STAMP_800MS,	// TIME_STAMP_800MS 调试的时候需要
	.buff.change_armor = false,
	.buff.fire = false,
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
void REVOLVER_pidParamsInit(Revolver_PID_t *pid)
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
void REVOLVER_stop(Revolver_PID_t *pid)
{
	int16_t pidOut[4] = {0, 0, 0, 0};
	
	/* 内环速度环最终输出 */
	Revolver_PID.Speed.out = 0;
	Revolver_PID.Out = 0;
	
#if REVOLVER_ID > 0x204	
	CAN2_send(0x1FF, pidOut);
#else
	CAN2_send(0x200, pidOut);
#endif
}

/**
 *	@brief	拨盘电机速度环
 */
void REVOLVER_Speed_pidCalculate(Revolver_PID_t *pid)
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
void REVOLVER_Angle_pidCalculate(Revolver_PID_t *pid)
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
void REVOLVER_pidOut(Revolver_PID_t *pid)
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
	CAN2_queueSend(0x1FF, pidOut);
#else
	CAN2_queueSend(0x200, pidOut);
#endif	
}

/**
 *	@brief	速度环卡弹处理方式(计时判断+反转尝试)
 */
void REVOLVER_Speed_bulletStuck(Revolver_PID_t *pid, Revolver_Info_t *info)
{
	static uint16_t stuck_time = 0;
	static uint16_t turnback_time = 0;
	static uint8_t	stuck_recover_flag = 0;
	
	if(stuck_recover_flag == 0) {	// 没有启动卡弹恢复尝试
		if(pid->Speed.out > REVOLVER_STUCK_PID_OUT
		   && pid->Speed.feedback < REVOLVER_STUCK_SPEED) {	// PID输出过大而速度反馈较小则认为卡弹
			stuck_time++;	
		} else {
			stuck_time = 0;
		}
		if(stuck_time > REVOLVER_STUCK_TIME) {	// 连续卡弹超过60ms
			stuck_time = 0;
			info->Shoot.stuck_count++;
			stuck_recover_flag = 1;
			info->Shoot.num = 0;	// 屏蔽打弹指令
		}
	} else {	// 卡弹恢复
		pid->Speed.target = -4000;
		turnback_time++;
		if(turnback_time > REVOLVER_TURNBACK_TIME) {	// 倒转完成
			turnback_time = 0;
			stuck_recover_flag = 0;
		}
	}
}

/**
 *	@brief	位置环卡弹处理方式(计时判断+反转尝试)
 */
void REVOLVER_Angle_bulletStuck(Revolver_PID_t *pid, Revolver_Info_t *info)
{
	static uint16_t stuck_time = 0;
	static uint16_t turnback_time = 0;
	static uint8_t	stuck_recover_flag = 0;
	
	if(stuck_recover_flag == 0) {	// 没有启动卡弹恢复尝试
		if(pid->Speed.out > REVOLVER_STUCK_PID_OUT
		   && pid->Speed.feedback < REVOLVER_STUCK_SPEED) {	// PID输出过大而速度反馈较小则认为卡弹
			stuck_time++;	
		} else {
			stuck_time = 0;
		}
		if(stuck_time > REVOLVER_STUCK_TIME) {	// 连续卡弹超过60ms
			stuck_time = 0;
			info->Shoot.stuck_count++;
			stuck_recover_flag = 1;
			info->Shoot.num = 0;	// 屏蔽打弹指令
			//pid->Angle.target = ANGLE_AN_BULLET;	// 反转1格
			pid->AngleRampBuffer = pid->Angle.target - ANGLE_AN_BULLET;
		}
	} else {	// 卡弹恢复
		turnback_time++;
		if(turnback_time > REVOLVER_TURNBACK_TIME) {	// 倒转完成
			pid->Angle.target = pid->Angle.feedback;
			pid->AngleRampBuffer = pid->Angle.target;
			turnback_time = 0;
			stuck_recover_flag = 0;
		}
	}
}

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief 设置拨盘的控制行为
 */
void REVOLVER_setAction(Revolver_Action_t action)
{
	Revolver.action = action;
}

/**
 *	@brief	返回实时射击时间
 */
uint32_t REVOLVER_getRealShootTime(void)
{
	return Revolver.Shoot.real_time;
}

/**
 *	@brief	发弹量+1
 */
void REVOLVER_addShootCount(void)
{
	Revolver.Shoot.total_count++;
}


/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #遥控# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	遥控设置拨盘电机转动(完成射击动作)
 *	@note	SW1_DOWN(前提是摩擦轮开启)
 */
void REMOTE_setRevolverShoot(RC_Ctl_t *remoteInfo)
{
	static uint8_t prev_sw1 = RC_SW_DOWN;
	//static uint8_t prev_sw2 = RC_SW_MID;
	uint8_t	sw1, sw2;
	sw1 = remoteInfo->rc.s1;
	sw2 = remoteInfo->rc.s2;
	
	if(FRICTION_ifReady()) {
		if(Revolver.pidMode == REVO_POSI_MODE) {	// 点射模式
			if(sw1 == RC_SW_DOWN && prev_sw1 != RC_SW_DOWN) 
			{
				Revolver.Shoot.freq = 8;
				Revolver.Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
			}
		} else if(Revolver.pidMode == REVO_SPEED_MODE) {	// 连射模式
			if(sw1 == RC_SW_DOWN) {
				Revolver.state = REVO_STATE_ON;
				Revolver.Shoot.freq = 15;
				Revolver_PID.Speed.target = SPEED_AN_BULLET_PER_SECOND * Revolver.Shoot.freq;	// 发/s
			} else {
				Revolver.state = REVO_STATE_OFF;
				Revolver_PID.Speed.target = 0;
			}
		}
	}
	prev_sw1 = sw1;
	
	if(sw2 == RC_SW_MID) {
		/* 速度环->位置环*/
		if(Revolver.pidMode == REVO_SPEED_MODE) {
			Revolver.pidMode = REVO_POSI_MODE;
			Revolver_PID.Angle.target = Revolver_PID.Angle.feedback;	// 防止切换模式的时候动一下
			Revolver_PID.AngleRampBuffer = Revolver_PID.Angle.target;
		}
	} else if(sw2 == RC_SW_DOWN) {
		/* 位置环->速度环 */
		if(Revolver.pidMode == REVO_POSI_MODE) {
			Revolver.pidMode = REVO_SPEED_MODE;
			Revolver.state = REVO_STATE_OFF;
			Revolver_PID.Speed.target = 0;
			//Revolver_PID.Speed.out = 0;	// s1_down,s2_down -> s2_mid -> s1_mid -> s2_down => 拨盘会正转一小步(小于一格)
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

void KEY_setRevolverShoot(RC_Ctl_t *remoteInfo)
{	
//	/* 摩擦轮开启的时候才响应拨盘 */
//	if(FRICTION_ifReady()) 
//	{		
//		/* 按下鼠标左键时累计时间 */
//		if(IF_MOUSE_PRESSED_LEFT)
//		{
//			if(Time_Pressed_Shoot < TIME_STAMP_6000MS)	// 防止长时间按变量溢出
//					Time_Pressed_Shoot++;
//		}
//		
//		if(IF_KEY_PRESSED_B && !IF_MOUSE_PRESSED_LEFT & !IF_KEY_PRESSED_B)
//		{
//			/* 推家模式 */
//			Revolver.pidMode = REVO_POSI_MODE;
//			Revolver.action = SHOOT_HIGH_F_LOW_S;
//		}
//		else if(GIMBAL_ifBuffMode())
//		{
//			/* 打符模式 */
//			Revolver.pidMode = REVO_POSI_MODE;
//			Revolver.action = SHOOT_BUFF;
//			/* 切换模式时清除左键响应时间 */
//			Time_Pressed_Shoot = 0;
//		}
//		else if(GIMBAL_ifAutoMode())
//		{
//			/* 自瞄模式 */
//			Revolver.pidMode = REVO_POSI_MODE;
//			Revolver.action = SHOOT_AUTO;
//			/* 切换模式时清除左键响应时间 */
//			Time_Pressed_Shoot = 0;
//		}
//		else if(IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z) 
//		{
//			/* 三连发模式 */
//			if(Time_Pressed_Shoot >= TRIPLE_SHOOT_PRESSED_TIME) 
//			{	// 首次启动需要长按左键250ms
//				//Time_Pressed_Shoot = 0;
//				Revolver.pidMode = REVO_SPEED_MODE;	// 速度环
//				Revolver.action = SHOOT_TRIPLE;		// 三连发
//				Revolver.state = REVO_STATE_ON;		// 开启拨盘旋转
//			} 
//		} 
//		else if((Time_Pressed_Shoot > TIME_STAMP_10MS && Time_Pressed_Shoot < TRIPLE_SHOOT_PRESSED_TIME)
//				&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z)
//		{
//			/* 点射模式 */
//			Time_Pressed_Shoot = 0;
//			Revolver.pidMode = REVO_POSI_MODE;
//			Revolver.action = SHOOT_SINGLE;
//		}
//		else 
//		{
//			/* 常规控制模式 */
//			/* 速度环切位置环 */
//			if(Revolver.pidMode == REVO_SPEED_MODE) {
//				Revolver_PID.Angle.target = Revolver_PID.Angle.feedback;	// 防止切换模式的时候动一下
//				Revolver_PID.AngleRampBuffer = Revolver_PID.Angle.target;
//			}
//			Time_Pressed_Shoot = 0;
//			Revolver.pidMode = REVO_POSI_MODE;
//			Revolver.action = SHOOT_NORMAL;
//			Revolver.state = REVO_STATE_OFF;		// 停止拨盘旋转			
//		}
//	}
//	/* 未开启摩擦轮屏蔽指令*/
//	else	
//	{
//		Time_Pressed_Shoot = 0;
//		Revolver.pidMode = REVO_POSI_MODE;
//		Revolver.action = SHOOT_NORMAL;		
//	}
}

/* #拨盘# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief 	结合当前热量情况计算最大发弹量(0<=allow_num<=target_num)
 *	@return allow_num( 0 ~ target_num)
 */
uint8_t REVOLVER_heatLimit(Revolver_Info_t *info, uint8_t target_num)
{
	uint16_t remain_heat;	// 剩余热量
	uint8_t  allow_num;		// 允许的发弹数量
	
	if(target_num == 0)	// 防止出错
		return 0;
	
	if(info->Shoot.suicide_mode == true)	// 热量超限扣血换取射频
		return target_num;
	
	remain_heat = info->Shoot.heat_limit - info->Shoot.heat_real;
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
 *	@brief	拨盘电机PID控制器
 *	@note	(前提是摩擦轮开启)
 */
uint16_t js_shoot_num = 0;
void REVOLVER_pidControlTask(void)
{
	/* 未开摩擦轮 */
	if(FRICTION_ifOpen() == false) {
		g_Revolver_Motor_Info.angle_sum = 0;	// 累加角度值清零
		REVOLVER_pidParamsInit(&Revolver_PID);
		REVOLVER_stop(&Revolver_PID);
	}
	else {
		if(Revolver.pidMode ==  REVO_SPEED_MODE) // 连射模式
		{	
			if(Revolver.state == REVO_STATE_ON) {	// 开启拨盘旋转
				if(REVOLVER_heatLimit(&Revolver, 1))
					Revolver_PID.Speed.target = SPEED_AN_BULLET_PER_SECOND * Revolver.Shoot.freq;	// 每秒一颗的转速乘以射频
				else
					Revolver_PID.Speed.target = 0;
			} else if(Revolver.state == REVO_STATE_OFF) {	// 关闭拨盘旋转
				Revolver_PID.Speed.target = 0;
			}
			/* 卡弹判断 */
			REVOLVER_Speed_bulletStuck(&Revolver_PID, &Revolver);
			/* 速度环计算 */
			REVOLVER_Speed_pidCalculate(&Revolver_PID);
		} 
		else if(Revolver.pidMode ==  REVO_POSI_MODE) // 点射模式
		{	
			/* 卡弹判断 */
			REVOLVER_Angle_bulletStuck(&Revolver_PID, &Revolver);	
			if(Revolver.Shoot.num > 0) {	// 需要打弹数量
				js_shoot_num++;
				Revolver.Shoot.num--;
				Revolver_PID.AngleRampBuffer += ANGLE_AN_BULLET;
				Revolver.Shoot.real_time = xTaskGetTickCount();
			}
			/* 位置环计算(让目标缓慢逼近最终目标) */
			Revolver_PID.Angle.target = RAMP_float(Revolver_PID.AngleRampBuffer, Revolver_PID.Angle.target, ANGLE_AN_BULLET_RAMP);
			REVOLVER_Angle_pidCalculate(&Revolver_PID);
			Revolver_PID.Speed.target = Revolver_PID.Angle.out;
			REVOLVER_Speed_pidCalculate(&Revolver_PID);
		}
		/*----最终输出----*/
		REVOLVER_pidOut(&Revolver_PID);	
	} 
}

/**
 *	@brief	拨盘电机常规控制
 *	@note		默认不打弹
 */
static uint16_t Time_Pressed_Shoot = 0;	
void REVOLVER_normalControl(Revolver_Info_t *info)
{
	/* 摩擦轮没有就绪屏蔽打弹指令 */
	if(FRICTION_ifReady() == false) {
		Time_Pressed_Shoot = 0;
		Revolver.pidMode = REVO_POSI_MODE;
		Revolver.action = SHOOT_NORMAL;
		return;
	}
	
	/* 按下鼠标左键时累计时间 */
	if(IF_MOUSE_PRESSED_LEFT)
	{
		if(Time_Pressed_Shoot < TIME_STAMP_6000MS)	// 防止长时间按变量溢出
				Time_Pressed_Shoot++;
	}

	if( !IF_MOUSE_PRESSED_LEFT
			&& (Time_Pressed_Shoot > TIME_STAMP_10MS)
			&& (Time_Pressed_Shoot < TRIPLE_SHOOT_PRESSED_TIME) ) 
	{
		/* 点射 */
		Revolver.pidMode = REVO_POSI_MODE;	// 位置环
		Revolver.state = REVO_STATE_OFF;		// 关拨盘旋转
		REVOLVER_singleShootControl(info);
	}
	else if( (Time_Pressed_Shoot >= TRIPLE_SHOOT_PRESSED_TIME) )
	{
		/* 连射 */
		Revolver.pidMode = REVO_SPEED_MODE;	// 速度环
		Revolver.state = REVO_STATE_ON;			// 开拨盘旋转
		REVOLVER_tripleShootControl(info);
	}
	else
	{
		/* 防止出错 */
		/* 速度环切位置环 */
		if(Revolver.pidMode == REVO_SPEED_MODE) {
			Revolver_PID.Angle.target = Revolver_PID.Angle.feedback;	// 防止切换模式的时候动一下
			Revolver_PID.AngleRampBuffer = Revolver_PID.Angle.target;
		}
		Time_Pressed_Shoot = 0;
		Revolver.pidMode = REVO_POSI_MODE;
		Revolver.action = SHOOT_NORMAL;
		Revolver.state = REVO_STATE_OFF;		// 停止拨盘旋转
	}	
}

/**
 *	@brief	拨盘电机单发控制
 *	@note		点射
 */
void REVOLVER_singleShootControl(Revolver_Info_t *info)
{
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	//响应间隔计时
	
	current_time = xTaskGetTickCount();
	
	/* # 怎么根据性能点调整射频? */
	info->Shoot.interval = TIME_STAMP_1000MS / info->Shoot.freq;
	
	if((respond_time < current_time)
			&& (info->Shoot.num == 0))
	{
		respond_time = current_time + info->Shoot.interval;
		info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
	}

	/* 松开左键后返回常规控制 */
	if(!IF_MOUSE_PRESSED_LEFT) {
		Time_Pressed_Shoot = 0;
		Revolver.action = SHOOT_NORMAL;
	}
}

/**
 *	@brief	拨盘电机三连发控制
 *	@note		三连发
 */
void REVOLVER_tripleShootControl(Revolver_Info_t *info)
{
//	/* # 怎么根据性能点调整射频? */
//	if(info->Shoot.heat_real >= 160) {
//		info->Shoot.freq = 14;
//	} else {
//		info->Shoot.freq = 8;
//	}	
	
	/* 松开左键后返回常规控制 */
	if(!IF_MOUSE_PRESSED_LEFT) {	
		Time_Pressed_Shoot = 0;
		Revolver.action = SHOOT_NORMAL;
	}
}

/**
 *	@brief	秘技之暴雨梨花
 *	@note		# RM2020对抗赛规则里面枪口热量与子弹初速度无关（固定一颗为10）！
 */
void REVOLVER_comboShootControl(Revolver_Info_t *info)
{
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	//响应间隔计时	
	
	current_time = xTaskGetTickCount();
	
	/* # 怎么根据性能点调整射频? */
	info->Shoot.interval = TIME_STAMP_1000MS/15;	//最快一秒15发
	
	if((respond_time < current_time) 
			&& (info->Shoot.num == 0))
	{
		respond_time = current_time + info->Shoot.interval;
		info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
	}	
	
	/* 松开B键后返回常规控制 */
	if(!IF_KEY_PRESSED_B) {	
		Time_Pressed_Shoot = 0;
		Revolver.action = SHOOT_NORMAL;
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
void REVOLVER_buffShootControl(Revolver_Info_t *info)
{
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	// 响应间隔计时
	static uint16_t manual_pressed_left = 0;	// 手动打弹左键响应

	/* #位置环 */
	Revolver.pidMode = REVO_POSI_MODE;
	Revolver.state = REVO_STATE_OFF;
	
	current_time = xTaskGetTickCount();

	/* 是否更换装甲板 */
	if(VISION_getFlagStatus(VISION_FLAG_CHANGE_ARMOR) == true) {
		info->buff.change_armor = true;
	}
	
	/* 是否切换到第四块装甲板 */
	info->buff.change_armor_4 = VISION_getFlagStatus(VISION_FLAG_CHANGE_ARMOR_4);
	
	if(test_buff_bullet == 0)
	{
		/* 松开右键自动打弹 */
		if(!IF_MOUSE_PRESSED_RIGH)
		{
			manual_pressed_left = 0;
			/* 未识别到符 */
			if(VISION_getFlagStatus(VISION_FLAG_LOCK_BUFF) == false)
			{
				info->buff.lost_time++;
				if(info->buff.lost_time > info->buff.lost_time_boundary) {
					info->buff.fire = false;
					info->buff.lock_time = 0;
				}
				
	//			if(info->buff.change_armor == true)
	//			{
	//				info->buff.lock_time = 0;
	//				info->buff.lost_time = 0;
	//			}
			}
			/* 识别到符 */
			else
			{
				info->buff.lost_time = 0;
				
				/* 刚切换了装甲板稳定一段时间不打弹 */
				if(info->buff.change_armor_delay > 0)
				{
					info->buff.change_armor_delay--;
				}

				if(info->buff.change_armor_delay == 0)		
				{					
					/* 刚切换了装甲板 */
					if(info->buff.change_armor == true)
					{
						info->buff.change_armor = false;
						info->buff.fire = false;	// 禁止射击
						info->buff.lock_time = 0;	// 跟踪装甲板时间清零
						respond_time = current_time-1;// 刷新射击时间
						info->buff.change_armor_delay = info->buff.change_armor_delay_boundary;	// 切换后先稳定一段时间
					} 
					/* 未切换装甲板 */
					else if(info->buff.change_armor == false)
					{
						/* 云台已跟上目标 */
						if(GIMBAL_BUFF_chaseReady()) 
						{
							info->buff.lock_time++;
							/* 云台稳定锁定装甲板 */
							if(info->buff.lock_time > info->buff.lock_time_boundary)
							{
								/* 可以开炮 */
								info->buff.fire = true;
							}
						}
						/* 云台落后目标 */
						else
						{
							/* 禁止开炮 */
							info->buff.lock_time = 0;
							info->buff.fire = false;
						}
					}
					
					/* 响应时间已到 & 可以开炮 & 非刚切换装甲板 & 上一颗子弹已打出去 */
					if((respond_time < current_time) &&
						info->buff.fire == true && 
						info->buff.change_armor == false &&
						info->Shoot.num == 0) 
					{
						respond_time = current_time + info->buff.respond_interval;
						info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
					}
				}
			}
			
			/* # 测试的时候默认这里拨一个就打一颗出去(之后可以利用裁判系统的射速反馈来判断是否射出) */
			if(info->buff.change_armor_4 == true) {
				if(info->Shoot.num != 0) {
					if(first_into_armor_4 == 0) {
						first_into_armor_4 = xTaskGetTickCount();
					}
				}
				if(first_into_armor_4 != 0) {
					respond_into_armor_4 = xTaskGetTickCount();
					if(respond_into_armor_4 > first_into_armor_4 + TIME_STAMP_100MS) {	// 适当延时模拟发弹延迟
						VISION_setFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
					}
				}
			} else {
				VISION_clearFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
				first_into_armor_4 = 0;
			}
		}
		/* 按住右键接管打弹 */
		else
		{
			/* 防止手动切换成自动的时候马上打弹 */
			info->buff.fire = false;	// 禁止自动射击
			info->buff.lock_time = 0;	// 跟踪装甲板时间清零
			if(IF_MOUSE_PRESSED_LEFT) 
			{
				if(manual_pressed_left < TIME_STAMP_10S)	// 防止长时间按变量溢出
					manual_pressed_left++;
			}
			else
			{
				if(manual_pressed_left > TIME_STAMP_10MS) {	// 左键抬起后才打出
					info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
				}
				
				/* # 测试的时候默认这里拨一个就打一颗出去(之后可以利用裁判系统的射速反馈来判断是否射出) */
				if(info->buff.change_armor_4 == true) {
					if(info->Shoot.num != 0) {
						if(first_into_armor_4 == 0) {
							first_into_armor_4 = xTaskGetTickCount();
						}
					}
					if(first_into_armor_4 != 0) {
						respond_into_armor_4 = xTaskGetTickCount();
						if(respond_into_armor_4 > first_into_armor_4 + TIME_STAMP_100MS) {// 适当延时模拟发弹延迟
							VISION_setFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
						}
					}
				} else {
					VISION_clearFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
					first_into_armor_4 = 0;
				}
				
				manual_pressed_left = 0;
			}
		}
	}
	else
	{
		/* 防止手动切换成自动的时候马上打弹 */
		info->buff.fire = false;	// 禁止自动射击
		info->buff.lock_time = 0;	// 跟踪装甲板时间清零
		if(IF_MOUSE_PRESSED_LEFT) 
		{
			if(manual_pressed_left < TIME_STAMP_10S)	// 防止长时间按变量溢出
				manual_pressed_left++;
		}
		else
		{
			if(manual_pressed_left > TIME_STAMP_10MS) {	// 左键抬起后才打出
				info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
			}
			manual_pressed_left = 0;
		}		
	}
}

/**
 *	@brief	拨盘电机自瞄控制
 *	@note		自瞄预测
 */
void REVOLVER_autoShootControl(Revolver_Info_t *info)
{
	static portTickType current_time     = 0;
	static uint32_t respond_time_Stop    = 0;//响应间隔计时，静止
	static uint32_t respond_time_MobiPre = 0;//响应间隔计时，移动预测	

	/* #位置环 */
	Revolver.pidMode = REVO_POSI_MODE;
	Revolver.state = REVO_STATE_OFF;
	
	current_time = xTaskGetTickCount();
	
	/* 自爆模式(紧急情况下以热量超限扣血换取射频 */
	if(IF_KEY_PRESSED_B) {
		info->Shoot.freq = 20;
		info->Shoot.suicide_mode = true;
	} else {
		info->Shoot.suicide_mode = false;
	}

	/* 根据射频计算响应间隔 */
	info->Shoot.interval = TIME_STAMP_1000MS / info->Shoot.freq;
	
	/* 自瞄移动预测已开启 */
	if( GIMBAL_ifAutoMobiPre() == true) {
		//info->Shoot.interval = TIME_STAMP_1000MS/10;
		/* 自瞄开火条件 */
		if(GIMBAL_AUTO_chaseReady() == true				// 预测到位
				&& respond_time_MobiPre < current_time// 射击响应
					&& info->Shoot.num == 0							// 上一颗已打出
						&& IF_MOUSE_PRESSED_LEFT)					// 左键按下
		{
			respond_time_MobiPre = current_time + info->Shoot.interval;
			info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
		}
		else{
			info->Shoot.num = 0;	// 预测未到位，禁止打弹
		}
	}
	/* 自瞄移动预测未开启 */
	else if( GIMBAL_ifAutoMobiPre() == false) {
		//info->Shoot.interval = TIME_STAMP_1000MS/5;
		/* 自瞄开火条件 */
		if(GIMBAL_AUTO_chaseReady() == true				// 预测到位
				&& respond_time_Stop < current_time		// 射击响应
					&& info->Shoot.num == 0							// 上一颗已打出
						&& IF_MOUSE_PRESSED_LEFT)					// 左键按下
		{
			respond_time_Stop = current_time + info->Shoot.interval;
			info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
		}
		else{
			info->Shoot.num = 0;	// 预测未到位，禁止打弹
		}		
	}
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	拨盘电机读取裁判系统信息
 */
void REVOLVER_recordJudgeInfo(Revolver_Info_t *revo_info, Judge_Info_t *judge_info)
{
	if(judge_info->data_valid == true) {
		revo_info->Shoot.heat_real = judge_info->PowerHeatData.shooter_heat0;
		revo_info->Shoot.heat_limit = judge_info->GameRobotStatus.shooter_heat0_cooling_limit;
		revo_info->Shoot.heat_cooling_rate = judge_info->GameRobotStatus.shooter_heat0_cooling_rate;	// 步兵<20%HP,冷却速率翻倍
	} else {
		//..暂不处理
	}
	
	switch(judge_info->GameRobotStatus.robot_level)
	{
		// 1级步兵
		case 1:
			revo_info->Shoot.freq = 8;
			break;
		// 2级步兵
		case 2:
			revo_info->Shoot.freq = 10;
			break;
		// 3级步兵
		case 3:
			revo_info->Shoot.freq = 12;
			break;
		// 防止出bug
		default:
			revo_info->Shoot.freq = 8;
			break;
	}
}

/**
 *	@brief	根据云台模式调整拨盘模式
 *	@note		# 与原有的控制逻辑存在冲突，暂时不用
 */
void REVOLVER_getInfo(void)
{
	static Gimbal_Mode_t now_mode;
	
	/* 获取裁判系统信息 */
	REVOLVER_recordJudgeInfo(&Revolver, &Judge_Info);
	
	/* 调整拨盘模式 */
	now_mode = GIMBAL_getMode();
	switch(now_mode)
	{
		case GIMBAL_MODE_NORMAL:
		case GIMBAL_MODE_RELOAD_BULLET:
			/*
				云台常规和对位时恢复常规控制
			*/
			REVOLVER_setAction(SHOOT_NORMAL);
			break;
		case GIMBAL_MODE_AUTO:
			/*
				云台自瞄时拨盘进入自瞄模式
			*/
			REVOLVER_setAction(SHOOT_AUTO);
			break;
		case GIMBAL_MODE_SMALL_BUFF:
		case GIMBAL_MODE_BIG_BUFF:
			/*
				云台打符时拨盘进入打符模式
			*/
			REVOLVER_setAction(SHOOT_BUFF);
			break;
		default:
			break;
	}
}

/**
 *	@brief	拨盘电机遥控控制任务
 */
void REVOLVER_rcControlTask(void)
{
	REMOTE_setRevolverShoot(&RC_Ctl_Info);
}

/**
 *	@brief	拨盘电机键盘控制任务
 */
void REVOLVER_keyControlTask(void)
{
	/* 键盘鼠标控制拨盘射击行为 */
	//KEY_setRevolverShoot(&RC_Ctl_Info);	
	switch(Revolver.action)
	{
		case SHOOT_NORMAL:
			REVOLVER_normalControl(&Revolver);
			break;
		case SHOOT_BUFF:
			REVOLVER_buffShootControl(&Revolver);
			break;
		case SHOOT_AUTO:
			REVOLVER_autoShootControl(&Revolver);
			break;
		default:
			break;
	}	
}

/**
 *	@brief	拨盘电机失控保护
 */
void REVOLVER_selfProtect(void)
{
	g_Revolver_Motor_Info.angle_sum = 0;	// 累加角度值清零
	REVOLVER_stop(&Revolver_PID);
	REVOLVER_pidParamsInit(&Revolver_PID);	
}

/**
 *	@brief	拨盘电机控制
 */
void REVOLVER_control(void)
{
	/*----信息读入----*/
	REVOLVER_getInfo();
	/*----期望修改----*/
	if(Flag.Remote.FLAG_mode == RC) {
		REVOLVER_rcControlTask();
	} else if(Flag.Remote.FLAG_mode == KEY) {
		REVOLVER_keyControlTask();
	}	
	
	/*----最终输出----*/
	REVOLVER_pidControlTask();
}

