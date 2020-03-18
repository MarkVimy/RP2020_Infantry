#include "judge2_0.h"


/*比赛相关信息*/

Interact_Target self_id = RED_SENTRY;	 /*缺省为红色哨兵，初始化应该通过裁判系统读取*/

ext_game_status_t game_status;	/*比赛状态*/

ext_game_result_t game_result;	/*比赛结果*/

ext_game_robot_HP_t game_robot_HP;	/*血量信息*/

ext_dart_status_t	dart_status;		/*飞镖状态*/

ext_event_data_t event_data;		/*事件信息*/		

ext_dart_remaining_time_t dart_remaining_time;	/*飞镖剩余时间*/

ext_game_robot_status_t game_robot_status;		/*机器人状态*/

ext_power_heat_data_t power_heat;			/*热量功率信息*/

ext_game_robot_pos_t game_robot_pos;	/*机器人位置信息*/

ext_buff_t buff;	/*buff状态*/

ext_robot_hurt_t robot_hurt;		/*机器人伤害信息*/

ext_shoot_data_t shoot_data;		/*射击信息*/

ext_bullet_remaining_t bullet_remaining;	/*子弹剩余数*/

ext_student_interactive_header_data_t interactive_data_header;		/*交互数据的标题头*/

robot_interactive_data_t send_interactive_buff;		/*要发送的交互数据包*/

robot_interactive_data_t receive_interactive_buff;	/*用于接收的交互数据包*/

ext_client_custom_graphic_delete_t graphic_delete;	/*删除图层*/

graphic_data_struct_t graphic_data;		/*图层对象信息*/

ext_client_custom_graphic_single_t draw_single_graphic;

ext_client_custom_graphic_double_t draw_double_grahpci;

ext_client_custom_graphic_five_t	draw_five_graphic;

ext_client_custom_graphic_seven_t	draw_seven_graphic;

ext_client_custom_character_t	draw_char_graphic;

uint16_t jdg_buf_length=0; 	/*裁判系统的接收数据长度*/
uint16_t cmd_id=0;	/*命令码*/
bool update_flag = false; 	/*更新标志位*/


/**
* @brief 裁判系统数据接收函数
* @param uint8_t * 接收数据包指针
* @return void
*	包括处理一包多帧的情况
*/
void Read_Judge_Info(uint8_t *rx_buf)
{
	uint8_t *p_buf = rx_buf;
	
	//数据命令码解析
	//拷贝帧头部分的数据
	if(rx_buf==NULL)
		return;
	if(rx_buf[0] == JUDGE_FRAME_HEADER)
	{
		//判断帧头
		if(Verify_CRC8_Check_Sum(rx_buf,5) == true)
		{
			//判断帧头CRC8
			//统计一帧数据长度,用于CR16校验
			jdg_buf_length = (*(rx_buf+1))+9;
			//
			if(Verify_CRC16_Check_Sum(rx_buf,jdg_buf_length) == true)
			{
				//判断帧尾CRC16
				//只有CRC16成功才是true
				cmd_id=(((uint16_t)rx_buf[6]) << 8 | rx_buf[5]);
				//获取数据命令码
				update_flag=true;
				//更新标志位
				switch(cmd_id)
				{
					case ID_game_state:          		//0x0001
						//比赛状态数据
						break;
					
					case ID_game_result:       //0x0002
						//比赛结果数据，比赛结束发送
						break;
					
					case ID_game_robot_HP:    				//0x0003
						//比赛机器人血量数据，1Hz发送
						break;
					
					case ID_dart_status:   //0x0004
						//飞镖发射状态，飞镖发射时发送
						break;
					
					case ID_ICRA_buff_debuff_zone_status:  //0x0005
						//人工智能挑战赛加成与惩罚区状态，1Hz
						break;
					
					case ID_event_data:      		//0x0101
						//场地事件数据，1Hz

						break;

					case ID_supply_projectile_action:      		//0x0102
						//场地补给站动作标识数据

						break;
					
					case ID_referee_warning:      		//0x0104
						//裁判警告数据，警告后发送
						break;
					
					case ID_dart_remaining_time:      			//0x0105
						//飞镖发射口倒计时，1Hz
						break;
					
					case ID_game_robot_state:      	//0x0201
						//机器人状态数据，10Hz
						break;
					
					case ID_power_heat_data:      			//0x0202
						//实时功率热量数据，50Hz
						break;
					
					case ID_game_robot_pos:      			//0x0203
						//机器人位置数据，10Hz
						break;	
					
					case ID_buff_musk:      			//0x0204
						//机器人增益数据，1Hz
						break;
					
					case ID_aerial_robot_energy:      			//0x0205
						//空中机器人能量状态数据，10Hz，只有空中机器人主控发送
						break;

					case ID_robot_hurt:      			//0x0206
						//伤害状态数据，伤害发生后发送
						break;

					case ID_shoot_data:      			//0x0207
						//实时射击数据，子弹发射后发送
						break;	

					case ID_bullet_remaining:      			//0x0208
						//弹丸剩余发送数，仅空中机器人，哨兵机器人以及ICRA机器人发送，1Hz
						break;		

					case ID_rfid_status:      			//0x0209
						//机器人RFID状态，1Hz
						break;	

					case ID_interactive_header_data:      			//0x0301
						//机器人交互数据，发送方触发发送
						break;									
					default:break;
				}
				if(*(rx_buf+jdg_buf_length)==JUDGE_FRAME_HEADER)
				{
					//如果一个数据包出现了多帧数据,则再次读取
					p_buf = rx_buf+jdg_buf_length;
					Read_Judge_Info(p_buf);
				}
			}			
		}	
	}		
}

/**
* @brief 绘制一个矩形
* @param  graphic *图层信息
					name	图层名
					layer 图层数
					color 图形颜色
					width 线宽
					start_x 开始点x坐标
					start_y 开始点y坐标
					diagonal_x 对角点x坐标
					diagonal_y 对角点y坐标
* @return NONE
*/
void Draw_Rectangle(graphic_data_struct_t* graphic,
										const uint8_t *name,
										uint8_t layer,
										Graphic_Color color,
										uint16_t width,
										uint16_t start_x,
										uint16_t start_y,
										uint16_t diagonal_x,
										uint16_t diagonal_y)
{
	Add_Graphic(graphic,name,layer,RECTANGLE,color,0,0,width,start_x,start_y,0,diagonal_x,diagonal_y);
}

/**
* @brief 绘制一条直线
* @param  graphic *图层信息
					name	图层名
					layer 图层数
					color 图形颜色
					width 线宽
					start_x 开始点x坐标
					start_y 开始点y坐标
					end_x 终点x坐标
					end_y 终点y坐标
* @return NONE
*/
void Draw_Line(graphic_data_struct_t* graphic,
							const uint8_t *name,
							uint8_t layer,
							Graphic_Color color,
							uint16_t width,
							uint16_t start_x,
							uint16_t start_y,
							uint16_t end_x,
							uint16_t end_y)
{
	Add_Graphic(graphic,name,layer,LINE,color,0,0,width,start_x,start_y,0,end_x,end_y);
}
 
/**
* @brief 绘制一个圆
* @param  graphic *图层信息
					name	图层名
					layer 图层数
					color 图形颜色
					width 线宽
					center_x 圆心x坐标
					center_y 圆心y坐标
					radius 半径
* @return NONE
*/
void Draw_Circle(graphic_data_struct_t* graphic,
							const uint8_t *name,
							uint8_t layer,
							Graphic_Color color,
							uint16_t width,
							uint16_t center_x,
							uint16_t center_y,
							uint16_t radius)
{
	Add_Graphic(graphic,name,layer,CIRCLE,color,0,0,width,center_x,center_y,radius,0,0);
}

/**
* @brief 绘制一个椭圆
* @param  graphic *图层信息
					name	图层名
					layer 图层数
					color 图形颜色
					width 线宽
					center_x 圆心x坐标
					center_y 圆心y坐标
					axis_x x半轴
					axis_y y半轴
* @return NONE
*/
void Draw_Oval(graphic_data_struct_t* graphic,
							const uint8_t *name,
							uint8_t layer,
							Graphic_Color color,
							uint16_t width,
							uint16_t center_x,
							uint16_t center_y,
							uint16_t axis_x,
							uint16_t axis_y)
{	
	Add_Graphic(graphic,name,layer,OVAL,color,0,0,width,center_x,center_y,0,axis_x,axis_y);
}


/**
* @brief 绘制一个圆弧
* @param  graphic *图层信息
					name	图层名
					layer 图层数
					color 图形颜色
					start_angle 起始角度
					end_angle 	终止角度
					width 线宽
					center_x 圆心x坐标
					center_y 圆心y坐标
					axis_x x半轴
					axis_y y半轴
* @return NONE
*/
void Draw_ARC(graphic_data_struct_t* graphic,
							const uint8_t *name,
							uint8_t layer,
							Graphic_Color color,
							uint16_t start_angle,
							uint16_t end_angle,
							uint16_t width,
							uint16_t center_x,
							uint16_t center_y,
							uint16_t axis_x,
							uint16_t axis_y)
{
	Add_Graphic(graphic,name,layer,ARC,color,start_angle,end_angle,width,center_x,center_y,0,axis_x,axis_y);
}

/**
* @brief 绘制浮点数
* @param  graphic *图层信息
					name	图层名
					layer 图层数
					color 图形颜色
					font_size	字体大小
					accuracy 小数点位数
					width 线宽
					start_x	起始x坐标
					start_y 起始y坐标
					number 显示的数字
* @return NONE
*/
void Draw_Float(graphic_data_struct_t* graphic,
								const uint8_t *name,
								uint8_t layer,
								Graphic_Color color,
								uint16_t font_size,
								uint16_t accuracy,
								uint16_t width,
								uint16_t start_x,
								uint16_t start_y,
								float number)
{
	float num_tmp_f = number;
	uint32_t num_tmp_u=0x00;
	memcpy(&num_tmp_u,&num_tmp_f,4);
	Add_Graphic(graphic,name,layer,FLOAT,color,font_size,accuracy,width,start_x,start_y,(num_tmp_u>>22)&0x3ff,(num_tmp_u>>11)&0x7ff,num_tmp_u&0x7ff);

}

/**
* @brief 绘制整数
* @param  graphic *图层信息
					name	图层名
					layer 图层数
					color 图形颜色
					font_size	字体大小
					width 线宽
					start_x	起始x坐标
					start_y 起始y坐标
					number 显示的数字
* @return NONE
*/
void Draw_Int(graphic_data_struct_t* graphic,
							const uint8_t *name,
							uint8_t layer,
							Graphic_Color color,
							uint16_t font_size,
							uint16_t width,
							uint16_t start_x,
							uint16_t start_y,
							int32_t number)
{
	int32_t num_tmp_i = number;
	uint32_t num_tmp_u=0x00;
	memcpy(&num_tmp_u,&num_tmp_i,4);
	Add_Graphic(graphic,name,layer,INT,color,font_size,0,width,start_x,start_y,(num_tmp_i>>22)&0x3ff,(num_tmp_i>>11)&0x7ff,num_tmp_i&0x7ff);
}



/**
* @brief 绘制字符
* @param  graphic *字符图层信息
					name	图层名
					layer 图层数
					color 图形颜色
					font_size	字体大小
					length	字符长度
					width 线宽
					start_x	起始x坐标
					start_y 起始y坐标
					character 字符串信息
* @return NONE
*/
void Draw_Char(ext_client_custom_character_t* graphic,
							const uint8_t *name,
							uint8_t layer,
							Graphic_Color color,
							uint16_t font_size,
							uint16_t length,
							uint16_t width,
							uint16_t start_x,
							uint16_t start_y,
							const uint8_t *character)
{
	Add_Graphic(&(graphic->grapic_data_struct),name,layer,CHAR,color,font_size,length,width,start_x,start_y,0,0,0);
	memcpy(graphic->data,character,length);
}



/**
* @brief 新增一个图层，填充图层信息
* @param  graphic *图层信息
					name	图层名
					layer 图层数
					type  图形类型
					color 图形颜色
					等等
* @return NONE
*/
void Add_Graphic(graphic_data_struct_t* graphic,
									const uint8_t* name,
									uint8_t layer,
									uint8_t type,
									uint8_t color,
									uint16_t start_angle,
									uint16_t end_angle,
									uint16_t width,
									uint16_t start_x,
									uint16_t start_y,
									uint16_t radius,
									uint16_t end_x,
									uint16_t end_y)
{
	graphic->color = color;
	for(uint8_t i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];
	graphic->layer = layer;
	graphic->graphic_tpye = type;
	graphic->operate_tpye = ADD;
	
	graphic->start_angle = start_angle;
	graphic->end_angle = end_angle;
	graphic->width = width;
	graphic->start_x = start_x;
	graphic->start_y = start_y;
	graphic->radius = radius;
	graphic->end_x = end_x;
	graphic->end_y = end_y;
}

/**
* @brief  修改一个图层，填充图层信息
* @param  graphic *图层信息
					name	图层名
					layer 图层数
					type  图形类型
					color 图形颜色
					等等
* @return NONE
*/
void Modify_Graphic(graphic_data_struct_t* graphic,
								  const uint8_t* name,
									uint8_t layer,
									uint8_t type,
									uint8_t color,
									uint16_t start_angle,
									uint16_t end_angle,
									uint16_t width,
									uint16_t start_x,
									uint16_t start_y,
									uint16_t radius,
									uint16_t end_x,
									uint16_t end_y)
{
	graphic->color = color;
	for(uint8_t i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	
	graphic->layer = layer;
	graphic->graphic_tpye = type;
	graphic->operate_tpye = MODIFY;
	
	graphic->start_angle = start_angle;
	graphic->end_angle = end_angle;
	graphic->width = width;
	graphic->start_x = start_x;
	graphic->start_y = start_y;
	graphic->radius = radius;
	graphic->end_x = end_x;
	graphic->end_y = end_y;
	
	/*因为不知道客户端修改未增加创建的图形会不会bug，因此在此先做判断，是否需要创建图形*/
}


/**
* @brief  删除一个图形，填充图层信息
* @param  graphic *图层信息
* @return NONE
*/
void Delete_Graphic(graphic_data_struct_t* graphic)
{
	graphic->operate_tpye = DELETE;
	/*直接删除即可*/
}


/**
* @brief 删除目标客户端某一图层的全部内容
* @param Interact_Target 目标客户端ID
					layer 删除的图层数
* @return NONE
*/
void Delete_Graphic_Layer(Interact_Target target,uint8_t layer)
{
	uint8_t data[17]={0};	/*6+2+9*/  	/*临时数组*/
	uint16_t data_length = LEN_INTERACT_delete_graphic;	 /*data字段的长度*/
	uint8_t CRC8=0x00;
	uint16_t CRC16=0x00;
	/*两个校验变量*/
	
	data[0] = 0xA5;
	data[1] = data_length>>8;
	data[2] = data_length;
	data[3] = 1;
	CRC8 = Get_CRC8_Check_Sum(data,4,0xff);
	data[4] = CRC8;
	/*SOF*/
	
	data[5] = ID_interactive_header_data>>8;
	data[6] = (uint8_t)ID_interactive_header_data;
	/*填充交互的cmd-ID*/
	
	interactive_data_header.data_cmd_id = INTERACT_ID_delete_graphic;
	interactive_data_header.receiver_ID = target;
	interactive_data_header.send_ID = self_id;
	memcpy(data+7,&interactive_data_header,6);
	/*填充data包头*/
	
	graphic_delete.operate_type = 1;
	graphic_delete.layer = layer;
	memcpy(data+13,&graphic_delete,2);
	/*填充data内容*/
	
	
	CRC16 = Get_CRC16_Check_Sum(data,15,0xffff);
	data[15] = CRC16>>8;
	data[16] = CRC16;
	/*获取CRC16值*/
	
	memcpy(&send_interactive_buff,data,17);
	Send_Interact_Data(&send_interactive_buff,17);
	/*通过串口发送*/
}



/**
* @brief 删除目标客户端所有图层的API函数
* @param Interact_Target 目标客户端ID
* @return NONE
*/
void Delete_All_Graphic_Layer(Interact_Target target)
{
	uint8_t data[17]={0};	/*6+2+9*/  	/*临时数组*/
	uint16_t data_length = LEN_INTERACT_delete_graphic;	 /*data字段的长度*/
	uint8_t CRC8=0x00;
	uint16_t CRC16=0x00;
	/*两个校验变量*/
	
	data[0] = 0xA5;
	data[1] = data_length>>8;
	data[2] = data_length;
	data[3] = 1;
	CRC8 = Get_CRC8_Check_Sum(data,4,0xff);
	data[4] = CRC8;
	/*SOF*/
	
	data[5] = ID_interactive_header_data>>8;
	data[6] = (uint8_t)ID_interactive_header_data;
	/*填充交互的cmd-ID*/
	
	interactive_data_header.data_cmd_id = INTERACT_ID_delete_graphic;
	interactive_data_header.receiver_ID = target;
	interactive_data_header.send_ID = self_id;
	memcpy(data+7,&interactive_data_header,6);
	/*填充data包头*/
	
	graphic_delete.operate_type = 2;
	graphic_delete.layer = 9;
	memcpy(data+13,&graphic_delete,2);
	/*填充data内容*/
	
	
	CRC16 = Get_CRC16_Check_Sum(data,15,0xffff);
	data[15] = CRC16>>8;
	data[16] = CRC16;
	/*获取CRC16值*/
	
	memcpy(&send_interactive_buff,data,17);
	Send_Interact_Data(&send_interactive_buff,17);
	/*通过串口发送*/
}



/**
* @brief 裁判系统交互数据的最终发送函数
* @param uint8_t * 发送的内容
* @return 发送结果
*/
bool Send_Interact_Data(robot_interactive_data_t *str,uint16_t length)
{
	if(length > 113)
		return false;
	/*数组溢出退出*/
	for(uint16_t i=0;i<length;i++)
	{
		UART5_SendChar(str->data[i]);   
	}
	return true;
}




/**
* @brief 修改已有的浮点数图形
* @param  graphic *图层信息
					number 显示的数字
* @return NONE
*/
void Modify_Float(graphic_data_struct_t* graphic,float number)
{
	float num_tmp_f = number;
	uint32_t num_tmp_u=0x00;
	memcpy(&num_tmp_u,&num_tmp_f,4);
	Modify_Graphic(graphic,graphic->graphic_name,graphic->layer,FLOAT,graphic->color,graphic->start_angle,graphic->end_angle,graphic->width,graphic->start_x,graphic->start_y,(num_tmp_u>>22)&0x3ff,(num_tmp_u>>11)&0x7ff,num_tmp_u&0x7ff);

}

/**
* @brief 修改已有的整数图形
* @param  graphic *图层信息
					number 显示的数字
* @return NONE
*/
void Modify_Int(graphic_data_struct_t* graphic,int32_t number)
{
	int32_t num_tmp_i = number;
	uint32_t num_tmp_u=0x00;
	memcpy(&num_tmp_u,&num_tmp_i,4);
	Modify_Graphic(graphic,graphic->graphic_name,graphic->layer,INT,graphic->color,graphic->start_angle,graphic->end_angle,graphic->width,graphic->start_x,graphic->start_y,(num_tmp_u>>22)&0x3ff,(num_tmp_u>>11)&0x7ff,num_tmp_u&0x7ff);
}



/**
* @brief 修改已有的字符图形
* @param  graphic *字符图层信息
					length	字符长度
					character 字符串信息
* @return NONE
*/
void Modify_Char(ext_client_custom_character_t* graphic,const uint8_t *character,uint16_t length)
{
	Modify_Graphic(&(graphic->grapic_data_struct),graphic->grapic_data_struct.graphic_name,\
			graphic->grapic_data_struct.layer,CHAR,graphic->grapic_data_struct.color,graphic->grapic_data_struct.start_angle,\
				graphic->grapic_data_struct.end_angle,graphic->grapic_data_struct.width,graphic->grapic_data_struct.start_x,\
					graphic->grapic_data_struct.start_y,0,0,0);
	memcpy(graphic->data,character,length);
}



















/**
* @brief 裁判系统处理任务
* @param void
* @return void
*	处理裁判系统是否离线等周期任务
*/
void Judge_Task(void)
{
	/*先判断是否连接到裁判系统*/


	
	/*未连接则一直等待，连接上之后，就向客户端发送绘制函数，初始化图层*/
	
	
	
	/*初始化图层后进入循环任务，根据实际需要更改显示数值。*/
	
	/*不建议改动非数字类图形，暂时也没有写这方面的函数，意义不大*/


}


