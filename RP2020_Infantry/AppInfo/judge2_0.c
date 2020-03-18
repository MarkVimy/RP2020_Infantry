#include "judge2_0.h"


/*���������Ϣ*/

Interact_Target self_id = RED_SENTRY;	 /*ȱʡΪ��ɫ�ڱ�����ʼ��Ӧ��ͨ������ϵͳ��ȡ*/

ext_game_status_t game_status;	/*����״̬*/

ext_game_result_t game_result;	/*�������*/

ext_game_robot_HP_t game_robot_HP;	/*Ѫ����Ϣ*/

ext_dart_status_t	dart_status;		/*����״̬*/

ext_event_data_t event_data;		/*�¼���Ϣ*/		

ext_dart_remaining_time_t dart_remaining_time;	/*����ʣ��ʱ��*/

ext_game_robot_status_t game_robot_status;		/*������״̬*/

ext_power_heat_data_t power_heat;			/*����������Ϣ*/

ext_game_robot_pos_t game_robot_pos;	/*������λ����Ϣ*/

ext_buff_t buff;	/*buff״̬*/

ext_robot_hurt_t robot_hurt;		/*�������˺���Ϣ*/

ext_shoot_data_t shoot_data;		/*�����Ϣ*/

ext_bullet_remaining_t bullet_remaining;	/*�ӵ�ʣ����*/

ext_student_interactive_header_data_t interactive_data_header;		/*�������ݵı���ͷ*/

robot_interactive_data_t send_interactive_buff;		/*Ҫ���͵Ľ������ݰ�*/

robot_interactive_data_t receive_interactive_buff;	/*���ڽ��յĽ������ݰ�*/

ext_client_custom_graphic_delete_t graphic_delete;	/*ɾ��ͼ��*/

graphic_data_struct_t graphic_data;		/*ͼ�������Ϣ*/

ext_client_custom_graphic_single_t draw_single_graphic;

ext_client_custom_graphic_double_t draw_double_grahpci;

ext_client_custom_graphic_five_t	draw_five_graphic;

ext_client_custom_graphic_seven_t	draw_seven_graphic;

ext_client_custom_character_t	draw_char_graphic;

uint16_t jdg_buf_length=0; 	/*����ϵͳ�Ľ������ݳ���*/
uint16_t cmd_id=0;	/*������*/
bool update_flag = false; 	/*���±�־λ*/


/**
* @brief ����ϵͳ���ݽ��պ���
* @param uint8_t * �������ݰ�ָ��
* @return void
*	��������һ����֡�����
*/
void Read_Judge_Info(uint8_t *rx_buf)
{
	uint8_t *p_buf = rx_buf;
	
	//�������������
	//����֡ͷ���ֵ�����
	if(rx_buf==NULL)
		return;
	if(rx_buf[0] == JUDGE_FRAME_HEADER)
	{
		//�ж�֡ͷ
		if(Verify_CRC8_Check_Sum(rx_buf,5) == true)
		{
			//�ж�֡ͷCRC8
			//ͳ��һ֡���ݳ���,����CR16У��
			jdg_buf_length = (*(rx_buf+1))+9;
			//
			if(Verify_CRC16_Check_Sum(rx_buf,jdg_buf_length) == true)
			{
				//�ж�֡βCRC16
				//ֻ��CRC16�ɹ�����true
				cmd_id=(((uint16_t)rx_buf[6]) << 8 | rx_buf[5]);
				//��ȡ����������
				update_flag=true;
				//���±�־λ
				switch(cmd_id)
				{
					case ID_game_state:          		//0x0001
						//����״̬����
						break;
					
					case ID_game_result:       //0x0002
						//����������ݣ�������������
						break;
					
					case ID_game_robot_HP:    				//0x0003
						//����������Ѫ�����ݣ�1Hz����
						break;
					
					case ID_dart_status:   //0x0004
						//���ڷ���״̬�����ڷ���ʱ����
						break;
					
					case ID_ICRA_buff_debuff_zone_status:  //0x0005
						//�˹�������ս���ӳ���ͷ���״̬��1Hz
						break;
					
					case ID_event_data:      		//0x0101
						//�����¼����ݣ�1Hz

						break;

					case ID_supply_projectile_action:      		//0x0102
						//���ز���վ������ʶ����

						break;
					
					case ID_referee_warning:      		//0x0104
						//���о������ݣ��������
						break;
					
					case ID_dart_remaining_time:      			//0x0105
						//���ڷ���ڵ���ʱ��1Hz
						break;
					
					case ID_game_robot_state:      	//0x0201
						//������״̬���ݣ�10Hz
						break;
					
					case ID_power_heat_data:      			//0x0202
						//ʵʱ�����������ݣ�50Hz
						break;
					
					case ID_game_robot_pos:      			//0x0203
						//������λ�����ݣ�10Hz
						break;	
					
					case ID_buff_musk:      			//0x0204
						//�������������ݣ�1Hz
						break;
					
					case ID_aerial_robot_energy:      			//0x0205
						//���л���������״̬���ݣ�10Hz��ֻ�п��л��������ط���
						break;

					case ID_robot_hurt:      			//0x0206
						//�˺�״̬���ݣ��˺���������
						break;

					case ID_shoot_data:      			//0x0207
						//ʵʱ������ݣ��ӵ��������
						break;	

					case ID_bullet_remaining:      			//0x0208
						//����ʣ�෢�����������л����ˣ��ڱ��������Լ�ICRA�����˷��ͣ�1Hz
						break;		

					case ID_rfid_status:      			//0x0209
						//������RFID״̬��1Hz
						break;	

					case ID_interactive_header_data:      			//0x0301
						//�����˽������ݣ����ͷ���������
						break;									
					default:break;
				}
				if(*(rx_buf+jdg_buf_length)==JUDGE_FRAME_HEADER)
				{
					//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
					p_buf = rx_buf+jdg_buf_length;
					Read_Judge_Info(p_buf);
				}
			}			
		}	
	}		
}

/**
* @brief ����һ������
* @param  graphic *ͼ����Ϣ
					name	ͼ����
					layer ͼ����
					color ͼ����ɫ
					width �߿�
					start_x ��ʼ��x����
					start_y ��ʼ��y����
					diagonal_x �Խǵ�x����
					diagonal_y �Խǵ�y����
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
* @brief ����һ��ֱ��
* @param  graphic *ͼ����Ϣ
					name	ͼ����
					layer ͼ����
					color ͼ����ɫ
					width �߿�
					start_x ��ʼ��x����
					start_y ��ʼ��y����
					end_x �յ�x����
					end_y �յ�y����
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
* @brief ����һ��Բ
* @param  graphic *ͼ����Ϣ
					name	ͼ����
					layer ͼ����
					color ͼ����ɫ
					width �߿�
					center_x Բ��x����
					center_y Բ��y����
					radius �뾶
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
* @brief ����һ����Բ
* @param  graphic *ͼ����Ϣ
					name	ͼ����
					layer ͼ����
					color ͼ����ɫ
					width �߿�
					center_x Բ��x����
					center_y Բ��y����
					axis_x x����
					axis_y y����
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
* @brief ����һ��Բ��
* @param  graphic *ͼ����Ϣ
					name	ͼ����
					layer ͼ����
					color ͼ����ɫ
					start_angle ��ʼ�Ƕ�
					end_angle 	��ֹ�Ƕ�
					width �߿�
					center_x Բ��x����
					center_y Բ��y����
					axis_x x����
					axis_y y����
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
* @brief ���Ƹ�����
* @param  graphic *ͼ����Ϣ
					name	ͼ����
					layer ͼ����
					color ͼ����ɫ
					font_size	�����С
					accuracy С����λ��
					width �߿�
					start_x	��ʼx����
					start_y ��ʼy����
					number ��ʾ������
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
* @brief ��������
* @param  graphic *ͼ����Ϣ
					name	ͼ����
					layer ͼ����
					color ͼ����ɫ
					font_size	�����С
					width �߿�
					start_x	��ʼx����
					start_y ��ʼy����
					number ��ʾ������
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
* @brief �����ַ�
* @param  graphic *�ַ�ͼ����Ϣ
					name	ͼ����
					layer ͼ����
					color ͼ����ɫ
					font_size	�����С
					length	�ַ�����
					width �߿�
					start_x	��ʼx����
					start_y ��ʼy����
					character �ַ�����Ϣ
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
* @brief ����һ��ͼ�㣬���ͼ����Ϣ
* @param  graphic *ͼ����Ϣ
					name	ͼ����
					layer ͼ����
					type  ͼ������
					color ͼ����ɫ
					�ȵ�
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
* @brief  �޸�һ��ͼ�㣬���ͼ����Ϣ
* @param  graphic *ͼ����Ϣ
					name	ͼ����
					layer ͼ����
					type  ͼ������
					color ͼ����ɫ
					�ȵ�
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
	
	/*��Ϊ��֪���ͻ����޸�δ���Ӵ�����ͼ�λ᲻��bug������ڴ������жϣ��Ƿ���Ҫ����ͼ��*/
}


/**
* @brief  ɾ��һ��ͼ�Σ����ͼ����Ϣ
* @param  graphic *ͼ����Ϣ
* @return NONE
*/
void Delete_Graphic(graphic_data_struct_t* graphic)
{
	graphic->operate_tpye = DELETE;
	/*ֱ��ɾ������*/
}


/**
* @brief ɾ��Ŀ��ͻ���ĳһͼ���ȫ������
* @param Interact_Target Ŀ��ͻ���ID
					layer ɾ����ͼ����
* @return NONE
*/
void Delete_Graphic_Layer(Interact_Target target,uint8_t layer)
{
	uint8_t data[17]={0};	/*6+2+9*/  	/*��ʱ����*/
	uint16_t data_length = LEN_INTERACT_delete_graphic;	 /*data�ֶεĳ���*/
	uint8_t CRC8=0x00;
	uint16_t CRC16=0x00;
	/*����У�����*/
	
	data[0] = 0xA5;
	data[1] = data_length>>8;
	data[2] = data_length;
	data[3] = 1;
	CRC8 = Get_CRC8_Check_Sum(data,4,0xff);
	data[4] = CRC8;
	/*SOF*/
	
	data[5] = ID_interactive_header_data>>8;
	data[6] = (uint8_t)ID_interactive_header_data;
	/*��佻����cmd-ID*/
	
	interactive_data_header.data_cmd_id = INTERACT_ID_delete_graphic;
	interactive_data_header.receiver_ID = target;
	interactive_data_header.send_ID = self_id;
	memcpy(data+7,&interactive_data_header,6);
	/*���data��ͷ*/
	
	graphic_delete.operate_type = 1;
	graphic_delete.layer = layer;
	memcpy(data+13,&graphic_delete,2);
	/*���data����*/
	
	
	CRC16 = Get_CRC16_Check_Sum(data,15,0xffff);
	data[15] = CRC16>>8;
	data[16] = CRC16;
	/*��ȡCRC16ֵ*/
	
	memcpy(&send_interactive_buff,data,17);
	Send_Interact_Data(&send_interactive_buff,17);
	/*ͨ�����ڷ���*/
}



/**
* @brief ɾ��Ŀ��ͻ�������ͼ���API����
* @param Interact_Target Ŀ��ͻ���ID
* @return NONE
*/
void Delete_All_Graphic_Layer(Interact_Target target)
{
	uint8_t data[17]={0};	/*6+2+9*/  	/*��ʱ����*/
	uint16_t data_length = LEN_INTERACT_delete_graphic;	 /*data�ֶεĳ���*/
	uint8_t CRC8=0x00;
	uint16_t CRC16=0x00;
	/*����У�����*/
	
	data[0] = 0xA5;
	data[1] = data_length>>8;
	data[2] = data_length;
	data[3] = 1;
	CRC8 = Get_CRC8_Check_Sum(data,4,0xff);
	data[4] = CRC8;
	/*SOF*/
	
	data[5] = ID_interactive_header_data>>8;
	data[6] = (uint8_t)ID_interactive_header_data;
	/*��佻����cmd-ID*/
	
	interactive_data_header.data_cmd_id = INTERACT_ID_delete_graphic;
	interactive_data_header.receiver_ID = target;
	interactive_data_header.send_ID = self_id;
	memcpy(data+7,&interactive_data_header,6);
	/*���data��ͷ*/
	
	graphic_delete.operate_type = 2;
	graphic_delete.layer = 9;
	memcpy(data+13,&graphic_delete,2);
	/*���data����*/
	
	
	CRC16 = Get_CRC16_Check_Sum(data,15,0xffff);
	data[15] = CRC16>>8;
	data[16] = CRC16;
	/*��ȡCRC16ֵ*/
	
	memcpy(&send_interactive_buff,data,17);
	Send_Interact_Data(&send_interactive_buff,17);
	/*ͨ�����ڷ���*/
}



/**
* @brief ����ϵͳ�������ݵ����շ��ͺ���
* @param uint8_t * ���͵�����
* @return ���ͽ��
*/
bool Send_Interact_Data(robot_interactive_data_t *str,uint16_t length)
{
	if(length > 113)
		return false;
	/*��������˳�*/
	for(uint16_t i=0;i<length;i++)
	{
		UART5_SendChar(str->data[i]);   
	}
	return true;
}




/**
* @brief �޸����еĸ�����ͼ��
* @param  graphic *ͼ����Ϣ
					number ��ʾ������
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
* @brief �޸����е�����ͼ��
* @param  graphic *ͼ����Ϣ
					number ��ʾ������
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
* @brief �޸����е��ַ�ͼ��
* @param  graphic *�ַ�ͼ����Ϣ
					length	�ַ�����
					character �ַ�����Ϣ
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
* @brief ����ϵͳ��������
* @param void
* @return void
*	�������ϵͳ�Ƿ����ߵ���������
*/
void Judge_Task(void)
{
	/*���ж��Ƿ����ӵ�����ϵͳ*/


	
	/*δ������һֱ�ȴ���������֮�󣬾���ͻ��˷��ͻ��ƺ�������ʼ��ͼ��*/
	
	
	
	/*��ʼ��ͼ������ѭ�����񣬸���ʵ����Ҫ������ʾ��ֵ��*/
	
	/*������Ķ���������ͼ�Σ���ʱҲû��д�ⷽ��ĺ��������岻��*/


}


