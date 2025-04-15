	/**@file  dr16.c
	* @brief    �豸��
	* @details  ��Ҫ�����������ڹ��������ṩ���ڳ�ʼ�����û��ص��ض���
	* @author      RyanJiao  any question please send mail to 1095981200@qq.com
	* @date        2021-10-9
	* @version     V1.0
	* @copyright    Copyright (c) 2021-2121  �й���ҵ��ѧCUBOTս��
	**********************************************************************************
	* @attention
	* Ӳ��ƽ̨: STM32H750VBT \n
	* SDK�汾��-++++
	* @par �޸���־:
	* <table>
	* <tr><th>Date        <th>Version  <th>Author    <th>Description
	* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>������ʼ�汾
	* </table>
	*
	**********************************************************************************
	 ==============================================================================
														How to use this driver  
	 ==============================================================================
	 

		********************************************************************************
		* @attention
		* Ӳ��ƽ̨: STM32H750VBT \n
		* SDK�汾��-++++
		* if you had modified this file, please make sure your code does not have many 
		* bugs, update the version NO., write dowm your name and the date, the most
		* important is make sure the users will have clear and definite understanding 
		* through your new brief.
		********************************************************************************
	*/

	#include "dr16.h"
	#include "check.h"
	#include "hardware_config.h"
	
	#include "control_logic.h"

	#include "check.h"
	#include "brain.h"
	#include "reset.h"
		#include "supercap.h"
	#define myABS(x) ( (x)>0?(x):-(x) )
	RC_Ctrl rc_Ctrl={
		.isUnpackaging = 0,
		.isOnline = 0
		
	};

	/**
		* @brief  ����dr16�Ľ��ջ���������
		*/
	uint8_t DR16_recData[DR16_rxBufferLengh];


	/**
		* @brief  ����dr16���ڻ��������ݽṹ
		*/
	UART_RxBuffer uart1_buffer={
		.Data = DR16_recData,
		.DataSize = DR16_rxBufferLengh
	};

	/**
  * @brief  ��������������Ƿ�Ϊ��Ч����
	* @param[in] RC_Ctl  ���ջ����������׵�ַ
  * @retval    RC_Ctl  ���ջ����������׵�ַ
  */
void PC_keybroad_filter(RC_Ctrl* RC_Ctl);
void DR16_DataUnpack(RC_Ctrl* rc_ctrl, uint8_t * recBuffer, uint16_t len);
//void KeyBoard_DataUnpack(RC_Ctrl* rc_ctrl, uint8_t * recBuffer, uint16_t len , Trace* Info_Vision ,Supercap* Cap);
	/**
		* @brief  ��ʼ�����ջ��������͵����ݣ��������Ͱ�����Ϣ����
		*/

	void DR16Init(RC_Ctrl* RC_Ctl)
	{
		RC_Ctl->rc.ch0=1024;
		RC_Ctl->rc.ch1=1024;
		RC_Ctl->rc.ch2=1024;
		RC_Ctl->rc.ch3=1024;
		RC_Ctl->rc.s1=3;
		RC_Ctl->rc.s2=3;
		RC_Ctl->rc.sw=1024;
		RC_Ctl->mouse.x=0;
		RC_Ctl->mouse.y=0;
		RC_Ctl->mouse.z=0;		
		RC_Ctl->key_Q_flag=0;
		RC_Ctl->key_E_flag=0;    //< �ϵ�ص���
		RC_Ctl->key_R_flag=0;
		RC_Ctl->key_F_flag=0;
		RC_Ctl->key_G_flag=0;
		RC_Ctl->key_Z_flag=0;
		RC_Ctl->key_X_flag=0;
		RC_Ctl->key_C_flag=0;
		RC_Ctl->key_V_flag=0;
		RC_Ctl->key_B_flag=0;
		RC_Ctl->key_ctrl_flag=0;
		RC_Ctl->Chassis_Y_Integ=0;//б�»��ֱ���
		RC_Ctl->Chassis_X_Integ=0;
		RC_Ctl->ShootNumber=1;
		RC_Ctl->Cruise_Mode = 0;
	}


	/**
		* @brief  ����dr16�Ľ��ջ���������
		*/
	uint8_t DR16_callback(uint8_t * recBuffer, uint16_t len)
	{

		DR16_DataUnpack(&rc_Ctrl, recBuffer, len);  //< callback�����ɸ�ʽ����
//		KeyBoard_DataUnpack(&rc_Ctrl, recBuffer, len , &Vision_Info  ,&super_cap);
//		rc_Ctrl.onlineCheckCnt = 0;
//		if (tim14_FPS.Receiver_FPS>1) check_robot_state.usart_state.Check_receiver = 0;
		return 0;
	}


	/**
		* @brief  ����dr16�Ľ��ջ���������, ����ȫ�ֱ���rc_Ctrl��ֵ���Թ�������������
		*/
	void DR16_DataUnpack(RC_Ctrl* rc_ctrl, uint8_t * recBuffer, uint16_t len)
	{ 
		
		rc_ctrl->isUnpackaging = 1;					//< �����ڼ䲻�����ȡ����
		uint8_t correct_num=0;	
		correct_num=0;
		if(((recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff)<=1684 && ((recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff)>=364)
			correct_num++;
		if((((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff)<=1684 && (((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff)>=364)
			correct_num++;
		if((((recBuffer[2] >> 6) | (recBuffer[3] << 2) |(recBuffer[4] << 10)) & 0x07ff)<=1684 && (((recBuffer[2] >> 6) | (recBuffer[3] << 2) |(recBuffer[4] << 10)) & 0x07ff)>=364)
			correct_num++;
		if((((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff)<=1684 && (((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff)>=364)
			correct_num++;
		if((((recBuffer[5] >> 4)& 0x000C) >> 2)==1 || (((recBuffer[5] >> 4)& 0x000C) >> 2)==2 || (((recBuffer[5] >> 4)& 0x000C) >> 2)==3)
			correct_num++;
		if(((recBuffer[5] >> 4)& 0x0003)==1 || ((recBuffer[5] >> 4)& 0x0003)==2 || ((recBuffer[5] >> 4)& 0x0003)==3)
			correct_num++;
		if(correct_num==6)																																												//< ������������֤ 
		{  	
		check_robot_state.Check_Usart.Check_receiver_cnt=0;
		tim14_FPS.Receiver_cnt++;
		rc_ctrl->rc.s1_last=rc_ctrl->rc.s1;
			rc_ctrl->rc.ch0 = (recBuffer[0]| (recBuffer[1] << 8)) & 0x07ff; 																					//< Channel 0   ��8λ���3λ
			rc_ctrl->rc.ch1 = ((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff; 																	//< Channel 1   ��5λ���6λ
			rc_ctrl->rc.ch2 = ((recBuffer[2] >> 6) | (recBuffer[3] << 2) |(recBuffer[4] << 10)) & 0x07ff; 						//< Channel 2
			rc_ctrl->rc.ch3 = ((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff; 																	//< Channel 3
			rc_ctrl->rc.s1 = ((recBuffer[5] >> 4)& 0x000C) >> 2; 																											//!< Switch left
			rc_ctrl->rc.s2 = ((recBuffer[5] >> 4)& 0x0003); 				//��ȡ����5��3��4λ																									//!< Switch right
			rc_ctrl->rc.sw=(uint16_t)(recBuffer[16]|(recBuffer[17]<<8))&0x7ff; 
				
			if((rc_ctrl->rc.ch0>1020)&&(rc_ctrl->rc.ch0<1028))          //ң������Ʈ
				rc_ctrl->rc.ch0=1024;
			if((rc_ctrl->rc.ch1>1020)&&(rc_ctrl->rc.ch1<1028))
				rc_ctrl->rc.ch1=1024;
			if((rc_ctrl->rc.ch2>1020)&&(rc_ctrl->rc.ch2<1028))
				rc_ctrl->rc.ch2=1024;
			if((rc_ctrl->rc.ch3>1020)&&(rc_ctrl->rc.ch3<1028))
				rc_ctrl->rc.ch3=1024;
			
				/***********����ӳ��*************/
			rc_ctrl->mouse.x = recBuffer[6]  | (recBuffer[7] << 8);                       //< Mouse X axis   
			rc_ctrl->mouse.y = recBuffer[8]  | (recBuffer[9] << 8);                       //< Mouse Y axis     
			rc_ctrl->mouse.z = recBuffer[10] | (recBuffer[11] << 8);                      //< Mouse Z axis   
			rc_ctrl->mouse.press_l = recBuffer[12];                                       //< Mouse Left Is Press ?   
			rc_ctrl->mouse.press_r = recBuffer[13];                                       //< Mouse Right Is Press ? 
							
			if(rc_ctrl->mouse.x>25000)   rc_ctrl->mouse.x=25000;     																												//< �޷�
			if(rc_ctrl->mouse.x<-25000)  rc_ctrl->mouse.x=-25000;	
			if(rc_ctrl->mouse.y>25000)   rc_ctrl->mouse.y=25000;
			if(rc_ctrl->mouse.y<-25000)  rc_ctrl->mouse.y=-25000;
				
			rc_ctrl->keyboard.v = recBuffer[14]| (recBuffer[15] << 8);  									//< ��16������ֵ   

			rc_ctrl->key_W=recBuffer[14]&0x01;	
			rc_ctrl->key_S=(recBuffer[14]>>1)&0x01;					
			rc_ctrl->key_A=(recBuffer[14]>>2)&0x01;
			rc_ctrl->key_D=(recBuffer[14]>>3)&0x01;					
			rc_ctrl->key_B=(recBuffer[15]>>7)&0x01;
			rc_ctrl->key_V=(recBuffer[15]>>6)&0x01;				
			rc_ctrl->key_C=(recBuffer[15]>>5)&0x01;
			rc_ctrl->key_X=(recBuffer[15]>>4)&0x01;					
			rc_ctrl->key_Z=(recBuffer[15]>>3)&0x01;					
			rc_ctrl->key_G=(recBuffer[15]>>2)&0x01;			
			rc_ctrl->key_F=(recBuffer[15]>>1)&0x01;
			rc_ctrl->key_R=(recBuffer[15])&0x01;													
			rc_ctrl->key_E=(recBuffer[14]>>7)&0x01;
			rc_ctrl->key_Q=(recBuffer[14]>>6)&0x01;
			rc_ctrl->key_ctrl=(recBuffer[14]>>5)&0x01;
			rc_ctrl->key_shift=(recBuffer[14]>>4)&0x01;
			PC_keybroad_filter(rc_ctrl);				//< ����

					}
		else{}
		rc_ctrl->isUnpackaging = 0;					//< ������ɱ�־λ�������ȡ
	}

//	void KeyBoard_DataUnpack(RC_Ctrl* rc_ctrl, uint8_t * recBuffer, uint16_t len, Trace* Info_Vision,  Supercap* Cap)
//		{


//		

//		
//		if(rc_ctrl->key_shift==1)
//		{
//			Cap->cap_state.Supercap_Mode = 1;
//		}
//		else
//		{
//			Cap->cap_state.Supercap_Mode = 0;
//		}
//		
///*********************��������*****************************/		 
//		if(rc_ctrl->rc.sw>1400||rc_ctrl->key_R_flag%2==1)
//		{ 	
//			Cap->cap_state.Supercap_Mode = 1;
////			Swerve->Roll_Flag=1;
////			Swerve->Chassis_State = 0;
//		}
//		else if((rc_ctrl->rc.sw<1400&&rc_ctrl->rc.sw>600)||rc_ctrl->key_R_flag%2==0)
//		{
//			Cap->cap_state.Supercap_Mode = 0;
////			Swerve->Roll_Flag=0;
////			Swerve->Chassis_State = 1;
//		}	
///**********************�Ӿ�ģʽѡ��***************************/		 
//										
//		if(rc_ctrl->key_G_flag==1||rc_ctrl->rc.s2==1) 
//		{ 
//			Info_Vision->Hit_Mode=1;							//����ģʽ
//		}
//		else if(rc_ctrl->mouse.press_r_flag==1||rc_ctrl->rc.s2==2)
//		{
//			Info_Vision->Hit_Mode=2;							//����ģʽ
//		}
//		
//		if(rc_ctrl->rc.s1==2)Info_Vision->Lidar_Mode=1;
//		else Info_Vision->Lidar_Mode=0;
///**********���Rû��������ر����飬Gû�����¹رմ���******/		 
//		if(rc_ctrl->mouse.press_r_flag!=1&&rc_ctrl->rc.s2!=2)
//		 {	
//			if(Info_Vision->Hit_Mode!=1)		
//			{					
//				Info_Vision->Hit_Mode=0;				 
//			}		
//		 }
//		if(rc_ctrl->key_G_flag!=1&&rc_ctrl->rc.s2!=1)
//		 {	
//			if(Info_Vision->Hit_Mode!=2)		
//			{					
//				Info_Vision->Hit_Mode=0;
//			}						
//		 }

///*---------------------------------��������ʼ---------------------------------*/
////		myABS(rc_ctrl->rc.ch1-1024)>10? (chassis->Movement.DeltaVx=(rc_ctrl->rc.ch1-1024)-chassis->Movement.Vx) : (chassis->Movement.Vx);//f0
////		myABS(rc_ctrl->rc.ch0-1024)>10? (chassis->Movement.DeltaVy=(rc_ctrl->rc.ch0-1024)-chassis->Movement.Vy) : (chassis->Movement.Vy);//r0
//			
//	//��������ƽ��״̬
////		if(chassis->Movement.Vx>=0)
////		{
////			chassis->Movement.DeltaVx>0?  (chassis->Movement.Vx+=chassis->Movement.DeltaVx*vx_Sence):(chassis->Movement.Vx+=chassis->Movement.DeltaVx*vx_Sence*2);
////		}
////		else
////		{
////			chassis->Movement.DeltaVx>0?  (chassis->Movement.Vx+=chassis->Movement.DeltaVx*vx_Sence*2):(chassis->Movement.Vx+=chassis->Movement.DeltaVx*vx_Sence);
////		}

//////								
//////��������ǰ��״̬
////		if(chassis->Movement.Vy>=0)   
////		{
////			chassis->Movement.DeltaVy>0?  (chassis->Movement.Vy+=chassis->Movement.DeltaVy*vy_Sence):(chassis->Movement.Vy+=chassis->Movement.DeltaVy*vy_Sence*2);//ǰ����������ʱ ��С��ֵ������ 
////		}
////		else               //�������ڵ���״̬
////		{
////			chassis->Movement.DeltaVy>0?  (chassis->Movement.Vy+=chassis->Movement.DeltaVy*vy_Sence*2):(chassis->Movement.Vy+=chassis->Movement.DeltaVy*vy_Sence);
////		}		
////		
///*--------------------���Ҵ����Ծ�ļ�����ƿ�ʼ--------------------*/				
//		if((rc_ctrl->rc.ch1-1024)>300)   rc_ctrl->key_W=1;
//		else if((rc_ctrl->rc.ch1-1024)<-300) rc_ctrl->key_S=1;
//		
//		if((rc_ctrl->rc.ch0-1024)>300)   rc_ctrl->key_D=1;
//		else if((rc_ctrl->rc.ch0-1024)<-300)  rc_ctrl->key_A=1;

//		 /*ɲ��*/		
//		 if((rc_ctrl->key_S-rc_ctrl->key_W)==0)			
//		 {
//			 if(rc_ctrl->Chassis_Y_Integ>0)    //ǰ������
//			 {

//				 rc_ctrl->Chassis_Y_Integ-=ACE_SHACHE*15;  //Ace_SenseΪ���ٶ�������
//				 if(rc_ctrl->Chassis_Y_Integ<0)  rc_ctrl->Chassis_Y_Integ=0;
//			 }   
//			 else                        //���˼���
//			 {
//				 rc_ctrl->Chassis_Y_Integ+=ACE_SHACHE*15; 
//				 if(rc_ctrl->Chassis_Y_Integ>0)  rc_ctrl->Chassis_Y_Integ=0;					 }						
// 
//		 }
//		 else              //����
//		 { 
//					 
//			 rc_ctrl->Chassis_Y_Integ+=ACE_SENSE*0.3f*(rc_ctrl->key_S-rc_ctrl->key_W);//������ʻ  //Shoot_Info.bullet_cap_status=1;//������ 							 

//			 if(rc_ctrl->Chassis_Y_Integ>3)          rc_ctrl->Chassis_Y_Integ=3;
//			 else if (rc_ctrl->Chassis_Y_Integ<-3)   rc_ctrl->Chassis_Y_Integ=-3;									 
//		 }

//		/*    ���Ҽ��ټ��ٴ���                 */	 
//		{
//			if((rc_ctrl->key_D-rc_ctrl->key_A)==0)
//			{
//				 if(rc_ctrl->Chassis_X_Integ>0)    //�Ҽ���
//				 {
//							 rc_ctrl->Chassis_X_Integ-=ACE_SHACHE*15;
//							 if(rc_ctrl->Chassis_X_Integ<0)  rc_ctrl->Chassis_X_Integ=0;
//				 }
//				 else                       //�����
//				 {
//							 rc_ctrl->Chassis_X_Integ+=ACE_SHACHE*15;
//							 if(rc_ctrl->Chassis_X_Integ>0)  rc_ctrl->Chassis_X_Integ=0;
//				 }					 
//			}
//			else                               //����
//			{		
//				rc_ctrl->Chassis_X_Integ -= ACE_SENSE*0.3f*(rc_ctrl->key_A-rc_ctrl->key_D);		//������ʻ  //Shoot_Info.bullet_cap_status=1;//������ 				
//				if(rc_ctrl->Chassis_X_Integ>3)          rc_ctrl->Chassis_X_Integ=3;
//				else if (rc_ctrl->Chassis_X_Integ<-3)         rc_ctrl->Chassis_X_Integ=-3;											
//			}
//		}

///*---------------------------------���ոǿ���---------------------------------*/		
//		
////		if(((rc_ctrl->key_E==1)||(rc_ctrl->rc.s2==1))&&Holder.Motors6020.motor[1].Data.Ecd>=4500)
////		{
////			 shoot->BulletCap_Status_Flag=1;					/*�򿪵��ָ�*/
////		}
////		else if(rc_ctrl->key_E==0||rc_ctrl->rc.s2==3||Holder.Motors6020.motor[1].Data.Ecd<4500)
////		{
////			 shoot->BulletCap_Status_Flag=0;			
////		}
///*--------------------------Supercap_Charge_mode����ֵ���͸��˵���---------------------------------------*/		
//		
//		if(rc_ctrl->rc.s2==2)
//			//Cap->cap_state.Supercap_Charge_mode = 1;
//		if(rc_ctrl->rc.s2==3)
//			//Cap->cap_state.Supercap_Charge_mode = 0;
//			
///*--------------------------û����������û��---------------------------------------*/	
//		
//		if((rc_ctrl->key_X_flag%2==1)&&(rc_ctrl->key_ctrl_flag == 1)&&(rc_ctrl->key_V_flag%2!=1))
//		{
//			rc_ctrl->ShootNumberCut++; 
//			if(rc_ctrl->ShootNumberCut==1)
//			{
//				rc_ctrl->ShootNumber++;  					
//				if(rc_ctrl->ShootNumber>3)
//				{ 
//					rc_ctrl->ShootNumber = 1;
//				}		
//				Info_Vision->RobotModeFlag=1;
//			}
//			
//			if(rc_ctrl->ShootNumberCut>1)rc_ctrl->ShootNumberCut=1;				
//		}
//		
//		else if((rc_ctrl->key_X_flag%2!=1)||(rc_ctrl->key_ctrl_flag != 1))
//		{
//			Info_Vision->RobotModeFlag=0;
//			rc_ctrl->ShootNumberCut=0;
//		}

///*--------------------------��λ������λ�����ͼ������������---------------------------------------*/				
//		if((rc_ctrl->key_F_flag%2==1)&&(rc_ctrl->key_ctrl_flag == 1)&&(rc_ctrl->key_X_flag%2==1)&&(rc_ctrl->key_V_flag%2==1))
//		{
//			Brain_RobotToComputerCmd(6);
//			rc_ctrl->key_F_flag=0;
//			rc_ctrl->key_X_flag=0;
//			rc_ctrl->key_V_flag=0;
//		}
///*--------------------------��λ������λ�����ͻ����˴�����������---------------------------------------*/				
//		if((rc_ctrl->key_F_flag%2==1)&&(rc_ctrl->key_ctrl_flag != 1)&&(rc_ctrl->key_X_flag%2==1)&&(rc_ctrl->key_V_flag%2==1))
//		{
//			Brain_RobotToBrainCmd(5);
//			rc_ctrl->key_F_flag=0;
//			rc_ctrl->key_X_flag=0;
//			rc_ctrl->key_V_flag=0;
//		}
///*--------------------------û����������û��---------------------------------------*/				
//		
//		if(rc_ctrl->key_V_flag%2==1)	Info_Vision->buchang=1;		
//		else if(rc_ctrl->key_V_flag%2==0)	Info_Vision->buchang=0;
//			
//		if(rc_ctrl->key_shift==1||rc_ctrl->rc.sw>1400)	Cap->cap_state.Supercap_Mode = 1;
//		else	Cap->cap_state.Supercap_Mode = 0;
//		 
///*--------------------------û����������û��---------------------------------------*/										 
//		if(rc_ctrl->key_R_flag == 1&&rc_ctrl->key_ctrl_flag == 1){
//		 rc_ctrl->c +=1;
//		}
//		if(rc_ctrl->key_G_flag == 1&&rc_ctrl->key_ctrl_flag == 1){
//		 rc_ctrl->d +=1;
//		}



//		if(lidar_mode==2)lidar_mode=0;
//		/*����*/
//		if((rc_ctrl->rc.s1==2&&rc_ctrl->rc.s1_last==3)||(rc_ctrl->key_Q_flag==1&&rc_ctrl->key_ctrl_flag==0&&rc_ctrl->key_E_flag==0)){
//			rc_ctrl->OneShoot +=1;
//		}
////		rc_ctrl->rc.s2_last=rc_ctrl->rc.s2;
////		rc_ctrl->rc.s1_last=rc_ctrl->rc.s1;
//		
//		/*������*/
//		if(rc_ctrl->key_Q_flag==1&&rc_ctrl->key_ctrl_flag==1){
//		rc_ctrl->ThreeShoot+=3;
//		}
//		rc_ctrl->rc.s2_last=rc_ctrl->rc.s2;
////		rc_ctrl->rc.s1_last=rc_ctrl->rc.s1;
///*--------------------------������ģʽ---------------------------------------*/				
////		if((rc_ctrl->key_C_flag%2 == 1)&&(rc_ctrl->key_ctrl_flag != 1))
////		{
////			shoot->SuperHeatMode = 1;
////		}
////		else if((rc_ctrl->key_C_flag%2 == 0)||(rc_ctrl->key_ctrl_flag == 1))
////		{
////		  shoot->SuperHeatMode = 0;
////		}
///*--------------------------�����λ---------------------------------------*/	

//		if((rc_ctrl->key_shift==1)&&(rc_ctrl->key_B_flag%2 == 1)&&(rc_ctrl->key_ctrl_flag==1))	
//		{
//			SoftResetFlag=1;
//		}
//	
///*--------------------------û����---------------------------------------*/				

//		if(rc_ctrl->key_B_flag%2 == 1) //�����ڻ���ģʽ
//		{    
//			rc_ctrl->Cruise_Mode =1;
////			Swerve->Roll_Flag1=1;
//		}					

//		else if(rc_ctrl->key_B_flag%2 == 0)
//		{
////		    Swerve->Roll_Flag1=0;
//			rc_ctrl->Cruise_Mode = 0 ;
//		}
//	}
/**
	* @brief  ��������������Ƿ�Ϊ��Ч����
	*/
void PC_keybroad_filter(RC_Ctrl* RC_Ctl)
{
	static uint16_t key_W_cnt,key_A_cnt,key_S_cnt,key_D_cnt,key_ctrl_cnt,
								 key_shift_cnt,mouse_press_l_cnt,mouse_press_r_cnt,
								 key_C_cnt,key_F_cnt,key_G_cnt,key_Q_cnt,key_E_cnt,
									key_Z_cnt,key_V_cnt,key_X_cnt,key_B_cnt,key_R_cnt;
		/*   ֧�������� W A S D   */
		//< key_W
	if(RC_Ctl->key_W==1) 
	{
		key_W_cnt++;
		if(key_W_cnt==Key_Filter_Num)	
		{
			RC_Ctl->key_W_flag=1;	
		}	 
	}   
	else
	{
		RC_Ctl->key_W_flag=0;	
		key_W_cnt=0;	
	}	
			//key_A
	if(RC_Ctl->key_A==1) 
	{
		key_A_cnt++;
		if(key_A_cnt==Key_Filter_Num)	
		{
			RC_Ctl->key_A_flag=1;
		}	
	}

	else
	{	
		key_A_cnt=0;	
		RC_Ctl->key_A_flag=0;
	}
	//key_S
	if(RC_Ctl->key_S==1) 
	{
		key_S_cnt++;
		if(key_S_cnt==Key_Filter_Num)	
		{
			RC_Ctl->key_S_flag=1;	
		}			
	}	
	else
	{
		key_S_cnt=0;
		RC_Ctl->key_S_flag=0;
	}		
	//key_D
	if(RC_Ctl->key_D==1) 
	{
		key_D_cnt++;
		if(key_D_cnt==Key_Filter_Num)	
		{
			RC_Ctl->key_D_flag=1;			
		}	
	}		
	else
	{
		key_D_cnt=0;
		RC_Ctl->key_D_flag=0;
	}
	
		//key_B
	if(RC_Ctl->key_B==1) 
	{
		key_B_cnt++;
		if(key_B_cnt==Key_Filter_Num)	
		{
			RC_Ctl->key_B_flag=1;
		}	
	}	
	else
	{
		key_B_cnt=0;
		RC_Ctl->key_B_flag=0;
	}
	//key_C
	if(RC_Ctl->key_C==1) 
	{
		key_C_cnt++;
		if(key_C_cnt==3)	
		{
			RC_Ctl->key_C_flag=1;
		}	 
	}  
	else
	{
		key_C_cnt=0;
		RC_Ctl->key_C_flag=0;
	}	

		//key_R
	if(RC_Ctl->key_R==1) 
	{
		key_R_cnt++;
		if(key_R_cnt==Key_Filter_Num)	
		{
			RC_Ctl->key_R_flag++;			
		}	 
	}   else
	 {
		key_R_cnt=0;	
//		RC_Ctl.key_R_flag=0;
	 }
	//key_F
	if(RC_Ctl->key_F==1) 
	{
		key_F_cnt++;
		if(key_F_cnt==Key_Filter_Num)	
		{
			RC_Ctl->key_F_flag++;	
  if(RC_Ctl->key_F_flag>1) 
			RC_Ctl->key_F_flag=0;
		}	
	}	
	else
	{
		key_F_cnt=0;
	}
		//key_X
	if(RC_Ctl->key_X==1) 
	{
		key_X_cnt++;
		if(key_X_cnt==3)	
		{
		RC_Ctl->key_X_flag=1;			
//				RC_Ctl->key_X_flag++;
//				if(RC_Ctl->key_X_flag>1) 
//					RC_Ctl->key_X_flag=1;
		}	 
	}	
	else
	{
		key_X_cnt=0;
		RC_Ctl->key_X_flag=0;
	}
	
	//key_G
	if(RC_Ctl->key_G==1) 
	{
		key_G_cnt++;
		if(key_G_cnt==Key_Filter_Num)	
		{
			RC_Ctl->key_G_flag++;
		}	
	}	
	else
	{
		key_G_cnt=0;
//		RC_Ctl.key_G_flag=0;
	}	

	
	//key_Q
	if(RC_Ctl->key_Q==1) 
	{
		key_Q_cnt++;
		if(key_Q_cnt==Key_Filter_Num)	
		{
		RC_Ctl->key_Q_flag=1;			
		}	
		else
	 {
		RC_Ctl->key_Q_flag=0;	
	 }	
	}	
	else
	{
		key_Q_cnt=0;
		RC_Ctl->key_Q_flag=0;
	}
	//key_E
	if(RC_Ctl->key_E==1) 
	 {
		key_E_cnt++;
		if(key_E_cnt==Key_Filter_Num)	
		{
			RC_Ctl->key_E_flag++;	
		if(RC_Ctl->key_E_flag>1)  RC_Ctl->key_E_flag=0;			
		}	 
	 }
	 else
	 {
		key_E_cnt=0;
		//RC_Ctl.key_E_flag=0;
	 }	
	//key_Z
		if(RC_Ctl->key_Z==1) 
	{
		key_Z_cnt++;
		if(key_Z_cnt==Key_Filter_Num)	//RC_Ctl.key_Z_flag��0��λ�ÿ���˼��һ��
		{
//				RC_Ctl->key_Z_flag++;
//			if(RC_Ctl->key_Z_flag>1)	RC_Ctl->key_Z_flag=0;
			RC_Ctl->key_Z_flag=1;
		}			
	}	else
	 {
		key_Z_cnt=0;
		RC_Ctl->key_Z_flag=0;
	 } 
	//key_V
		if(RC_Ctl->key_V==1) 
	{
		key_V_cnt++;
		if(key_V_cnt==Key_Filter_Num)	
		{
			RC_Ctl->key_V_flag=1;		
		}			
	}	else
	 {
		key_V_cnt=0;	
		RC_Ctl->key_V_flag=0;
	 } 	 
	//key_ctrl
	if(RC_Ctl->key_ctrl==1) 
	{
		key_ctrl_cnt++;
		if(key_ctrl_cnt==Key_Filter_Num)	
		{
			RC_Ctl->key_ctrl_flag=1;
			key_ctrl_cnt=0;	
		}	
	} 
	else
	{
		RC_Ctl->key_ctrl_flag=0;
	}	 
	//key_shift 
	 if(RC_Ctl->key_shift==1) 
	{
		key_shift_cnt++;
		if(key_shift_cnt==Key_Filter_Num)	
		{
			RC_Ctl->key_shift_flag=1;
			key_shift_cnt=0;	
		}	
	 }	
	else
	{
		RC_Ctl->key_shift_flag=0;
	}
	 //mouse_l
	 if(RC_Ctl->mouse.press_l==1)
	 {
		mouse_press_l_cnt++;
		if(mouse_press_l_cnt==Key_Filter_Num)
		{
			RC_Ctl->mouse.press_l_flag=1;
			mouse_press_l_cnt=0;
		}
	 }
	 else
	 {
		 RC_Ctl->mouse.press_l_flag=0;
	 }
		 //mouse_r
	 if(RC_Ctl->mouse.press_r==1)
	 {
		mouse_press_r_cnt++;
		if(mouse_press_r_cnt==Key_Filter_Num)
		{
			RC_Ctl->mouse.press_r_flag=1;
			mouse_press_r_cnt=0;
		}
	 }
	 else
	 {
		 RC_Ctl->mouse.press_r_flag=0;
	 }
}

