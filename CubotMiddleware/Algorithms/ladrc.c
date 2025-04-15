#include "LADRC.h"
#include "filter.h"

LADRC_NUM   M3508_Angle;    //ϵͳ��Ҫ�����Ĳ��� 
LADRC_NUM   M3508_Speed; 
LADRC_NUM   M6020_Angle; 
LADRC_NUM   M6020_Angle_ZOH; 
LADRC_NUM M6020_Angle_Yaw1;
LADRC_NUM M6020_Angle_Yaw;
/**
   *@Brief default ������
   *@Brief ���ݾ��� ts��0.2~0.3֮�� Wo��Wcѡ�����С��������b0
   *@Date@WangShun 2022-05-28 2022-07-03����
---------------------------------------------------
		      LADRC default������									
---------------------------------------------------
---------------------------------------------------
	ts	|	h	|	r	|   wc   |   w0  |	b0
---------------------------------------------------
	0.1	|	h	|	r	|  100   |  400  |	b0
---------------------------------------------------
   0.157|	h	|	r	|   64   |  224~255  |	b0
---------------------------------------------------
   0.158|	h	|	r	|   63   |  253  |	b0
---------------------------------------------------
   0.159|	h	|	r	|   63   |  252  |	b0
---------------------------------------------------
	0.16|	h	|	r	|   63   |  250  |	b0
---------------------------------------------------
	0.17|	h	|	r	|   59   |  235  |	b0
---------------------------------------------------
	0.18|	h	|	r	|   56   |  222  |	b0
---------------------------------------------------
	0.2	|	h	|	r	|   50   |  200  |	b0
---------------------------------------------------
	0.21|	h	|	r	|   48   |  190  |	b0
---------------------------------------------------
	0.22|	h	|	r	|   45   |  182  |	b0
---------------------------------------------------
	0.23|	h	|	r	|   43   |  174  |	b0
---------------------------------------------------
	0.24|	h	|	r	|   42   |  167  |	b0
---------------------------------------------------
	0.25|	h	|	r	|   40   |  160  |	b0
---------------------------------------------------
	0.26|	h	|	r	|   38   |  154  |	b0
---------------------------------------------------
	0.27|	h	|	r	|   37   |  148  |	b0
---------------------------------------------------
	0.28|	h	|	r	|   36   |  144  |	b0
---------------------------------------------------
	0.29|	h	|	r	|   34   |  138  |	b0
---------------------------------------------------
	0.3	|	h	|	r	|   33   |  133  |	b0
---------------------------------------------------
	0.4	|	h	|	r	|   25   |  100  |	b0
---------------------------------------------------
	0.5	|	h	|	r	|   20   |   80  |	b0
---------------------------------------------------
--------------------------------------------------- 
*/
 
 /**
  * ����˵�� LADRC��ʼ�ο�ֵ
  * 		WangShun��2022-07-03����
  */	
const double LADRC_Unit[5][7]=
{
	{0.002,22,63,252,45},//3508�Ƕ�
	{0.001,20,50,200,1800},//3508�ٶ�
	{0.001,250,45,-3,250,0.003,0.00165},//Сyaw�Ƕ�
	{0.001,100,4,-2.5,250,1,0.03},//yaw�Ƕ�
	{0.001,10,2.5,-1,250,0.001,0.00165},//6020�Ƕ�
};
/**
  * ����˵����LADRC��ʼ��
  * 		WangShun��2022-07-03����
  */	
void LADRC_Init(LADRC_NUM *LADRC_TYPE)
{
	if(LADRC_TYPE == &M3508_Angle)
	{
		LADRC_TYPE->h = LADRC_Unit[0][0]; //��ʱʱ�估ʱ�䲽��
		LADRC_TYPE->r = LADRC_Unit[0][1]; //�����ٶȲ���
		LADRC_TYPE->wc = LADRC_Unit[0][2]; //�۲�������
		LADRC_TYPE->w0 = LADRC_Unit[0][3]; //״̬�����ʴ���
		LADRC_TYPE->b0 = LADRC_Unit[0][4]; //ϵͳ����
	}
	if(LADRC_TYPE == &M3508_Speed)
	{
		LADRC_TYPE->h = LADRC_Unit[1][0]; //��ʱʱ�估ʱ�䲽��
		LADRC_TYPE->r = LADRC_Unit[1][1]; //�����ٶȲ���
		LADRC_TYPE->wc = LADRC_Unit[1][2]; //�۲�������
		LADRC_TYPE->w0 = LADRC_Unit[1][3]; //״̬�����ʴ���
		LADRC_TYPE->b0 = LADRC_Unit[1][4]; //ϵͳ����
	}
	if(LADRC_TYPE == &M6020_Angle)
	{
		LADRC_TYPE->h = LADRC_Unit[2][0]; //��ʱʱ�估ʱ�䲽��
		LADRC_TYPE->r = LADRC_Unit[2][1]; //�����ٶȲ���
		LADRC_TYPE->Kp = LADRC_Unit[2][2]; //�۲�������
		LADRC_TYPE->Kd = LADRC_Unit[2][3]; //�۲�������
		LADRC_TYPE->w0 = LADRC_Unit[2][4]; //״̬�����ʴ���
		LADRC_TYPE->b0 = LADRC_Unit[2][5]; //ϵͳ����
	}
	if(LADRC_TYPE == &M6020_Angle_ZOH)
	{
		LADRC_TYPE->h = LADRC_Unit[2][0]; //��ʱʱ�估ʱ�䲽��
		LADRC_TYPE->r = LADRC_Unit[2][1]; //�����ٶȲ���
		LADRC_TYPE->Kp = LADRC_Unit[2][2]; //�۲�������
		LADRC_TYPE->Kd = LADRC_Unit[2][3]; //�۲�������
		LADRC_TYPE->beta = LADRC_Unit[2][6]; //״̬�����ʴ���
		LADRC_TYPE->b0 = LADRC_Unit[2][5]; //ϵͳ����
	}
		if(LADRC_TYPE == &M6020_Angle_Yaw1)
	{
		LADRC_TYPE->h = LADRC_Unit[2][0]; //��ʱʱ�估ʱ�䲽��
		LADRC_TYPE->r = LADRC_Unit[2][1]; //�����ٶȲ���
		LADRC_TYPE->Kp = LADRC_Unit[2][2]; //�۲�������
		LADRC_TYPE->Kd = LADRC_Unit[2][3]; //�۲�������
		LADRC_TYPE->beta = LADRC_Unit[2][6]; //״̬�����ʴ���
		LADRC_TYPE->b0 = LADRC_Unit[2][5]; //ϵͳ����
	}
			if(LADRC_TYPE == &M6020_Angle_Yaw)
	{
		LADRC_TYPE->h = LADRC_Unit[3][0]; //��ʱʱ�估ʱ�䲽��
		LADRC_TYPE->r = LADRC_Unit[3][1]; //�����ٶȲ���
		LADRC_TYPE->Kp = LADRC_Unit[3][2]; //�۲�������
		LADRC_TYPE->Kd = LADRC_Unit[3][3]; //�۲�������
		
		LADRC_TYPE->b0 = LADRC_Unit[3][5]; //ϵͳ����  
		LADRC_TYPE->beta = LADRC_Unit[3][6]; //״̬�����ʴ��� 0.3 0.5
	}
}
/**
  * ����˵����LADRCȱʡ
  * 		WangShun��2022-07-03����
  */
void LADRC_REST(LADRC_NUM *LADRC_TYPE1)
{
		LADRC_TYPE1->z1= 0; //��ʱʱ�估ʱ�䲽��
    LADRC_TYPE1->z2 =0; //�����ٶȲ���
    LADRC_TYPE1->z3=0; //�۲�������

}
/**
  * ��������void ADRC_TD(LADRC_NUM *LADRC_TYPE1,float Expect)
  * ����˵����LADRC����΢�ֲ���
  * @param[in]	��ڲ���������ֵExpect(v0)���ֵv1,v2
  * @par �޸���־
  * 		WangShun��2022-05-28����
  */
void LADRC_TD(LADRC_NUM *LADRC_TYPE1,float Expect)
{
    double fh= -LADRC_TYPE1->r * LADRC_TYPE1->r * (LADRC_TYPE1->v1-Expect)-2 * LADRC_TYPE1->r * LADRC_TYPE1->v2;
    LADRC_TYPE1->v1 += LADRC_TYPE1->v2 * LADRC_TYPE1->h;
    LADRC_TYPE1->v2 += fh * LADRC_TYPE1->h;//΢����
}
/**
  * ��������LADRC_ESO(LADRC_NUM *LADRC_TYPE1,float FeedBack)
  * ����˵����LADRC����״̬�۲���
  * @param[in]
  * @par �޸���־
  * 		WangShun��2022-07-03����
  */
void LADRC_ESO(LADRC_NUM *LADRC_TYPE,int16_t FeedBack)
{		
//	double z1k,z2k,z3k;
//	z1k = LADRC_TYPE->z1;
//	z2k = LADRC_TYPE->z2;
//	z3k = LADRC_TYPE->z3;
//	
//	LADRC_TYPE->z1 = LADRC_TYPE->h * z2k + square(LADRC_TYPE->h) * z3k/2 - y * (3 * LADRC_TYPE->beta - 3) + z1k * (3 * LADRC_TYPE->beta - 2) + LADRC_TYPE->b0 * square(LADRC_TYPE->h) * u / 2;
//	LADRC_TYPE->z2 = z2k + LADRC_TYPE->h * z3k + LADRC_TYPE->b0 * LADRC_TYPE->h * u + (y * square(LADRC_TYPE->beta - 1) * (LADRC_TYPE->beta + 5)) / (2 * LADRC_TYPE->h) - (z1k * square(LADRC_TYPE->beta - 1) * (LADRC_TYPE->beta + 5))/(2 * LADRC_TYPE->h);
//	LADRC_TYPE->z3 = z3k - (y * cube(LADRC_TYPE->beta - 1))/square(LADRC_TYPE->h) + (z1k * cube(LADRC_TYPE->beta - 1))/square(LADRC_TYPE->h);
}

double y_watch;
double u_watch;
void LADRC_ESO_ZOH(LADRC_NUM *LADRC_TYPE,double y,double u)
{	

	double z1k,z2k,z3k;
	z1k = LADRC_TYPE->z1;
	z2k = LADRC_TYPE->z2;
	z3k = LADRC_TYPE->z3;
	
	LADRC_TYPE->z1 = LADRC_TYPE->h * z2k + square(LADRC_TYPE->h) * z3k/2 - y * (3 * LADRC_TYPE->beta - 3) + z1k * (3 * LADRC_TYPE->beta - 2) + LADRC_TYPE->b0 * square(LADRC_TYPE->h) * u / 2;
	LADRC_TYPE->z2 = z2k + LADRC_TYPE->h * z3k + LADRC_TYPE->b0 * LADRC_TYPE->h * u + (y * square(LADRC_TYPE->beta - 1) * (LADRC_TYPE->beta + 5)) / (2 * LADRC_TYPE->h) - (z1k * square(LADRC_TYPE->beta - 1) * (LADRC_TYPE->beta + 5))/(2 * LADRC_TYPE->h);
	LADRC_TYPE->z3 = z3k - (y * cube(LADRC_TYPE->beta - 1))/square(LADRC_TYPE->h) + (z1k * cube(LADRC_TYPE->beta - 1))/square(LADRC_TYPE->h);
}
/**
   *@Brief  LADRC_LSEF
   *@Date   ���Կ�����
			WangShun��2022-07-03����
   */
void LADRC_LF(LADRC_NUM *LADRC_TYPE1,float y,float y_dot)
{
//    float Kp=LADRC_TYPE1->wc * LADRC_TYPE1->wc;
//    float Kd=2 * LADRC_TYPE1->wc;
	/**
       *@Brief  ���Կ�����������kd = 2wc
       *@Before Kd=3*LADRC_TYPE1->wc;
       *@Now    Kd=2*LADRC_TYPE1->wc;
       *@WangShun  2022-04-27  ע��
       */
    double e1=LADRC_TYPE1->v1 - y;
    double e2= - y_dot;
    double u0=LADRC_TYPE1->Kp * e1 + LADRC_TYPE1->Kd * e2;
	LADRC_TYPE1->Kppart=LADRC_TYPE1->Kp * e1/ LADRC_TYPE1->b0;
	LADRC_TYPE1->Kdpart=LADRC_TYPE1->Kd * e2/ LADRC_TYPE1->b0;
	
		LADRC_TYPE1->u0 = u0;
//    LADRC_TYPE1->u = (u0) / LADRC_TYPE1->b0+M6020_Angle_Yaw.z3/2;
	LADRC_TYPE1->u = (u0) / LADRC_TYPE1->b0;
		if(fabs(e1) < 1.5)
		LADRC_TYPE1->Kipart += LADRC_TYPE1->Ki* e1;
	else 	LADRC_TYPE1->Kipart = 0;
		LADRC_TYPE1->u += 	LADRC_TYPE1->Kipart;
	if(LADRC_TYPE1->u > 25000)
		LADRC_TYPE1->u = 25000;
	else if(LADRC_TYPE1->u < -25000)
		LADRC_TYPE1->u = -25000;
}
/**
  * LADRC���ƺ��� .
  * ������������ѭ���м���
  * @par ����
  * @par �޸���־
  * @WangShun  2022-07-03  ע��
  */
void LADRC_Loop(LADRC_NUM *LADRC_TYPE1,float* Expect,float* RealTimeOut)
{
	double  Expect_Value = *Expect;
	double  Measure = *RealTimeOut;
//    LADRC_TD(LADRC_TYPE1,Expect_Value);
//    LADRC_ESO(LADRC_TYPE1,Measure); 
//    LADRC_LF(LADRC_TYPE1);
}

void LADRC_ZOH_Loop(LADRC_NUM *LADRC_TYPE1,double Expect_Value,double RealTimeOut,double Measure,double Measure_D)
{
    LADRC_TD(LADRC_TYPE1,Expect_Value);
    LADRC_ESO_ZOH(LADRC_TYPE1,Measure,RealTimeOut); 
    LADRC_LF(LADRC_TYPE1,Measure,Measure_D);
}
