/***************************
Created by Wangshw 20150822
Modified by Wangshw 20160603
***************************/
#include "Extern.h"
extern unsigned int READ_EXTERNRAM(unsigned int Offset);	/*�������������ֶ�����ģ��*/
void TempCtrl_L1to6(unsigned char td);
void TempCtrl_L7(void);
void TempCtrl_L8(void);
void TempCtrl_L9(void);
void TempCtrl_L10(void);
void TempCtrl_L11(void);
/**************************************************************************
ģ������:	�����¿ز���ʵ��ģ��
ģ������:	1.���ɼ��¶�ֵ�����жϺͱȽ�;
			2.ʵ���¶ȿ��Ʋ��ԣ�������ȿ���ָ��;
			3.�Թ��Ͻ����жϣ����Լ������и�ֵ��
�������:	��
�������:	P[0~5]
����ֵ:		��
�޸ļ�¼:	20150822��������Ӧ������Ҫ��ʵ��24·���¶�23·���ȵĿ���
            20160603�޸ģ���Ӧ����״̬������Ҫ��ʵ��24·���¶�27·���ȵĿ���
����˵��:	
**************************************************************************/
/*********��·1:T1,T2;H1,H2*************/  
void TempCtrl_L1to6(unsigned char td)
{
	unsigned int Temp_value1, Temp_value2, T_max, T_min;

	/* �����������ж�,���·�����С�¶� */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM( ptHL_T1_Cmd[td][0]); 	/*   T01,T07,T21,T16,T04,T10  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value1 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */

	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM( ptHL_T1_Cmd[td][0] + 1); 	/*   T02,T08,T22,T17,T05,T11  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value2 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */

	HeatState = 1;                  /* ������ȷ��· */
	if((Temp_value1 > Sensor_Erk) || (Temp_value1 < Sensor_Erd))
	{
		if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))		/* ��·����·���·:���²���1 */
		{
			HeatState = 0;                  
			RetCode = 0x55;           /* �¿���״̬���븳ֵ 0x55*/
			pp[ptHL_T1_Cmd[td][1]] &= (unsigned char) ~ptHL_T1_Cmd[td][2];	/* �رռ���*/
			pp[ptHL_T1_Cmd[td][3]] &= (unsigned char) ~ptHL_T1_Cmd[td][4];	/* �رռ���*/
		}
		else	/* ��·�쳣:���ڿ�·���·,���²���2 */
		{
			T_max = Temp_value2;   /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
			T_min = Temp_value2; 
			if(RetCode > 0x80)
			{
				RetCode = 0x80;		/* ��·���д����������¿���״̬���븳ֵ */
			}
		}
	}
	else if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))	/* ��·�쳣:���ڿ�·���·,���²���2 */
	{
		T_max = Temp_value1;	/* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = Temp_value1; 
		if(RetCode > 0x80)
		{
			RetCode = 0x80;		/* ��·���д����������¿���״̬���븳ֵ */
		}
	}
	else		/* ��·����:�����ڿ�·���·,���²���3~11 */
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;   /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 
	}

	/* ���ȿ���   ����1 ����2  */
	if(HeatState)	/* �жϼ��Ȼ�·״̬ */
	{
		if((T_max < Argument.HT))		/* �¶ȳ������¹�����,���²���4 */
		{
			RetCode = 0x55;				/* �¿���״̬���븳ֵ0x55 */
			pp[ptHL_T1_Cmd[td][1]] &= (unsigned char) ~ptHL_T1_Cmd[td][2];	/* �رռ���*/
			pp[ptHL_T1_Cmd[td][3]] &= (unsigned char) ~ptHL_T1_Cmd[td][4];	/* �رռ���*/
		}
		else	/* �¶Ȳ��������¹�����*/
		{
			if(T_min >= Argument.LDT)		/* �¶ȵ��ڶ������� */
			{
				pp[ptHL_T1_Cmd[td][1]] |= (unsigned char) ptHL_T1_Cmd[td][2];	/* �򿪼��� */
				pp[ptHL_T1_Cmd[td][3]] |= (unsigned char) ptHL_T1_Cmd[td][4];	/* �򿪼���*/
			
				if(T_min > Argument.LT)		/* �¶ȵ��ڵ��¹�����*/
				{	
					if (TimerState_Cont > 600)
					{
						RetCode = 0x55;    /*��������10������:�ϱ�����,���²���4 */	
					}
				}
				else if (T_max <= Argument.HDT)		/* ���¶ȵ��ڶ������ޡ����¸��ڶ�������:���²���6*/
				{
					if(RetCode > 0x60)
					{
						RetCode = 0x60;    /* ���ڸߵ����쳣״̬���Լ�����ֵ */	
					}
				}
				else
				{
				    ;    /* ���账��:���²���5*/
				}	
			}
			else	/* �¶ȸ��ڶ������� */
			{	
				if(T_max <= Argument.HDT)	/* �¶ȸ��ڶ�������:���²���8 */
				{	
					pp[ptHL_T1_Cmd[td][1]] &= (unsigned char) ~ptHL_T1_Cmd[td][2];	/* �رռ���*/
					pp[ptHL_T1_Cmd[td][3]] &= (unsigned char) ~ptHL_T1_Cmd[td][4];	/* �رռ���*/
				}
				// /* �¶ȵ��ڶ�������:��·״̬����,���²���7 */
			}
		}
	}
}

void TempCtrl_L7(void)
{
	unsigned int Temp_value1, Temp_value2, Temp_value3, T_max, T_min;
	/* T3,T9,T13 �ؿ��� H5,H6(����5������6)*/
	/* �������쳣�ж� */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x12); 	/*   T03  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value1 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */

	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x18); 	/*   T09  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value2 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */

	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x1c); 	/*   T13  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value3 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */

	HeatState = 1;                  /* ������ȷ��· */
	if((Temp_value1 > Sensor_Erk) || (Temp_value1 < Sensor_Erd))
	{
		if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
		{
			if((Temp_value3 > Sensor_Erk) || (Temp_value3 < Sensor_Erd))
			{
				HeatState = 0;		/* ��·�쳣 */
				RetCode = 0x55;		/* �¿���״̬���븳ֵ */
				pp[2] &= (~0x06);	/* �رռ��� ����5������6*/                                  
			}
			else
			{
				T_max = Temp_value3 ;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
				T_min = Temp_value3 ; 
			}
		}
		else if((Temp_value3 > Sensor_Erk) || (Temp_value3 < Sensor_Erd))
		{
			T_max = Temp_value2 ;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
			T_min = Temp_value2 ; 
		}
		else
		{
			T_max = (Temp_value2 < Temp_value3) ? Temp_value2 : Temp_value3;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
			T_min = (Temp_value2> Temp_value3) ? Temp_value2 : Temp_value3; 
		}
		if(RetCode > 0x80)
		{
			RetCode = 0x80;        /* ��·���д����������¿���״̬���븳ֵ */
		}
	}
	else if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
	{
		if((Temp_value3 > Sensor_Erk) || (Temp_value3 < Sensor_Erd))
		{
			T_max = Temp_value1 ;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
			T_min = Temp_value1 ; 
		}
		else
		{
			T_max = (Temp_value1 < Temp_value3) ? Temp_value1 : Temp_value3;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
			T_min = (Temp_value1 > Temp_value3) ? Temp_value1 : Temp_value3; 
			if(RetCode > 0x80)
			{
				RetCode = 0x80;     /* ��·���д����������¿���״̬���븳ֵ */
			}
		}
	}
	else if(Temp_value3 > Sensor_Erk)
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 
		if(RetCode > 0x80)
		{
			RetCode = 0x80;     /* ��·���д����������¿���״̬���븳ֵ */
		}
	}
	else
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 

		T_max = (T_max < Temp_value3) ? T_max : Temp_value3;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = (T_min > Temp_value3) ? T_min : Temp_value3; 
	}
	
	/* ���ȿ���  ����5������6 */
	if(HeatState)	/* �жϼ��Ȼ�·״̬ */
	{
		if((T_max < Argument.HT))	/* �¶ȳ������¹����� */
		{
			RetCode = 0x55;			/* �¿���״̬���븳ֵ */
			pp[2] &= (~0x06);		/* �رռ��� ����5������6*/
		}
		else
		{
			if(T_min >= Argument.LDT)	/* �¶ȵ��ڶ������� */
			{
				pp[2] |= 0x06;			/* �򿪼��� ����5������6*/	

				if(T_min > Argument.LT)	 /* ����10������,���ϱ�����*/
				{	
					if(TimerState_Cont > 600)
					{
						RetCode = 0x55;    /* �¶ȵ��ڵ��¹�����*/	
					}
				}
				else if (T_max <= Argument.HDT)
				{
					if(RetCode > 0x60)
					{
						RetCode = 0x60;    /* ���ڸߵ����쳣״̬���Լ�����ֵ */	
					}
				}
				else
				{
				    ;    /* ���账�� */
				}	
			}
			else
			{	
				if(T_max <= Argument.HDT)	/* �¶ȸ��ڶ������� */
				{	
					pp[2] &= (~0x06);		/* �رռ��� ����5������6*/
				}
			}
		}
	}
}
void TempCtrl_L8(void)
{
	unsigned int Temp_value1, Temp_value2,  T_max, T_min;
	/* T15,T20,�ؿ��� H37(����37)*/
	/* �������쳣�ж� */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x1e); 	/*   T15  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value1 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x23); 	/*   T20  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value2 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */
	HeatState = 1;                  /* ������ȷ��· */
	if((Temp_value1 > Sensor_Erk) || (Temp_value1 < Sensor_Erd))
	{
		if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
		{
				HeatState = 0;			/* ��·�쳣 */
				RetCode = 0x55;			/* �¿���״̬���븳ֵ */
				pp[3] &= (~0x10);		/* �رռ��� ����37*/                                  
		}
		else
		{
			T_max = Temp_value2;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
			T_min = Temp_value2; 
			if(RetCode > 0x80)
			{
				RetCode = 0x80;        /* ��·���д����������¿���״̬���븳ֵ */
			}
		}
	}
	else if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
	{
		T_max = Temp_value1;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = Temp_value1; 
		if(RetCode > 0x80)
		{
			RetCode = 0x80;     /* ��·���д����������¿���״̬���븳ֵ */
		}
	}
	else
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 
	}
	
	/* ���ȿ���  ����5������6 */
	if(HeatState)	/* �жϼ��Ȼ�·״̬ */
	{
		
		if((T_max < Argument.HT))		/* �¶ȳ������¹����� */
		{
			RetCode = 0x55;				/* �¿���״̬���븳ֵ */
			pp[3] &= (~0x10);			/* �رռ��� ����37*/
		}
		else
		{
			if(T_min >= Argument.LDT)		/* �¶ȵ��ڶ������� */
			{
				pp[3] |= 0x10;				/* �򿪼��� ����5������6*/	

				if(T_min > Argument.QLT)	 /* ����10������,���ϱ�����*/
				{
					if(TimerState_Cont > 600)
					{
						RetCode = 0x55;    /* �¶ȵ��ڵ��¹�����*/	
					}
				}
				else if (T_max <= Argument.HDT)
				{
					if(RetCode > 0x60)
					{
						RetCode = 0x60;    /* ���ڸߵ����쳣״̬���Լ�����ֵ */	
					}
				}
				else
				{
				    ;    /* ���账�� */
				}	
			}
			else
			{	
				if(T_max <= Argument.HDT)	/* �¶ȸ��ڶ������� */
				{	
					pp[3] &= (~0x10);		/* �رռ��� ����37*/
				}
			}
		}
	}
}
void TempCtrl_L9(void)
{
	unsigned int Temp_value1, Temp_value2,  T_max, T_min;
	/* T14,T18,�ؿ��� H23��H24��H25��H26��H33��H34��H35��H36(����23��24��25��26��33��34��35��36)*/
	/* �������쳣�ж� */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x1d); 	/*   T15  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value1 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x21); 	/*   T20  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value2 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */
	HeatState = 1;                  /* ������ȷ��· */
	if((Temp_value1 > Sensor_Erk) || (Temp_value1 < Sensor_Erd))
	{
		if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
		{
			HeatState = 0;				/* ��·�쳣 */
			RetCode = 0x55;				/* �¿���״̬���븳ֵ */
			pp[5] &= (~0x1e);			/* �رռ��� ����23,����24,����25,����26  */
			pp[4] &= (~0x02);			/* �رռ��� ����33 */
			pp[3] &= (~0xe0);			/* �رռ��� ����34,����35������36 */
		}
		else
		{
			T_max = Temp_value2;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
			T_min = Temp_value2; 
			if(RetCode > 0x80)
			{
				RetCode = 0x80;        /* ��·���д����������¿���״̬���븳ֵ */
			}
		}
	}
	else if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
	{
		T_max = Temp_value1;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = Temp_value1; 
		if(RetCode > 0x80)
		{
			RetCode = 0x80;     /* ��·���д����������¿���״̬���븳ֵ */
		}
	}
	else
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 
	}
	
	/* ���ȿ��� */
	if(HeatState)	/* �жϼ��Ȼ�·״̬ */
	{
		
		if((T_max < Argument.HT))		/* �¶ȳ������¹����� */
		{
			RetCode = 0x55;				/* �¿���״̬���븳ֵ */
			pp[5] &= (~0x1e);			/* �رռ��� ����23,����24,����25,����26  */
			pp[4] &= (~0x02);			/* �رռ��� ����33 */
			pp[3] &= (~0xe0);			/* �رռ��� ����34,����35������36 */
		}
		else
		{

			if(T_min >= Argument.CLDT)	/* �¶ȵ��ڶ������� */
			{
				pp[5] |= 0x1e;			/* ���� ����23,����24,����25,����26  */
				pp[4] |= 0x02;			/* ���� ����33 */
				pp[3] |= 0xe0;			/* ���� ����34,����35������36 */

				if(T_min > Argument.LT)	 /* ����10������,���ϱ�����*/
				{
					if(TimerState_Cont > 600)
					{
						RetCode = 0x55;    /* �¶ȵ��ڵ��¹�����*/	
					}
				}
				else if (T_max <= Argument.HDT)
				{
					if(RetCode > 0x60)
					{
						RetCode = 0x60;    /* ���ڸߵ����쳣״̬���Լ�����ֵ */	
					}
				}
				else
				{
				    ;    /* ���账�� */
				}	
			}
			else
			{	
				if(T_max <= Argument.HDT)		/* �¶ȸ��ڶ������� */
				{	
					pp[5] &= (~0x1e);			/* �رռ��� ����23,����24,����25,����26  */
					pp[4] &= (~0x02);			/* �رռ��� ����33 */
					pp[3] &= (~0xe0);			/* �رռ��� ����34,����35������36 */
				}
			}
		}
	}
}

void TempCtrl_L10(void)
{
	unsigned int Temp_value1, Temp_value2, Temp_value3, T_max, T_min;
	/* T6,T12,T24 �ؿ��� H15,H18(����15������18)*/
	/* �������쳣�ж� */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x15); 	/*   T06  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value1 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x1b); 	/*   T12  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value2 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x27); 	/*   T24  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value3 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */
	HeatState = 1;                  /* ������ȷ��· */
	if((Temp_value1 > Sensor_Erk) || (Temp_value1 < Sensor_Erd))
	{
		if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
		{
			if((Temp_value3 > Sensor_Erk) || (Temp_value3 < Sensor_Erd))
			{
				HeatState = 0;			/* ��·�쳣 */
				RetCode = 0x55;			/* �¿���״̬���븳ֵ */
				pp[0] &= (~0x48);		/* �رռ��� ����15������18*/                                  
			}
			else
			{
				T_max = Temp_value3;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
				T_min = Temp_value3; 
			}
		}
		else if((Temp_value3 > Sensor_Erk) || (Temp_value3 < Sensor_Erd))
		{
			T_max = Temp_value2;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
			T_min = Temp_value2; 
		}
		else
		{
			Temp_value1 = Temp_value2;		/* �쳣���������ݴ��� */
		}
		if(RetCode > 0x80)
		{
			RetCode = 0x80;        /* ��·���д����������¿���״̬���븳ֵ */
		}
	}
	else if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
	{
		if((Temp_value3 > Sensor_Erk) || (Temp_value3 < Sensor_Erd))
		{
			T_max = Temp_value1;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
			T_min = Temp_value1; 
		}
		else
		{
			T_max = (Temp_value1 < Temp_value3) ? Temp_value1 : Temp_value3;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
			T_min = (Temp_value1 > Temp_value3) ? Temp_value1 : Temp_value3; 
			if(RetCode > 0x80)
			{
				RetCode = 0x80;     /* ��·���д����������¿���״̬���븳ֵ */
			}
		}
	}
	else if(Temp_value3 > Sensor_Erk)
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 
		if(RetCode > 0x80)
		{
			RetCode = 0x80;     /* ��·���д����������¿���״̬���븳ֵ */
		}
	}
	else
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 

		T_max = (T_max < Temp_value3) ? T_max : Temp_value3;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = (T_min > Temp_value3) ? T_min : Temp_value3; 
	}
	
	/* ���ȿ���  ����15������18 */
	if(HeatState)	/* �жϼ��Ȼ�·״̬ */
	{
		
		if((T_max < Argument.HT))		/* �¶ȳ������¹����� */
		{
			RetCode = 0x55;				/* �¿���״̬���븳ֵ */
			pp[0] &= (~0x48);			/* �رռ��� ����15������18*/
		}
		else
		{
			if(T_min >= Argument.LDT)		/* �¶ȵ��ڶ������� */
			{
				pp[0] |= 0x48;				/* �򿪼��� ����15������18*/	

				if(T_min > Argument.LT)
				{	
					if(TimerState_Cont > 600)
					{
						RetCode = 0x55;    /* �¶ȵ��ڵ��¹�����*/	
					}
				}
				else if (T_max <= Argument.HDT)
				{
					if(RetCode > 0x60)
					{
						RetCode = 0x60;    /* ���ڸߵ����쳣״̬���Լ�����ֵ */
					}
				}
				else
				{
				    ;    /* ���账�� */
				}	
			}
			else
			{	
				if(T_max <= Argument.HDT)		/* �¶ȸ��ڶ������� */
				{	
					pp[0] &= (~0x48);			/* �رռ��� ����15������18*/
				}
			}
		}
	}
}
void TempCtrl_L11(void)
{
	unsigned int Temp_value1, Temp_value2,  T_max, T_min;
	/* T19,T23,�ؿ��� H16�� H17(����16������17)*/
	/* �������쳣�ж� */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x22); 	/*   T19  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value1 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */
	AncTotal.AncuInt.AncuIntL = READ_EXTERNRAM(AnC_Buf + 0x26); 	/*   T23  */
	AncTotal.AncuInt.AncuIntH = 0;
	Temp_value2 = (unsigned int)((AncTotal.AncuLong << 9) / AncStandard);  /* У��ϵ������ 512��Ӧ5V  */
	HeatState = 1;                  /* ������ȷ��· */
	if((Temp_value1 > Sensor_Erk) || (Temp_value1 < Sensor_Erd))
	{
		if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
		{
				HeatState = 0;                  /* ��·�쳣 */
				RetCode = 0x55;           /* �¿���״̬���븳ֵ */
				pp[0] &= (~0x30);                                           /* �رռ��� ����16������17*/                                  
		}
		else
		{
			T_max = Temp_value2;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
			T_min = Temp_value2; 
			if(RetCode > 0x80)
			{
				RetCode = 0x80;        /* ��·���д����������¿���״̬���븳ֵ */
			}
		}
	}
	else if((Temp_value2 > Sensor_Erk) || (Temp_value2 < Sensor_Erd))
	{
		T_max = Temp_value1;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = Temp_value1; 
		if(RetCode > 0x80)
		{
			RetCode = 0x80;     /* ��·���д����������¿���״̬���븳ֵ */
		}
	}
	else
	{
		T_max = (Temp_value1 < Temp_value2) ? Temp_value1 : Temp_value2;	  /* �¶���ֵ�ж� ���¶�ϵ��,T_maxΪģ������Сֵ */
		T_min = (Temp_value1 > Temp_value2) ? Temp_value1 : Temp_value2; 
	}
	
	/* ���ȿ���  ����5������6 */
	if(HeatState)	/* �жϼ��Ȼ�·״̬ */
	{
		
		if((T_max < Argument.HT))                                      /* �¶ȳ������¹����� */
		{
			RetCode = 0x55;           /* �¿���״̬���븳ֵ */
			pp[0] &= (~0x30);                                           /* �رռ��� ����16������17*/
		}
		else
		{
			if(T_min >= Argument.LDT)                                    /* �¶ȵ��ڶ������� */
			{
				pp[0] |= 0x30;                                          /* �򿪼��� ����16������17*/	

				if(T_min > Argument.QLT)	 /* ����10������,���ϱ�����*/
				{
					if(TimerState_Cont > 600)
					{
						RetCode = 0x55;    /* �¶ȵ��ڵ��¹�����*/	
					}
				}
				else if (T_max <= Argument.HDT)
				{
					if(RetCode > 0x60)
					{
						RetCode = 0x60;    /* ���ڸߵ����쳣״̬���Լ�����ֵ */	
					}
				}
				else
				{
				    ;    /* ���账�� */
				}	
			}
			else
			{	
				if(T_max <= Argument.HDT)                               /* �¶ȸ��ڶ������� */
				{	
					pp[0] &= (~0x30);                                   /* �رռ��� ����16������17*/
				}
			}
		}
	}
}
