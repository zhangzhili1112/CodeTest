#include "Extern.h"

extern void WRITE_EXTERNRAM(unsigned int Offset, unsigned int Data);/*�жϳ�������д����ģ��*/
extern unsigned int READ_EXTERNRAM(unsigned int Offset);			  /*�жϳ������ֶ�����ģ��*/
const unsigned char TD_Index[Sample_Total] =
{
	0x00,0x10,0x20,0x30,0x40,0x50,0x70,0x80,0x90,	//һ��ͨ��=0;����ͨ��:0�¶�У׼��ѹ\1���µ�01\2���µ�17\3����ͨ��1״̬\4����ͨ��17״̬\5����ͨ��25״̬\7+12V������ѹ\8+5V������ѹ\9-12V������ѹ,��9·
		 0x11,0x21,0x31,0x41,0x51,					//һ��ͨ��=1;����ͨ��:              1���µ�02\2���µ�18\3����ͨ��2״̬\4����ͨ��18״̬\5����ͨ��26״̬,��5·
		 0x12,0x22,0x32,							//һ��ͨ��=2;����ͨ��:              1���µ�03\2���µ�19\3����ͨ��3״̬,��3·
		 0x13,0x23,0x33,							//һ��ͨ��=3;����ͨ��:              1���µ�04\2���µ�20\3����ͨ��4״̬,��3·
		 0x14,0x24,0x34,0x44,						//һ��ͨ��=4;����ͨ��:              1���µ�05\2���µ�21\3����ͨ��5״̬\4����ͨ��21״̬,��4·
		 0x15,0x25,0x35,0x45,						//һ��ͨ��=5;����ͨ��:              1���µ�06\2���µ�22\3����ͨ��6״̬\4����ͨ��22״̬,��4·
		 0x16,0x26,     0x46,0x56,					//һ��ͨ��=6;����ͨ��:              1���µ�07\2���µ�23\               4����ͨ��23״̬\5����ͨ��31״̬,��4·
		 0x17,0x27,     0x47,0x57,					//һ��ͨ��=7;����ͨ��:              1���µ�08\2���µ�24\               4����ͨ��24״̬\5����ͨ��32״̬,��4·
		 0x18,               0x58,					//һ��ͨ��=8;����ͨ��:              1���µ�09\                                         5����ͨ��33״̬,��2·
		 0x19,               0x59,					//һ��ͨ��=9;����ͨ��:              1���µ�10\                                         5����ͨ��34״̬,��2·
		 0x1A,     0x3A,     0x5A,					//һ��ͨ��=A;����ͨ��:              1���µ�11\          3����ͨ��11״̬\               5����ͨ��35״̬,��3·
		 0x1B,     0x3B,     0x5B,					//һ��ͨ��=B;����ͨ��:              1���µ�12\          3����ͨ��12״̬\               5����ͨ��36״̬,��3·
		 0x1C,     0x3C,     0x5C,					//һ��ͨ��=C;����ͨ��:              1���µ�13\          3����ͨ��13״̬\               5����ͨ��37״̬,��3·
		 0x1D,     0x3D,							//һ��ͨ��=D;����ͨ��:              1���µ�14\          3����ͨ��14״̬,��2·
		 0x1E,     0x3E,							//һ��ͨ��=E;����ͨ��:              1���µ�15\          3����ͨ��15״̬,��2·
		 0x1F,     0x3F,							//һ��ͨ��=F;����ͨ��:              1���µ�16\          3����ͨ��16״̬,��2·
};		//�ܹ�55·

void AnCollect(void)
{
	unsigned char register i,j,td;
	unsigned int register Rtmp,TotalRtmp;
	TotalRtmp = 0;
	AncMax = 0;
	AncMin = 0x7ff;
	td = TD_Index[TimerCount];
	XBYTE[ADDR_CJTD] = td & 0x0F;   						/*�ɼ�ͨ��һ��ѡ��*/
	_nop_();	/*�յȴ�*//*��ʱ����������*/
	_nop_();	/*�յȴ�*/
	XBYTE[ADDR_CJTD] = td;   		/*�ɼ�ͨ������ѡ��*/
	_nop_();	/*�յȴ�*//*��ʱ����������*/
	for (i = 0; i < 6; i++)/*�ɼ�6��,ȥ��1ȥ��1,ʣ��4������ƽ��ֵ*/
	{
		AD_CONVST = 0;  		   /*����ת��ģʽ*/
		_nop_();                  /* ��ʱ,�������� */
		_nop_();
		AD_CONVST = 1;  		   /*����ת��ģʽ*/                  
		_nop_();                  /* ��ʱ,�������ţ��ȴ�ת����� */
		_nop_();
		AD_RFS = 0; 			   /*��������֡ͬ��,��ʼ������*/
		_nop_();                  /* ��ʱ,�������� */
		_nop_();
		for (Rtmp = 0,j = 0; j < 16; j++)	  /* �������ݳ�ʼ��,�ɼ�16λ���� */ 
		{
			Rtmp <<= 1;
			SDATA = 1;		/*������ǰ��������λ���ø�*/
			SCLK = 0;   	    	 /* �½��ض��� */
			if(SDATA)
			{
				Rtmp |=1;    
			}
			SCLK = 1;   			 /* ����������clock out */
		}
		AD_RFS = 1; 			   /*��������֡ͬ��,��ȡ���ݽ���*/
		_nop_();                  /* ��ʱ,�������� */
		if (0x800 == (0x800 & Rtmp))   //Tzh �ж��Ƿ�������ĵ�������Ϊ��ֵ��
		{
			Rtmp = 0;
		}
		TotalRtmp += (unsigned long)Rtmp;

		if (AncMax < Rtmp)  	  /*ͳ�������Сֵ*/
		{
			AncMax = Rtmp;
		}
		if(AncMin > Rtmp)
		{
			AncMin = Rtmp;
		}
	}
	switch(td)	/* �洢������������[0-3]:JZ/5VTM/12VTM/12VTM*  [0x10-0x6f]:32·��������+40·������״̬ */
	{
	case 0x70:
		td = 1;
		break;
	case 0x80:
		td = 2;
		break;
	case 0x90:
		td = 3;
		break;
	default:
		break;
	}
	/* �Բɼ�����ģ���������˲����� */
	TotalRtmp = TotalRtmp - AncMax - AncMin + 2;		/*�޳����ֵ����Сֵ*///Tzh Ϊʲô+2
	WRITE_EXTERNRAM((unsigned int) (AnC_Buf + td), (TotalRtmp >> 2));  /* ���ɼ����ݴ洢����ʱ�洢�� */
}
void PowerDetect(void)
{
	unsigned int register Rtmp;
	Rtmp = READ_EXTERNRAM(AnC_Buf + 1);		//+12V������ѹ
	if(((Base_P12 - Error_THRESHOLD) > Rtmp) ||  ((Base_P12 + Error_THRESHOLD) < Rtmp))
	{
		Status |= 0x10;		//Status.4=1
	}
	else
	{
		Status &= 0x0f;		//Status.4=0
	}
	Rtmp = READ_EXTERNRAM(AnC_Buf + 2);		//+5V������ѹ
	if(((Base_5V - Error_THRESHOLD) > Rtmp) ||  ((Base_5V + Error_THRESHOLD) < Rtmp))
	{
		Status |= 0x20;		//Status.5=1
	}
	else
	{
		Status &= 0x1f;		//Status.5=0
	}
	Rtmp = READ_EXTERNRAM(AnC_Buf + 3);		//-12V������ѹ
	if(((Base_S12 - Error_THRESHOLD) > Rtmp) ||  ((Base_S12 + Error_THRESHOLD) < Rtmp))
	{
		Status |= 0x40		//Status.6=1
	}
	else
	{
		Status &= 0x3f		//Status.6=0
	}
	Rtmp = READ_EXTERNRAM(AnC_Buf + 0);		//�¶�У׼��ѹ
	if(((Base_Sensor - Error_THRESHOLD) > Rtmp) ||  ((Base_Sensor + Error_THRESHOLD) < Rtmp))
	{
		Status |= 0x80;		//Status.7=1
	}
	else
	{
		Status &= 0x7f;		//Status.7=0
	}
}										
/**************************************************************************
ģ������:	��ʱ�ж�ģ��                
ģ������:   1.ÿ�����ж�1��
			2.��24·�����źš�27·����״̬������ѹ��4·������ѹ���вɼ���ÿ�����1·�źŲɼ�6�Σ�����ȡ0���޳����ֵ����Сֵ������ȡƽ����
			3.�ɼ���ɺ���֤AD��·�ɼ�+5V������ֵ512����+12V������ֵ1229����-12V������ֵ1229��������ѹ���¶�У׼��ѹ������ֵ512������ȷ��
			4.�ɼ���ɺ����òɼ���ɱ�־��
			5.�Դ��ڳ�ʱ�����жϣ��������ͺͽ��ա�
�������:	��
�������:	��
����ֵ:		��
�޸ļ�¼:   
����˵��:
**************************************************************************/ 
void Timer0_INT(void) interrupt 1 using 1
{

	TR0 = 0;			  /*ֹͣ��ʱ��0����*/
	TH0 = TH_1ms;		  /*��ʱ��0ʱ��1ms��ʼ��*/
	TL0 = TL_1ms; 		  
	TR0 = 1;			  /*������ʱ��0����*/
	synTime.TA ++;		  /*��ʱ�������ۼ�*/
	if (ReceiveingFlag)
	{
		ReceiveCount ++;
		if (ReceiveCount > OverTime)
		{
			SM2 =1;
			ReceiveingFlag = 0;		//ֹͣ��ʱ
			ReceiveCount = 0;
		}
	}
	if (SendingFlag)
	{
		SendCount ++;
		if (SendCount > OverTime)
		{
			CommuniationDisable = 1;		//�Ͽ�ͨѶ��·
			SendingFlag = 0;		//ֹͣ��ʱ
			SendCount = 0;
		}
	}
	TimerCount++;		  /*�ڲ���ʱ�������ۼ�*/
	if (CtrlPeriod == TimerCount)
	{
		TimerCount = 0;

		TimerState_Cont ++;
	}
	if ((Sample_Total > TimerCount) && (wkStatus != vaild_Status))
	{
		AnCollect();
	}

	else if (Sample_Total == TimerCount)
	{
		PowerDetect();
	}
	else  if ((Sample_Total +1 ) == TimerCount)
	{
		ctrlFlag = 1��
	}
	else if((CtrlPeriod - 50) == TimerCount)
	{
		ctrlFlag = 0��
	}
	else
	{
		//�����
	}
}
