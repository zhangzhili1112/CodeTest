/****************************************
��Ҫ����Ϊ��
1.�ɼ�����һ��Ϊ0x00   			   --ADCλ��[0]
				0x10-0x2f[32·��������ɼ�]--ADCλ��[0x10-0x2f]
				0x30-0x43[1-20·������״̬�ɼ�]--ADCλ��[0x30-0x43]
				0x50-0x63[21-40·��������ɼ�]--ADCλ��[0x50-0x63]
				0x70--5VTM, 			   --ADCλ��[1]
				0x80--12VTM,			   --ADCλ��[2]
				0x90--12VTM*,   		   --ADCλ��[3]
2.ָ�����
  ע������--�ӵ�ַ26-2����--ADR��+ADR��+������
  ע������--�ӵ�ַ24-8����--��ֵ��Ԥ��ֵ�����޸�
  ָ������--�ӵ�ַ20-3����--ͬ��ʱ��+ָ��(2a2a[��λ];2b2b[��������];2c2c[��������];2d2d[ȫ����������];2e2e[�����Զ�Ѳ��];1a1a[�������];1b1b[��������]
  �Լ���--�ӵ�ַ10-24����
  ͬ��ʱ��--�ӵ�ַ14-2����
3.���ݴ洢
ʹ��1553B��RAM�ռ�Ϊ����RAMƫ�Ƶ�ַ:0x500-0xFDF
4.�Զ��¶ȿ���
����Ԥ���¿ز��Խ��м��ȿ���

Created by Wangshw 20150822
Modified by Wangshw 20160603
******************************************/
#include "Global.h"

/*****************�ڲ�RAM�洢��ַ��*******************************************/

/*****************����ӳ���****************************************************/
void Exec_Loop(void);   			 /*ѭ��ִ��ģ��*/
void ClearDog(void);				/*���Ź���λ��ģ��*/
/*-------------��ʼ��ģ�������ģ��----------------------------*/
void InitProgram(void); 			/*�����ʼ���ӳ���*/

extern void LoadArg(void);			/*װ������ӳ���*/
extern void SaveArg(void);     		/*��������ӳ���*/

void WRITE_EXTERNRAM(unsigned int Offset, unsigned int Data);/*��������������д����ģ��*/
unsigned int READ_EXTERNRAM(unsigned int Offset);	/*�������������ֶ�����ģ��*/
/*-------------�¿�ʵ�ּ����ܼ��ģ��------------------------- */
extern void TempCtrl_L1to6(unsigned char td);
extern void TempCtrl_L7(void);
extern void TempCtrl_L8(void);
extern void TempCtrl_L9(void);
extern void TempCtrl_L10(void);
extern void TempCtrl_L11(void);
char code reserve[8] _at_ 0x03;		//�ⲿ�ж�0--���Ź��������
/**************************************************************************
ģ������:	��ʼ�����Լ�ģ��
ģ������:	1.�Լ�����״̬���г�ʼ����;
			2.��RAM���м�������;
			3.ִ�����߳�ʼ������;
			4.CPU״̬��ʼ������
����:		1-main()
�������:	��
�������:	��
����ֵ:		��
�޸ļ�¼:	
����˵��:   	
**************************************************************************/ 
void InitProgram(void)
{
	unsigned char i,tChar;
	CommuniationDisable = 1;	//�Ͽ�ͨѶ��·
	ExternInt0_Enable = 0;		/*���Ź��ж����벻����*/
	AD_CONVST = 1;				/*AD������ת�� */
	AD_RFS = 1;					/*��������֡��ͬ��*/
	SCLK = 1;					/*ADʱ��ƽʱΪ��*/
	SDATA = 0;					/*AD��������������*/
	ICWRDisable = 1;			/*�������дI2C*/
	ICLK = 0;					/*I2Cʱ��ƽʱΪ��*/
	ICDATA = 0;					/*I2C��������������*/
	/*--------------------������״̬����Ϊ�ر�----------------------------*/ 
	pp[0] = 0;
	pp[1] = 0;
	pp[2] = 0;
	pp[3] = 0;
	pp[4] = 0;
	pp[5] = 0;
	XBYTE[ADDR_IOSEL1] = pp[0];  			 /*40�����������*/ 
	XBYTE[ADDR_IOSEL2] = pp[1];  
	XBYTE[ADDR_IOSEL3] = pp[2];
	XBYTE[ADDR_IOSEL4] = pp[3];
	XBYTE[ADDR_IOSEL5] = pp[4];
	XBYTE[ADDR_IOSEL6] = pp[5];
	if(0x00 == (Status & 0x08)) 
	{
		for(tChar = RamFlag , RamStartAdr = XDATASTART , i = 0 ; i < XBLOCKTOTAL ; i++ , tChar >>= 1 , RamStartAdr + =XBLOCKLEN)
		{
			if((tChar & 0x01) == 0) break;
		}
	}
	else
	{
		RamStartAdr = 0;
		wkStatus = vaild_Status;
	}
	SendBuff.BuffAdr = RamStartAdr + SendBufStart;
	ReceiveBufStart.BuffAdr = RamStartAdr + ReceiveBufStart;
	if(HAVEDONE != RestoreFlag)//Tzh ������û�д���HAVEDONE�����
	{
		if(0x00 == (Status & 0x08))
		{
			wkStatus = Idel_Status;
			WRITE_EXTERNRAM(AnC_Buf,(unsigned int)Base_Sensor);
			RetCode = 0xAA;
		}
		synTime.TA = 0;
	}
	TimerCount = CtrlPeriod -1 ; //�����жϿ�ʼ����
	ReceiveingFlag = 0;
	SendingFlag = 0;
	SentDone = 0;
	ctrlFlag = 0;
	LoadArg();
	RestoreFlag = HAVEDONE;        //Tzh RestoreFlag��HAVEDONE�⣬δ������ֵ��
	RestoreFlagB0 = HAVEDONE;
	RestoreFlagB1 = HAVEDONE;
}

/**************************************************************************
ģ������:	������ģ��
ģ������:	1.���ó�ʼ��ģ���ϵͳ���г�ʼ��;
			2.�Լ����Զ������Լ��¶ȿ��Ƶı������г�ʼ����
			3.����ѭ��ִ��ģ��;
�ȳ�:		3-InitProgram()/ClearDog()/Exec_Loop()��
����:		1-main()
�������:	��
�������:	��
����ֵ:		��
�޸ļ�¼:	
����˵��:	
**************************************************************************/ 
void main(void)
{
	unsigned char i,tChar;
	IE = 0;	/*ȫ���жϽ�ֹ*/ 
	TCON = 0;
	InitProgram();         /*�����ʼ��ģ��*/

	/*-----------------------------------------------------------------------*/ 
	TMOD = 0x21;					/*T1�����ڷ�ʽ2,T0������ʽ1*/
	TH0 = 0xFF;						/*��ʱ��������Լ17΢����붨ʱ�ж�*/
	TL0 = 0xF0; 		  
	IP = 3;							/*�ж����ȼ�:INTO(�ⲿ0)��INT1(ʱ��0)��INT4(����)*/
	TR0 = 1;						/*����ET0����*/
	IE = 0x82;						/*����ET0�жϣ����ж�*/
	while(!ctrlFlag);				/*�ȴ��ɼ���һ������*/ //Tzh ��������δ֪
	IE = 0;	/*ȫ���жϽ�ֹ*/ 
 
	TH1 = 0xFD;						/*���ڲ�����9600*/
	TL1 = 0xFD;
	PCON = 0x7F;					/*�����ʲ��ӱ�*/
	TR1 = 1;						/*����ET1����*/
	SCON = 0xD0;
	SM2 = 1;
	ClearDog();						/*���Ź���λ*/
	ExternInt0_Enable = 1;			/*���Ź��ж���������*/
	IT0 = 0;						/*�ж�(INT0)��ƽ����*/
	RI = 0;							/*��������жϱ�־*/
	TI = 0;
	IE = 0x93;						/*�����Ź�(EX0--INT0)�жϡ�ʱ��(ET0--INT1)�жϡ������ж�(ES--INT4)�����ж�*/
	while (1)
	{
		
		while(!ctrlFlag);
		switch(wkStatus)
		{
		case Auto_TempCtrl_Status:	/*�¿س���ѭ��ִ��ģ��*/
			Exec_wkLoop();
			break;
		case Test_Loop_Status:		/*Ѱ�����ִ��ģ��*/
			Exec_AutoProcess();
			Test_Loop_Status &= Test_Status;
			break;
		case Save_Arg_Status:		/*�������*/
			EA = 0;
			SaveArg();
			wkStatus = Idel_Status;
			EA = 1;
			break;
		default:
			break;
		}
		ClearDog(); 	  /*���Ź���λ*/
	}
}

/**************************************************************************
ģ������:	ѭ��ִ��ģ��
ģ������:	1.�������ģ��;
			2.���������ϱ�־λ����;
			3.ģ�����ɼ�;
			4.��Mode_FlagΪ0ʱ�����������¶ȿ���;
			5.ִ������ָ�������ִ��;
			6.����ͨ���Զ�Ѳ���ӳ���;
			7.�����Լ���������֡.
�ȳ�:		5-TempCtrl_L1to6()/TempCtrl_L7()/TempCtrl_L8()/TempCtrl_L9()/TempCtrl_L10()/TempCtrl_L11()
����:		1-main()
�������:	��
�������:	��
����ֵ:		��
�޸ļ�¼:	
����˵��:	
**************************************************************************/ 
void Exec_wkLoop(void)
{	
	unsigned char td; 	
	AncStandard = READ_EXTERNRAM(AnC_Buf);
	for(td = 0 ; td < 6 ; td ++)
	{
		if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
		{
			TempCtrl_L1to6(td);
		}
		else
		{
			break;
		}
	}
	if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
	{
		TempCtrl_L7();
	}
	if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
	{
		TempCtrl_L8();
	}
	if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
	{
		TempCtrl_L9();
	}
	if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
	{
		TempCtrl_L10();
	}
	if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
	{
		TempCtrl_L11();
	}
	if(ctrlFlag && (Auto_TempCtrl_Status == wkStatus))
	{
		XBYTE[ADDR_IOSEL1] = pp[0];                          /* ADDR_IOSEL1 ���ȿ������ */
		XBYTE[ADDR_IOSEL2] = pp[1];                          /* ADDR_IOSEL2 ���ȿ������ */
		XBYTE[ADDR_IOSEL3] = pp[2];                          /* ADDR_IOSEL3 ���ȿ������ */
		XBYTE[ADDR_IOSEL4] = pp[3];                          /* ADDR_IOSEL4 ���ȿ������ */
		XBYTE[ADDR_IOSEL5] = pp[4];                          /* ADDR_IOSEL5 ���ȿ������ */
		XBYTE[ADDR_IOSEL6] = pp[5];                          /* ADDR_IOSEL6 ���ȿ������ */
	}
}


/*************************************************************************
ģ������: �������Լ�ѭ����������
ģ������: 1.��40·����ͨ������ѭ�����
		  2.ÿ������ͨ����ʱ��1s,�رա�����˳����ԣ�ֹͣѭ��
����:	  0-��	
�������: ��	
����ֵ:	  ��	
�޸ļ�¼:	
����˵��:	
**************************************************************************/
void Exec_AutoProcess(void)
{
	unsigned int i;
	XBYTE[ADDR_IOSEL1] = 0x00;  			 /*40�����������*/ 
	XBYTE[ADDR_IOSEL2] = 0x00;  
	XBYTE[ADDR_IOSEL3] = 0x00;
	XBYTE[ADDR_IOSEL4] = 0x00;
	XBYTE[ADDR_IOSEL5] = 0x00;
	XBYTE[ADDR_IOSEL6] = 0x00;
	/*����1���½������ӳ�,����Ϊ1�������ؿ�ʼ*/
	while(ctrlFlag);		//�ȵ��½���
	while(!ctrlFlag);		//�ȵ�������
	for(i = 0 ; i < 40 ; i++)
	{
		XBYTE[HEAT_ADR[i]] = OPEN_CMD[i];
		ClearDog(); 	  /*���ܵȴ���1��,���Ź���λ*/
		while(ctrlFlag);
		while(!ctrlFlag);
		XBYTE[HEAT_ADR[i]] = 0;
		if(Test_Loop_Status != wkStatus)	break;
	}
}


/*******************************************************************************
ģ������:  д������ģ��
ģ������:  ������д����
�ȳ�:	   0-��	
�������:  ��	
����ֵ:	   ��	
�޸ļ�¼:	
����˵��:	
*******************************************************************************/
void WRITE_EXTERNRAM(unsigned int Offset, unsigned int Data)
{
	XBYTE[RamStartAdr + (Offset * 2)] = (unsigned char) Data;		/*д���ֽ� */
	XBYTE[RamStartAdr + (Offset * 2) + 1] = (unsigned char) (Data >> 8);/*д���ֽ� */
}
/*******************************************************************************
ģ������:  ��������ģ��
ģ������:  �����ֶ�����
�ȳ�:	   0-	
����:	   3-BusInitial/BusRamTest/CheckBusStatus	
�������:  Offset	
�������:  ��	
����ֵ:	   ��	
�޸ļ�¼:	
����˵��:	
*******************************************************************************/
unsigned int READ_EXTERNRAM(unsigned int Offset)
{
	unsigned char LByte;
	unsigned char HByte;
	HByte = XBYTE[RamStartAdr + (Offset * 2) + 1]; /*�������ֽ�*/
	LByte = XBYTE[RamStartAdr + (Offset * 2)];	 /*�������ֽ�*/
	return (unsigned int) ((HByte << 8) + LByte);	/*���ض�������*/
}
/**************************************************************************
ģ������: ���Ź���λ��ģ��
ģ������: �����Ź���������λ��
�ȳ�:	  1_nop_() 
����:	  1)/main()/
�������: ��			
�������: ��	
����ֵ:	  ��	
�޸ļ�¼: ��	
����˵��:
**************************************************************************/
void ClearDog(void)
{
	ADDR_DOG = 0;		/*���Ź���λ*/
	_nop_();			/*��ʱ,���ٱ���2��ָ������*/
	_nop_();			
	ADDR_DOG = 1;		/*�����Ź���ʱ�������ж�*/
}
