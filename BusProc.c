#include "Extern.h"
extern void RestoreTmpArg(void);
/**************************************************************************
ģ������:	������������Ӧ��
ģ������:	���һ֡��������
�������:	Ӧ��������
�������:	��
����ֵ:		��
ͨѶ����֡�ṹ:	�¿����ӵ�ַ(9bits)��������(1Byte-retCmdCode)�����ݳ���L=0�����У��=0.
�޸ļ�¼:	
����˵��:	�յ����͵�����ַ����ָ��(�ӵ�ַmyAdr),��֯��Ӧ����Ӧ��֡,100���뷢��.���100������δ����,������֡.
**************************************************************************/ 

void sendFrame(unsigned char retCmdCode)
{
	if(SendingFlag) return;
	SendingFlag = 1;
	SendCount = 0;
	SentDone = 1;
	CommuniationDisable = 0;
	SendBuff.CLength = 0;
	SendBuff.Cmd = cmdCode;      //Tzh ��δ������ڷ���FrameErr�ȴ���Ӧ��֡��Ӧ�ڸ���ʹ���βδ����retCmdCode������cmdCodeδ���ж���
	SendBuff.Length = 0;
	TI = 0;
	TB8 = 1;
	SBUF = myAdr;
}
/**************************************************************************
ģ������:	�������߷����Լ���
ģ������:	���һ֡��������
�������:	Ӧ��������
�������:	��
����ֵ:		��
ͨѶ����֡�ṹ:	�¿����ӵ�ַ(9bits)��������(1Byte-SelfTest_ANSW)�����ݳ���L=1������(1Bytes:selfTestStatus)�����У��=selfTestStatus.
�޸ļ�¼:	
����˵��:	�յ����͵�����ַ�Լ�ָ��(�ӵ�ַmyAdr),��֯�Լ�����Ӧ��֡,100���뷢��.���100������δ����,������֡.
**************************************************************************/ 

void sendTestResult(void)
{
	if(SendingFlag) return;
	SendingFlag = 1;
	SendCount = 0;
	SentDone = 1;
	CommuniationDisable = 0;
	SendBuff.CLength = 0;
	SendBuff.Cmd = SelfTest_ANSW;
	SendBuff.Length = 1;
	TI = 0;
	TB8 = 1;
	SBUF = myAdr;
}
/**************************************************************************
ģ������:	�������߷���ң��
ģ������:	���ң��֡��������
�������:	��
�������:	��
����ֵ:		��
ͨѶ����֡�ṹ:	�¿����ӵ�ַ(9bits)��������(1Byte)�����ݳ���L=62�����ݡ����У��.
�޸ļ�¼:	
����˵��:	�յ����͵�����ַң��ָ��(�ӵ�ַmyAdr),��֯ң��֡,100���뷢��.���100������δ����,������֡.
**************************************************************************/ 

void sendTM(void)
{
	int i,j;
	unsigned char channelstatus;
	ICUnion tmp;

	if(SendingFlag) return;
	EA = 0;
	/*�¶����ݷ���0x10~0x27*/
	for(i = 0 ; i < 24 ; i++)		//���µ�1~24�¶�ֵ:ÿ���¶�ֵ��Чλ12λ����4λΪ0;���ֽ���ǰ�����ֽ��ں�

	{
		tmp.in = READ_EXTERNRAM(AnC_Buf + i +0x10);
		XBYTE[ReceiveBuff.BuffAdr+6+i*2] = tmp.ch[0];
		XBYTE[ReceiveBuff.BuffAdr+7+i*2] = tmp.ch[1];
	}
	XBYTE[ReceiveBuff.BuffAdr+0] = synTime.TS[2];	//�¿����ڲ�ʱ��:���ֵ��ֽ�
	XBYTE[ReceiveBuff.BuffAdr+1] = synTime.TS[3];	//���ָ��ֽ�
	XBYTE[ReceiveBuff.BuffAdr+2] = synTime.TS[0];	//���ֵ��ֽ�
	XBYTE[ReceiveBuff.BuffAdr+3] = synTime.TS[1];	//���ָ��ֽ�
	XBYTE[ReceiveBuff.BuffAdr+4] = Status;
	XBYTE[ReceiveBuff.BuffAdr+5] = RetCode;
	tmp.in = READ_EXTERNRAM(AnC_Buf);
	XBYTE[ReceiveBuff.BuffAdr+54] = tmp.ch[0];
	XBYTE[ReceiveBuff.BuffAdr+55] = tmp.ch[1];
	/*ͨ��״̬���ݷ���0x30~0x47��0x50~0x5f*/
	for(j = 0 ; j < 3; j++)
	{
		for (channelstatus = 0 , i = 0 ; i < 8 ; i++ , channelstatus >>= 1)	//ȡ0x30~0x47
		{
			tmp.in = READ_EXTERNRAM(AnC_Buf + 0x30 + j* 8 + i);
			if(tmp.in < HEAT_THRESHOLD)//Tzh Ӧ���Ǵ��ڣ�503���
			{
				channelstatus |= 0x80;
			}
		}
		XBYTE[ReceiveBuff.BuffAdr+56+j] = channelstatus;
	}
	for(j = 0 ; j < 2; j++)
	{
		for (channelstatus = 0 , i = 0 ; i < 8 ; i++ , channelstatus >>= 1)	//ȡ0x30~0x47
		{
			tmp.in = READ_EXTERNRAM(AnC_Buf + 0x50 + j* 8 + i);
			if(tmp.in  < HEAT_THRESHOLD)
			{
				channelstatus |= 0x80;
			}
		}
		XBYTE[ReceiveBuff.BuffAdr+59+j] = channelstatus;
	}
	EA = 1;
	SendingFlag = 1;
	SendCount = 0;
	SentDone = 1;
	CommuniationDisable = 0;
	SendBuff.CLength = 0;
	SendBuff.Cmd = TM_ANSW;
	SendBuff.Length = 61;
	TI = 0;
	TB8 = 1;
	SBUF = myAdr;
}

/**************************************************************************
ģ������:	���������ж�ģ��
ģ������:	�����жϼĴ���
�������:	��
�������:	��
����ֵ:		��
ͨѶ����֡�ṹ:	�¿����ӵ�ַ(9bits)��������(1Byte)�����ݳ���L������(L_Bytes)�����У��(1Byte--�����������У��).
�޸ļ�¼:	
����˵��:	�յ����͵�����ַָ��(�ӵ�ַmyAdr)��㲥ָ��(�ӵ�ַBoardCastAdr),15��������.���15������λ����,������֡.
**************************************************************************/ 
void SBusService(void) interrupt 4 using 2
{
	unsigned char register rChar;
	unsigned char register i,j;
	unsigned char register switch_index;
	ICUnion register rArg;
	if(RI == 1)
	{
		RI = 0;//Tzh �����ж��������ˣ��Ƿ���ܵ����ֽ������µ��ж�
		rChar = SBUF;
		if(RB8 == 1)
		{
			if (rChar == myAdr || rChar == BoardCastAdr)
			{
				SM2 = 0;
				ReceiveBuff.CLength = 0;
				ReceiveBuff.CXor = 0;
				ReceiveBuff.Cmd = 0;
				ReceiveBuff.Length = 0;
				ReceiveBuff.StartAdr = 0;
				ReceiveCount = 0;
				ReceiveingFlag = 1;		//��ʱ��ʼ
				if(rChar == myAdr)		//��������
				{
					BoardcastFlag = 0;
				}
				else					//�㲥����
				{
					BoardcastFlag = 1;
				}
			}
		}
		else
		{
			switch(ReceiveBuff.CLength)
			{
			case 0:
					if(BoardcastFlag && (rChar != SYN_CMD))		//�㲥��ͬ������:������֡,����Ӧ
					{
						SM2 =1;
						ReceiveingFlag = 0;		//ֹͣ��ʱ
					}
					else
					{
						ReceiveBuff.Cmd = rChar;
						ReceiveBuff.CLength ++;
					}
					break;
			case 1:
					if (rChar > Max_Frame_dL)					//���ݳ��ȴ���40�ֽ�:������֡
					{
						SM2 =1;
						ReceiveingFlag = 0;		//ֹͣ��ʱ
					}
					else		//��¼���ݳ���,��ʼ������
					{
						ReceiveBuff.Length = rChar;
						ReceiveBuff.CXor = 0;
						ReceiveBuff.StartAdr = 0;
						ReceiveBuff.CLength ++;
					}
					break;
			default:
					ReceiveBuff.CXor ^ = rChar;
					if(vaild_Status == wkStatus)
					{
						if (ReceiveBuff.StartAdr == ReceiveBuff.Length)
						{
							ReceiveingFlag = 0;		//ֹͣ��ʱ
							SM2 =1;
							sendTestResult();	//�ϱ��Լ���:�����Լ���֡,��������
						}
						else
						{
							ReceiveBuff.StartAdr++;
						}
					}
					else if (ReceiveBuff.StartAdr == ReceiveBuff.Length)		//�������
					{
						ReceiveingFlag = 0;		//ֹͣ��ʱ
						SM2 =1;
						if(ReceiveBuff.CXor == 0)		//У����ȷ
						{
							switch(ReceiveBuff.Cmd)
							{
							case SYN_CMD:		//ͬ������    //Tzh �������3.1��������ͬ��������Ӧ���ڵȴ�״̬������ڽ��յ�ͬ�������δ�Ե�ǰ״̬�Ƿ�Ϊ�ȴ�״̬�����ж�
								if(ReceiveBuff.Length == SYN_CMD_L)
								{
									EA = 0;
									synTime.TS[0] = XBYTE[ReceiveBuff.BuffAdr+3];	//���ֵ��ֽ�//Tzh ����ڽ���ʱ��ͬ��ʱ������ʱ��Ϊ�ȸߺ��С�����У������ֵʱΪ�ȸߺ�ʹ������
									synTime.TS[1] = XBYTE[ReceiveBuff.BuffAdr+2];	//���ָ��ֽ�
									synTime.TS[2] = XBYTE[ReceiveBuff.BuffAdr+1];	//���ֵ��ֽ�
									synTime.TS[3] = XBYTE[ReceiveBuff.BuffAdr+0];	//���ָ��ֽ�
									EA = 1;
								}
								else	//֡��ʽ��Ч
								{
									if(!BoardcastFlag)		//��������,����֡�ṹ��//Tzh ����3.3.2.1��������ͬ��ָ���Ϊ�㲥Ҳ��Ϊ���㣬���յ�����ͬ��ָ�����ݳ��ȴ����Ƿ�Ӧ�����������δ��ȷ
									{
										sendFrame((unsigned char)FrameErr);
									}
								}
								break;
							case SelfTest_CMD:		//ȡ�Լ�������
								if(ReceiveBuff.Length == SelfTest_CMD_L)
								{
									sendTestResult();	//�ϱ��Լ���:�����Լ���֡,��������
								}
								else	//֡��ʽ��Ч
								{
									sendFrame((unsigned char)FrameErr);
								}
								break;
							case TM_CMD:			//״̬ң������
								if(ReceiveBuff.Length == TM_CMD_L)
								{
									sendTM();			//�ϱ�ң��:����ң��֡,��������
								}
								else	//֡��ʽ��Ч
								{
									sendFrame((unsigned char)FrameErr);
								}
								break;
							case WKStart_CMD:		//�����Զ��¿�����//Tzh �������3.1����δ�����Զ��¿�״̬������Ӧ�Զ��¿�����
								if(ReceiveBuff.Length == WKStart_CMD_L)
								{
									if((Idel_Status == wkStatus) || (Auto_TempCtrl_Status == wkStatus))  //��ǰ״̬Ϊ����̬/�Զ��¿�̬,����ת��Ϊ�¿�̬
									{
										if(Idel_Status == wkStatus)
										{
											RetCode = 0xAA;			/*�¿���״̬�ָ�����*/
											/*40�����������*/
											XBYTE[ADDR_IOSEL1] = 0x00;  			  
											XBYTE[ADDR_IOSEL2] = 0x00;  
											XBYTE[ADDR_IOSEL3] = 0x00;
											XBYTE[ADDR_IOSEL4] = 0x00;
											XBYTE[ADDR_IOSEL5] = 0x00;
											XBYTE[ADDR_IOSEL6] = 0x00;
											pp[0] = 0;
											pp[1] = 0;
											pp[2] = 0;
											pp[3] = 0;
											pp[4] = 0;
											pp[5] = 0;
										}
										wkStatus = Auto_TempCtrl_Status;	//���õ�ǰ״̬Ϊ�¿�̬
										sendFrame((unsigned char)WKStart_ANSW);		//״̬ת���ɹ�,���������Զ��¿�Ӧ��
									}
									else	//��ǰ״̬��Ϊ����̬,ά�ֵ�ǰ̬
									{
										sendFrame((unsigned char)TimErr);	//����ת��״̬:ʱ�����
									}
								}
								else	//֡��ʽ��Ч
								{
									sendFrame((unsigned char)FrameErr);
								}

								break;
							case TestStart_CMD:		//������������
								if(ReceiveBuff.Length == TestStart_CMD_L)
								{
									if(Auto_TempCtrl_Status != wkStatus)  //��ǰ״̬Ϊ����̬/����̬,����ת��Ϊ����̬
									{
										wkStatus |= Test_Status;	//���õ�ǰ״̬Ϊ����̬
										sendFrame((unsigned char)TestStart_ANSW);		//״̬ת���ɹ�,������������Ӧ��
									}
									else	//��ǰ״̬��Ϊ����̬,ά�ֵ�ǰ̬
									{
										sendFrame((unsigned char)TimErr);	//����ת��״̬:ʱ�����
									}
								}
								else	//֡��ʽ��Ч
								{
									sendFrame((unsigned char)FrameErr);
								}
								break;
							case WKorTestStop_CMD:		//ֹͣ�Զ��¿�����,��ֹͣ����������ͬ
								if(ReceiveBuff.Length == WKorTestStop_CMD_L)
								{
									if((Idel_Status == wkStatus) || (Save_Arg_Status == wkStatus))		//��ǰ״̬Ϊ����̬
									{
										sendFrame((unsigned char)illCMD_ANSW);
									}
									else if(Auto_TempCtrl_Status == wkStatus)	//��ǰ״̬Ϊ�Զ��¿�̬
									{
										sendFrame((unsigned char)WKStop_ANSW);	//ֹͣ�Զ��¿�Ӧ��
									}
									else	//��ǰ״̬Ϊ����̬
									{
										sendFrame((unsigned char)TestStop_ANSW);	//ֹͣ����Ӧ��
									}
									wkStatus = Idel_Status;				//���õ�ǰ״̬Ϊ����
								}
								else	//֡��ʽ��Ч
								{
									sendFrame((unsigned char)FrameErr);		//Ӧ��:������Ч
								}
								break;
							case LCStart_CMD:	//����Ѳ������
								if(ReceiveBuff.Length == LCStart_CMD_L)
								{
									if((Test_Status == wkStatus) || (Test_Loop_Status == wkStatus))	//��ǰ״̬Ϊ����̬
									{
										if(Test_Status == wkStatus)//Tzh ��·ִ�д򿪡��رգ�������Ϊȫ���ر�
										{
											XBYTE[ADDR_IOSEL1] = 0x00;  			  
											XBYTE[ADDR_IOSEL2] = 0x00;  
											XBYTE[ADDR_IOSEL3] = 0x00;
											XBYTE[ADDR_IOSEL4] = 0x00;
											XBYTE[ADDR_IOSEL5] = 0x00;
											XBYTE[ADDR_IOSEL6] = 0x00;
											pp[0] = 0;
											pp[1] = 0;
											pp[2] = 0;
											pp[3] = 0;
											pp[4] = 0;
											pp[5] = 0;
										}
										wkStatus = Test_Loop_Status;	//���õ�ǰ״̬Ϊ����̬Ѳ��
										sendFrame((unsigned char)LCStart_ANSW);		//������������Ѳ��Ӧ��
									}
									else  //��ǰ״̬��Ϊ����̬
									{
										sendFrame((unsigned char)TimErr);	//����Ѳ��:ʱ�����
									}
								}
								else	//֡��ʽ��Ч
								{
									sendFrame((unsigned char)FrameErr);
								}
								break;
							case DisChannel_CMD:	//����ͨ���Ͽ�����
								if(ReceiveBuff.Length == DisChannel_CMD_L)
								{
									if(Test_Status == wkStatus)	//��ǰ״̬Ϊ����̬
									{
										/*����ͨ���Ͽ�*/
										pp[0] = 0;
										pp[1] = 0;
										pp[2] = 0;
										pp[3] = 0;
										pp[4] = 0;
										pp[5] = 0;
										XBYTE[ADDR_IOSEL1] = pp[0];
										XBYTE[ADDR_IOSEL2] = pp[1];
										XBYTE[ADDR_IOSEL3] = pp[2];
										XBYTE[ADDR_IOSEL4] = pp[3];
										XBYTE[ADDR_IOSEL5] = pp[4];
										XBYTE[ADDR_IOSEL6] = pp[5];
										sendFrame((unsigned char)DisChannel_ANSW);		//�����������Լ���ͨ���Ͽ�Ӧ��
									}
									else  //��ǰ״̬��Ϊ����̬
									{
										sendFrame((unsigned char)TimErr);	//���ܶϿ�:ʱ�����
									}
								}
								else	//֡��ʽ��Ч
								{
									sendFrame((unsigned char)FrameErr);
								}
								break;
							case EnChannel_CMD:	//����ͨ����ͨ����
								if(ReceiveBuff.Length == EnChannel_CMD_L)
								{
									if(Test_Status == wkStatus)	//��ǰ״̬Ϊ����̬
									{
										/*����ͨ����ͨ*/
										pp[0] = IO_CMD1;  			  
										pp[1] = IO_CMD2;  
										pp[2] = IO_CMD3;
										pp[3] = IO_CMD4;
										pp[4] = IO_CMD5;
										pp[5] = IO_CMD6;
										XBYTE[ADDR_IOSEL1] = pp[0];  			  
										XBYTE[ADDR_IOSEL2] = pp[1];  
										XBYTE[ADDR_IOSEL3] = pp[2];
										XBYTE[ADDR_IOSEL4] = pp[3];
										XBYTE[ADDR_IOSEL5] = pp[4];
										XBYTE[ADDR_IOSEL6] = pp[5];
										sendFrame((unsigned char)EnChannel_ANSW);		//���ͼ���ͨ����ͨ����Ӧ��
									}
									else  //��ǰ״̬��Ϊ����̬
									{
										sendFrame((unsigned char)TimErr);	//���ܽ�ͨ����ͨ��:ʱ�����
									}
								}
								else	//֡��ʽ��Ч
								{
									sendFrame((unsigned char)FrameErr);
								}
								break;
							case DEAChannel_CMD:	//��Ҫ����Ƹ�����ͨ������
								if(ReceiveBuff.Length == DEAChannel_CMD_L)
								{
									if(Test_Status == wkStatus)	//��ǰ״̬Ϊ����̬
									{
										/*����ͨ����Ҫ���ͨ/�Ͽ�*/
										sendFrame((unsigned char)DEAChannel_ANSW);		//���Ͱ�Ҫ����Ƹ�����ͨ������Ӧ��
										for(switch_index = 0 , i = 0 ; i < DEAChannel_CMD_L ; i++)
										{
											rChar = XBYTE[ReceiveBuff.BuffAdr + i];
											for(j =0 ; j < 8; j++ , switch_index++)
											{
												if(1 == (rChar & 0x01))
												{
													pp[HEAT_INDEX[switch_index]] |= OPEN_CMD[switch_index];
												}
												else
												{
													pp[HEAT_INDEX[switch_index]] &= ~OPEN_CMD[switch_index];
												}
												rChar >>= 1;
											}
										}
										XBYTE[ADDR_IOSEL1] = pp[0];
										XBYTE[ADDR_IOSEL2] = pp[1];
										XBYTE[ADDR_IOSEL3] = pp[2];
										XBYTE[ADDR_IOSEL4] = pp[3];
										XBYTE[ADDR_IOSEL5] = pp[4];
										XBYTE[ADDR_IOSEL6] = pp[5];
									}
									else  //��ǰ״̬��Ϊ����̬
									{
										sendFrame((unsigned char)TimErr);	//:ʱ�����
									}
								}
								else	//֡��ʽ��Ч
								{
									sendFrame((unsigned char)FrameErr);
								}
								break;
							case DESChannel_CMD:	//��Ҫ����Ƶ�����ͨ������//Tzh ����ָ���ʽ֡����δ�����жϣ�����Ӧ��FrameErr
								if(Test_Status == wkStatus)  //��ǰ״̬Ϊ����̬
								{
									rCmdErr = 0;
									for(i = 0 ; i < ReceiveBuff.Length ; i++)
									{
										switch_index = (XBYTE[ReceiveBuff.BuffAdr + i] & (~SwitchMask));//Tzh ��仰���˸����ţ�
										if((switch_index == 0) || (switch_index > 40 ))
										{
											rCmdErr = 1;
											break;
										}
									}
									if(rCmdErr)
									{
										sendFrame((unsigned char)DESChannel_Err);		//���ͼ���ͨ����Χ��
									}
									else
									{
										sendFrame((unsigned char)DESChannel_ANSW);		//���Ͱ�Ҫ����Ƶ�����ͨ������Ӧ��
										for(i = 0 ; i < ReceiveBuff.Length ; i++)
										{
											rChar = XBYTE[ReceiveBuff.BuffAdr + i];
											switch_index = (rChar & (~SwitchMask)) - 1;
											rChar &= SwitchMask;
											if(rChar == SwitchOn)
											{
												pp[HEAT_INDEX[switch_index]] |= OPEN_CMD[switch_index];
											}
											else if(rChar == SwitchOff)
											{
												pp[HEAT_INDEX[switch_index]] &= ~OPEN_CMD[switch_index];
											}
											else
											{
											}
										}
										XBYTE[ADDR_IOSEL1] = pp[0];
										XBYTE[ADDR_IOSEL2] = pp[1];
										XBYTE[ADDR_IOSEL3] = pp[2];
										XBYTE[ADDR_IOSEL4] = pp[3];
										XBYTE[ADDR_IOSEL5] = pp[4];
										XBYTE[ADDR_IOSEL6] = pp[5];
									}
								}
								else	//��ǰ״̬��Ϊ����̬
								{
									sendFrame((unsigned char)TimErr);	//���ܽ�ͨ����ͨ��:ʱ�����
								}
								break;
							case ArgLoad_CMD:	//����װ������
								if(ReceiveBuff.Length == ArgLoad_CMD_L)
								{
									if(Idel_Status == wkStatus)  //��ǰ״̬Ϊ�ȴ�̬
									{
										rArg.ch[0] = XBYTE[ReceiveBuff.BuffAdr + 0];
										rArg.ch[1] = XBYTE[ReceiveBuff.BuffAdr + 1];
										Argument.LDT = rArg.in;//Tzh ����������װ��ָ����δ��ȷ�ߵ��ֽڴ��˳�򣬿��ܵ������ݴ���
										rArg.ch[0] = XBYTE[ReceiveBuff.BuffAdr + 2];
										rArg.ch[1] = XBYTE[ReceiveBuff.BuffAdr + 3];
										Argument.HDT = rArg.in;
										rArg.ch[0] = XBYTE[ReceiveBuff.BuffAdr + 4];
										rArg.ch[1] = XBYTE[ReceiveBuff.BuffAdr + 5];
										Argument.CLDT = rArg.in;
										rArg.ch[0] = XBYTE[ReceiveBuff.BuffAdr + 6];
										rArg.ch[1] = XBYTE[ReceiveBuff.BuffAdr + 7];
										Argument.LT = rArg.in;
										rArg.ch[0] = XBYTE[ReceiveBuff.BuffAdr + 8];
										rArg.ch[1] = XBYTE[ReceiveBuff.BuffAdr + 9];
										Argument.HT = rArg.in;
										rArg.ch[0] = XBYTE[ReceiveBuff.BuffAdr + 10];
										rArg.ch[1] = XBYTE[ReceiveBuff.BuffAdr + 11];
										Argument.QLT = rArg.in;

										rChar = 0xC0;
										//���¹�����LT�����¶�������LDT�����¶�������HDT�����¹�����HT
										if((Argument.LT >= Argument.LDT) || (Argument.LDT >= Argument.HDT) || (Argument.HDT >= Argument.HT))
										{
											rChar += 1;
										}
										//���¹�����LT��������¶�������CLDT�����¶�������HDT
										if((Argument.LT >= Argument.CLDT) || (Argument.CLDT >= Argument.HDT))
										{
											rChar += 2;
										}
										//��ƿ���¹�����QLT�����¶�������LDT
										if(Argument.QLT >= Argument.LDT)
										{
											rChar += 4;
										}
										if(rChar == 0xC0)
										{
											wkStatus = Save_Arg_Status;
											sendFrame((unsigned char)ArgLoad_ANSW);		//���Ͳ���װ������Ӧ��
										}
										else
										{
											sendFrame(rChar);		//���Ͳ�����Ӧ��
											RestoreTmpArg();		//�ָ�����
										}
									}
									else	//��ǰ״̬��Ϊ�ȴ�̬
									{
										sendFrame((unsigned char)TimErr);	//:ʱ�����
									}
								}
								else	//֡��ʽ��Ч
								{
									sendFrame((unsigned char)FrameErr);
								}
								break;
							default:	//����ʶ�������
								BoardcastFlag = 0;
								sendFrame((unsigned char)CmdErr);	//�����������
								break;

							}
						}
						else	//У�����
						{
							if(!BoardcastFlag)		//�ǹ㲥��ʽ:Ӧ��У���
							{
								sendFrame((unsigned char)XorErr);
							}
						}
					}
					else
					{
						XBYTE[ReceiveBuff.BuffAdr+ReceiveBuff.StartAdr] = rChar;
						ReceiveBuff.StartAdr++;
					}
					break;
			}
		}
	}
	else if(TI == 1)
	{
		TI = 0;
		if(SendingFlag)
		{
			if(SentDone)
			{
				TB8 = 0;
				SentDone = 0;
			}
			switch(SendBuff.CLength)
			{
			case 0:
				SBUF = SendBuff.Cmd;
				SendBuff.CLength++;
				break;
			case 1:
				SBUF = SendBuff.Length;
				SendBuff.CXor = 0;
				SendBuff.StartAdr = 0;
				SendBuff.CLength++;
				break;
			default:
				if(SendBuff.Length > SendBuff.StartAdr)
				{
					if(1 == SendBuff.Length)
					{
						SBUF = Status;
						SendBuff.CXor ^ = RetCode;
					}
					else
					{
						SBUF = XBYTE[SendBuff.BuffAdr+SendBuff.StartAdr];
						SendBuff.CXor ^ = XBYTE[SendBuff.BuffAdr+SendBuff.StartAdr];
					}
				}
				else if(SendBuff.Length == SendBuff.StartAdr)
				{
					SBUF = SendBuff.CXor;
				}
				else
				{
					SendingFlag = 0;
					CommuniationDisable = 1;
				}
				SendBuff.StartAdr++;
				break;
			}
		}

	}
}

