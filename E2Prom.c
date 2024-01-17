#include "Extern.h"
void SaveArg(void);
void LoadArg(void);
void SaveTmpArg(void);
void RestoreTmpArg(void);
//-----------------------------------------------------------------------------
// IO��ģ��I2Cʱ�򣬹�6������
//-----------------------------------------------------------------------------

void StartI2C(void)
{
    ICDATA = 1;        //������ʼ�����������ź�//Tzh �����������Ӧ����ʱ���ٷ�����
	ICLK = 1;        
	_nop_();
	ICDATA = 0;        //������ʼ�ź�
	_nop_();
	ICLK = 0;        //ǯסI2C���ߣ�׼�����ͻ��������
	_nop_();
	_nop_();
}

void StopI2C(void)
{
    ICDATA = 0;        //���ͽ��������������ź�//Tzh ͬ��ʼ
	ICLK = 1;        
	_nop_();
	ICDATA = 1;        //������ʼ�ź�
	_nop_();
	ICLK = 0;        //���ͽ���
	_nop_();
	_nop_();
}

void WriteByte(unsigned char uchar_sbc)
{
    unsigned char uchar_BitCnt;

	for(uchar_BitCnt = 0 ; uchar_BitCnt < 8 ; uchar_BitCnt ++ , uchar_sbc <<= 1)    //Ҫ���͵����ݳ���8λ
	{
	    if(0x80 == (uchar_sbc & 0x80))               //�жϷ���λ
		{
			ICDATA = 1;
		}
        else
		{
			ICDATA = 0;
		}
		ICLK = 1;                                           //��ʱ����Ϊ�ߣ�֪ͨ����������ʼ��������λ
		_nop_();
		_nop_();
		_nop_();

		ICLK = 0;
		_nop_();
	}

	ICDATA = 1;		//�������Ӧ��
	ICLK=0;
	_nop();
	_nop();
}

unsigned char ReadByte(void)
{
    unsigned char uchar_Retc;
	unsigned char uchar_BitCnt;

	for(uchar_BitCnt = 0 ,uchar_Retc = 0; uchar_BitCnt < 8 ; uchar_BitCnt ++ , uchar_Retc <<= 1)
	{
		ICLK = 0;                                             //��ʱ����Ϊ�ͣ�׼����������λ
		_nop_();

		ICLK = 1;                                             //��ʱ����Ϊ�ߣ�ʹ��������������Ч
		_nop();
		_nop();
		if(ICDATA == 1)
		{
			uchar_Retc = uchar_Retc + 1;
		}
	}
    ICLK = 0;
	_nop_();
	_nop_();
	return(uchar_Retc);                                      //�����յ�������
}


//-----------------------------------------------------------------------------
// ��E2PROMд������,���ô˺���ǰ������ж�
//-----------------------------------------------------------------------------
void WriteE2pByte(unsigned int uint_Addr,unsigned char uchar_DataWrite)      
{
    unsigned char uchar_AddrMsb;
    unsigned char uchar_AddrLsb;

    uchar_AddrLsb = (unsigned char)uint_Addr;
    uchar_AddrMsb = (unsigned char)(uint_Addr >> 8);
    //ICWRDisable = 0;                                          //�����д

    StartI2C();
    WriteByte(0xA6);                       //�����ĸ�4λΪA,��ַΪ��21(11)����дλ��0��0��д��״̬����
    WriteByte(uchar_AddrMsb);
    WriteByte(uchar_AddrLsb);
    WriteByte(uchar_DataWrite);
    StopI2C();
    //ICWRDisable = 1;                               //��ֹ��д
}


//-----------------------------------------------------------------------------
// ��ȡE2PROM������,���ô˺���ǰ������ж�
//-----------------------------------------------------------------------------
unsigned char ReadE2pByte(unsigned int uint_Addr)      //ֻ��������ȡ������
{
    unsigned char uchar_AddrMsb;
    unsigned char uchar_AddrLsb;

    unsigned char uchar_AddrReadData;

    uchar_AddrLsb = uint_Addr;								
    uchar_AddrMsb = uint_Addr >> 8;							//Tzh ��дȡ��ַ����һ��
    
    //ICWRDisable = 0;                                          //�����д

    StartI2C();
    WriteByte(0xA7);                                  //�����ĸ�4λΪA,��ַΪ��21(11)����дλ��0��1������״̬����
    WriteByte(uchar_AddrMsb);
    WriteByte(uchar_AddrLsb);
    uchar_AddrReadData = ReadByte();
    StopI2C();

    //ICWRDisable = 1;                                          //��ֹ��д

    return uchar_AddrReadData;
}
//-----------------------------------------------------------------------------
// �����¿ز���������,���ô˺���ǰ������ж�
//-----------------------------------------------------------------------------
void SaveTmpArg(void)
{
	if(0x00 == (Status & 0x08))
	{
		Write_EXTERNRAM(ArgAdr+0,Argument.LDT);
		Write_EXTERNRAM(ArgAdr+1,Argument.HDT);
		Write_EXTERNRAM(ArgAdr+2,Argument.CLDT);
		Write_EXTERNRAM(ArgAdr+3,Argument.LT);
		Write_EXTERNRAM(ArgAdr+4,Argument.HT);
		Write_EXTERNRAM(ArgAdr+5,Argument.QLT);
	}
}
//-----------------------------------------------------------------------------
// �ָ��������¿ز���,���ô˺���ǰ������ж�
//-----------------------------------------------------------------------------
void RestoreTmpArg(void)
{
	if(0x08 == (Status & 0x08))
	{
		Argument.LDT = 0x206;
		Argument.HDT = 0x6D;
		Argument.CLDT = 0x18C;
		Argument.LT = 0x360;
		Argument.HT = 0x39;
		Argument.QLT = 0x3BD;
	}
	else
	{
		Argument.LDT = Read_EXTERNRAM(ArgAdr+0);
		Argument.HDT = Read_EXTERNRAM(ArgAdr+1);
		Argument.CLDT = Read_EXTERNRAM(ArgAdr+2);
		Argument.LT = Read_EXTERNRAM(ArgAdr+3);
		Argument.HT = Read_EXTERNRAM(ArgAdr+4);
		Argument.QLT = Read_EXTERNRAM(ArgAdr+5);
	}
}
void SaveArg(void)
{
	ICUnion tmp;

	ICWRDisable = 0;	 //�����д
	tmp.in = Argument.LDT;
	WriteE2pByte(3,tmp.ch[0]);
	WriteE2pByte(4,tmp.ch[1]);
	tmp.in = Argument.HDT;
	WriteE2pByte(5,tmp.ch[0]);
	WriteE2pByte(6,tmp.ch[1]);
	tmp.in = Argument.CLDT;
	WriteE2pByte(7,tmp.ch[0]);
	WriteE2pByte(8,tmp.ch[1]);
	tmp.in = Argument.LT;
	WriteE2pByte(9,tmp.ch[0]);
	WriteE2pByte(10,tmp.ch[1]);
	tmp.in = Argument.HT;
	WriteE2pByte(11,tmp.ch[0]);
	WriteE2pByte(12,tmp.ch[1]);
	tmp.in = Argument.QLT;
	WriteE2pByte(13,tmp.ch[0]);
	WriteE2pByte(14,tmp.ch[1]);
	WriteE2pByte(1,0x90);
	WriteE2pByte(2,0xEB);
	ICWRDisable = 1;	//��ֹ��д
	SaveTmpArg();
}
void LoadArg(void)
{
	ICUnion tmp;

	ICWRDisable = 0;	 //�����д
	tmp.ch[0] = ReadE2pByte(1);
	tmp.ch[1] = ReadE2pByte(2);
	if(0xEB90 == tmp.in)//Tzh ��ѯ������洢˳��
	{
		tmp.ch[0] = ReadE2pByte(3);
		tmp.ch[1] = ReadE2pByte(4);
		Argument.LDT = tmp.in;
		tmp.ch[0] = ReadE2pByte(5);
		tmp.ch[1] = ReadE2pByte(6);
		Argument.HDT = tmp.in;
		tmp.ch[0] = ReadE2pByte(7);
		tmp.ch[1] = ReadE2pByte(8);
		Argument.CLDT = tmp.in;
		tmp.ch[0] = ReadE2pByte(9);
		tmp.ch[1] = ReadE2pByte(10);
		Argument.LT = tmp.in;
		tmp.ch[0] = ReadE2pByte(11);
		tmp.ch[1] = ReadE2pByte(12);
		Argument.HT = tmp.in;
		tmp.ch[0] = ReadE2pByte(13);
		tmp.ch[1] = ReadE2pByte(14);
		Argument.QLT = tmp.in;
	}
	else
	{
		Argument.LDT = 0x206;
		Argument.HDT = 0x6D;
		Argument.CLDT = 0x18C;
		Argument.LT = 0x360;
		Argument.HT = 0x39;
		Argument.QLT = 0x3BD;
	}
	ICWRDisable = 1;	//��ֹ��д
	SaveTmpArg();
}
