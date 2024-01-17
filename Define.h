/****************************************
Created	by lgl	at 20181201
Modified	by	lgl	at 20181230
******************************************/
#include <math.h>
#include <absacc.h>
#include <intrins.h>
#ifndef __DEFINE_H__
#define __DEFINE_H__
#define oclock 11059200
#define myAdr 0x21		//�¿����ӵ�ַ
#define BoardCastAdr 0xff	//�㲥�ӵ�ַ
#define	XDATASTART	0x00	//�ⲿ�Ĵ�����ʼ��ַΪ0
#define	XDATALEN	0x8000	//�ⲿ�Ĵ�������(32KBytes)
#define	XBLOCKTOTAL	0x04	//�ⲿ�Ĵ�������
#define	XBLOCKLEN	0x2000	//�ⲿ�Ĵ���ÿ�鳤��(8KBytes)
#define HAVEDONE 0xEB	
#define CtrlPeriod	2000		//��������1000����
/*****************���Ƶ�ַ��**********************************************/
#define ADDR_CJTD   	0xA000  	 /*�ɼ�ͨ�����õ�ַ*/
#define Sample_Total	55		/*�ɼ��ź�����*/
#define ADDR_IOSEL1 	0xA004  	 /*IOSEL1:����ͨ��14~20*/
#define ADDR_IOSEL2 	0xA008  	 /*IOSEL2:����ͨ��7~13*/
#define ADDR_IOSEL3 	0xA00C  	 /*IOSEL3:����ͨ��1~6*/
#define ADDR_IOSEL4 	0xA010  	 /*IOSEL4:����ͨ��34~40*/
#define ADDR_IOSEL5 	0xA014  	 /*IOSEL5:����ͨ��27~33*/
#define ADDR_IOSEL6 	0xA018  	 /*IOSEL6:����ͨ��21~26*/
#define IO_CMD1	0xFE  	 /*����ָ��14~20*/
#define IO_CMD2	0xFE  	 /*����ָ��7~13*/
#define IO_CMD3	0x7E  	 /*����ָ��1~6*/
#define IO_CMD4	0xFE  	 /*����ָ��34~40*/
#define IO_CMD5	0xFE  	 /*����ָ��27~33*/
#define IO_CMD6	0x7E  	 /*����ָ��21~26*/
/*****************ʹ�õı�������0x500-0xFDF*****************************************/
#define AnC_Buf 		0x900   	 /* 55����--ģ�����ɼ�������ʱ�洢��from:0x1000            */
#define ArgAdr		    0xA00        /* �¿ز�����ʱ����� */
#define ReceiveBufStart 0x1000
#define SendBufStart	0x1080
#define HEAT_THRESHOLD    0x0300	 /* �����������о���ֵ:���ڵ���HEAT_THRESHOLD--��;����--��*/
#define Sensor_Erk        0x03fa     /* �������жϿ�·��ֵ:-40*/
#define Sensor_Erd        0x001e     /* �������ж϶�·��ֵ:100*/
#define Error_THRESHOLD	0x33		/*������ֵ51*/
#define Base_Sensor		0x200		/*�¶�У׼��ѹ����ֵ512*/
#define Base_5V			0x200		/*+5V����ֵ512*/
#define Base_P12		0x4cd		/*+12V����ֵ1229*/
#define Base_S12		0x4cd		/*-12V����ֵ1229*/
#define SwitchMask	0xC0		//����״̬λ
#define SwitchOn	0xC0		//���ؿ�--��ͨ
#define SwitchOff	0x00		//���ع�--�Ͽ�
/************�����*************/
#define SYN_CMD 0xEB		//�㲥ͬ������--�ϵ��Լ��Ժ�,����״̬
#define SYN_CMD_L	0x04
#define SelfTest_CMD 0x11	//�Լ�����--�ϵ��Լ��Ժ�,����״̬
#define	SelfTest_CMD_L 0
#define SelfTest_ANSW 0x11	//�Լ�����Ӧ��
#define	TM_CMD 0xEE			//ң���ѯ����--����״̬
#define	TM_CMD_L 0
#define TM_ANSW 0xEE		//ң���ѯӦ��
#define WKStart_CMD 0xAA	//�����Զ��¿�����--����״̬,�ڴ桢AD�Լ��޴���
#define	WKStart_CMD_L 0
#define WKStart_ANSW 0xAA	//�����Զ��¿�Ӧ��
#define WKorTestStop_CMD 0x55		//ֹͣ�Զ��¿�����,��ֹͣ������ͬ
#define WKStop_ANSW 0x5A	//ֹͣ�Զ��¿�Ӧ��
#define TestStart_CMD 0xBB	//������������--����״̬,�ڴ桢AD�Լ��޴���
#define	TestStart_CMD_L 0
#define TestStart_ANSW 0xBB	//��������Ӧ��
#define TestStop_ANSW 0xA5	//ֹͣ����Ӧ��
#define	WKorTestStop_CMD_L 0
#define illCMD_ANSW 0x55	//����״̬�յ�ֹͣ����Ӧ��
#define LCStart_CMD 0xB1	//����Ѳ������--����״̬,�ڴ桢AD�Լ��޴���
#define	LCStart_CMD_L 0
#define LCStart_ANSW 0x1B	//����Ѳ��Ӧ��
#define DisChannel_CMD 0xB0		//����ͨ���Ͽ�����--����״̬
#define	DisChannel_CMD_L 0
#define DisChannel_ANSW 0x0B	//����ͨ���Ͽ�Ӧ��
#define EnChannel_CMD 0xBF		//����ͨ����ͨ����--����״̬
#define	EnChannel_CMD_L 0
#define EnChannel_ANSW 0xFB		//����ͨ����ͨӦ��
#define DEAChannel_CMD 0xBA		//��Ҫ����Ƹ�����ͨ������--����״̬
#define	DEAChannel_CMD_L 0x5
#define DEAChannel_ANSW 0xAB	//��Ҫ����Ƹ�����ͨ��Ӧ��
#define DESChannel_CMD 0xBC		//��Ҫ����Ƶ�����ͨ������--����״̬
#define DESChannel_ANSW 0xCB	//��Ҫ����Ƶ�����ͨ��Ӧ��
#define ArgLoad_CMD 0xFF		//����װ������--����״̬,�ڴ桢AD�Լ��޴���
#define ArgLoad_CMD_L 12
#define ArgLoad_ANSW 0xFF		//����װ��Ӧ��
#define XorErr 0xCA		//���У���
#define TimErr 0xCB		//ʱ���
#define CmdErr 0xCC		//�����
#define FrameErr 0xCD		//֡�ṹ(����)��
#define DESChannel_Err 0xC8	//����ͨ����Χ
#define OverTime	0x64	//�շ���ʱ100����
#define Max_Frame_dL 40		//�������֡�ṹ������ݳ���
/*********************���幤��״̬*****************************/
#define	Auto_TempCtrl_Status	0x0F	//�Զ��¿�״̬
#define	Test_Status	0xF0	//����״̬
#define	Test_Loop_Status	0xF1	//Ѱ��״̬
#define	Idel_Status	0x00	//����״̬
#define Save_Arg_Status	0x01	//д����
#define vaild_Status 0xFF	//����״̬,û�п����ڴ�
/*********************����1ms��ʱ����*****************************************/
#define	TH_1ms	0xFC		//��ʱԼ975.5΢�� 
#define	TL_1ms	0x7D		//��ʱ����0xFC67�������ж�,Լִ��22��ָ������(��Ӧ�ж�Լ4������,�����ֳ�12������--ACC��B��DPTR��PSW�����üĴ����飬��ʱ����λ6������)
//�������ݽṹ
typedef struct {
	unsigned int LDT;	//�������¶�������
	unsigned int HDT;	//��������
	unsigned int CLDT;	//������¶�������
	unsigned int LT;	//�������¹�����
	unsigned int HT;	//���¹�����
	unsigned int QLT;	//��ƿ���¹�����
} ArgStruct;
typedef	struct {
	unsigned char CLength;
	unsigned char CXor;
	unsigned char Cmd;
	unsigned char Length;
	unsigned int BuffAdr;
	unsigned int StartAdr;
} SeriBuff;
typedef union {
	unsigned char TS[4];
	unsigned long TA;
} TimeCount;
typedef union {
	unsigned long AncuLong;
	struct {
		unsigned int AncuIntL,AncuIntH;
	} AncuInt;
} TempleStr;
typedef union {
	unsigned int in;
	unsigned char ch[2];
} ICUnion;
/* ��������ַ�б� */
const unsigned int code HEAT_ADR[40] = 
{
	ADDR_IOSEL3, ADDR_IOSEL3, ADDR_IOSEL3, ADDR_IOSEL3, ADDR_IOSEL3, ADDR_IOSEL3,				/*0xA00C:01,02,03,04,05,06   */
	ADDR_IOSEL2, ADDR_IOSEL2, ADDR_IOSEL2, ADDR_IOSEL2, ADDR_IOSEL2, ADDR_IOSEL2, ADDR_IOSEL2,	/*0xA008:07,08,09,10,11,12,13*/
	ADDR_IOSEL1, ADDR_IOSEL1, ADDR_IOSEL1, ADDR_IOSEL1, ADDR_IOSEL1, ADDR_IOSEL1, ADDR_IOSEL1,	/*0xA004:14,15,16,17,18,19,20*/
	ADDR_IOSEL6, ADDR_IOSEL6, ADDR_IOSEL6, ADDR_IOSEL6, ADDR_IOSEL6, ADDR_IOSEL6,				/*0xA018:21,22,23,24,25,26   */ 
	ADDR_IOSEL5, ADDR_IOSEL5, ADDR_IOSEL5, ADDR_IOSEL5, ADDR_IOSEL5, ADDR_IOSEL5, ADDR_IOSEL5,	/*0xA014:27,28,29,30,31,32,33*/
	ADDR_IOSEL4, ADDR_IOSEL4, ADDR_IOSEL4, ADDR_IOSEL4, ADDR_IOSEL4, ADDR_IOSEL4, ADDR_IOSEL4	/*0xA010:34,35,36,37,38,39,40*/
};
/* ����������ָ���б� */
const unsigned char code OPEN_CMD[40] = 
{
/*  1.....................................................8*/
		0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 
		0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 
		0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02,
		0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 
		0x80, 0x40, 0x20, 0x10,0x08, 0x04, 0x02, 
		0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02
};
const unsigned char HEAT_INDEX[40] =
{/*      1       2       3       4       5       6       7       8*/
	2,2,2,2,2,2,	/*01,02,03,04,05,06   */
	1,1,1,1,1,1,1,	/*07,08,09,10,11,12,13*/
	0,0,0,0,0,0,0,	/*14,15,16,17,18,19,20*/
	5,5,5,5,5,5,	/*21,22,23,24,25,26   */
	4,4,4,4,4,4,4,	/*27,28,29,30,31,32,33*/
	3,3,3,3,3,3,3	/*34,35,36,37,38,39,40*/
};
const unsigned int code ptHL_T1_Cmd[6][5] = {
	/*SampleAdrstart,	HeatAdr-PP,	HeatCmd,	HeatAdr-PP,	HeatCmd*/
	{AnC_Buf+0x10,		2,			0x40,		2,			0x80},	//T01��T02; J01��J02:0xA00C,0x40,0xA00C,0x20
	{AnC_Buf+0x16,		1,			0x08,		1,			0x04},	//T07��T08; J11��J12:0xA008,0x08,0xA008,0x04
	{AnC_Buf+0x24,		4,			0x08,		4,			0x04},	//T21��T22;	J31��J32:0xA014,0x08,0xA014,0x04
	{AnC_Buf+0x1F,		5,			0x40,		5,			0x20},	//T16��T17; J21��J22:0xA018,0x40,0xA018,0x20
	{AnC_Buf+0x13,		2,			0x10,		2,			0x08},	//T04��T05;	J03��J04:0xA00C,0x10,0xA00C,0x08
	{AnC_Buf+0x19,		1,			0x02,		0,			0x08}	//T10��T11;	J13��J14:0xA008,0x02,0xA004,0x80
};
#endif
