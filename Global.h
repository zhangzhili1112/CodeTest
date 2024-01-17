/****************************************
Created	by lgl	at 20181201
Modified	by	lgl	at 20181230
******************************************/
#include <reg51.h>
#include "Define.h"
#ifndef __Global_H__
#define __Global_H__
/*****************P�ڶ���****************************************************/
sbit ExternInt0_Enable = P3^2;				/*�ⲿ�ж�(���Ź�)����*/
sbit AD_CONVST = P1 ^ 2;					/*ADת������[0->1��AD����ת��,1-ת��������]*/ //Tzh �������������Ϊ������ת��Ϊ1������ע��Ϊת��������
sbit AD_RFS = P1 ^ 3;						/*AD���ݶ�����[0-֡ͬ��]*/
sbit SCLK = P1 ^ 4;							/*AD��ȡ��������ʱ��SCLK*/
sbit ICLK = P1 ^ 5;							/*I2Cʱ��*/
sbit ICWRDisable = P1 ^ 6;					/*I2C�������д*/
sbit ADDR_DOG = P1 ^ 7;						/*���Ź�����*/
sbit SDATA = P3 ^ 4;						/*AD��ȡ�Ĵ���SDATA*/
sbit ICDATA = P3 ^ 5;						/*I2C����λ*/
sbit CommuniationDisable = P3 ^ 7;			/*����ͨѶ��·�Ͽ�*/	
/********************************�Զ������**********************************/
unsigned char idata Status _at_ 0x20;         /*�Լ���״̬��*/
unsigned char idata RetCode _at_ 0x21;
unsigned char idata RamFlag _at_ 0x22;        /*����ճ������ַճ�����־����4��*/
unsigned char idata wkStatus _at_ 0x23;
unsigned int idata TimerState_Cont _at_ 0x24;
unsigned char idata pp[6] _at_ 0x26 = {0, 0, 0, 0, 0, 0};  /*����ָ�����״̬��¼       */
unsigned char bdata iBase _at_ 0x2c;
unsigned char idata RestoreFlag _at_ 0x2d;
unsigned char idata RestoreFlagB0 _at_ 0x2e;
unsigned char idata RestoreFlagB1 _at_ 0x2f;
unsigned int idata RamStartAdr _at_ 0x30;	/*�����ڴ���ʼ��ַ*/
unsigned int idata TimerCount _at_ 0x32;       /* ��ʱ�������ۼ�ֵ                 */
unsigned char idata ReceiveCount _at_0x34;//Tzh ����������ʶ����
unsigned char idata SendCount _at_0x35;
unsigned int idata AncMax _at_ 0x36;
unsigned int idata AncMin _at_ 0x38;
unsigned int idata AncStandard _at_ 0x3a;
unsigned TempleStr idata AncTotal _at_ 0x3c;

ArgStruct idata Argument _at_ 0x40;
TimeCount idata synTime _at_ 0x4C;

SeriBuff idata SendBuff _at_ 0x50;
SeriBuff idata ReceiveBuff _at_ 0x58;

sbit ReceiveingFlag = iBase ^ 0;
sbit SendingFlag = iBase ^ 1;
sbit SentDone = iBase ^ 2;
sbit BoardcastFlag = iBase ^ 3;
sbit rCmdErr = iBase ^ 4;
sbit HeatState = iBase ^ 5;
sbit ctrlFlag = iBase ^ 7;
#endif
