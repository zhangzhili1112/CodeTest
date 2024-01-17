/****************************************
Created	by lgl	at 20181201
Modified	by	lgl	at 20181230
******************************************/
#include <reg51.h>
#include "Define.h"
#ifndef __Global_H__
#define __Global_H__
/*****************P口定义****************************************************/
sbit ExternInt0_Enable = P3^2;				/*外部中断(看门狗)输入*/
sbit AD_CONVST = P1 ^ 2;					/*AD转换启动[0->1－AD启动转换,1-转换过程中]*/ //Tzh 软件需求中描述为不允许转换为1，代码注释为转换过程中
sbit AD_RFS = P1 ^ 3;						/*AD数据读启动[0-帧同步]*/
sbit SCLK = P1 ^ 4;							/*AD读取串行数据时钟SCLK*/
sbit ICLK = P1 ^ 5;							/*I2C时钟*/
sbit ICWRDisable = P1 ^ 6;					/*I2C不允许读写*/
sbit ADDR_DOG = P1 ^ 7;						/*看门狗控制*/
sbit SDATA = P3 ^ 4;						/*AD读取的串行SDATA*/
sbit ICDATA = P3 ^ 5;						/*I2C数据位*/
sbit CommuniationDisable = P3 ^ 7;			/*串性通讯链路断开*/	
/********************************自定义变量**********************************/
unsigned char idata Status _at_ 0x20;         /*自检结果状态字*/
unsigned char idata RetCode _at_ 0x21;
unsigned char idata RamFlag _at_ 0x22;        /*数据粘连、地址粘连块标志，共4块*/
unsigned char idata wkStatus _at_ 0x23;
unsigned int idata TimerState_Cont _at_ 0x24;
unsigned char idata pp[6] _at_ 0x26 = {0, 0, 0, 0, 0, 0};  /*控制指令输出状态记录       */
unsigned char bdata iBase _at_ 0x2c;
unsigned char idata RestoreFlag _at_ 0x2d;
unsigned char idata RestoreFlagB0 _at_ 0x2e;
unsigned char idata RestoreFlagB1 _at_ 0x2f;
unsigned int idata RamStartAdr _at_ 0x30;	/*可用内存起始地址*/
unsigned int idata TimerCount _at_ 0x32;       /* 定时器计数累加值                 */
unsigned char idata ReceiveCount _at_0x34;//Tzh 编译器可能识别不了
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
