#ifndef __function_h_
#define __function_h_
#include "stm32f10x_lib.h"
//系统状态全局变量
#define Sys_Undefined 0
#define Sys_Running 1
#define Sys_Idle 	2
#define Sys_SelfCheck 3
#define Sys_Error 	4
//collin add20140902
//#define Sys_Start 5
//end
//collin add20140917
#define Sys_Print_Yes 6
#define Sys_Print_No 7
//end
//collin add20141009
#define Sys_Function 8
//end
//collin add20141030
//#define USART_OPTION 1
//end
#define abs(A) (((A)>0)?(A):(-(A)))
extern u16	A1,A2,A3,A4;
extern 	u16 Temperature[6];
extern u32 C13[60],C12[60];
//collin modify20141025
extern s16 dob;
//extern double dob;
//end
//collin add20141119
extern u8 flag_print;
extern int ID_NUM;
extern u8 flag_delete;
//end
extern long long C12_PJ,C13_PJ,C12_BZWC,C13_BZWC;
extern u8	N[8],M[13];				//常数项个个位（个位，十位，百位，千位等等）的值
extern u32	YH08;
extern u8	ch_selected,ZF;	//正负号;
extern u8	qualityCount;//质控次数；
extern u8	sampleOn;
extern s16	BZWC,DOB1,DOB2,DOB3,DOB4,DOB5,DOB6,DOB_ZK[10],DOB_PJ;//质量控制结果保存;//质量控制标准误差，
extern u16	worktime;
extern u8	flag_gb,sjxg,pdzxg;
extern u8	K1,K2,K3,K4,K5,K6,K7,K8,K9,K0,K_D,K_J,K_Z,K_Y,K_S,K_X,MENU,COUNT,CONFIRM,REMOVE,RETURN,STOP,PRINT,INQUIRY,INQUIRY_PRINT;
extern u16	Num,DiapIndex;
extern u8  flag_dob,flag_cz,cuozhi;//DOB分界值；错值判定标志；错值判断标准
extern u8	sampleChanged;
extern u8	flag_err[6];
extern u8 ID[30];
extern	   tm timer;

extern volatile unsigned int Sys_Status;
//extern u8	m_bRequestCf;

extern u8 	Cf[128] ;
extern u8 	Abs_backup[320] ;
extern double absC12_backup[40];
extern double absC13_backup[40];
extern double absc12_st[5];
extern double absc13_st[5];
extern double c12_st[5];
extern double c13_st[5];
extern double m_lCfC12_st[6];
extern double m_lCfC13_st[6];

extern u32 	ParamT;
extern u16	ParamP,PressType;
extern u8	m_bMeasureBreak;
extern u8 	m_bIgnoreConcentrationLow; 


extern volatile int LastWord;
extern u16	m_bSaveCf;
extern 	u8	m_mesureType,upperCount;

extern 	u8	reSendData;
extern 	u16	checkOver;

extern 	u32	Intensitys[18];
//5ms定时器变量
extern volatile u32 TimingDelay;
extern u16	Second_1,Second_2;
//串口通信变量
#define FrameLength 24
#define BufferSize FrameLength*10+1
extern volatile unsigned short SendBackup[21][10];
extern volatile unsigned char SendBuffer[BufferSize];
extern volatile unsigned char ReadBuffer[FrameLength+30];
extern volatile unsigned int PHead,PEnd;
extern volatile unsigned int QueryFlag;
extern volatile unsigned int SendLength;
extern volatile unsigned int SendCounter;
//串口通信变量uart1
#define FrameLength 24
#define BufferSize FrameLength*10+1
extern volatile unsigned short SendBackup_uart1[21][10];
extern volatile unsigned char SendBuffer_uart1[BufferSize];
extern volatile unsigned char ReadBuffer_uart1[FrameLength+30];
extern volatile unsigned int PHead_uart1,PEnd_uart1;
extern volatile unsigned int QueryFlag_uart1;
extern volatile unsigned int SendLength_uart1;
extern volatile unsigned int SendCounter_uart1;
extern u8 m_bBusing;
extern u16 chopperDelaytimes;
//ADC采样设定参数

extern u8	ADCIndex;
extern u16	ADCResult[36];							 
//系统时钟变量
extern u8 SelfChecking;
extern volatile u32 SelfCheckCount;
//样品在位标志
extern volatile u8 checkChannel;
//collin add20141125
extern volatile u8 ledChannel;
//end
extern volatile u8 SamplePosition1,SamplePosition2,SamplePosition3,SamplePosition4,SamplePosition5,SamplePosition6;
extern u8	keynum,sleeptime,testcount,ch_tested[6],flag_count;
//定义电磁阀端口
#ifndef Valve_GPIO
	#define Valve_GPIO
	#define GPIO_Valve1 		GPIOF
	#define GPIO_Valve2_3 		GPIOG
	#define GPIO_Valve4_12 		GPIOE
	#define GPIO_Valve13_18		GPIOD
	#define GPIO_Pin_Valve1 	GPIO_Pin_15	
	#define GPIO_Pin_Valve2 	GPIO_Pin_0	
	#define GPIO_Pin_Valve3 	GPIO_Pin_1	
	#define GPIO_Pin_Valve4 	GPIO_Pin_7
	#define GPIO_Pin_Valve5 	GPIO_Pin_8	
	#define GPIO_Pin_Valve6 	GPIO_Pin_9
	#define GPIO_Pin_Valve7 	GPIO_Pin_10	
	#define GPIO_Pin_Valve8 	GPIO_Pin_11	
	#define GPIO_Pin_Valve9 	GPIO_Pin_12	
	#define GPIO_Pin_Valve10 	GPIO_Pin_13				 
 	#define GPIO_Pin_Valve11 	GPIO_Pin_14
	#define GPIO_Pin_Valve12 	GPIO_Pin_15
	#define GPIO_Pin_Valve13 	GPIO_Pin_10
	#define GPIO_Pin_Valve14 	GPIO_Pin_11
	#define GPIO_Pin_Valve15 	GPIO_Pin_12
	#define GPIO_Pin_Valve16 	GPIO_Pin_13
	#define GPIO_Pin_Valve17 	GPIO_Pin_14
	#define GPIO_Pin_Valve18 	GPIO_Pin_15
#endif
#ifndef GPIO_LED
	#define GPIO_LED1 GPIOC
	#define GPIO_LED GPIOF
	#define LED_Pin_1 GPIO_Pin_0
	#define LED_Pin_2 GPIO_Pin_10
	#define LED_Pin_3 GPIO_Pin_9
	#define LED_Pin_4 GPIO_Pin_8
	#define LED_Pin_5 GPIO_Pin_7
	#define LED_Pin_6 GPIO_Pin_6
#endif
//定义ADC变换通道
#ifndef ADCCH
	#define ADCCH
	#define ADC_T42 	14//14
	#define ADC_T18 	15//15
	#define ADC_FS 		2//15
	#define ADC_T_10A 	9
	#define ADC_T_10B 	8
//	#define ADC_ROOM 	1
//	#define ADC_Humidity 	3
#endif

#ifndef GPIO_SPEAKER
	#define GPIO_SPEAKER GPIOF
	#define GPIO_Pin_SPEAKER GPIO_Pin_14
#endif
#ifndef GPIO_ERROR
	#define GPIO_ERROR 	GPIOF
	#define ERROR_Pin 	GPIO_Pin_13
#endif
#ifndef GPIO_TripSwitch
	#define GPIO_TripSwitch GPIOF
	#define GPIO_Pin_TripSwitch GPIO_Pin_12
	#define GPIO_PORT_SOURCE_TripSwitch GPIO_PortSourceGPIOF
	#define GPIO_PIN_SOURCE_TripSwitch GPIO_PinSource12
	#define EXTI_LINE_TripSwitch EXTI_Line12
#endif

/*
Motorun汽缸电机驱动函数
En=1/0 1为启动励磁
FPulseNum为脉冲个数
Cw=FORWARD/BACKWORD
IfConv=0/1 是否在过程中进行采集
FStartSpeed 启动速度，单位r/s
FTargeSpeed 目标速度，单位r/s
FStepTimes_1 加速过程的过程数
FStepCount_1 每个加速过程持续脉冲数
FEndSpeed 结束速度，单位r/s
FStepTimes_2减速过程的过程数
FStepCount_2 每个减速过程持续脉冲数
u8 vnum1,u8 vnum2,u8 vnum3 三个电磁阀状态
*/
void Motorun(u8 En,u16 FPulseNum,u8 Cw,u8 IfConv,
			u8 FStartSpeed,u8 FTargeSpeed,u8 FStepTimes_1,u16 FStepCount_1,
			u8 FEndSpeed,u8 FStepTimes_2,u16 FStepCount_2,u16 FAddedNum,
			u8 vnum1,u8 vnum2,u8 vnum3);
void Motorun_1(u8 En,u16 FPulseNum,u8 Cw,u8 IfConv,
			 u8 FStartSpeed,u8 FTargeSpeed,u8 FStepTimes_1,u16 FStepCount_1,
			 u8 FEndSpeed,u8 FStepTimes_2,u16 FStepCount_2,u16 FAddedNum);
void ReadS_GPIO(void);

void ReadS_V(void);
//气缸驱动函数初始化
void Motor_Config(void);
//蜂鸣器初始化
void Speaker_Config(void);
//蜂鸣器函数，Num发声次数
void Speaker(u8 Num);
//电磁阀函数
void Valve(unsigned char Vnum);
//电磁阀GPIO设定
void Valve_GPIO_Config(void);
//100ms系统时钟初始化
void Sys_100msClock_Config(void);
//队列函数
void Inqueue(unsigned char byte);
unsigned char Exqueue(void);
unsigned char QLength(void);
//队列函数 for uart1
void Inqueue_uart1(unsigned char byte);
unsigned char Exqueue_uart1(void);
unsigned char QLength_uart1(void);
//5ms系统时钟初始化
void Delay5ms_Config(void);
//延时5ms*nTime

//语音芯片初始化
void AK5385_Config(void);
//语音芯片转换
void AK_Conv(void);
//切光片电机初始化
void LightModulation_Config(void);
//切光片电机控制status=1开启status=0停止
void LightModulation(u8 status);
/*
模数转换通道，返回16次平均后的AD值
ch=ADC_FS 返回压力表AD值
*/
unsigned int ADC_Press(unsigned char ch);
void SamplePosition(void);
void SendPosition(short positionChanged);
//模数转换配置
void LED(u8 Num, u8 Status);
void ADC_Config(void);
void GPIO_Configuration(void);
void RCC_Configuration(void);
void NVIC_Configuration(void);
void PRINT_Init (void);

//collin modify20141028
//uart1 and uart3 配置
void Uart1_Config(void);
void Uart3_Config(void);
//end
//发送语音芯片数据
void SendData(void);
//配置外部中断
void EXTI_Config(void);
void	Set_time(void);
void 	Ch_PDZ(void);               //错置判定值修改

void	dis_set_inf1(unsigned char flag);							//常数项切换显示程序

void 	pprint(unsigned char ch);
void	print_test(void);
void	print_time(void);
void 	print_filter(s16 Filter);											//过滤器检测
void 	print_result_qc(void);									  //打印质控结果
void 	print_result_search(void);									
void 	print_result(void);									
void 	print_power(u8 count,u8 type);
void 	print_error(u8 type);
void 	print_pressure(u16 p,u8 f);
void 	print_date_time(void);                                         //打印日期时间
void	print_temperature(u8 type);

unsigned int ADC(unsigned char ch);
//u16 weedavg(u16 a[ADCNUM],u8 num) ;
void deleteRecord(void);
void SendDataTemperature(void);

void sampleScan(void);

u8 RTC_Get_Week(u16 year,u8 month,u8 day);
#endif
