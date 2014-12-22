#include "function.h" 
#include "math.h" 
#include "limits.h" 
#include "motion.h"
#include "lcd.h"
#include "spi_flash.h" 
#include "sysdate.h"
#include "hpir.h"
#include "i2c_ee.h"
#define ABS(A) (((A)>0)?(A):(-(A)))
u8 const mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
unsigned char table[] ="0123456789";
unsigned char RQ[]="日期：";
unsigned char SJ[]="时间："; 
unsigned char YQBH[]="仪器编号";
unsigned char GQJC[]="光强检测";
unsigned char GLQXNBZ[]= "     过滤器效能不足!    ";
unsigned char GHGLQ[]  = "      请更换过滤器!     ";
unsigned char GLQXNLH[]= "     过滤器效能良好!    ";
unsigned char GLQXN[]=   "    过滤器效能：";
unsigned char WDJC[]= "       温度检测         ";
unsigned char WDZC[]=    "结果：   温度正常！     ";
unsigned char WDYC[]=  "结果：   温度异常！     ";
unsigned char YLZC[]=  "结果：  气密性正常      ";
unsigned char LQ[]=    "结果：    漏气 !        ";
unsigned char YLJC[]=  "        压力检测        ";
unsigned char DYJJC[]= "       打印机检测       ";
unsigned char GLQJC[]= "       过滤器检测       ";
unsigned char DQND[]="底气浓度：";
unsigned char YQND[]="样气浓度：";
unsigned char JCJG[]= "       检测结果        ";
//unsigned char JCJG[]=  "     幽门螺杆菌（Hp）   ";
unsigned char DOB[]="DOB(‰) = ";
unsigned char table1[]="------------------------";
unsigned char table2[]="************************";
unsigned char T12[]=   "  C12= ";
unsigned char T13[]=   "  C13= ";
unsigned char GQ13[]=  "时间   C12     C13    ";
unsigned char   KG[]="   ";
unsigned char   KG1[]="  ";
unsigned char GQJG[]=  "      光强检测结果      ";
unsigned char D[]=".";
unsigned char BFH[]="％";
unsigned char YPBH[]="样品编号: ";
//collin modify20141124
//unsigned char DOBFJZ[]="DOB分界值:";
unsigned char DOBFJZ[]="DOB正常范围<";
//end 
unsigned char J[]="+-:";
unsigned char YIN[]="阴性";
unsigned char YANG[]="阳性";
unsigned char KH[]="()";
unsigned char NDD[] ="结   果：气体浓度太低！";
unsigned char QDBC[] ="结   果： 气袋被拔出！ ";
unsigned char TDH[]="检测通道： ";
unsigned char DYZC[]="       打印正常!       ";
unsigned char ZLKZ[]="     质量控制第";
unsigned char CI[]="次";
unsigned char CS[]="次数";
unsigned char BZWC1[]="  标准误差：  ";
unsigned char PJZ[]="    平均值：  ";
unsigned char ZK[]=  "      质量控制结果     ";
unsigned char GQPJ[]=  "光强平均值：    ";
unsigned char GQBZWC[]=  "光强标准误差：    ";
unsigned char BYXS[]=  "变异系数：    ";
unsigned char MPA[]=  "MPa";
unsigned char DQYL[]=  "当前压力值：";
unsigned char JYR[]=  "检验人员：_____________";
unsigned char XM[]=   "姓    名：_____________";
unsigned char JG[]=   "检测结果：";
tm timer;
u8	ID[30];
u8	sampleChanged;
u8  flag_dob,flag_cz,cuozhi;//DOB分界值；错值判定标志；错值判断标准
//采样设定
u8		ADCIndex;
u16		ADCResult[36];
u16		ADCConv[ADCNUM]; 
u8	keynum,sleeptime,testcount;
u8	flag_err[6];
u16	Num=0,DiapIndex=1;
s16	BZWC,DOB1,DOB2,DOB3,DOB4,DOB5,DOB6;//质量控制标准误差，
//全局系统变量
extern volatile unsigned int Sys_Status	= Sys_Undefined;
extern u8 m_bChopper;
u16 chopperDelaytimes=0;
u8 prnerr=0;
volatile u8 cw_status=FORWARD;
//切光片电机加速
volatile unsigned int Step_LM=0; 
const unsigned int Speedup[200]={62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766, 62766,56978,51962,47586,43746,40357,37351,34673,32278,30125,28185,26429,24835,23384,22059,20846,19733,18709,17765,16893,16085,15335,14639,13991,13386,12821,12293,11798,11334,10898,10488,10102,9738,9394,9070,8763,8472,8197,7936,7688,7452,7228,7015,6811,6618,6433,6257,6088,5927,5773,5626,5485,5349,5220,5095,4976,4861,4751,4645,4543,4445,4351,4260,4173,4088,4007,3929,3853,3780,3709,3641,3575,3512,3450,3391,3333,3277,3223,3171,3121,3072,3024,2978,2933,2890,2848,2807,2768,2729,2692,2656,2621,2587,2554,2521,2490,2460,2430,2401,2373,2346,2320,2294,2269,2245,2221,2198,2176,2154,2133,2112,2092,2073,2054,2035,2017,2000,1983,1966,1950,1934,1919,1904,1890,1876,1862,1849,1836,1823,1811,1799,1787,1776,1765,1755,1745,1735,1725,1716,1706,1698,1689,1681,1673,1665,1658,1650,1643,1637,1630,1624,1618,1612,1607,1601,1596,1591,1587,1582,1578,1574,1570,1567,1563,1560,1557,1554,1552,1549,1547,1545,1543,1542,1540,1539,1538,1537,1536,1536,1536,1536};
//28054,25815,23906,22260,20827,19566,18450,17454,16560,15753,15022,14355,13744,13184,12668,12190,11747,11335,10951,10593,10257,9941,9645,9365,9102,8853,8617,8393,8181,7979,7787,7603,7429,7262,7102,6950,6803,6663,6529,6400,6275,6156,6041,5930,5823,5720,5621,5525,5432,5342,5255,5171,5090,5011,4934,4860,4788,4718,4651,4585,4520,4458,4397,4338,4281,4225,4171,4117,4066,4015,3966,3918,3871,3825,3780,3737,3694,3652,3611,3572,3533,3494,3457,3420,3385,3350,3315,3282,3249,3216,3185,3154,3123,3093,3064,3035,3007,2979,2952,2925,2899,2873,2848,2823,2799,2775,2751,2728,2705,2682,2660,2639,2617,2596,2576,2555,2535,2515,2496,2477,2458,2440,2421,2403,2386,2368,2351,2334,2317,2301,2284,2268,2253,2237,2222,2206,2191,2177,2162,2148,2134,2120,2106,2092,2079,2065,2052,2039,2027,2014,2001,1989,1977,1965,1953,1941,1930,1918,1907,1896,1885,1874,1863,1852,1842,1831,1821,1811,1801,1791,1781,1771,1761,1752,1742,1733,1724,1715,1706,1697,1688,1679,1670,1662,1653,1645,1637,1628,1620,1612,1604,1596,1588,1581,1573,1565,1558,1550,1543,1536}	 ;
//AK5351 变量//AK5351 变量
volatile unsigned int RxLeftIdx = 0;
volatile unsigned int RxRightIdx = 0;
volatile u16 AK_Left_Data[DataLength*2]={0};
volatile u16 AK_Right_Data[DataLength*2]={0};
volatile int LastCh=-1;
volatile int LastWord=0;
//5ms定时器变量
volatile u32 TimingDelay=0;
volatile unsigned char V[20];
volatile unsigned char G[20];
//串口通信变量
volatile unsigned char SendBuffer[BufferSize]={0};
volatile unsigned char ReadBuffer[FrameLength+30]={0};
volatile unsigned int PHead=0,PEnd=0;
volatile unsigned int QueryFlag=1;
volatile unsigned int SendLength=0;
volatile unsigned int SendCounter=0;
//串口1通信变量
volatile unsigned char SendBuffer_uart1[BufferSize]={0};
volatile unsigned char ReadBuffer_uart1[FrameLength+30]={0};
volatile unsigned int PHead_uart1=0,PEnd_uart1=0;
volatile unsigned int QueryFlag_uart1=1;
volatile unsigned int SendLength_uart1=0;
volatile unsigned int SendCounter_uart1=0; 
//汽缸电机变量
volatile u8 flagPress;
volatile u16 PulseNum=0;
volatile u16 PulseCount=0;
volatile u8 StartSpeed=0;
volatile u8 TargeSpeed=0;
volatile u8 StepTimes_1=0;
volatile u16 StepCount_1=0;
volatile u8 EndSpeed=0;
volatile u8 StepTimes_2=0;
volatile u16 StepCount_2=0;
volatile u16 AddedNum=0;
u8  flag_moto=0;
u16	Second_1,Second_2;
u8	m_bBusing;
//样品在位变量
extern volatile u8 SamplePosition1=1,SamplePosition2=1,SamplePosition3=1,SamplePosition4=1,SamplePosition5=1,SamplePosition6=1;
	
//系统时钟变量
extern u8 SelfChecking=0;
extern volatile u32 SelfCheckCount=0;
volatile u8 checkChannel=0;
//collin add20141125
volatile u8 ledChannel=0;
//end
#define TIMx TIM3 
volatile unsigned short SendBackup[21][10]={0};
u8 	ToMeasPosition[12];
u8	ch_checked[6];
u16 Temperature[6];
double absC12[40];
double absC13[40];

double absc12_st[5];
double absc13_st[5];
double c12_st[5];
double c13_st[5];
double m_lCfC12_st[6];
double m_lCfC13_st[6];

double m_lCfC12[6];
double m_lCfC13[6];

double dt1_st[3];
double dt2_st[3];

//collin add20141031
double m_lCfC12_backup[6];
double m_lCfC13_backup[6];
//end
//collin modify20141014
//double consC12[20]= {0.501,0.803,1.101,1.402,1.688,2.010,2.298,2.605,2.903,3.197,3.502,3.803,4.105,4.395,4.706,4.997,5.278,5.609,5.911,6.213};
//double consC13[20]= {0.54108,0.86724,1.18908,1.51416,1.82304,2.1708,2.48184,2.8134,3.13524,3.45276,3.78216,4.10724,4.4334,4.7466,5.08248,5.39676,5.70024,6.05772,6.38388,6.71004};

double consC12[20]= {0.504,0.802,1.11,1.40,1.71,2.00,2.31,2.61,2.93,3.19,3.51,3.78,4.12,4.44,4.71,4.92,5.32,5.61,5.93,6.24};
double consC13[20]={0.54432,0.86616,1.1988,1.512,1.8468,2.16,2.4948,2.8188,3.1644,3.4452,3.7908,4.0824,4.4496,4.7952,5.0868,5.3136,5.7456,6.0588,6.4044,6.7392};
//end
u16 serial;
u16 Pressure0=0,Pressure1=0,Pressure2=0;
u8  TripSwitchStatus=0;
s16 	m=0;
double	Percent=0.600;
double 	Per=0.600;
u8  	reTest;
u8  functioncheck;
u8	reSendData;
u16	checkOver;
u32	Intensity1,Intensity2;
u32	Intensitys[18];
u8	m_bDilution;
u8	m_bChanged;
u8	m_DilutionChannel;
u8	m_mesureType;
u8	sampleTooless;
u32	QualifyTimes;
u8	Trip;
u16 temp1=3620;
u16 temp2=3500;
float C12BABS=0;
float C12SABS=0;
float m_lC12B=0;
float m_lC12S=0;
double xbc12,xbc13, xsc12,xsc13;
double cbc12,cbc13,csc12,csc13;
double ratebase,ratesample;
double 	m_lCf[9]={0} ;
u8 Go=0 ;
//collin modify20141025
s16 dob=0;
//double dob = 0;
//end

s16 DOB_ZK[10],DOB_PJ;//质量控制结果保存

//collin add20141014
u16 debug_collin = 0;
u8   debug1_collin = 0;
u8   debug2_collin = 0;
//end

//collin add20141011
int funtion_complete_reboot_flag = 0;
//end

//collin add20141124
double I13bs = 0;
double Tbs = 0;
s16 dob_debug=0;
double ratebase_debug,ratesample_debug;
//end
void ChannelMeas(u8 Pnum)
{
    u8 i=0,j=0,k=0;
    u8 Pno=Pnum%10;
    Pnum = (u8)(Pnum/10);
    ToMeasPosition[0]	= 0;
    if(sampleTooless==1)
        return;
    switch(Pno)
    {
    case	1:
	ledChannel = 0;
	LED(1,1);
	LED(2,1);
        AirFirst();
        Intensitys[0]=Intensity1;
        Intensitys[1]=Intensity2;
		//collin add20141105
		  //加入帧头
        Inqueue(0x02);
        //加入命令标志
        Inqueue(0x01);
        Inqueue(Intensity1&0xff);
        Inqueue((Intensity1>>8)&0xff);
        Inqueue((Intensity1>>16)&0xff);
        Inqueue((Intensity1>>24)&0xff);
        Inqueue(Intensity2&0xff);
        Inqueue((Intensity2>>8)&0xff);
        Inqueue((Intensity2>>16)&0xff);
        Inqueue((Intensity2>>24)&0xff);
         for(k=0; k<8; k++)
            Inqueue(0);
        Inqueue(0x1);//buf18
        Inqueue(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue(0);
        Inqueue(0x03);
		Delay5ms(100);
		//end

		//collin add20141105
		  //加入帧头
        Inqueue_uart1(0x02);
        //加入命令标志
        Inqueue_uart1(0x01);
        Inqueue_uart1(Intensity1&0xff);
        Inqueue_uart1((Intensity1>>8)&0xff);
        Inqueue_uart1((Intensity1>>16)&0xff);
        Inqueue_uart1((Intensity1>>24)&0xff);
        Inqueue_uart1(Intensity2&0xff);
        Inqueue_uart1((Intensity2>>8)&0xff);
        Inqueue_uart1((Intensity2>>16)&0xff);
        Inqueue_uart1((Intensity2>>24)&0xff);
         for(k=0; k<8; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x1);//buf18
        Inqueue_uart1(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(100);
		//end
        break;
    case 	2:
	//collin add20141125
	ledChannel = 1;
	//end
        BaseFirst(Pnum,m);
        if(sampleTooless==1|| m_bMeasureBreak==1)
            break;
        Intensitys[2]=Intensity1;
        Intensitys[3]=Intensity2;
		//collin add20141105
		  //加入帧头
        Inqueue(0x02);
        //加入命令标志
        Inqueue(0x01);
        Inqueue(Intensity1&0xff);
        Inqueue((Intensity1>>8)&0xff);
        Inqueue((Intensity1>>16)&0xff);
        Inqueue((Intensity1>>24)&0xff);
        Inqueue(Intensity2&0xff);
        Inqueue((Intensity2>>8)&0xff);
        Inqueue((Intensity2>>16)&0xff);
        Inqueue((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue(0);
        Inqueue(0x02);//buf18
        Inqueue(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue(0);
        Inqueue(0x03);
		Delay5ms(100);
		//end
		//collin add20141105
		  //加入帧头
        Inqueue_uart1(0x02);
        //加入命令标志
        Inqueue_uart1(0x01);
        Inqueue_uart1(Intensity1&0xff);
        Inqueue_uart1((Intensity1>>8)&0xff);
        Inqueue_uart1((Intensity1>>16)&0xff);
        Inqueue_uart1((Intensity1>>24)&0xff);
        Inqueue_uart1(Intensity2&0xff);
        Inqueue_uart1((Intensity2>>8)&0xff);
        Inqueue_uart1((Intensity2>>16)&0xff);
        Inqueue_uart1((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x02);//buf18
        Inqueue_uart1(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(100);
		//end
        C12BABS = -log10((float)Intensitys[2] / ((float)Intensitys[0]));
        m_lC12B = ((((m_lCfC12[5] * C12BABS + m_lCfC12[4]) * C12BABS + m_lCfC12[3]) * C12BABS + m_lCfC12[2]) * C12BABS + m_lCfC12[1]) * C12BABS + m_lCfC12[0];
        if(m_bIgnoreConcentrationLow==0)
        {
            if(m_lC12B<0.3)
            {
                m_bMeasureBreak=2;//底气浓度过低，中止测量
				m_bBusing=0;
				flag_err[checkChannel-1]=1;
				LED(checkChannel,1);
				checkChannel=0;
				if(m_mesureType==4)ch_selected=0;
            }
        }
	//collin add20141125
	ledChannel = 0;
	LED(1,1);
	LED(2,1);
	//end
        break;
    case 	3:
        AirFirst();
		//AirSecond();
        Intensitys[4]=Intensity1;
        Intensitys[5]=Intensity2;
		//collin add20141105
		  //加入帧头
        Inqueue(0x02);
        //加入命令标志
        Inqueue(0x01);
        Inqueue(Intensity1&0xff);
        Inqueue((Intensity1>>8)&0xff);
        Inqueue((Intensity1>>16)&0xff);
        Inqueue((Intensity1>>24)&0xff);
        Inqueue(Intensity2&0xff);
        Inqueue((Intensity2>>8)&0xff);
        Inqueue((Intensity2>>16)&0xff);
        Inqueue((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue(0);
        Inqueue(0x03);//buf18
        Inqueue(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue(0);
        Inqueue(0x03);
		Delay5ms(100);
		//end
		//collin add20141105
		  //加入帧头
        Inqueue_uart1(0x02);
        //加入命令标志
        Inqueue_uart1(0x01);
        Inqueue_uart1(Intensity1&0xff);
        Inqueue_uart1((Intensity1>>8)&0xff);
        Inqueue_uart1((Intensity1>>16)&0xff);
        Inqueue_uart1((Intensity1>>24)&0xff);
        Inqueue_uart1(Intensity2&0xff);
        Inqueue_uart1((Intensity2>>8)&0xff);
        Inqueue_uart1((Intensity2>>16)&0xff);
        Inqueue_uart1((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);//buf18
        Inqueue_uart1(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(100);
		//end
        break;
    case 	4:
	//collin add20141125
	ledChannel = 2;
	//end
        SampleFirst(Pnum,m);
        if(sampleTooless==1|| m_bMeasureBreak==1)
            break;
        Intensitys[6]=Intensity1;
        Intensitys[7]=Intensity2;
		//collin add20141105
		  //加入帧头
        Inqueue(0x02);
        //加入命令标志
        Inqueue(0x01);
        Inqueue(Intensity1&0xff);
        Inqueue((Intensity1>>8)&0xff);
        Inqueue((Intensity1>>16)&0xff);
        Inqueue((Intensity1>>24)&0xff);
        Inqueue(Intensity2&0xff);
        Inqueue((Intensity2>>8)&0xff);
        Inqueue((Intensity2>>16)&0xff);
        Inqueue((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue(0);
        Inqueue(0x04);//buf18
        Inqueue(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue(0);
        Inqueue(0x03);
		Delay5ms(100);
		//end

		//collin add20141105
		  //加入帧头
        Inqueue_uart1(0x02);
        //加入命令标志
        Inqueue_uart1(0x01);
        Inqueue_uart1(Intensity1&0xff);
        Inqueue_uart1((Intensity1>>8)&0xff);
        Inqueue_uart1((Intensity1>>16)&0xff);
        Inqueue_uart1((Intensity1>>24)&0xff);
        Inqueue_uart1(Intensity2&0xff);
        Inqueue_uart1((Intensity2>>8)&0xff);
        Inqueue_uart1((Intensity2>>16)&0xff);
        Inqueue_uart1((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x04);//buf18
        Inqueue_uart1(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(100);
		//end

        C12SABS = -log10((float)Intensitys[6] /( (float)Intensitys[4]));
        m_lC12S = ((((m_lCfC12[5] * C12SABS + m_lCfC12[4]) * C12SABS + m_lCfC12[3]) * C12SABS + m_lCfC12[2]) * C12SABS + m_lCfC12[1]) * C12SABS + m_lCfC12[0];
        if(m_bIgnoreConcentrationLow==0)
        {
            if(m_lC12S<0.3)
            {
                m_bMeasureBreak=3;//样气浓度过低，中止测量
				m_bBusing=0;
				flag_err[checkChannel-1]=1;
				LED(checkChannel,1);
				checkChannel=0;
				if(m_mesureType==4)ch_selected=0;
                break;	 
            }
        }
        //	if(m_bDilution)
		/*	
        {
			C12BABS = -log10((float)Intensitys[2] / ((float)Intensitys[0]));
	    	m_lC12B = ((((m_lCfC12[5] * C12BABS + m_lCfC12[4]) * C12BABS + m_lCfC12[3]) * C12BABS + m_lCfC12[2]) * C12BABS + m_lCfC12[1]) * C12BABS + m_lCfC12[0];
			 //collin mask20141025
			 
	        if ((ABS(m_lC12B - m_lC12S) > 0.05)&&(m_mesureType==1||m_mesureType==4))
	        {
	        	m_bDilution = 1;
				if(m_lC12B <2.2|| m_lC12S<2.2)
				{
					Per = 0.900;
				}
				else if(m_lC12B>4.0&& m_lC12S>4.0)
				{
					Per = 0.750;
				}
				else
				{
					Per = 0.800;
				}
				//	Per=850;
	            if (m_lC12B > m_lC12S)
	            {
	                Percent =  m_lC12S / m_lC12B * Per ;
	                m_bChanged = 1;
	            }
	            else
	            {
	                Percent =  m_lC12B / m_lC12S * Per;
	                m_bChanged = 0;
	            }
	        }

	        else
            {
                m_bDilution = 0;
            }
			
			//end
        }
		*/
		//collin add20141125
	ledChannel = 0;
	LED(1,1);
	LED(2,1);
	//end
        break;
    case 	5:
        Air();
        Intensitys[8]=Intensity1;
        Intensitys[9]=Intensity2;
		//collin add20141105
		  //加入帧头
        Inqueue(0x02);
        //加入命令标志
        Inqueue(0x01);
        Inqueue(Intensity1&0xff);
        Inqueue((Intensity1>>8)&0xff);
        Inqueue((Intensity1>>16)&0xff);
        Inqueue((Intensity1>>24)&0xff);
        Inqueue(Intensity2&0xff);
        Inqueue((Intensity2>>8)&0xff);
        Inqueue((Intensity2>>16)&0xff);
        Inqueue((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue(0);
        Inqueue(0x05);//buf18
        Inqueue(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue(0);
        Inqueue(0x03);
		Delay5ms(100);
		//end

		//collin add20141105
		  //加入帧头
        Inqueue_uart1(0x02);
        //加入命令标志
        Inqueue_uart1(0x01);
        Inqueue_uart1(Intensity1&0xff);
        Inqueue_uart1((Intensity1>>8)&0xff);
        Inqueue_uart1((Intensity1>>16)&0xff);
        Inqueue_uart1((Intensity1>>24)&0xff);
        Inqueue_uart1(Intensity2&0xff);
        Inqueue_uart1((Intensity2>>8)&0xff);
        Inqueue_uart1((Intensity2>>16)&0xff);
        Inqueue_uart1((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x05);//buf18
        Inqueue_uart1(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(100);
		//end
				C12BABS = -log10((float)Intensitys[2] / ((float)(Intensitys[4]* 0.5+Intensitys[0]*0.5)));
	    m_lC12B = ((((m_lCfC12[5] * C12BABS + m_lCfC12[4]) * C12BABS + m_lCfC12[3]) * C12BABS + m_lCfC12[2]) * C12BABS + m_lCfC12[1]) * C12BABS + m_lCfC12[0];

        C12SABS = -log10((float)Intensitys[6] /( (float)(Intensitys[4]*0.7+Intensitys[8]* 0.3)));
        m_lC12S = ((((m_lCfC12[5] * C12SABS + m_lCfC12[4]) * C12SABS + m_lCfC12[3]) * C12SABS + m_lCfC12[2]) * C12SABS + m_lCfC12[1]) * C12SABS + m_lCfC12[0];

		//	if(m_bDilution)
		{
	        if ((ABS(m_lC12B - m_lC12S) > 0.05)&&(m_mesureType==1||m_mesureType==4))
	        {
	            m_bDilution = 1;
				if(m_lC12B <2.2|| m_lC12S<2.2)
				{
					Per = 0.900;
				}
				else if(m_lC12B>4.0&& m_lC12S>4.0)
				{
					Per = 0.750;
				}
				else
				{
					Per = 0.800;
				}
	            if (m_lC12B > m_lC12S)
	            {
	                Percent =  m_lC12S / m_lC12B * Per ;
	                m_bChanged = 1;
	            }
	            else   
	            {
	                Percent =  m_lC12B / m_lC12S * Per;
	                m_bChanged = 0;
	            }	             
	        }
	        else
	        {
	         	m_bDilution = 0;
	    	}		
		}	
        break;
    case 	6:
	//collin add20141125
	ledChannel = 1;
	//end
        if(m_bDilution==1)
        {
            if(m_bChanged)
            {
                Sample(Pnum+10,m,Percent,1);//	Pno =8;
            }
            else
            {
                Sample(Pnum+10,m,Per,1);
            }
        }
        else
            Sample(Pnum+10,m,Per,0);
        Intensitys[10]=Intensity1;
        Intensitys[11]=Intensity2;
		//collin add20141105
		  //加入帧头
        Inqueue(0x02);
        //加入命令标志
        Inqueue(0x01);
        Inqueue(Intensity1&0xff);
        Inqueue((Intensity1>>8)&0xff);
        Inqueue((Intensity1>>16)&0xff);
        Inqueue((Intensity1>>24)&0xff);
        Inqueue(Intensity2&0xff);
        Inqueue((Intensity2>>8)&0xff);
        Inqueue((Intensity2>>16)&0xff);
        Inqueue((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue(0);
        Inqueue(0x06);//buf18
        Inqueue(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue(0);
        Inqueue(0x03);
		Delay5ms(100);
		//end

		//collin add20141105
		  //加入帧头
        Inqueue_uart1(0x02);
        //加入命令标志
        Inqueue_uart1(0x01);
        Inqueue_uart1(Intensity1&0xff);
        Inqueue_uart1((Intensity1>>8)&0xff);
        Inqueue_uart1((Intensity1>>16)&0xff);
        Inqueue_uart1((Intensity1>>24)&0xff);
        Inqueue_uart1(Intensity2&0xff);
        Inqueue_uart1((Intensity2>>8)&0xff);
        Inqueue_uart1((Intensity2>>16)&0xff);
        Inqueue_uart1((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x06);//buf18
        Inqueue_uart1(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(100);
		//end

		//collin add20141125
		ledChannel = 0;
		LED(1,1);
		LED(2,1);
		//end
        break;
    case 	7:

        Air();
        Intensitys[12]=Intensity1;
        Intensitys[13]=Intensity2;
		//collin add20141105
		  //加入帧头
        Inqueue(0x02);
        //加入命令标志
        Inqueue(0x01);
        Inqueue(Intensity1&0xff);
        Inqueue((Intensity1>>8)&0xff);
        Inqueue((Intensity1>>16)&0xff);
        Inqueue((Intensity1>>24)&0xff);
        Inqueue(Intensity2&0xff);
        Inqueue((Intensity2>>8)&0xff);
        Inqueue((Intensity2>>16)&0xff);
        Inqueue((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue(0);
        Inqueue(0x07);//buf18
        Inqueue(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue(0);
        Inqueue(0x03);
		Delay5ms(100);
		//end

		//collin add20141105
		  //加入帧头
        Inqueue_uart1(0x02);
        //加入命令标志
        Inqueue_uart1(0x01);
        Inqueue_uart1(Intensity1&0xff);
        Inqueue_uart1((Intensity1>>8)&0xff);
        Inqueue_uart1((Intensity1>>16)&0xff);
        Inqueue_uart1((Intensity1>>24)&0xff);
        Inqueue_uart1(Intensity2&0xff);
        Inqueue_uart1((Intensity2>>8)&0xff);
        Inqueue_uart1((Intensity2>>16)&0xff);
        Inqueue_uart1((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x07);//buf18
        Inqueue_uart1(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(100);
        break;
    case 	8:
		//collin add20141125
	ledChannel = 2;
	//end
        if(m_bDilution==1)
        {
            if(m_bChanged)
                Sample(Pnum+20,m,Per,1);
            else
                Sample(Pnum+20,m,Percent,1);
        }
        else
            Sample(Pnum+20,m,Per,0);
        Intensitys[14]=Intensity1;
        Intensitys[15]=Intensity2;
		//collin add20141105
		  //加入帧头
        Inqueue(0x02);
        //加入命令标志
        Inqueue(0x01);
        Inqueue(Intensity1&0xff);
        Inqueue((Intensity1>>8)&0xff);
        Inqueue((Intensity1>>16)&0xff);
        Inqueue((Intensity1>>24)&0xff);
        Inqueue(Intensity2&0xff);
        Inqueue((Intensity2>>8)&0xff);
        Inqueue((Intensity2>>16)&0xff);
        Inqueue((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue(0);
        Inqueue(0x08);//buf18
        Inqueue(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue(0);
        Inqueue(0x03);
		Delay5ms(100);
		//end

		//collin add20141105
		  //加入帧头
        Inqueue_uart1(0x02);
        //加入命令标志
        Inqueue_uart1(0x01);
        Inqueue_uart1(Intensity1&0xff);
        Inqueue_uart1((Intensity1>>8)&0xff);
        Inqueue_uart1((Intensity1>>16)&0xff);
        Inqueue_uart1((Intensity1>>24)&0xff);
        Inqueue_uart1(Intensity2&0xff);
        Inqueue_uart1((Intensity2>>8)&0xff);
        Inqueue_uart1((Intensity2>>16)&0xff);
        Inqueue_uart1((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x08);//buf18
        Inqueue_uart1(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(100);
		//end

			//collin add20141125
	ledChannel = 0;
	LED(1,1);
	LED(2,1);
	//end
        break;
    case 	9:
        Air();
        Valve(161);
        Valve(150);
        Intensitys[16]=Intensity1;
        Intensitys[17]=Intensity2;
		//collin add20141105
		  //加入帧头
        Inqueue(0x02);
        //加入命令标志
        Inqueue(0x01);
        Inqueue(Intensity1&0xff);
        Inqueue((Intensity1>>8)&0xff);
        Inqueue((Intensity1>>16)&0xff);
        Inqueue((Intensity1>>24)&0xff);
        Inqueue(Intensity2&0xff);
        Inqueue((Intensity2>>8)&0xff);
        Inqueue((Intensity2>>16)&0xff);
        Inqueue((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue(0);
        Inqueue(0x09);//buf18
        Inqueue(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue(0);
        Inqueue(0x03);
		Delay5ms(100);
		//end

		//collin add20141105
		  //加入帧头
        Inqueue_uart1(0x02);
        //加入命令标志
        Inqueue_uart1(0x01);
        Inqueue_uart1(Intensity1&0xff);
        Inqueue_uart1((Intensity1>>8)&0xff);
        Inqueue_uart1((Intensity1>>16)&0xff);
        Inqueue_uart1((Intensity1>>24)&0xff);
        Inqueue_uart1(Intensity2&0xff);
        Inqueue_uart1((Intensity2>>8)&0xff);
        Inqueue_uart1((Intensity2>>16)&0xff);
        Inqueue_uart1((Intensity2>>24)&0xff);
        for(k=0; k<8; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x09);//buf18
        Inqueue_uart1(0x09);//buf19
        for(k=0; k<3; k++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(100);
		//end

        Sys_Status = Sys_Idle;
        
        m_bBusing=0;
        GPIO_WriteBit(GPIO_M1MO,GPIO_Pin_M1MO,Bit_SET);
        Speaker(1);
        break;
    default:
        break;
    }
    AK_Left_Data[8]	= m_mesureType+Pno<<8;
	if(Pno!=0)
	{
		for(i=0;i<10;i++)
			SendBackup[Pno-1][i]=AK_Left_Data[i];
	}
    if(m_bMeasureBreak==0)
    {
        if(Pno==9)
        {	 
			for(i=0;i<9;i++)
			{
				for(j=0;j<10;j++)
				{
					AK_Left_Data[j]=SendBackup[i][j];
				}
				Second_1=0;
				SendData();
				Delay5ms(10);
			} 
            //1.吸光度

			//collin modify20141024
			
            xbc12=-log10((float)(Intensitys[10]<<1) / ((float)Intensitys[8]+(float)Intensitys[12]));
            xsc12=-log10((float)(Intensitys[14]<<1) / ((float)Intensitys[12]+(float)Intensitys[16]));

            xbc13=-log10((float)(Intensitys[11]<<1) / ((float)Intensitys[9]+(float)Intensitys[13]));
            xsc13=-log10((float)(Intensitys[15]<<1) / ((float)Intensitys[13]+(float)Intensitys[17]));
			 
			/*
			xbc12=-log10((float)(Intensitys[10]) / ((float)Intensitys[8]*0.4667+(float)Intensitys[12]*0.5333));
            xsc12=-log10((float)(Intensitys[14]) / ((float)Intensitys[12]*0.4832+(float)Intensitys[16]*0.5168));

            xbc13=-log10((float)(Intensitys[11]) / ((float)Intensitys[9]*0.4667+(float)Intensitys[13]*0.53333));
            xsc13=-log10((float)(Intensitys[15]) / ((float)Intensitys[13]*0.4832+(float)Intensitys[17]*0.5168));
			 */
			//end
            if(m_mesureType==8)
            {				
                absC12[serial]	= (xbc12+xsc12)/2;
                //absC12[(serial<<2)+1]= xsc12;
                absC13[serial]	= (xbc13+xsc13)/2;
                //absC13[(serial<<2)+1]= xsc13;
				serial++ ;
                Inqueue(0x02);
                //加入命令标志
                Inqueue(0xFF);
                Inqueue(0x00);
                Inqueue(0x46);
                Inqueue(serial);
                for(i=0; i<FrameLength-6; i++)
                    Inqueue(0);
                Inqueue(0x03);
                Delay5ms(50);

				Inqueue_uart1(0x02);
                //加入命令标志
                Inqueue_uart1(0xFF);
                Inqueue_uart1(0x00);
                Inqueue_uart1(0x46);
                Inqueue_uart1(serial);
                for(i=0; i<FrameLength-6; i++)
                    Inqueue_uart1(0);
                Inqueue_uart1(0x03);
                Delay5ms(50);

                if(serial>=20)
                {
					double dt1[3];
					double dt2[3];
                    //hpir(吸光度，浓度，样品数量，输出系数，次数）
                    hpir(absC12,consC12, 20, m_lCfC12, 5,dt1);
                    hpir(absC13,consC13, 20, m_lCfC13, 5,dt2);
					//保存吸光度到Flash
					for(i=0;i<20;i++)
					{
						for (j=0; j<8; j++)  
						{  
						   Abs_backup[i*8+j] = *((char *)(&absC12[i]) + j);
						}
					} 
					for(i=0;i<20;i++)
					{
						for (j=0; j<8; j++)  
						{  
						   Abs_backup[i*8+160+j] = *((char *)(&absC13[i]) + j);
						}
					}
					SPI_FLASH_SectorErase(0x7D0000);
					SPI_FLASH_BufferWrite(Abs_backup,0x7D0000,320); 
					//end
					//写入参数序列
					for(i=0;i<6;i++)
					{
						for (j=0; j<8; j++)  
						{  
						   Cf[i*8+32+j] = *((char *)(&m_lCfC12[i]) + j);
						}
					} 
					for(i=0;i<6;i++)
					{
						for (j=0; j<8; j++)  
						{  
						   Cf[i*8+80+j] = *((char *)(&m_lCfC13[i]) + j);
						}
					} 
					m_bBusing=1; 
					//写入存储器
					I2C_EE_BufferWrite(Cf, 0,128);
					m_bBusing=0; 
					Inqueue(0x02);
	                //加入命令标志
	                Inqueue(0xFF);
	                Inqueue(0x00);
	                Inqueue(0x47);
	                Inqueue((u32)(dt1[0]*100000)&0xFF);
					Inqueue(((u32)(dt1[0]*100000)>>8)&0xFF);
					Inqueue(((u32)(dt1[0]*100000)>>16)&0xFF);
					Inqueue(((u32)(dt1[0]*100000)>>24)&0xFF);
					Inqueue((u32)(dt2[0]*100000)&0xFF);
					Inqueue(((u32)(dt2[0]*100000)>>8)&0xFF);
					Inqueue(((u32)(dt2[0]*100000)>>16)&0xFF);
					Inqueue(((u32)(dt2[0]*100000)>>24)&0xFF);
	                for(i=0; i<FrameLength-13; i++)
	                    Inqueue(0);
	                Inqueue(0x03);
	                Delay5ms(200);

					Inqueue_uart1(0x02);
	                //加入命令标志
	                Inqueue_uart1(0xFF);
	                Inqueue_uart1(0x00);
	                Inqueue_uart1(0x47);
	                Inqueue_uart1((u32)(dt1[0]*100000)&0xFF);
					Inqueue_uart1(((u32)(dt1[0]*100000)>>8)&0xFF);
					Inqueue_uart1(((u32)(dt1[0]*100000)>>16)&0xFF);
					Inqueue_uart1(((u32)(dt1[0]*100000)>>24)&0xFF);
					Inqueue_uart1((u32)(dt2[0]*100000)&0xFF);
					Inqueue_uart1(((u32)(dt2[0]*100000)>>8)&0xFF);
					Inqueue_uart1(((u32)(dt2[0]*100000)>>16)&0xFF);
					Inqueue_uart1(((u32)(dt2[0]*100000)>>24)&0xFF);
	                for(i=0; i<FrameLength-13; i++)
	                    Inqueue_uart1(0);
	                Inqueue_uart1(0x03);
	                Delay5ms(200);
					
					for(i=4;i<16;i++)  
					{
						//发送标准曲线
						Inqueue(0x02);
						Inqueue(0xFF);
						Inqueue(0x00);
						Inqueue(0x43);
						Inqueue(i);
						for(j=0;j<8;j++)
							Inqueue(Cf[j+i*8]);
						for(j=0;j<FrameLength-14;j++)
							Inqueue(0);
						Inqueue(0x03);
						Delay5ms(20);
						
						Inqueue_uart1(0x02);
						Inqueue_uart1(0xFF);
						Inqueue_uart1(0x00);
						Inqueue_uart1(0x43);
						Inqueue_uart1(i);
						for(j=0;j<8;j++)
							Inqueue_uart1(Cf[j+i*8]);
						for(j=0;j<FrameLength-14;j++)
							Inqueue_uart1(0);
						Inqueue_uart1(0x03);
						Delay5ms(20);						
					} 

					while(1)
					{

						//collin add20141011
						//制作标准曲线完成重启命令字
						if (funtion_complete_reboot_flag != 5)
						{
						Inqueue(0x02);
                		Inqueue(0xFF);
                		Inqueue(0x00);
                		Inqueue(0xf0);
                		for(i=0; i<FrameLength-5; i++)
                    		Inqueue(0);
                		Inqueue(0x03);
                		Delay5ms(10);

						Inqueue_uart1(0x02);
                		Inqueue_uart1(0xFF);
                		Inqueue_uart1(0x00);
                		Inqueue_uart1(0xf0);
                		for(i=0; i<FrameLength-5; i++)
                    		Inqueue_uart1(0);
                		Inqueue_uart1(0x03);
                		Delay5ms(10);

						funtion_complete_reboot_flag++;
						}
						//end											
					}
				}
            }
            else
            {
                //2.浓度				
                cbc12 = ((((m_lCfC12[5] * xbc12 + m_lCfC12[4]) * xbc12 + m_lCfC12[3]) * xbc12 + m_lCfC12[2]) * xbc12 + m_lCfC12[1]) * xbc12 + m_lCfC12[0];
                csc12 = ((((m_lCfC12[5] * xsc12 + m_lCfC12[4]) * xsc12 + m_lCfC12[3]) * xsc12 + m_lCfC12[2]) * xsc12 + m_lCfC12[1]) * xsc12 + m_lCfC12[0];
                cbc13 = ((((m_lCfC13[5] * xbc13 + m_lCfC13[4]) * xbc13 + m_lCfC13[3]) * xbc13 + m_lCfC13[2]) * xbc13 + m_lCfC13[1]) * xbc13 + m_lCfC13[0];
                csc13 = ((((m_lCfC13[5] * xsc13 + m_lCfC13[4]) * xsc13 + m_lCfC13[3]) * xsc13 + m_lCfC13[2]) * xsc13 + m_lCfC13[1]) * xsc13 + m_lCfC13[0];
				//根据浓度定位5个点，做标准曲线
				//collin mask20141222
				/*			

				for(i=19;i>=0;i--)
				{
				if(consC12[i]<((cbc12+csc12)/2))
				{
				for(j=0;j<5;j++)
				{
				absc12_st[j]= absC12_backup[i-2+j];
				absc13_st[j]= absC13_backup[i-2+j];
				c12_st[j] = consC12[i-2+j];
				c13_st[j] = consC13[i-2+j];
				}
				break;
				}
				}
				hpir(absc12_st,c12_st, 5, m_lCfC12_st, 5,dt1_st);
                hpir(absc13_st,c13_st, 5, m_lCfC13_st, 5,dt2_st);
				cbc12 = ((((m_lCfC12_st[5] * xbc12 + m_lCfC12_st[4]) * xbc12 + m_lCfC12_st[3]) * xbc12 + m_lCfC12_st[2]) * xbc12 + m_lCfC12_st[1]) * xbc12 + m_lCfC12_st[0];
                csc12 = ((((m_lCfC12_st[5] * xsc12 + m_lCfC12_st[4]) * xsc12 + m_lCfC12_st[3]) * xsc12 + m_lCfC12_st[2]) * xsc12 + m_lCfC12_st[1]) * xsc12 + m_lCfC12_st[0];
                cbc13 = ((((m_lCfC13_st[5] * xbc13 + m_lCfC13_st[4]) * xbc13 + m_lCfC13_st[3]) * xbc13 + m_lCfC13_st[2]) * xbc13 + m_lCfC13_st[1]) * xbc13 + m_lCfC13_st[0];
                csc13 = ((((m_lCfC13_st[5] * xsc13 + m_lCfC13_st[4]) * xsc13 + m_lCfC13_st[3]) * xsc13 + m_lCfC13_st[2]) * xsc13 + m_lCfC13_st[1]) * xsc13 + m_lCfC13_st[0];
				*/
				//end
				
				//3.浓度比
				//collin modify20141013
				/*
                ratebase	= cbc13*100.0/cbc12*10.0;	 //107.7-->1.077
                ratesample	= csc13*100.0/csc12*10.0;
				*/
				//collin add20141121
				/*
				if (m_mesureType == 4)
				{
				ratebase = (float)Intensitys[9]/(float)Intensitys[8]*1000.0;
				ratesample = (float)Intensitys[13]/(float)Intensitys[12]*1000.0;
				}
				else
				{
				*/
				//end
							//collin add 20141124
							
			//c13底气和样气光强值的差
			 
			//xbc12=-log10((float)(Intensitys[10]<<1) / ((float)Intensitys[8]+(float)Intensitys[12]));
			//I13bs= Intensitys[11]*(1/pow(10,1.12372*xbc12*0.05)-1)/1000.0;
			//I13bs = 1.12372*0.2*(xbc12+xsc12)/2.0/1000.0;
			
			//I13bs = 1.12372*0.1/1000.0;
			//Tbs = xsc13/xsc12-xbc13/xbc12;
			//if ((m_mesureType == 4) && (I13bs > abs((float)Intensitys[11]-((float)Intensitys[15] - abs(((float)Intensitys[9]+(float)Intensitys[13])/2-((float)Intensitys[13]+(float)Intensitys[17])/2)) )))
			//if ((m_mesureType == 4) && (I13bs > abs(xsc13/xsc12-xbc13/xbc12)))
			//{
			/*
			xbc12=-log10((float)(Intensitys[10]<<1) / ((float)Intensitys[8]+(float)Intensitys[12]));
			xbc13=-log10((float)(Intensitys[11]<<1) / ((float)Intensitys[9]+(float)Intensitys[13]));

			xsc12=-log10((((float)Intensitys[10] - abs(((float)Intensitys[8]+(float)Intensitys[12])/2.0-((float)Intensitys[12]+(float)Intensitys[16])/2.0)) *2)/ ((float)Intensitys[12]+(float)Intensitys[16]));
			xsc13=-log10((((float)Intensitys[11] - abs(((float)Intensitys[9]+(float)Intensitys[13])/2.0-((float)Intensitys[13]+(float)Intensitys[17])/2.0)) *2)/ ((float)Intensitys[13]+(float)Intensitys[17]));
			*/
		//	ratebase = xbc13/xbc12*1000.0;
		//	ratesample = xsc13/xsc12*1000.0;
			//}
		//	else
		//	{
			  
			//end
				ratebase	= cbc13/cbc12*1000.0;	 //107.7-->1.077
                ratesample	= csc13/csc12*1000.0;
			//	ratebase_debug =  xbc13/xbc12*1000.0;
			//	ratesample_debug = xsc13/xsc12*1000.0;
			//	}
				//ratebase	= cbc13/cbc12/10000.0;	 //107.7-->1.077
                //ratesample	= csc13/csc12/10000.0;
				//end
                //4.dob	 *100
				//collin modify20141024
                dob=(s16)((ratesample-ratebase)*100.0/1.12372);
				//dob_debug = ((ratesample_debug-ratebase_debug)*100.0/1.12372);
				/*
				if ((ratebase/1077.0) > 1)
                   dob=((ratesample-ratebase)*100.0/1.12372)/(ratebase/1077.0);
                else
                   dob=((ratesample-ratebase)*100.0/1.12372);
				   */
				//end

				if(m_mesureType==4)
				{
					DOB_ZK[qualityCount-1]=dob;
					keynum=0;
					while(qualityCount==10)
					{
						s32 DOB_PJ_temp=0;
						double BZWC_temp=0.0;
						BZWC=0;
						ZF=0;
						if(ch_selected)
						{
							checkChannel=0;
							LED(1,0);	
						}
						for(i=0;i<qualityCount;i++)
						{
							DOB_PJ_temp+=DOB_ZK[i];
						}
						DOB_PJ=(s16)((float)DOB_PJ_temp/(float)qualityCount);
						for(i=0;i<qualityCount;i++)
						{
							BZWC_temp+=(DOB_ZK[i]-DOB_PJ)*(DOB_ZK[i]-DOB_PJ);
						}
						BZWC=(s16)(sqrt(BZWC_temp/qualityCount)/10.0);	 
						if(DOB_PJ<=-10)
						{	
							ZF=1;
							DOB_PJ=abs(DOB_PJ);							
							DOB_PJ=(DOB_PJ/10);						
						}
						else
						{
						 	DOB_PJ=abs(DOB_PJ);							
							DOB_PJ=(DOB_PJ/10);	
						}
						//collin add20141017
					//十组测量值发送
					 Inqueue(0x02);
                	Inqueue(0xFF);
                	Inqueue(0xf9);
					Inqueue(DOB_ZK[0]&0xff);
					Inqueue((DOB_ZK[0]>>8)&0xff);
					Inqueue(DOB_ZK[1]&0xff);
					Inqueue((DOB_ZK[1]>>8)&0xff);
					Inqueue(DOB_ZK[2]&0xff);
					Inqueue((DOB_ZK[2]>>8)&0xff);
					Inqueue(DOB_ZK[3]&0xff);
					Inqueue((DOB_ZK[3]>>8)&0xff);
					Inqueue(DOB_ZK[4]&0xff);
					Inqueue((DOB_ZK[4]>>8)&0xff);
					Inqueue(DOB_ZK[5]&0xff);
					Inqueue((DOB_ZK[5]>>8)&0xff);
					Inqueue(DOB_ZK[6]&0xff);
					Inqueue((DOB_ZK[6]>>8)&0xff);
					Inqueue(DOB_ZK[7]&0xff);
					Inqueue((DOB_ZK[7]>>8)&0xff);
					Inqueue(DOB_ZK[8]&0xff);
					Inqueue((DOB_ZK[8]>>8)&0xff);
					Inqueue(DOB_ZK[9]&0xff);
					Inqueue((DOB_ZK[9]>>8)&0xff);					
                	Inqueue(0x03);
                	Delay5ms(10);
						//end

						//十组测量值发送
					 Inqueue_uart1(0x02);
                	Inqueue_uart1(0xFF);
                	Inqueue_uart1(0xf9);
					Inqueue_uart1(DOB_ZK[0]&0xff);
					Inqueue_uart1((DOB_ZK[0]>>8)&0xff);
					Inqueue_uart1(DOB_ZK[1]&0xff);
					Inqueue_uart1((DOB_ZK[1]>>8)&0xff);
					Inqueue_uart1(DOB_ZK[2]&0xff);
					Inqueue_uart1((DOB_ZK[2]>>8)&0xff);
					Inqueue_uart1(DOB_ZK[3]&0xff);
					Inqueue_uart1((DOB_ZK[3]>>8)&0xff);
					Inqueue_uart1(DOB_ZK[4]&0xff);
					Inqueue_uart1((DOB_ZK[4]>>8)&0xff);
					Inqueue_uart1(DOB_ZK[5]&0xff);
					Inqueue_uart1((DOB_ZK[5]>>8)&0xff);
					Inqueue_uart1(DOB_ZK[6]&0xff);
					Inqueue_uart1((DOB_ZK[6]>>8)&0xff);
					Inqueue_uart1(DOB_ZK[7]&0xff);
					Inqueue_uart1((DOB_ZK[7]>>8)&0xff);
					Inqueue_uart1(DOB_ZK[8]&0xff);
					Inqueue_uart1((DOB_ZK[8]>>8)&0xff);
					Inqueue_uart1(DOB_ZK[9]&0xff);
					Inqueue_uart1((DOB_ZK[9]>>8)&0xff);					
                	Inqueue_uart1(0x03);
                	Delay5ms(10);

						//collin add20141014
					Inqueue(0x02);
                	Inqueue(0xFF);
                	Inqueue(0x00);
                	Inqueue(0xf6);
					Inqueue(DOB_PJ&0xff);
					Inqueue((DOB_PJ>>8)&0xff);
					Inqueue(BZWC&0xff);
					Inqueue((BZWC>>8)&0xff);
					Inqueue(ZF);
                	for(i=0; i<FrameLength-10; i++)
                    	Inqueue(0);
                	Inqueue(0x03);
                	Delay5ms(10);

						//collin add20141014
					Inqueue_uart1(0x02);
                	Inqueue_uart1(0xFF);
                	Inqueue_uart1(0x00);
                	Inqueue_uart1(0xf6);
					Inqueue_uart1(DOB_PJ&0xff);
					Inqueue_uart1((DOB_PJ>>8)&0xff);
					Inqueue_uart1(BZWC&0xff);
					Inqueue_uart1((BZWC>>8)&0xff);
					Inqueue_uart1(ZF);
                	for(i=0; i<FrameLength-10; i++)
                    	Inqueue_uart1(0);
                	Inqueue_uart1(0x03);
                	Delay5ms(10);

					qualityCount = 0;
					//end								 
						if(ch_selected)print_result_qc(); 	
						ch_selected=0;
						if(keynum>0)qualityCount=0;

						if(chopperDelaytimes>=6000)
				        {
				            chopperDelaytimes=0;
				            LightModulation(OFF);
				        }					
					}					
				}
                else
				{
					Num++;
					GetDate();
					GetTime();
					//collin mask20141025
					ID[0]= (dob&0xFF00)>>8;
					ID[1]=  dob&0x00FF;
					//end					
					ID[2]= (Num&0xFF00)>>8;
					ID[3]=  Num&0x00FF;
					ID[5]= ((u16)(cbc12*1000)&0xFF00)>>8;
					ID[4]=  (u16)(cbc12*1000)&0x00FF;
					ID[6]= checkChannel;
					ID[18]= ((u16)(csc12*1000)&0xFF00)>>8;
					ID[17]=  (u16)(csc12*1000)&0x00FF;
		            ID[7]=timer.w_year/10%10;
		            ID[8]=timer.w_year%10;
		            ID[9]=timer.w_month/10;
		            ID[10]=timer.w_month%10;
		            ID[11]=timer.w_date/10;
		            ID[12]=timer.w_date%10;
		            ID[13]=timer.hour/10;
		            ID[14]=timer.hour%10;
		            ID[15]=timer.min/10;
		            ID[16]=timer.min%10;
					ID[20]=flag_dob;
					SPI_FLASH_SectorErase(0x7E0000);
					SPI_FLASH_BufferWrite(ID,0x7E0000,30);
					SPI_FLASH_BufferWrite(ID,0x010000+Num*30,30);
					//collin mask20140917
					//print_result();
					//end
				}
                QueryFlag	= 1;
                //加入帧头
                Inqueue(0x02);
                //加入命令标志
                Inqueue(0x01);
                Inqueue(0x00);
                Inqueue(0x40);
				//collin mask20141025
                Inqueue(dob&0xff);
                Inqueue((dob>>8)&0xff);
				//end
                Inqueue(((u16)(m_lC12B*1000))&0xff);
                Inqueue(((u16)(m_lC12B*1000)>>8)&0xff);
                Inqueue(((u16)(m_lC12S*1000))&0xff);
                Inqueue(((u16)(m_lC12S*1000)>>8)&0xff);
                Inqueue(checkChannel);
				Inqueue(((u16)(cbc12*1000))&0xff);
                Inqueue(((u16)(cbc12*1000)>>8)&0xff);
                Inqueue(((u16)(csc12*1000))&0xff);
                Inqueue(((u16)(csc12*1000)>>8)&0xff);
                Inqueue(((u16)(cbc13*1000))&0xff);
                Inqueue(((u16)(cbc13*1000)>>8)&0xff);
                Inqueue(((u16)(csc13*1000))&0xff);
                Inqueue(((u16)(csc13*1000)>>8)&0xff);
                Inqueue(i+1);//buf19
                Inqueue(m_mesureType);//buf20
				if(cbc12<0.5)
					Inqueue(1);
				else if(csc12<0.5)
					Inqueue(2);
				else
					Inqueue(0);					
                Inqueue(0);
                Inqueue(0x03);
                Delay5ms(400);

				QueryFlag_uart1	= 1;
                //加入帧头
                Inqueue_uart1(0x02);
                //加入命令标志
                Inqueue_uart1(0x01);
                Inqueue_uart1(0x00);
                Inqueue_uart1(0x40);
				//collin mask20141025
                Inqueue_uart1(dob&0xff);
                Inqueue_uart1((dob>>8)&0xff);
				//end
                Inqueue_uart1(((u16)(m_lC12B*1000))&0xff);
                Inqueue_uart1(((u16)(m_lC12B*1000)>>8)&0xff);
                Inqueue_uart1(((u16)(m_lC12S*1000))&0xff);
                Inqueue_uart1(((u16)(m_lC12S*1000)>>8)&0xff);
                Inqueue_uart1(checkChannel);
				Inqueue_uart1(((u16)(cbc12*1000))&0xff);
                Inqueue_uart1(((u16)(cbc12*1000)>>8)&0xff);
                Inqueue_uart1(((u16)(csc12*1000))&0xff);
                Inqueue_uart1(((u16)(csc12*1000)>>8)&0xff);
                Inqueue_uart1(((u16)(cbc13*1000))&0xff);
                Inqueue_uart1(((u16)(cbc13*1000)>>8)&0xff);
                Inqueue_uart1(((u16)(csc13*1000))&0xff);
                Inqueue_uart1(((u16)(csc13*1000)>>8)&0xff);
                Inqueue_uart1(i+1);//buf19
                Inqueue_uart1(m_mesureType);//buf20
				if(cbc12<0.5)
					Inqueue_uart1(1);
				else if(csc12<0.5)
					Inqueue_uart1(2);
				else
					Inqueue_uart1(0);					
                Inqueue_uart1(0);
                Inqueue_uart1(0x03);
                Delay5ms(400);
				

				//	print_result();

					if(m_mesureType == 1)
				while(Sys_Status != Sys_Print_Yes && Sys_Status != Sys_Print_No);
				if (Sys_Status == Sys_Print_Yes)
				{
					print_result();
					Sys_Status = Sys_Running;
					}
					else if (Sys_Status == Sys_Print_No)
					{
					   Sys_Status = Sys_Running;
					}
            }
			LED(checkChannel,0);
			//collin add20141027
			LED(2,0);
			//end
			ch_checked[checkChannel]=1;
			checkChannel=0;
        }
    }
}

void LeakageCheck(u8 Go)
{	
   	 u8 i,j; 	
	 u16 Press[10]={0},Y;
	  if(Go==0)
 {
	
	Inqueue(0x02);
		//加入命令标志
	Inqueue(0xFF);
	Inqueue(0x00);
	Inqueue(0x45);
	Inqueue(0);	 
	for(i=0;i<FrameLength-6;i++)
		Inqueue(0);
	Inqueue(0x03);

	Inqueue_uart1(0x02);
		//加入命令标志
	Inqueue_uart1(0xFF);
	Inqueue_uart1(0x00);
	Inqueue_uart1(0x45);
	Inqueue_uart1(0);	 
	for(i=0;i<FrameLength-6;i++)
		Inqueue_uart1(0);
	Inqueue_uart1(0x03);

/* 	Motorun(0,T,FORWARD,0,      3,7,8,20,  3,6,20,0,     91,151,160);      //抽气

	Delay5ms(300);	
	Motorun(0,T-m,BACKWARD,0,   3,3,0,0,  3,0,0,0,      90,151,161);
	Delay5ms(200);				  
	Motorun(0,m,BACKWARD,0,     2,3,1,20,     2,4,50,200,   90,151,161);
	Valve(150);	 */
 	for(i=0;i<10;i++)
	{
		Delay5ms(200);         //延时
		Press[i] =ADC_Press(ADC_FS);
		Y=(u16)(((((float)(Press[i] + 28) * 3 / 4096) - 1) / 4)*1000);
		//debug_collin = Y;	
		Inqueue(0x02);
		Inqueue(0xFF);
		Inqueue(0x00);
		Inqueue(0x45);
		Inqueue(i+1);
		//collin modify20141014
		/*
		Inqueue((u8)(Press[i]%256));
		Inqueue((u8)(Press[i]/256));	 
		*/
		//debug1_collin = Y&0xFF;
		//debug2_collin = (Y>>8)&0xFF;
		Inqueue(Y&0xFF);
		Inqueue((Y>>8)&0xFF);
		for(j=0;j<FrameLength-8;j++)
		//end
			Inqueue(0);
		Inqueue(0x03);
		
		Inqueue_uart1(0x02);
		Inqueue_uart1(0xFF);
		Inqueue_uart1(0x00);
		Inqueue_uart1(0x45);
		Inqueue_uart1(i+1);
		//collin modify20141014
		/*
		Inqueue((u8)(Press[i]%256));
		Inqueue((u8)(Press[i]/256));	 
		*/
		//debug1_collin = Y&0xFF;
		//debug2_collin = (Y>>8)&0xFF;
		Inqueue_uart1(Y&0xFF);
		Inqueue_uart1((Y>>8)&0xFF);
		for(j=0;j<FrameLength-8;j++)
		//end
			Inqueue_uart1(0);
		Inqueue_uart1(0x03);
			
	}
	Delay5ms(10);
   	Inqueue(0x02);
	//加入命令标志
	Inqueue(0xFF);
	Inqueue(0x00);
	Inqueue(0x45);
	Inqueue(11);	
	if((Press[0]-Press[9])>100)
	{	
		Inqueue(1);
		print_pressure(Y,2);
	} 
	else
	{	
		Inqueue(0);
		print_pressure(Y,1);

	}
	for(i=0;i<FrameLength-7;i++)
		Inqueue(0);
	Inqueue(0x03);
//	Valve(151);
//	Valve(160);//排气  
	Delay5ms(100);

	Inqueue_uart1(0x02);
	Inqueue_uart1(0xFF);
	Inqueue_uart1(0x00);
	Inqueue_uart1(0x45);
	Inqueue_uart1(11);	
	if((Press[0]-Press[9])>100)
	{	
		Inqueue_uart1(1);
	} 
	else
	{	
		Inqueue_uart1(0);
	}
	for(i=0;i<FrameLength-7;i++)
		Inqueue_uart1(0);
	Inqueue_uart1(0x03);
	Delay5ms(100);
	}
}
  u16 n	= 0;
/*-----------------------------------
试打气体确定脉冲数（应在开机自检过程执行,测量4~6次？）
ADC基准电压3.0V；
压力表输出电压与表显压力满足关系式 y=4x+1；
AD值与压力的关系 AD=（4x+1）*4096/3.0；
打到目标压力（0.4Mpa）的脉冲数n=（目标压力/试测压力）*试测脉冲；
可得n=8738133/(AD-1365);
其中AD为试测时读取的AD值；
1365即为标准大气压时的AD值（pressure0），可多次测量取平均；
-------------------------------------*/
s16 Test(void)
{ 
	int	i;

	n	= 0;
	m_lCf[6]=2190;
	Go=0;
	//报告工作状态
	QueryFlag	= 1;
	Inqueue(0x02);
	Inqueue(0xFF);
	Inqueue(0x00);
	Inqueue(0x22);	
	for(i=0;i<FrameLength-5;i++)
		Inqueue(0);
	Inqueue(0x03);	
	Valve(151); 	
	Valve(160);
	Delay5ms(40);

	//报告工作状态
	QueryFlag_uart1	= 1;
	Inqueue_uart1(0x02);
	Inqueue_uart1(0xFF);
	Inqueue_uart1(0x00);
	Inqueue_uart1(0x22);	
	for(i=0;i<FrameLength-5;i++)
		Inqueue_uart1(0);
	Inqueue_uart1(0x03);	
	Valve(151); 	
	Valve(160);
	Delay5ms(40);

   	if(m_lCf[7]<3000||m_lCf[7]>5000)  
   		m_lCf[7]=5000;
   	T= m_lCf[7];
	while(Go<=3)
	{
		//放气
		Valve(151);
		Valve(160);//排气
		Valve(170);
		//collin add20141114
		//Valve(80);
		//end
		Valve(180);
		Delay5ms(200);
		Pressure0=ADC_Press(ADC_FS);				//读取压力表初始AD值
		Motorun(0,2000,FORWARD,0,     30,60,5,20,     30,6,20,0,        91,150,160);
		Delay5ms(10);
		//collin recover20141022
		Motorun(0,2000,BACKWARD,0,    30,60,5,20,     30,6,20,200,      90,151,160);
		//Motorun(1,2000,BACKWARD,0,    30,60,5,20,     30,6,20,200,      90,151,160);
		//end
		Delay5ms(20);
		//从阀门11吸气
		Motorun(0,2000,FORWARD,0,     30,60,5,20,     30,6,20,0,        91,150,160);
		Delay5ms(10);
		//collin recover20141022
		Motorun(0,2000,BACKWARD,0,    30,60,5,20,     30,6,20,200,      90,151,160);
		//Motorun(1,2000,BACKWARD,0,    30,60,5,20,     30,6,20,200,      90,151,160);
		//end
		Delay5ms(20);
		Motorun(0,T,FORWARD,0,     30,60,5,20,     30,7,20,0,        91,150,161);
		Delay5ms(300);
		Motorun(1,T,BACKWARD,0,    20,30,1,20,     20,4,50,200,      90,151,161);
	//	Delay5ms(400);
		//关闭阀门9保持样品池气压
		Valve(150);
		LeakageCheck(Go);
		Pressure1=ADC_Press(ADC_FS);         //读取压力表试测AD值
		Pressure1=Pressure1-Pressure0;
		if(m_lCf[8]==400)
		{
		// 	if(m_lCf[6]<1700||m_lCf[6]>2200)  m_lCf[6]=1900;
	   	    if(m_lCf[6]<2000||m_lCf[6]>2400)
			{
			 	m_lCf[6]=2190;
				temp1=3620;
				temp2=3500;
			}
		}
		else if(m_lCf[8]==300)
		{
			if(m_lCf[6]<1300||m_lCf[6]>1800)
			{
			  	m_lCf[6]=1580;
				temp1=3250;
				temp2=2850;
			}
		}
		else if(m_lCf[8]==200)
		{
			if(m_lCf[6]<800||m_lCf[6]>1400)
			{
				m_lCf[6]=1100;
				temp1=2520;
				temp2=2400;
			}
		}
		
		n=(unsigned int)(m_lCf[6]*T/Pressure1);   // 需根据实验结果校正	1084 3800   1110 0000
		Valve(151);
		Valve(160);
		Delay5ms(200);
 		Motorun(0,T,FORWARD,0,      30,60,8,20,  30,6,20,0,     91,151,160);      //抽气
		Delay5ms(300);
		//collin recover20141015	
		Motorun(0,T-n,BACKWARD,0,   30,30,0,0,  30,0,0,0,      90,151,160);
		//Motorun(1,T-n,BACKWARD,0,   30,30,0,0,  30,0,0,0,      90,151,160);
		//end
		Delay5ms(200);				  
		Motorun(1,n,BACKWARD,0,     20,30,1,20,     20,4,50,200,   90,151,161);
		Valve(150);
	//	LeakageCheck(Go);  
	   	Delay5ms(1000);
		Pressure2=ADC_Press(ADC_FS);
	  //  if(Pressure2<3380&&Pressure2>3200)
 	    if(Pressure2<temp1&&Pressure2>temp2)
          { Go=6; }
		  else 
		  {  Go++; }
	}
	m=T-n;  //扣除m个脉冲后反推即可得到0.4MPa气压
	//放气
	Valve(151);
	Valve(160);//排气
	Valve(170);
	Valve(180);
	Delay5ms(200);
	//全部关闭
	Valve(150);
	Delay5ms(20);
	//报告工作状态
	QueryFlag	= 1;
	Inqueue(0x02);
	Inqueue(0xFF);
	Inqueue(0x00);
	Inqueue(0x32);	
 	i= Pressure1+Pressure0;
	//collin add20141014
	i = (u16)(((((float)(i + 28) * 3 / 4096) - 1) / 4)*1000);
	debug1_collin = i&0xff;
	debug2_collin = (i>>8)&0xff;
	 Inqueue(i&0xff);
	 Inqueue((i>>8)&0xff);
	//end
	//Inqueue((u8)(i/256));
	//Inqueue((u8)(i%256));
	Inqueue(Go);	
	for(i=0;i<FrameLength-8;i++)
		Inqueue(0);
	Inqueue(0x03);	
	Delay5ms(10);

	QueryFlag_uart1	= 1;
	Inqueue_uart1(0x02);
	Inqueue_uart1(0xFF);
	Inqueue_uart1(0x00);
	Inqueue_uart1(0x32);	
 	i= Pressure1+Pressure0;
	//collin add20141014
	i = (u16)(((((float)(i + 28) * 3 / 4096) - 1) / 4)*1000);
	debug1_collin = i&0xff;
	debug2_collin = (i>>8)&0xff;
	 Inqueue_uart1(i&0xff);
	 Inqueue_uart1((i>>8)&0xff);
	//end
	//Inqueue((u8)(i/256));
	//Inqueue((u8)(i%256));
	Inqueue_uart1(Go);	
	for(i=0;i<FrameLength-8;i++)
		Inqueue_uart1(0);
	Inqueue_uart1(0x03);	
	Delay5ms(10);

	reTest = 0;
	return m;
}
/*-----------------------------------
初步参比气体测量子程序
 3次参比气洗气过程	 4~8r/s	 脉冲数：3250
 1次测量过程
-------------------------------------*/
void  AirFirst(void)
	{
	Valve(171);
	Valve(181);	   //1270-->1500
	Motorun(0,1500,FORWARD,0,  30,60,8,20,  30,2,20,0,     81,150,160);      //抽气
	Motorun(0,1500,BACKWARD,0, 30,60,8,20,  30,2,20,200,  80,151,170);      //打气,清洗
	Delay1ms(4);
	Valve(150);	 //2540-->3000
	Motorun(0,3000,FORWARD,0,      30,60,8,20,  30,2,20,0,     81,0,171);      //抽气
	Delay1ms(444);
	Valve(80);	
	Valve(151);
	Valve(170);
	Valve(180);
	Delay1ms(52);															
	Motorun(0,3000,BACKWARD,1,     27,27,0,0,  30,0,0,0,  0,0,0); 
	Delay1ms(75);
}


//collin add20141031
/*-----------------------------------
初步参比气体测量子程序
 3次参比气洗气过程	 4~8r/s	 脉冲数：3250
 1次测量过程
-------------------------------------*/
void  AirSecond(void)
{
	u8	i; 
	//for(i=0;i<2;i++)
	Valve(171);
	Valve(181);
	for(i=0;i<4;i++)
	{
		Motorun(0,1250,FORWARD,0,  30,60,8,20,  30,2,20,0,     81,150,160);      //抽气
		Delay5ms(10);
		Motorun(0,1250,BACKWARD,0, 30,60,8,20,  30,2,20,1200,  80,151,160);      //打气,清洗
	}
	Delay5ms(10);
	Motorun(0,3000,FORWARD,0,      30,60,8,20,  30,2,20,0,     81,150,160);      //抽气
	Delay5ms(100);
	Valve(170);
	Valve(180);
	Motorun(0,3000,BACKWARD,1,     30,30,0,0,  30,0,0,0,  80,151,160); 
	Delay5ms(10); 
}
//end
void  	Cartridge(u8 wash)
{
    u8 i;
    u8 OpenValve=0,CloseValve=0;
    switch(wash)
    {
    case 8:
        Valve(171);
        Valve(181);
        OpenValve=81,CloseValve=80;
        break;
    case 9:
        OpenValve=91,CloseValve=90;
        break;
    default:
        break;
    }

    for(i=0; i<2; i++)
    {
        Motorun(1,3250,FORWARD,0,      30,60,8,20,      20,6,20,0, 	 OpenValve, 150, 160);   //抽气 4~8r/s	脉冲数:1500
        Motorun(1,3250,BACKWARD,0,     30,60,8,20,      20,6,20,200,	 CloseValve, 151, 160);  //打气，底气清洗
    }
    Motorun(1,T,FORWARD,0,      30,50,8,20,      20,6,20,0, 	 OpenValve, 150, 160);   //抽气，满行程
    Delay5ms(300);
    Valve(170);
    Valve(180);
    Motorun(1, m,  BACKWARD,0,   30,30,0,0,      30,0,0,0,	CloseValve, 151, 160);  //反推m个脉冲  ,速度4~5r/s
    Delay5ms(200);
    Motorun(1,T-m,BACKWARD,0,   20,30,1,20, 20,5,50,200,	 0, 0, 161);  //打气至底
    Valve(150);
    Delay5ms(1000);         //共延时5s等气压稳定
    //转换发送数据
    AK_Conv();
    Valve(151);
    Valve(160);//排气
    Delay5ms(100);
    Valve(150);
}
/*-----------------------------------
参比气体测量子程序
 3次参比气洗气过程	 4~8r/s	 脉冲数：3250
 1次测量过程
-------------------------------------*/
void  Air(void)
{
	u8 i; 
	Valve(171);
	Valve(181);		//3180-->3500
	Motorun(0,3500,FORWARD,0,  30,65,8,20, 30,5,20,0,    81,150,160);      //抽气
	Motorun(0,3500,BACKWARD,0, 30,65,8,20, 30,5,20,200,  80,151,170);      //打气,清洗
	for(i=0;i<2;i++)
	{
		Motorun(0,3500,FORWARD,0,  30,65,8,20, 30,5,20,0,    81,150,171);      //抽气
		Motorun(0,3500,BACKWARD,0, 30,65,8,20, 30,5,20,200,  80,151,170);      //打气,清洗
		Delay1ms(4);
	}

	Motorun(0,3500,FORWARD,0,      30,65,8,20,  30,5,20,0,   81,150,171);      //抽气
	Delay1ms(514);
	Valve(170);
	Valve(180);
	Motorun(0,3500,BACKWARD,1,     27,27,0,1,  30,0,20,200, 80,151,170);      //打气，慢推1~3r/s，过程中测参比气体吸光度
	Delay1ms(200);	
	Valve(150);
	Delay1ms(23);
}

/*-----------------------------
初步样气测量程序
-----------------------------------*/
void	SampleFirst(u8 Pnum,u16 m)
{
	u8 OpenValve=0,CloseValve=0;
	switch(Pnum)
	{ 	 
		case 1:
			OpenValve=21,CloseValve=20;
			break;
		case 2:
			OpenValve=41,CloseValve=40;
			break;
		case 3:
			OpenValve=61,CloseValve=60;
			break;
		case 4:
			OpenValve=101,CloseValve=100;
			break;
		case 5:
			OpenValve=121,CloseValve=120;
			break;
		case 6:
			OpenValve=141,CloseValve=140;
			break;
		default:
			break;
	}

	Valve(171);	
	Valve(181);		  //1570-->1800
	Motorun(0,1800,FORWARD,0,      30,60,8,20,      30,6,20,0, 	 OpenValve, 150, 160);   //抽气 4~8r/s	脉冲数:1500
	Valve(CloseValve); Valve(170);	
	Delay1ms(22);
	Valve(151);
	Delay1ms(2);
	Motorun(0,1800,BACKWARD,0,     30,60,8,20,      30,6,20,200,	 0, 0, 0);  //打气，底气清洗
	Valve(171);
	Valve(OpenValve);
	Delay1ms(2);
	Valve(150);
	Delay1ms(27);	
	
  	Motorun(0,T,FORWARD,0,      30,60,8,20,      30,6,20,0, 	 0, 0, 0);   //抽气，满行程
	
	Delay1ms(1532);
	
	Valve(CloseValve);
	Delay1ms(25);
	CheckSamplePress();
	Valve(170);
	Valve(151);
	Delay1ms(2);	
	Motorun(0, m,  BACKWARD,0,     30,30,0,0,      30,0,0,0,	 	 0, 0, 0);  //反推m个脉冲  ,速度4~5r/s
	Delay1ms(1024);
	Valve(161);
	Delay1ms(54);
	Motorun(1,T-m,BACKWARD,0,   20,30,1,20, 20,5,50,200	, 0, 0, 0);  //打气至底
	Delay1ms(480);
	Valve(150);
	Delay1ms(664);         
    //转换发送数据
	AK_Conv(); Valve(151);
	Valve(160);//排气  
	Delay1ms(183);	
}
void	BaseFirst(u8 Pnum,u16 m)
{
	u8 OpenValve=0,CloseValve=0;
	switch(Pnum)
   	{ 
   		case 1:
    		OpenValve=11,CloseValve=10;
      		break;
   		case 2:
     		OpenValve=31,CloseValve=30;
      		break;
   		case 3:
     		OpenValve=51,CloseValve=50;
    		break;
   		case 4:
      		OpenValve=71,CloseValve=70;
      		break;
		case 5:
			OpenValve=111,CloseValve=110;
			break;
		case 6:
			OpenValve=131,CloseValve=130;
			break;
   		default:
      		break;
	}

	Valve(171);	
	Valve(181);	//1570-->1800
	Motorun(0,1800,FORWARD,0,      30,60,8,20,      30,6,20,0, 	 OpenValve, 150, 160);   //抽气 4~8r/s	脉冲数:1500
	Valve(CloseValve);
	Valve(170);
	Delay1ms(22);
	Valve(151);
	Delay1ms(2);
	Motorun(0,1800,BACKWARD,0,     30,60,8,20,      30,6,20,200,	 0, 0, 0);  //打气，底气清洗
	Valve(171);
	Valve(OpenValve);
	Delay1ms(2);
	Valve(150);
	Delay1ms(27);	
	
  	Motorun(0,T,FORWARD,0,      30,60,8,20,      30,6,20,0, 	 0, 0, 0);   //抽气，满行程
	
	Delay1ms(1532);
	
	Valve(CloseValve);
	Delay1ms(25);
	CheckSamplePress();
	Valve(170);
	Valve(151);
	Delay1ms(2);	
	Motorun(0, m,  BACKWARD,0,     30,30,0,0,      30,0,0,0,	 	 0, 0, 0);  //反推m个脉冲  ,速度4~5r/s
	Delay1ms(1024);
	Valve(161);
	Delay1ms(54);
	Motorun(1,T-m,BACKWARD,0,   20,30,1,20, 20,5,50,200	, 0, 0, 0);  //打气至底
	Delay1ms(480);
	Valve(150);

	Delay1ms(1362);         
    //转换发送数据
	AK_Conv(); Valve(151);
	Valve(160);//排气  
	Delay1ms(43);
}
/*-----------------------------------
底气测量子程序
1：测量第一路通道底气
2：测量第二路通道底气
3：测量第三路通道底气
4：测量第四路通道底气
单个通道底气洗气一次
一次测量
Pnum 通道数  n 目标压力的脉冲数
----------------------------------*/
void Baseline(u8 Pnum,u16 pulse,u16 percent)
{
    u8 	OpenValve=21,CloseValve=20;
    u8	TempValveOpen,TempValveClose;
    u16	i;

    /*	switch(Pnum)
    	{
    		case 11:
    			OpenValve=11,CloseValve=10;
    			break;
    		case 12:
    			OpenValve=31,CloseValve=30;
    			break;
    		case 13:
    			OpenValve=51,CloseValve=50;
    			break;
    		case 14:
    			OpenValve=71,CloseValve=70;
    			break;
    		case 15:
    			OpenValve=111,CloseValve=110;
    			break;
    		case 16:
    			OpenValve=131,CloseValve=130;
    			break;
    		case 21:
    			OpenValve=21,CloseValve=20;
    			break;
    		case 22:
    			OpenValve=41,CloseValve=40;
    			break;
    		case 23:
    			OpenValve=61,CloseValve=60;
    			break;
    		case 24:
    			OpenValve=101,CloseValve=100;
    			break;
    		case 25:
    			OpenValve=121,CloseValve=120;
    			break;
    		case 26:
    			OpenValve=141,CloseValve=140;
    			break;
    		default:
    			break;
    	}
    	 */
    TempValveOpen	= 91;
    TempValveClose	= 90;	 //空气

    i	= 175*percent/10;
    Motorun(1,    i,FORWARD,0, 20,65,8,20,  20,6,20,0,  OpenValve, 150, 161);   //抽气 4~8r/s	脉冲数:1500
    Delay5ms(100);
    Valve(CloseValve);
    Motorun(1,1750-i,FORWARD,0,20,65,8,20, 20,6,20,0, 	 TempValveOpen, 150, 161);   //抽气 4~8r/s	脉冲数:1500
    Delay5ms(100);
    Motorun(1,1750,BACKWARD,0, 20,65,8,20, 20,6,20,200, TempValveClose, 151, 160);  //打气，底气清洗
    Delay5ms(100);
    i	= 50*percent;
    Motorun(1,     i,FORWARD,0,20,65,8,20,  20,6,20,0, OpenValve, 150, 161);   //抽气，
    Delay5ms(200);
    Valve(CloseValve);
    Motorun(1,T-i,FORWARD,0,20,65,8,20, 20,6,20,0, 	 TempValveOpen, 150, 161);   //抽气 4~8r/s	脉冲数:1500
    Delay5ms(200);
    Motorun(1,T,BACKWARD,0,   20,30,1,20, 20,1,20,200, TempValveClose, 151, 161);  //打气
    Motorun(1,T,FORWARD,0, 	 20,65,8,20, 20,6,20,0, 	 0, 0, 0);   //抽气
    Motorun(1,T,BACKWARD,0,   20,30,1,20, 20,1,20,200,	 0, 0, 0);  //打气至底
    Motorun(1,T,FORWARD,0, 	 20,65,8,20, 20,6,20,0, 	 0, 0, 0);   //抽气

    Delay5ms(300);
    Motorun(1, pulse,  BACKWARD,0,     40,50,2,20,  30,1,20,0,	 CloseValve, 151, 160);  //反推m个脉冲  ,速度4~5r/s
    Delay5ms(200);
    Motorun(1,T-pulse,BACKWARD,0,   30,30,0,20,  30,0,50,200,0, 0, 161);  //打气至底
    Delay5ms(100);
    Valve(150);
    Delay5ms(1000);         //共延时5s等气压稳定
    //转换发送数据
    AK_Conv();
    Valve(160);//排气
    Delay5ms(100);
    Valve(151);
}
void CheckSamplePress(void)
{
//	u8 i;
//	Pressure2 =ADC_Press(ADC_FS);
//	if(Pressure2<1000)
//	{
//		//报告工作状态
//		QueryFlag	= 1;
//		//加入帧头
//		Inqueue(0x02);
//		//加入命令标志
//		Inqueue(0xFF);
//		Inqueue(0x00);
//		Inqueue(0x26);
//		for(i=0;i<FrameLength-5;i++)
//			Inqueue(0);
//		Inqueue(0x03);
//		sampleTooless =	1;
    //LightModulation(OFF);
//	}
}
/*-----------------------------------
样气测量子程序
1：测量第一路通道底气
2：测量第二路通道底气
3：测量第三路通道底气
4：测量第四路通道底气

单个通道样气洗气一次
一次测量
----------------------------------*/
void Sample(u8 Pnum,u16 pulse,double percent,u8 t)
{
	u8 	OpenValve=0,CloseValve=0;
	u8	TempValveOpen	= 91;
	u8	TempValveClose	= 90;	
	u16	i;

	switch(Pnum)
	{
		case 11:
			OpenValve=11,CloseValve=10;
			break;
		case 12:
			OpenValve=31,CloseValve=30;
			break;
		case 13:
			OpenValve=51,CloseValve=50;
			break;
		case 14:
			OpenValve=71,CloseValve=70;
			break;
		case 15:
			OpenValve=111,CloseValve=110;
			break;	
		case 16:
			OpenValve=131,CloseValve=130;
			break;								 
		case 21:
			OpenValve=21,CloseValve=20;
			break;
		case 22:
			OpenValve=41,CloseValve=40;
			break;
		case 23:
			OpenValve=61,CloseValve=60;
			break;
		case 24:
			OpenValve=101,CloseValve=100;
			break;
		case 25:
			OpenValve=121,CloseValve=120;
			break;
		case 26:
			OpenValve=141,CloseValve=140;
			break;
		default:
			break;
	}
	if(percent>0.95)
		t=0;
	Valve(171);
	Valve(181);
	if(t==0)
	{	
	    Motorun(0,2875,FORWARD,0,  30,60,8,20,  30,6,20,0, OpenValve, 150, 160);   //抽气 4~8r/s	脉冲数:1500
    	Delay5ms(300);	 //200
		Motorun(0,2875,BACKWARD,0, 30,60,8,20,  30,6,20,200,CloseValve, 151, 160);  //打气，底气清洗 
		Delay5ms(10);
  		Motorun(0,T,FORWARD,0,  30,60,8,20,  30,6,20,0,  OpenValve, 151, 161);   //抽气，满行程
		Delay5ms(300);
	}
	else
	{				
		i	= (u16)(2875*percent);
		Motorun(0,    i,FORWARD,0, 30,60,4,20,  30,4,20,0,  OpenValve, 150, 160);   //抽气 4~8r/s	脉冲数:1500
		Delay5ms(300);
		Valve(CloseValve);
		Motorun(0,2875-i,FORWARD,0,30,30,0,0, 30,0,0,0, 	 TempValveOpen, 150, 160);   //抽气 4~8r/s	脉冲数:1500
		Delay5ms(300);
		Motorun(0,2875,BACKWARD,0, 30,30,8,20, 30,6,20,200, TempValveClose, 151, 160);  //打气，底气清洗
		Delay5ms(10);

		i	= (u16)(T*percent);
  		Motorun(0, i,FORWARD,0,30,60,4,20,  30,4,20,0, OpenValve, 150, 160);   //抽气，
		Delay5ms(300);
		Valve(CloseValve);
		Motorun(0,T-i,FORWARD,0,30,30,0,0, 30,0,0,0, 	 TempValveOpen, 150, 160);   //抽气 4~8r/s	脉冲数:1500
		Delay5ms(300);
		Valve(170);
		Motorun(1,T,BACKWARD,0,  20,30,1,20, 20,5,50,200, TempValveClose, 151, 161);  //打气
		Motorun(0,T,FORWARD,0, 	 30,60,8,20, 30,6,20,0, 	 0, 0, 0);   //抽气 		
		Motorun(1,T,BACKWARD,0,  20,30,1,20, 20,5,50,200, 0, 0, 0);  //打气
		Delay5ms(400);
		Motorun(0,T,FORWARD,0, 	 30,60,8,20, 30,6,20,0, 	 0, 0, 0);   //抽气 
	}
	Valve(CloseValve);
	CheckSamplePress();
	Valve(170);
	Motorun(0, pulse,  BACKWARD,0,     30,30,0,0,  30,0,0,0,	 CloseValve, 151, 160);  //反推m个脉冲  ,速度4~5r/s
	Delay5ms(300);	   //200
	Motorun(1,T-pulse,BACKWARD,0,   20,30,1,20,  20,5,50,200,0, 0, 161);  //打气至底

	Delay1ms(480);
	Valve(150);
//	if(Pnum/10==2)
//		Delay1ms(5700);         //共延时5s等气压稳定
//	else
		Delay1ms(5000);
    //转换发送数据
	AK_Conv();
	Valve(151);
	Valve(160);//排气  
	Delay5ms(200);
}

void Motor_Init(void)
{
    u8 i;
    u8 Trip2;
    QueryFlag	= 1;
    //加入帧头
    Inqueue(0x02);
    //加入命令标志
    Inqueue(0xFF);
    Inqueue(0x00);
    Inqueue(0x1D);
    for(i=0; i<FrameLength-5; i++)
        Inqueue(0);
    Inqueue(0x03);
    Delay5ms(40);

	QueryFlag_uart1	= 1;
    //加入帧头
    Inqueue_uart1(0x02);
    //加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0x1D);
    for(i=0; i<FrameLength-5; i++)
        Inqueue_uart1(0);
    Inqueue_uart1(0x03);
    Delay5ms(40);

    Trip2= IsTripSwitchClosed() ;
    sampleTooless =0;
    if(Trip2)
    {
        Motorun(  0,800,FORWARD,0,      20,50,5,20,      30,3,20,0,        91,150,161);

        Trip2= IsTripSwitchClosed() ;
        if(Trip2)
        {
            /*显示光电开关出错*/
            Sys_Status=Sys_Error;
        }
        else
        {
            Sys_Status=Sys_SelfCheck;
            Motorun(  0,800,BACKWARD,0,     20,50,5,20,      30,3,20,200,      90,151,160);
        }
    }
    else
    {
        TripSwitchStatus=0;
        Motorun(  0,6000,BACKWARD,0,     20,30,0,0,      30,0,0,200,      90,151,160);
        if(!TripSwitchStatus)
        {
            /*显示光电开关出错*/
            Sys_Status=Sys_Error;
        }
    }

    QueryFlag	= 1;
    //加入帧头
    Inqueue(0x02);
    //加入命令标志
    Inqueue(0xFF);
    Inqueue(0x00);
    if(Sys_Status==Sys_Error)
        Inqueue(0x1E);	//出错标志
    else
        Inqueue(0x1F);	//正常标志
    for(i=0; i<FrameLength-5; i++)
        Inqueue(0);
    Inqueue(0x03);
    Delay5ms(10);

	QueryFlag_uart1	= 1;
    //加入帧头
    Inqueue_uart1(0x02);
    //加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    if(Sys_Status==Sys_Error)
        Inqueue_uart1(0x1E);	//出错标志
    else
        Inqueue_uart1(0x1F);	//正常标志
    for(i=0; i<FrameLength-5; i++)
        Inqueue_uart1(0);
    Inqueue_uart1(0x03);
    Delay5ms(10);
}
void SamplePosition()
{
	unsigned char sended=0;
	u8	positionChanged	= 0;
	if(SamplePosition1!=GPIO_ReadInputDataBit(GPIO_SamplePosition,SamplePosition_Pin_1))
	{
		SamplePosition1=!SamplePosition1;
		if(checkChannel==1&&SamplePosition1)
		{
			sended=1;
			m_bMeasureBreak=1;
		}
		positionChanged+=1;
		ch_checked[0]=0;
		//collin modify20141027
		/*
		if(!SamplePosition1&&checkChannel!=1)
			LED(1,1);
		else
			LED(1,0);
		*/
		if(!SamplePosition1&&checkChannel!=1)
		{
			LED(1,1);
			//LED(2,1);
		}			
		else
		{
			LED(1,0);
			//LED(2,0);
		}
			
		//end

	}
	//collin mask20141027
	
	if(SamplePosition2!=GPIO_ReadInputDataBit(GPIO_SamplePosition,SamplePosition_Pin_2))
	{
		SamplePosition2=!SamplePosition2;
		if(checkChannel==2&&SamplePosition2)
		{
			sended=1;
			m_bMeasureBreak=1;
		}
		positionChanged+=0x02;
		ch_checked[1]=0;
		if(!SamplePosition2&&checkChannel!=2)
			LED(2,1);
		else
			LED(2,0);
	}
	
	//end

	if(m_bMeasureBreak==1)
	{ 
		u8 i=0;
		u8 buf[24];
		if(checkChannel)flag_err[checkChannel-1]=1;
		if(m_mesureType==4)ch_selected=0;
		if(sended)
		{
			sended=0;
			LED(1,0);
			LED(2,0);
			Inqueue(0x02);
			//加入命令标志
			Inqueue(0xFF);
			Inqueue(0x00);
			Inqueue(0x27);
			Inqueue(checkChannel);					
			for(i=0;i<FrameLength-6;i++)
				Inqueue(0);
			Inqueue(0x03);
			checkChannel=0;	

			Inqueue_uart1(0x02);
			//加入命令标志
			Inqueue_uart1(0xFF);
			Inqueue_uart1(0x00);
			Inqueue_uart1(0x27);
			Inqueue_uart1(checkChannel);					
			for(i=0;i<FrameLength-6;i++)
				Inqueue_uart1(0);
			Inqueue_uart1(0x03);

		}	
	}
	if(positionChanged)
	{
		u8 i=0;
		u8 buf[24];
		buf[0]=0x02;
		buf[1]=0xFF;
		buf[2]=0x00;
		buf[3]=0x07;
			buf[4]=0;
		if(!SamplePosition1)
			buf[4]=0x01;
		if(!SamplePosition2)
			buf[4]+=0x02;
		if(!SamplePosition3)
			buf[4]+=0x04;
		if(!SamplePosition4)
			buf[4]+=0x08;
		if(!SamplePosition5)
			buf[4]+=0x10;
		if(!SamplePosition6)
			buf[4]+=0x20;  	
		buf[5]=positionChanged;
		for(i=6;i<23;i++)
			buf[i]=0;
		buf[23]=0x03;
		/*	
		for(i=0;i<24;i++)
			Inqueue(buf[i]);
			*/
	}
}
u8 posis;
void SendPosition(short positionChanged)
{
		u8 i=0;
		u8 buf[24];
		posis=0;
		buf[0]=0x02;
		buf[1]=0xFF;
		buf[2]=0x00;
		buf[3]=0x07;
		if(!SamplePosition1)
			posis+=0x01;
		if(!SamplePosition2)
			posis+=0x02;
		if(!SamplePosition3)
			posis+=0x04;
		if(!SamplePosition4)
			posis+=0x08;
		if(!SamplePosition5)
			posis+=0x10;
		if(!SamplePosition6)
			posis+=0x20;  	
		buf[4]=posis;
		buf[5]=positionChanged;
		for(i=6;i<23;i++)
			buf[i]=0;
		buf[23]=0x03;	

		for(i=0;i<24;i++)
			Inqueue(buf[i]);
}
u8 IsTripSwitchClosed(void)
{
	return GPIO_ReadInputDataBit(GPIO_TripSwitch,GPIO_Pin_TripSwitch);
}

void EXTI_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_TripSwitch;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIO_TripSwitch, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PORT_SOURCE_TripSwitch, GPIO_PIN_SOURCE_TripSwitch);

	EXTI_InitStructure.EXTI_Line = EXTI_LINE_TripSwitch;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}
void SendDataTemperature(void)
{
	u8	i;
	//当未有TXE开启时关闭串口查询
//	while(USART1->CR1&0x0080);
	QueryFlag	= 1;
	Inqueue(0x02);
	Inqueue(0xFF);
	Inqueue(0x00);
	Inqueue(0x1A);
	Inqueue((Temperature[0]>>8)&0xFF);
	Inqueue(Temperature[0]&0xFF);
	Inqueue((Temperature[1]>>8)&0xFF);
	Inqueue(Temperature[1]&0xFF);
	Inqueue((Temperature[2]>>8)&0xFF);
	Inqueue(Temperature[2]&0xFF);
	Inqueue((Temperature[3]>>8)&0xFF);
	Inqueue(Temperature[3]&0xFF);
	Inqueue((Temperature[4]>>8)&0xFF);
	Inqueue(Temperature[4]&0xFF);
	Inqueue((Temperature[5]>>8)&0xFF);
	Inqueue(Temperature[5]&0xFF);
	for(i=0;i<FrameLength-17;i++)
		Inqueue(0);
	Inqueue(0x03);
	
		QueryFlag_uart1	= 1;
	Inqueue_uart1(0x02);
	Inqueue_uart1(0xFF);
	Inqueue_uart1(0x00);
	Inqueue_uart1(0x1A);
	Inqueue_uart1((Temperature[0]>>8)&0xFF);
	Inqueue_uart1(Temperature[0]&0xFF);
	Inqueue_uart1((Temperature[1]>>8)&0xFF);
	Inqueue_uart1(Temperature[1]&0xFF);
	Inqueue_uart1((Temperature[2]>>8)&0xFF);
	Inqueue_uart1(Temperature[2]&0xFF);
	Inqueue_uart1((Temperature[3]>>8)&0xFF);
	Inqueue_uart1(Temperature[3]&0xFF);
	Inqueue_uart1((Temperature[4]>>8)&0xFF);
	Inqueue_uart1(Temperature[4]&0xFF);
	Inqueue_uart1((Temperature[5]>>8)&0xFF);
	Inqueue_uart1(Temperature[5]&0xFF);
	for(i=0;i<FrameLength-17;i++)
		Inqueue_uart1(0);
	Inqueue_uart1(0x03);		 
}

void ReadS_GPIO(void) //发送GPIO口状态
{  
	u16 i,j;
   	for(i=0,j=1;i<=15;i++)
	{		
		j=2^i; 
		G[i]=GPIO_ReadInputDataBit(GPIOA, j);	
	}
	Inqueue_uart1(0x02);//加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0xA1);
	for(i=0;i<16;i++)
	{	
		Inqueue_uart1(G[i]);				
	}
	Inqueue_uart1(0);
	Inqueue_uart1(0);		
	Inqueue_uart1(0);
    Inqueue_uart1(0x03);
	Delay5ms(20);
	for(i=0,j=1;i<=15;i++)
	{		
		j=2^i;
		G[i]=GPIO_ReadInputDataBit(GPIOB, j);	
	}
	Inqueue_uart1(0x02);//加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0xA2);
	for(i=0;i<16;i++)
	{	
		Inqueue_uart1(G[i]);				
	}
	Inqueue_uart1(0);
	Inqueue_uart1(0);		
	Inqueue_uart1(0);
    Inqueue_uart1(0x03);
	Delay5ms(20);
	for(i=0,j=1;i<=15;i++)
	{		
		j=2^i;
		G[i]=GPIO_ReadInputDataBit(GPIOC, j);	
	}
	Inqueue_uart1(0x02);//加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0xA3);
	for(i=0;i<16;i++)
	{	
		Inqueue_uart1(G[i]);				
	}
	Inqueue_uart1(0);
	Inqueue_uart1(0);		
	Inqueue_uart1(0);
    Inqueue_uart1(0x03);
	Delay5ms(20);
	for(i=0,j=1;i<=15;i++)
	{		
		j=2^i;
		G[i]=GPIO_ReadInputDataBit(GPIOD, j);	
	}
	Inqueue_uart1(0x02);//加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0xA4);
	for(i=0;i<16;i++)
	{	
		Inqueue_uart1(G[i]);				
	}
	Inqueue_uart1(0);
	Inqueue_uart1(0);		
	Inqueue_uart1(0);
    Inqueue_uart1(0x03);
	Delay5ms(20);
	for(i=0,j=1;i<=15;i++)
	{		
		j=2^i;
		G[i]=GPIO_ReadInputDataBit(GPIOE, j);	
	}
	Inqueue_uart1(0x02);//加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0xA5);
	for(i=0;i<16;i++)
	{	
		Inqueue_uart1(G[i]);				
	}
	Inqueue_uart1(0);
	Inqueue_uart1(0);		
	Inqueue_uart1(0);
    Inqueue_uart1(0x03);
	Delay5ms(20);
	for(i=0,j=1;i<=15;i++)
	{		
		j=2^i;
		G[i]=GPIO_ReadInputDataBit(GPIOF, j);	
	}
	Inqueue_uart1(0x02);//加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0xA6);
	for(i=0;i<16;i++)
	{	
		Inqueue_uart1(G[i]);				
	}
	Inqueue_uart1(0);
	Inqueue_uart1(0);		
	Inqueue_uart1(0);
    Inqueue_uart1(0x03);
	Delay5ms(20);
}
void ReadS_V(void)		//发送气阀状态
{	
	u8 i;
	V[0]=GPIO_ReadInputDataBit(GPIO_Valve1, GPIO_Pin_Valve1);
	V[1]=GPIO_ReadInputDataBit(GPIO_Valve2_3,  GPIO_Pin_Valve2);
	V[2]=GPIO_ReadInputDataBit(GPIO_Valve2_3,  GPIO_Pin_Valve3);
	V[3]=GPIO_ReadInputDataBit(GPIO_Valve4_12, GPIO_Pin_Valve4);
	V[4]=GPIO_ReadInputDataBit(GPIO_Valve4_12, GPIO_Pin_Valve5);
	V[5]=GPIO_ReadInputDataBit(GPIO_Valve4_12, GPIO_Pin_Valve6);
	V[6]=GPIO_ReadInputDataBit(GPIO_Valve4_12, GPIO_Pin_Valve7);
	V[7]=GPIO_ReadInputDataBit(GPIO_Valve4_12, GPIO_Pin_Valve8);
	V[8]=GPIO_ReadInputDataBit(GPIO_Valve4_12, GPIO_Pin_Valve9);
	V[9]=GPIO_ReadInputDataBit(GPIO_Valve4_12, GPIO_Pin_Valve10);
	V[10]=GPIO_ReadInputDataBit(GPIO_Valve4_12, GPIO_Pin_Valve11);
	V[11]=GPIO_ReadInputDataBit(GPIO_Valve4_12, GPIO_Pin_Valve12);
	V[12]=GPIO_ReadInputDataBit(GPIO_Valve13_18,GPIO_Pin_Valve13);
	V[13]=GPIO_ReadInputDataBit(GPIO_Valve13_18,GPIO_Pin_Valve14);
	V[14]=GPIO_ReadInputDataBit(GPIO_Valve13_18,GPIO_Pin_Valve15);
	V[15]=GPIO_ReadInputDataBit(GPIO_Valve13_18,GPIO_Pin_Valve16);
	V[16]=GPIO_ReadInputDataBit(GPIO_Valve13_18,GPIO_Pin_Valve17);
	V[17]=GPIO_ReadInputDataBit(GPIO_M1EN, GPIO_Pin_M1EN);
	V[18]=GPIO_ReadInputDataBit(GPIO_M2EN, GPIO_Pin_M2EN);
	Inqueue_uart1(0x02);//加入命令标志
    Inqueue_uart1(0xFF);
    Inqueue_uart1(0x00);
    Inqueue_uart1(0xA7);
	for(i=0;i<19;i++)
	{
		Inqueue_uart1(V[i]);
	}
    Inqueue_uart1(0x03);
    Delay5ms(20);
}

void Speaker_Config(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SPEAKER;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIO_SPEAKER, &GPIO_InitStructure);
	GPIO_WriteBit(GPIO_SPEAKER, GPIO_Pin_SPEAKER, Bit_RESET);
}
void Speaker(u8 Num)
{
	while(Num-->0)
	{
		GPIO_WriteBit(GPIO_SPEAKER, GPIO_Pin_SPEAKER, Bit_SET);
		//500ms
		Delay5ms(100);
		GPIO_WriteBit(GPIO_SPEAKER, GPIO_Pin_SPEAKER, Bit_RESET);
		Delay5ms(100);
	}
}
void Delay1ms(u32 nTime)
{
  	/* Enable the SysTick Counter 允许SysTick计数器*/
  	SysTick_CounterCmd(SysTick_Counter_Enable);
  
  	TimingDelay = nTime;

 	while(TimingDelay != 0)
    	;  //等待计数至0

  /* Disable the SysTick Counter 禁止SysTick计数器*/
 // SysTick_CounterCmd(SysTick_Counter_Disable);
  /* Clear the SysTick Counter 清零SysTick计数器*/
	SysTick_CounterCmd(SysTick_Counter_Clear);
}
void Valve_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Valve1 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIO_Valve1, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Valve2;
	GPIO_Init(GPIO_Valve2_3, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Valve8 | GPIO_Pin_Valve9;
	GPIO_Init(GPIO_Valve4_12, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_Valve14 |GPIO_Pin_Valve15 |GPIO_Pin_Valve16|GPIO_Pin_Valve17 |GPIO_Pin_Valve18;;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
	GPIO_Init(GPIO_Valve13_18, &GPIO_InitStructure);

	//所有阀门低电平
	GPIO_WriteBit(GPIO_Valve1, GPIO_Pin_Valve1,  Bit_RESET);
	GPIO_WriteBit(GPIO_Valve2_3,  GPIO_Pin_Valve2,  Bit_RESET);
	GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve8,  Bit_RESET);
	GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve9,  Bit_RESET);
	GPIO_WriteBit(GPIO_Valve13_18,GPIO_Pin_Valve14, Bit_RESET);
	GPIO_WriteBit(GPIO_Valve13_18,GPIO_Pin_Valve15, Bit_RESET);
	GPIO_WriteBit(GPIO_Valve13_18,GPIO_Pin_Valve16, Bit_RESET);
	GPIO_WriteBit(GPIO_Valve13_18,GPIO_Pin_Valve17, Bit_RESET);
	GPIO_WriteBit(GPIO_Valve13_18,GPIO_Pin_Valve18, Bit_RESET);
}

void Valve(unsigned char Vnum)
{
	switch(Vnum)
	{
	case 0:		
		break;
	case 10:
		GPIO_WriteBit(GPIO_Valve1, GPIO_Pin_Valve1, Bit_RESET);
		break;
	case 11:
		GPIO_WriteBit(GPIO_Valve1, GPIO_Pin_Valve1, Bit_SET);
		break;
	case 20:
		GPIO_WriteBit(GPIO_Valve2_3, GPIO_Pin_Valve2, Bit_RESET);
		break;
	case 21:
		GPIO_WriteBit(GPIO_Valve2_3, GPIO_Pin_Valve2, Bit_SET);
		break;
	case 30:
		GPIO_WriteBit(GPIO_Valve2_3, GPIO_Pin_Valve3, Bit_RESET);
		break;
	case 31:
		GPIO_WriteBit(GPIO_Valve2_3, GPIO_Pin_Valve3, Bit_SET);
		break;
	case 40:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve4, Bit_RESET);
		break;
	case 41:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve4,  Bit_SET);
		break;
	case 50:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve5, Bit_RESET);
		break;
	case 51:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve5,  Bit_SET);
		break;
	case 60:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve6, Bit_RESET);
		break;
	case 61:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve6, Bit_SET);
		break;
	case 70:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve7, Bit_RESET);
		break;
	case 71:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve7, Bit_SET);
		break;
	case 80:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve8, Bit_RESET);
		break;
	case 81:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve8, Bit_SET);
		break;
	case 90:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve9, Bit_RESET);
		break;
	case 91:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve9, Bit_SET);
		break;
	case 100:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve10, Bit_RESET);
		break;
	case 101:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve10, Bit_SET);
		break;
	case 110:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve11, Bit_RESET);
		break;
	case 111:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve11, Bit_SET);
		break;
	case 120:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve12,  Bit_RESET);
		break;
	case 121:
		GPIO_WriteBit(GPIO_Valve4_12, GPIO_Pin_Valve12,  Bit_SET);
		break;
	case 130:
		GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve13,  Bit_RESET);
		break;
	case 131:
		GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve13,  Bit_SET);
		break;
	case 140:
		GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve14,  Bit_RESET);
		break;
	case 141:
		GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve14,  Bit_SET);
		break;
	case 150:
		GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve15,  Bit_RESET);
		break;
	case 151:
		GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve15,  Bit_SET);
		break;
	case 160:
		GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve16,  Bit_RESET);
		break;
	case 161:
		GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve16,  Bit_SET);
		break;
	case 170:
	//collin modify20141114
		GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve17,  Bit_RESET);
		//GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve17,  Bit_SET);
			//end
		break;
	case 171:
		GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve17,  Bit_SET);
		break;
	case 180:
		GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve18,  Bit_RESET);
		break;
	case 181:
		GPIO_WriteBit(GPIO_Valve13_18, GPIO_Pin_Valve18,  Bit_SET);
		break;
	default:
		break;
	}
}

void SendData(void)
{
//collin modify20141029 usart3-->usart1
	//当未有TXE开启时关闭串口查询
	while(USART1->CR1&0x0080);
	QueryFlag_uart1	= 0;
	SendLength_uart1	= FrameLength;
	SendCounter_uart1	= 0;								   
	//启动串口发送中断
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	while(USART3->CR1&0x0080);
	QueryFlag	= 0;
	SendLength	= FrameLength;
	SendCounter	= 0;
	//启动串口发送中断
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	//end	
}

void Sys_100msClock_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	//周期100ms
	TIM_TimeBaseStructure.TIM_Period	= 6143; 
	TIM_TimeBaseStructure.TIM_Prescaler = 999;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
	//计数器上溢中断设定
	TIM_ClearFlag(TIMx, TIM_FLAG_Update);/*清除更新标志位*/
	TIM_ARRPreloadConfig(TIMx, DISABLE);/*预装载寄存器的内容被立即传送到影子寄存器 */
	//启动定时器
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);   
	TIM_Cmd(TIMx, ENABLE);
}

//collin modify20141028
void Uart1_Config()
{
	USART_InitTypeDef USART_InitStructure;	
	GPIO_InitTypeDef GPIO_InitStructure;
		/* Configure USARTx_Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USARTx_Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_BaudRate = 256000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* Configure the USART1 */ 
	USART_Init(USART1, &USART_InitStructure);
	/* Enable the USART1 */
	USART_Cmd(USART1, ENABLE);
	//启动串口接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);		
}

void Uart3_Config()
{
	USART_InitTypeDef USART_InitStructure;	
	GPIO_InitTypeDef GPIO_InitStructure;
		/* Configure USARTx_Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure USARTx_Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	USART_InitStructure.USART_BaudRate = 9600;
	//USART_InitStructure.USART_BaudRate = 256000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* Configure the USART3 */ 
	USART_Init(USART3, &USART_InitStructure);
	/* Enable the USART3 */
	USART_Cmd(USART3, ENABLE);
	//启动串口接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);		
}
//end

void Inqueue(unsigned char byte)
{
	if(PHead-1==PEnd || PHead+BufferSize-1==PEnd)
		return;//循环队列已满

	SendBuffer[PEnd++]=byte;
	if(PEnd>=BufferSize) 
		PEnd-=BufferSize;
}

void Inqueue_uart1(unsigned char byte)
{
	if(PHead_uart1-1==PEnd_uart1 || PHead_uart1+BufferSize-1==PEnd_uart1)
		return;//循环队列已满

	SendBuffer_uart1[PEnd_uart1++]=byte;
	if(PEnd_uart1>=BufferSize) 
		PEnd_uart1-=BufferSize;
}

unsigned char Exqueue(void)
{
	unsigned char ReturnChar;
	if(PEnd==PHead)	//如果空队列
		return 0;
	ReturnChar=SendBuffer[PHead++];
	if(PHead>=BufferSize) PHead-=BufferSize;
	return ReturnChar;
}

unsigned char Exqueue_uart1(void)
{
	unsigned char ReturnChar_uart1;
	if(PEnd_uart1==PHead_uart1)	//如果空队列
		return 0;
	ReturnChar_uart1=SendBuffer_uart1[PHead_uart1++];
	if(PHead_uart1>=BufferSize) PHead_uart1-=BufferSize;
	return ReturnChar_uart1;
} 

unsigned char QLength(void)
{
	if((int)(PEnd-PHead)>=0)
		return PEnd-PHead;
	else
		return PEnd-PHead+BufferSize;
}

unsigned char QLength_uart1(void)
{
	if((int)(PEnd_uart1-PHead_uart1)>=0)
		return PEnd_uart1-PHead_uart1;
	else
		return PEnd_uart1-PHead_uart1+BufferSize;
}		

void Delay5ms_Config(void)
{
	/* Disable SysTick Counter */
	SysTick_CounterCmd(SysTick_Counter_Disable);        //失能计数器 

	/* Disable the SysTick Interrupt */
	SysTick_ITConfig(DISABLE);   //关闭中断

	/* Configure HCLK clock as SysTick clock source */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);   //8分频
	/* SysTick interrupt each 1000 Hz with HCLK equal to 61.44MHz */
	SysTick_SetReload(3840);   //周期10ms
	/* Enable the SysTick Interrupt */
	SysTick_ITConfig(ENABLE);  //打开中断
}

void Delay5ms(u32 nTime)
{
	/* Enable the SysTick Counter 允许SysTick计数器*/
	SysTick_CounterCmd(SysTick_Counter_Enable);

	TimingDelay = 5*nTime;

	while(TimingDelay != 0)
		;  //等待计数至0

	/* Disable the SysTick Counter 禁止SysTick计数器*/
	// SysTick_CounterCmd(SysTick_Counter_Disable);
	/* Clear the SysTick Counter 清零SysTick计数器*/
	SysTick_CounterCmd(SysTick_Counter_Clear);
}
void LED(u8 Num, u8 Status)
{
	Status=!Status;
	switch(Num)
	{
	case 1:
		GPIO_WriteBit(GPIO_LED1, LED_Pin_1, (BitAction)(Status));
		break;
	case 2:
		GPIO_WriteBit(GPIO_LED, LED_Pin_2, (BitAction)(Status));
		break;
	case 3:
		GPIO_WriteBit(GPIO_LED, LED_Pin_3, (BitAction)(Status));
		break;
	case 4:
		GPIO_WriteBit(GPIO_LED, LED_Pin_4, (BitAction)(Status));
		break;
	case 5:
		GPIO_WriteBit(GPIO_LED, LED_Pin_5, (BitAction)(Status));
		break;
	case 6:
		GPIO_WriteBit(GPIO_LED, LED_Pin_6, (BitAction)(Status));
		break;
	default:
		break;
	}
}


void GPIO_Configuration(void)
{		  	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = ERROR_Pin ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIO_ERROR, &GPIO_InitStructure);
	GPIO_WriteBit(GPIO_ERROR, ERROR_Pin, Bit_SET);

	GPIO_InitStructure.GPIO_Pin = LED_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIO_LED, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIO_LED1, &GPIO_InitStructure);
	/*LCD控制口*/
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	/*LCD数据口*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
   	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_10|GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}

void RCC_Configuration(void)
{
	/* RCC system reset(for debug purpose) */
	ErrorStatus HSEStartUpStatus;
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK/2 */
		RCC_PCLK2Config(RCC_HCLK_Div2);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* PLLCLK = 12.288MHz * 5 = 61.44 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_5);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x08)
		{}
	}

	/* Enable ALL peripheral clocks --------------------------------------------------*/
	//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_ALL, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1|RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3|RCC_APB1Periph_SPI2|RCC_APB1Periph_USART3, ENABLE);
	//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALL,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD
		|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOG|RCC_APB2Periph_TIM1|RCC_APB2Periph_ADC1
		|RCC_APB2Periph_USART1,ENABLE);
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
	/* Set the Vector Table base location at 0x20000000 */
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
#endif

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* SPI2 IRQ Channel configuration */
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//行程开关外部中断
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQChannel;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

	/* TIM1 update Channel configuration */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//timer2
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//usart1 
	//collin open20141028
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//usart3	 
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//timer3
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	   										  
}
void PRINT_Init (void)
{
   u16 i=500;
	while(BUSY)
	{
		i--;
		if(i==0)
		{			
			prnerr=1;
			break;
		}	
	}
	if(!prnerr)
	{
		BUSY = 1;		 
		STB  = 1; 
		pprint(0x1b);
		pprint(0x40);     
	}
//	pprint(0x1c);pprint(0x26);  
}
void pprint(unsigned char ch)                             
{ 	  
	u16 i=3000;
	unsigned char  tValue; 
	while(BUSY) 
	{
	 	i--;
		if(i<10)return;			
	}; 
	tValue=~ch; 
	GPIOD->BSRR=(unsigned int)ch; 
	GPIOD->BRR=(unsigned int)tValue;          
	STB=0;                                              
	Delay(200);
	STB=1;                                               
}
u8 const table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //月修正数据表
u8 RTC_Get_Week(u16 year,u8 month,u8 day)
{
    u16 temp2;
    u8 yearH,yearL;
    yearH=year/100; yearL=year%100;
    // 如果为21世纪，年份数加100
    if (yearH>19)yearL+=100;
    // 所过闰年数只算1900年之后的
    temp2=yearL+yearL/4;
    temp2=temp2%7;
    temp2=temp2+day+table_week[month-1];
    if (yearL%4==0&&month<3)temp2--;
    return(temp2%7);
}

void Set_time(void)
{  
    if((M[1]*10+M[2])>99){M[1]=0;M[2]=0;}
    if((M[3]*10+M[4])<1){M[3]=1;M[4]=2;}
    if((M[3]*10+M[4])>12){M[3]=0;M[4]=1;}
    if((M[5]*10+M[6])<1){M[5]=mon_table[(M[3]*10+M[4])-1]/10;M[6]=mon_table[(M[3]*10+M[4])-1]%10;}
    if((M[5]*10+M[6])>mon_table[(M[3]*10+M[4]-1)]){M[5]=0;M[6]=1;}
    if((M[7]*10+M[8])>23){M[7]=0;M[8]=0;}
    if((M[9]*10+M[10])>59){M[9]=0;M[10]=0;}
    if((M[11]*10+M[12])>59){M[11]=0;M[12]=0;}
    if(CONFIRM)
    {
        write_ds(0x09,(M[1]*10+M[2]));
		Delay5ms(10);
		write_ds(0x08,(M[3]*10+M[4]));
		Delay5ms(10);
		write_ds(0x07,(M[5]*10+M[6]));
		Delay5ms(10);
		write_ds(0x04,(M[7]*10+M[8]));
		Delay5ms(10);
		write_ds(0x02,(M[9]*10+M[10]));
		Delay5ms(10);
		write_ds(0x00,(M[11]*10+M[12]));
		Delay5ms(10);
		timer.week=RTC_Get_Week(2000+timer.w_year,timer.w_month,timer.w_date);
		sjxg=0;
    }
}


void print_test(void)											//打印检测
{	
	u8 i; 
//	pprint(0x1b);pprint(0x38);pprint(0x00);               	//调用汉字出库指令
	pprint(0x1c);pprint(0x26);  
	pprint(0x1b);pprint(0x55);pprint(2);		            //字体大小
	pprint(0x1b);pprint(0x31);pprint(10);                 	//行间距
    print_time();
    for(i=0;i<strlen(table1);i++)pprint(table1[i]); 	    //"________"	
	pprint(0x0d);				                            //回车
	for(i=0;i<strlen(DYZC);i++)pprint(DYZC[i]); 			//＂打印正常!＂  	
	pprint(0x0d);				                            //回车 
	for(i=0;i<strlen(table2);i++)pprint(table2[i]); 	    //"********"
	pprint(0x0d);				                        	//回车 
	for(i=0;i<strlen(DYJJC);i++)pprint(DYJJC[i]);		   	//打印机检测    	
	pprint(0x0d);				                            //回车 
	pprint(0x0d);				                            //回车 
	pprint(0x0d);				                       		//回车     
}
void print_time(void)                                         //打印日期时间
{
    u8 i;
    GetDate();
	GetTime();
	pprint(0x1c);pprint(0x26);
    pprint(0x0d);				                            //回车
    for(i=0;i<strlen(SJ);i++)pprint(SJ[i]); 	            //"时间:"       
    pprint(table[2]);
	pprint(table[0]);
	pprint(table[timer.w_year/10%10]);
	pprint(table[timer.w_year%10]);
    pprint(J[1]);
    pprint(table[timer.w_month/10]);
	pprint(table[timer.w_month%10]);
    pprint(J[1]);
    pprint(table[timer.w_date/10]);
	pprint(table[timer.w_date%10]);
    for(i=0;i<strlen(KG1);i++)pprint(KG1[i]); 	            //"空格"    
    pprint(table[timer.hour/10]);
	pprint(table[timer.hour%10]);
    pprint(J[2]);
    pprint(table[timer.min/10]);
	pprint(table[timer.min%10]);
    pprint(0x0d);				                            //回车  
}
void print_pressure(u16 p,u8 f)
{
  	u8 i;
	pprint(0x1c);pprint(0x26);		
//  pprint(0x1b);pprint(0x38);pprint(0x00);               	//调用汉字出库指令 
	pprint(0x1b);pprint(0x55);pprint(2);		            //字体大小
	pprint(0x1b);pprint(0x31);pprint(10);                 	//行间距	
  	if(f==0)
	{
		pprint(0x0d);				                            //回车
		for(i=0;i<strlen(DQYL);i++)pprint(DQYL[i]); 	        //"当前压力值："
    	pprint(table[p/1000]);
    	pprint(D[0]);
    	pprint(table[p/100%10]);
    	pprint(table[p/10%10]);
    	pprint(table[p%10]);
    	for(i=0;i<strlen(MPA);i++)pprint(MPA[i]); 	            //"MPa"	break;
	}
	else
	{
		pprint(0x0d);				                            //回车
		if(f==2){for(i=0;i<strlen(LQ);i++)pprint(LQ[i]);}//漏气
        if(f==1){for(i=0;i<strlen(YLZC);i++)pprint(YLZC[i]);}//压力正常  
        pprint(0x0d); 
        print_date_time();
        for(i=0;i<strlen(table2);i++)pprint(table2[i]); 	    //"*************" 	
		pprint(0x0d);
		for(i=0;i<strlen(YLJC);i++)pprint(YLJC[i]); 	        //" 压强检测  "
		pprint(0x0d);
		pprint(0x0d);
		pprint(0x0d);
	}   
}
void print_error(u8 type)
{
	u8 i;
	pprint(0x1c);pprint(0x26);
//	pprint(0x1b);pprint(0x38);pprint(0x00);               	//调用汉字出库指令 	 类型
	pprint(0x1b);pprint(0x55);pprint(2);		            //字体大小
	pprint(0x1b);pprint(0x31);pprint(10);                 	//行间距
	pprint(0x0d);				                            //回车 
	pprint(0x0d);				                            //回车
	for(i=0;i<strlen(DOBFJZ);i++)pprint(DOBFJZ[i]); 		//＂DOB分界值＂
	pprint(table[flag_dob/10]);										//4.0
	pprint(D[0]);
	pprint(table[flag_dob%10]);
	pprint(0x0d);				                       		//回车    
	for(i=0;i<strlen(table1);i++)pprint(table1[i]); 	    //"__________"
    print_time();//打印检测时间
	pprint(0x0d);				                            //回车
	for(i=0;i<strlen(TDH);i++)pprint(TDH[i]); 			   	//检测通道：
	pprint(table[checkChannel]);
	
	pprint(0x0d);
	if(type)
		for(i=0;i<strlen(NDD);i++)pprint(NDD[i]); 			   	//浓度低
	else 
		for(i=0;i<strlen(QDBC);i++)pprint(QDBC[i]); 			   	//气袋被拔出	
	pprint(0x0d);
	for(i=0;i<strlen(table2);i++)pprint(table2[i]); 	    //"********"
	pprint(0x0d);				                        	//回车 
	for(i=0;i<strlen(JCJG);i++)pprint(JCJG[i]); 			//＂检测结果＂
	pprint(0x0d);				                            //回车
	pprint(0x0d);				                            //回车 
	pprint(0x0d);				                       		//回车 			
}
void print_power(u8 count,u8 type)
{
	u8 i;
	long long C12_PJ_temp=0;
	long long C13_PJ_temp=0;
	s32 C12_BZWC_temp=0;
	s32 C13_BZWC_temp=0;
	pprint(0x1b);pprint(0x38);pprint(0x00);               	//调用汉字出库指令 
	pprint(0x1b);pprint(0x55);pprint(2);		            //字体大小
	pprint(0x1b);pprint(0x31);pprint(2);                 	//行间距	
	if(type==0)				                           
	{	
		pprint(0x0d);										  //回车 
		pprint(table[count/10]);
		pprint(table[count%10]);
		pprint(table[0]);
		for(i=0;i<strlen(KG);i++)pprint(KG[i]); 	    //"   "	
		if(Intensity1>=99999)
		{
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
		}
		else
		{
			pprint(table[Intensity1/10000]);
			pprint(table[Intensity1/1000%10]);
			pprint(table[Intensity1/100%10]);
			pprint(table[Intensity1/10%10]);
			pprint(table[Intensity1%10]);
		}
		for(i=0;i<strlen(KG);i++)pprint(KG[i]); 	    //"   "	
		if(Intensity2>=99999)
		{
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);//光强太大显示99999
		}
		else
		{
			pprint(table[Intensity2/10000]);
			pprint(table[Intensity2/1000%10]);
			pprint(table[Intensity2/100%10]);
			pprint(table[Intensity2/10%10]);
			pprint(table[Intensity2%10]);
		}
	}	
	if(count==60)
	{	
		pprint(0x0d);										  //回车
		C12_PJ=0;C13_PJ=0;
		for(i=0;i<count;i++)
		{
			C12_PJ_temp+=C12[i];
			C13_PJ_temp+=C13[i];
		}
		C12_PJ=(long long)(C12_PJ_temp/count);
		C13_PJ=(long long)(C13_PJ_temp/count);
		for(i=0;i<count;i++)
		{
			C12_BZWC_temp+=(C12[i]-C12_PJ)*(C12[i]-C12_PJ);
		}
		for(i=0;i<count;i++)
		{
			C13_BZWC_temp+=(C13[i]-C13_PJ)*(C13[i]-C13_PJ);
		}
		C12_BZWC= (long long)sqrt(C12_BZWC_temp/count);
		C13_BZWC= (long long)sqrt(C13_BZWC_temp/count);
		pprint(0x0d);				                            //回车
		if(type==0)
		{
			for(i=0;i<strlen(table1);i++)pprint(table1[i]); 	    //"——————————" 		
			pprint(0x0d);				                            //回车
			for(i=0;i<strlen(GQ13);i++)pprint(GQ13[i]); 			//＂次数	C12		C13＂
			pprint(0x0d);											//回车
		}		
		for(i=0;i<strlen(table1);i++)pprint(table1[i]); 	    //"——————————" 		
		pprint(0x0d);				                            //回车
		pprint(0x1b);pprint(0x31);pprint(16);                 	//行间距
		for(i=0;i<strlen(T13);i++)pprint(T13[i]); 			//＂C13=＂
		pprint(table[(C13_BZWC*1000/C13_PJ)/10]);
		pprint(D[0]);
		pprint(table[(C13_BZWC*1000/C13_PJ)%10]);
	 	//pprint(0x0d);				                            //回车
		 for(i=0;i<strlen(KG1);i++)pprint(KG1[i]); 	            //"空格" 
		for(i=0;i<strlen(T12);i++)pprint(T12[i]); 			//＂C12=＂
		pprint(table[(C12_BZWC*1000/C12_PJ)/10]);
		pprint(D[0]);
		pprint(table[(C12_BZWC*1000/C12_PJ)%10]);
		pprint(0x0d);				                            //回车
		for(i=0;i<strlen(BYXS);i++)pprint(BYXS[i]); 			//"变异系数"
		pprint(0x0d);				                            //回车
		for(i=0;i<strlen(T13);i++)pprint(T13[i]); 			//＂C13=＂
		if(C13_BZWC>9999)
		{
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
		}
        else  if(C13_BZWC>999)
		{
			pprint(table[C13_BZWC/1000]);
			pprint(table[C13_BZWC/100%10]);
			pprint(table[C13_BZWC/10%10]);
			pprint(table[C13_BZWC%10]);
		}
        else  if(C13_BZWC>99)
		{
			pprint(table[C13_BZWC/100]);
			pprint(table[C13_BZWC/10%10]);
			pprint(table[C13_BZWC%10]);
		}
        else  if(C13_BZWC>9)
		{
			pprint(table[C13_BZWC/10]);
			pprint(table[C13_BZWC%10]);
		}
        else
		{
			pprint(table[C13_BZWC]);
		}
		//pprint(0x0d);				                            //回车
		 for(i=0;i<strlen(KG1);i++)pprint(KG1[i]); 	            //"空格" 
		for(i=0;i<strlen(T12);i++)pprint(T12[i]); 			//＂C12=＂
		if(C12_BZWC>9999)
		{
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
		}
        else  if(C12_BZWC>999)
		{
			pprint(table[C12_BZWC/1000]);
			pprint(table[C12_BZWC/100%10]);
			pprint(table[C12_BZWC/10%10]);
			pprint(table[C12_BZWC%10]);
		}
        else  if(C12_BZWC>99)
		{
			pprint(table[C12_BZWC/100]);
			pprint(table[C12_BZWC/10%10]);
			pprint(table[C12_BZWC%10]);
		}
        else  if(C12_BZWC>9)
		{
			pprint(table[C12_BZWC/10]);
			pprint(table[C12_BZWC%10]);
		}
        else
		{
			pprint(table[C12_BZWC]);
		}				
		pprint(0x0d);				                            //回车
		for(i=0;i<strlen(GQBZWC);i++)pprint(GQBZWC[i]); 			//＂光强标准误差＂
		pprint(0x0d);											//回车
			for(i=0;i<strlen(T13);i++)pprint(T13[i]); 			//＂C13=＂
		if(C13_PJ>99999)
		{
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
		}
		else
		{
			pprint(table[C13_PJ/10000]);
			pprint(table[C13_PJ/1000%10]);
			pprint(table[C13_PJ/100%10]);
			pprint(table[C13_PJ/10%10]);
			pprint(table[C13_PJ%10]);
		}	
		pprint(0x0d);				                            //回车
		for(i=0;i<strlen(T12);i++)pprint(T12[i]); 			//＂C12=＂
		if(C12_PJ>99999)
		{
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
			pprint(table[9]);
		}
		else
		{
			pprint(table[C12_PJ/10000]);
			pprint(table[C12_PJ/1000%10]);
			pprint(table[C12_PJ/100%10]);
			pprint(table[C12_PJ/10%10]);
			pprint(table[C12_PJ%10]);
		}	
		pprint(0x0d);				                            //回车
		for(i=0;i<strlen(GQPJ);i++)pprint(GQPJ[i]); 			//＂光强平均值：＂
		pprint(0x0d);				                            //回车
        print_date_time();
		for(i=0;i<strlen(table2);i++)pprint(table2[i]); 	    //"********"
		pprint(0x0d);											//回车
		for(i=0;i<strlen(GQJG);i++)pprint(GQJG[i]); 			//＂光强检测结果＂
	   	pprint(0x0d);				                            //回车
//		pprint(0x0d);				                            //回车
//		pprint(0x0d);				                            //回车

		//collin add20141016
        Inqueue(0x02);
        Inqueue(0xff);
		Inqueue(0x00);
		Inqueue(0xf8);
        Inqueue(C12_PJ&0xff);
        Inqueue((C12_PJ>>8)&0xff);
        Inqueue((C12_PJ>>16)&0xff);
        Inqueue((C12_PJ>>24)&0xff);
        Inqueue(C13_PJ&0xff);
        Inqueue((C13_PJ>>8)&0xff);
        Inqueue((C13_PJ>>16)&0xff);
        Inqueue((C13_PJ>>24)&0xff);
		Inqueue(C12_BZWC&0xff);
		Inqueue((C12_BZWC>>8)&0xff);
		Inqueue(C13_BZWC&0xff);
		Inqueue((C13_BZWC>>8)&0xff);
        for(i=0; i<7; i++)
            Inqueue(0);
        Inqueue(0x03);
		Delay5ms(100);
		//end

		//collin add20141016
        Inqueue_uart1(0x02);
        Inqueue_uart1(0xff);
		Inqueue_uart1(0x00);
		Inqueue_uart1(0xf8);
        Inqueue_uart1(C12_PJ&0xff);
        Inqueue_uart1((C12_PJ>>8)&0xff);
        Inqueue_uart1((C12_PJ>>16)&0xff);
        Inqueue_uart1((C12_PJ>>24)&0xff);
        Inqueue_uart1(C13_PJ&0xff);
        Inqueue_uart1((C13_PJ>>8)&0xff);
        Inqueue_uart1((C13_PJ>>16)&0xff);
        Inqueue_uart1((C13_PJ>>24)&0xff);
		Inqueue_uart1(C12_BZWC&0xff);
		Inqueue_uart1((C12_BZWC>>8)&0xff);
		Inqueue_uart1(C13_BZWC&0xff);
		Inqueue_uart1((C13_BZWC>>8)&0xff);
        for(i=0; i<7; i++)
            Inqueue_uart1(0);
        Inqueue_uart1(0x03);
		Delay5ms(100);
		//end
	}		
}

void print_result(void)											//打印机检测
{	
	u8 i;
	DOB1=dob;
	pprint(0x1c);pprint(0x26);
//	pprint(0x1b);pprint(0x38);pprint(0x00);               	//调用汉字出库指令 
	pprint(0x1b);pprint(0x55);pprint(2);		            //字体大小
	pprint(0x1b);pprint(0x31);pprint(10);                 	//行间距
	pprint(0x0d);				                            //回车 
	pprint(0x0d);				                            //回车
	for(i=0;i<strlen(DOBFJZ);i++)pprint(DOBFJZ[i]); 		//＂DOB分界值＂
	pprint(table[ID[20]/10]);										//4.0
	pprint(D[0]);
	pprint(table[ID[20]%10]);
	pprint(0x0d);				                       		//回车    
	for(i=0;i<strlen(table1);i++)pprint(table1[i]); 	    //"__________"
    print_time();//打印检测时间
    for(i=0;i<strlen(JYR);i++)pprint(JYR[i]); 			   	//"检验人：_________"
    pprint(0x0d);
	//collin mask20141124
	/*				                       		//回车
	for(i=0;i<strlen(TDH);i++)pprint(TDH[i]); 			   	//检测通道：
	pprint(table[ID[6]]);	
	pprint(0x0d);
	*/
	//end				                        	//回车 
	for(i=0;i<strlen(JG);i++)pprint(JG[i]); 			   	//结果
	if((DOB1/10)>=flag_dob||((DOB1/10)<-cuozhi&&flag_cz==1&&(DOB1/10)<-flag_dob))
	{
		for(i=0;i<strlen(YANG);i++)
		pprint(YANG[i]);
	}
	else
	{
		for(i=0;i<strlen(YIN);i++)pprint(YIN[i]);
	}
	pprint(0x0d);				                            //回车
	for(i=0;i<strlen(DOB);i++)pprint(DOB[i]); 				//＂DOB(‰)=＂	
    DOB1=DOB1/10;
    if(DOB1<0)
    {
        if(flag_cz==0||(DOB1>=-cuozhi&&flag_cz==1))pprint(J[1]);
        DOB1=abs(DOB1);		
    }   
    if(DOB1>=10000)
	{
		pprint(table[(u16)(DOB1/1000%10)]);
		pprint(table[(u16)(DOB1/100%10)]);
		pprint(table[(u16)(DOB1/10%10)]);
		pprint(D[0]);
		pprint(table[(u16)(DOB1%10)]);
	}
	else if(DOB1>=1000)
	{
		pprint(table[(u16)(DOB1/1000)]);
		pprint(table[(u16)(DOB1/100%10)]);
		pprint(table[(u16)(DOB1/10%10)]);
		pprint(D[0]);
		pprint(table[(u16)(DOB1%10)]);
	}
	else	if(DOB1>=100)
	{
		pprint(table[(u16)(DOB1/100)]);
		pprint(table[(u16)(DOB1/10%10)]);
		pprint(D[0]);
		pprint(table[(u16)(DOB1%10)]);
	}
	else
	{
		pprint(table[(u16)(DOB1/10)]);
		pprint(D[0]);
		pprint(table[(u16)(DOB1%10)]);	
	}
	pprint(0x0d);				                       		//回车
	for(i=0;i<strlen(YQND);i++)pprint(YQND[i]); 			//＂样气浓度＂
    if((ID[17]+ID[18]*256)>9999)
    {
        pprint(table[(ID[17]+ID[18]*256)/10000]); 
        pprint(table[(ID[17]+ID[18]*256)/1000%10]);
        pprint(D[0]); 
        pprint(table[(ID[17]+ID[18]*256)/100%10]);
        pprint(table[(ID[17]+ID[18]*256)/10%10]);
    	pprint(table[(ID[17]+ID[18]*256)%10]); 
    }
    else
    {
    	pprint(table[(ID[17]+ID[18]*256)/1000]);
        pprint(D[0]); 
        pprint(table[(ID[17]+ID[18]*256)/100%10]);
        pprint(table[(ID[17]+ID[18]*256)/10%10]);
    	pprint(table[(ID[17]+ID[18]*256)%10]); 
    }
	for(i=0;i<strlen(BFH);i++)pprint(BFH[i]);				//％
	pprint(0x0d);				                       		//回车
	for(i=0;i<strlen(DQND);i++)pprint(DQND[i]); 			//＂底气浓度＂
	if((ID[4]+ID[5]*256)>9999)
    {
        pprint(table[(ID[4]+ID[5]*256)/10000]); 
        pprint(table[(ID[4]+ID[5]*256)/1000%10]);
        pprint(D[0]); 
        pprint(table[(ID[4]+ID[5]*256)/100%10]);
        pprint(table[(ID[4]+ID[5]*256)/10%10]);
    	pprint(table[(ID[4]+ID[5]*256)%10]); 
    }
    else
    {
    	pprint(table[(ID[4]+ID[5]*256)/1000]);
        pprint(D[0]); 
        pprint(table[(ID[4]+ID[5]*256)/100%10]);
        pprint(table[(ID[4]+ID[5]*256)/10%10]);
    	pprint(table[(ID[4]+ID[5]*256)%10]); 
    }
	for(i=0;i<strlen(BFH);i++)pprint(BFH[i]);				//％ 
	pprint(0x0d);				                       		//回车
    for(i=0;i<strlen(XM);i++)pprint(XM[i]); 			//＂姓名:_________＂
    pprint(0x0d);				                       		//回车
	for(i=0;i<strlen(YPBH);i++)pprint(YPBH[i]); 			//＂样品编号＂
	pprint(table[Num/10000]);
	pprint(table[Num/1000%10]);
	pprint(table[Num/100%10]);
	pprint(table[Num/10%10]);
	pprint(table[Num%10]);
	pprint(0x0d);				                        	//回车
	for(i=0;i<strlen(table2);i++)pprint(table2[i]); 	    //"********"
	pprint(0x0d);				                        	//回车 
	for(i=0;i<strlen(JCJG);i++)pprint(JCJG[i]); 			//＂检测结果＂
	pprint(0x0d);				                            //回车
	pprint(0x0d);				                            //回车 
	pprint(0x0d);				                       		//回车 
}

void print_result_search(void)											//打印机检测
{	
	u8 i;
	//collin add20141120
	DOB1 = (ID[0]<<8)+ID[1];
	//end 
	pprint(0x1c);pprint(0x26);
//	pprint(0x1b);pprint(0x38);pprint(0x00);               	//调用汉字出库指令 
	pprint(0x1b);pprint(0x55);pprint(2);		            //字体大小
	pprint(0x1b);pprint(0x31);pprint(10);                 	//行间距
	pprint(0x0d);				                            //回车
	for(i=0;i<strlen(DOBFJZ);i++)pprint(DOBFJZ[i]); 		//＂DOB分界值＂
	pprint(table[ID[20]/10]);										//4.0
	pprint(D[0]);
	pprint(table[ID[20]%10]);
//	pprint(table[ID[19]/10%10]);										//4.0
//	pprint(D[0]);
//	pprint(table[ID[19]%10]);
	pprint(0x0d);				                       		//回车
	for(i=0;i<strlen(table1);i++)pprint(table1[i]); 	    //"__________"
	pprint(0x0d);				                        	//回车
    for(i=0;i<strlen(SJ);i++)pprint(SJ[i]); 	            //"时间:"       
    pprint(table[2]);
	pprint(table[0]);
	pprint(table[ID[7]]);
	pprint(table[ID[8]]);
    pprint(J[1]);
    pprint(table[ID[9]]);
	pprint(table[ID[10]]);
    pprint(J[1]);
    pprint(table[ID[11]]);
	pprint(table[ID[12]]);
    for(i=0;i<strlen(KG1);i++)pprint(KG1[i]); 	            //"空格"
    pprint(table[ID[13]]);
	pprint(table[ID[14]]);
    pprint(J[2]);
    pprint(table[ID[15]]);
	pprint(table[ID[16]]);   
    pprint(0x0d);				                            //回车 
	//collin mask2014
	/*   
	for(i=0;i<strlen(TDH);i++)pprint(TDH[i]); 			   	//检测通道：
	pprint(table[ID[6]]);							
	pprint(0x0d);
	*/
	//end				                        	//回车 
	for(i=0;i<strlen(JG);i++)pprint(JG[i]); 			   	//结果
	//collin modify20141121
	//if(DOB1>=ID[20]||((DOB1)>cuozhi&&flag_cz==1&&ZF==1)){for(i=0;i<strlen(YANG);i++)pprint(YANG[i]);}
	if(abs(DOB1/10)>=ID[20]||((abs(DOB1))>cuozhi&&flag_cz==1&&ZF==1)){for(i=0;i<strlen(YANG);i++)pprint(YANG[i]);}
	//end
	else{for(i=0;i<strlen(YIN);i++)pprint(YIN[i]);}
	pprint(0x0d);				                            //回车
	for(i=0;i<strlen(DOB);i++)pprint(DOB[i]); 				//＂DOB(‰)=＂
	//collin add20141120
	DOB1=DOB1/10;
	 if(DOB1<0)
    {
        if(flag_cz==0||(DOB1>=-cuozhi&&flag_cz==1))pprint(J[1]);
        DOB1=abs(DOB1);		
    }  
	//end	
	if(ZF==1)
	{
		pprint(J[1]);						
	}
	if(DOB1>=1000)
	{
		if(DOB1>9999)
		{
			DOB1=9999;
		}
		pprint(table[(u16)(DOB1/1000)]);
		pprint(table[(u16)(DOB1/100%10)]);
		pprint(table[(u16)(DOB1/10%10)]);
		pprint(D[0]);
		pprint(table[(u16)(DOB1%10)]);
	}
	else if(DOB1>=100)
	{
		pprint(table[(u16)(DOB1/100)]);
		pprint(table[(u16)(DOB1/10%10)]);
		pprint(D[0]);
		pprint(table[(u16)(DOB1%10)]);
	}
	else
	{
		pprint(table[(u16)(DOB1/10)]);
		pprint(D[0]);
		pprint(table[(u16)(DOB1%10)]);	
	}
	pprint(0x0d);				                       		//回车
	for(i=0;i<strlen(YQND);i++)pprint(YQND[i]); 			//＂样气浓度＂
	if((ID[17]+ID[18]*256)>9999)
    {
        pprint(table[(ID[17]+ID[18]*256)/10000]); 
        pprint(table[(ID[17]+ID[18]*256)/1000%10]);
        pprint(D[0]); 
        pprint(table[(ID[17]+ID[18]*256)/100%10]);
        pprint(table[(ID[17]+ID[18]*256)/10%10]);
    	pprint(table[(ID[17]+ID[18]*256)%10]); 
    }
    else
    {
    	pprint(table[(ID[17]+ID[18]*256)/1000]);
        pprint(D[0]); 
        pprint(table[(ID[17]+ID[18]*256)/100%10]);
        pprint(table[(ID[17]+ID[18]*256)/10%10]);
    	pprint(table[(ID[17]+ID[18]*256)%10]); 
    }
	for(i=0;i<strlen(BFH);i++)pprint(BFH[i]);				//％
	pprint(0x0d);				                       		//回车
	for(i=0;i<strlen(DQND);i++)pprint(DQND[i]); 			//＂底气浓度＂
	if((ID[4]+ID[5]*256)>9999)
    {
        pprint(table[(ID[4]+ID[5]*256)/10000]); 
        pprint(table[(ID[4]+ID[5]*256)/1000%10]);
        pprint(D[0]); 
        pprint(table[(ID[4]+ID[5]*256)/100%10]);
        pprint(table[(ID[4]+ID[5]*256)/10%10]);
    	pprint(table[(ID[4]+ID[5]*256)%10]); 
    }
    else
    {
    	pprint(table[(ID[4]+ID[5]*256)/1000]);
        pprint(D[0]); 
        pprint(table[(ID[4]+ID[5]*256)/100%10]);
        pprint(table[(ID[4]+ID[5]*256)/10%10]);
    	pprint(table[(ID[4]+ID[5]*256)%10]); 
    }
	for(i=0;i<strlen(BFH);i++)pprint(BFH[i]);				//％
	pprint(0x0d);				                       		//回车
	for(i=0;i<strlen(YPBH);i++)pprint(YPBH[i]); 			//＂样品编号＂
	//collin modify20141121
	/*
	pprint(table[DiapIndex/10000]);
	pprint(table[DiapIndex/1000%10]);
	pprint(table[DiapIndex/100%10]);
	pprint(table[DiapIndex/10%10]);
	pprint(table[DiapIndex%10]);
	*/
	pprint(table[ID_NUM/10000]);
	pprint(table[ID_NUM/1000%10]);
	pprint(table[ID_NUM/100%10]);
	pprint(table[ID_NUM/10%10]);
	pprint(table[ID_NUM%10]);
    pprint(0x0d);				                            //回车
	for(i=0;i<strlen(table2);i++)pprint(table2[i]); 	    //"***********"
	pprint(0x0d);				                        	//回车
	for(i=0;i<strlen(JCJG);i++)pprint(JCJG[i]); 			//＂检测结果＂
	ZF=0;
	pprint(0x0d);				                            //回车
	pprint(0x0d);				                            //回车 
	pprint(0x0d);				                       		//回车 
}
void print_result_qc(void)									  //打印质控结果
{
	u8 i;
	pprint(0x1c);pprint(0x26);
//	pprint(0x1b);pprint(0x38);pprint(0x00);               	//调用汉字出库指令 
	pprint(0x1b);pprint(0x55);pprint(2);		            //字体大小
	pprint(0x1b);pprint(0x31);pprint(10);                 	//行间距
	pprint(0x0d);				                            //回车
	for(i=0;i<strlen(BZWC1);i++)pprint(BZWC1[i]); 			//＂标准误差：＂							
	if(BZWC>=100)
	{
		pprint(table[9]);
		pprint(D[0]);
		pprint(table[9]);
	}
	else
	{
		pprint(table[(u16)(BZWC/10)]);
		pprint(D[0]);
		pprint(table[(u16)(BZWC%10)]);	
	}
	pprint(0x0d);				                            //回车
	for(i=0;i<strlen(PJZ);i++)pprint(PJZ[i]); 				//＂  平均值：＂
	if(ZF==1)pprint(J[1]);
	if(DOB_PJ>999)
	{
		pprint(table[9]);
		pprint(table[9]);
		pprint(D[0]);
		pprint(table[9]);
	}
	else if(DOB_PJ>99)
	{
		pprint(table[(u16)(DOB_PJ/100)]);
		pprint(table[(u16)(DOB_PJ/10%10)]);
		pprint(D[0]);
		pprint(table[(u16)(DOB_PJ%10)]);
	}
	else
	{
		pprint(table[(u16)(DOB_PJ/10)]);
		pprint(D[0]);
		pprint(table[(u16)(DOB_PJ%10)]);	
	} 
	pprint(0x0d);				                            //回车
    print_date_time();
	for(i=0;i<strlen(table2);i++)pprint(table2[i]); 	    //"********"
	pprint(0x0d);				                        	//回车 
	for(i=0;i<strlen(ZK);i++)pprint(ZK[i]); 			//＂质量控制结果＂
	pprint(0x0d);				                            //回车
	pprint(0x0d);				                            //回车
	pprint(0x0d);				                            //回车
}
void print_filter(s16 Filter)											//过滤器检测
{	
	u8 i;
	pprint(0x1c);pprint(0x26); 
//	pprint(0x1b);pprint(0x38);pprint(0x00);               	//调用汉字出库指令 
	pprint(0x1b);pprint(0x55);pprint(2);		            //字体大小
	pprint(0x1b);pprint(0x31);pprint(10);                 	//行间距
    print_time();	
    for(i=0;i<strlen(table1);i++)pprint(table1[i]); 	    //"________"
	pprint(0x0d);				                            //回车
	for(i=0;i<strlen(GLQXN);i++)pprint(GLQXN[i]); 			//＂过滤器效能：＂
	pprint(table[Filter/1000]);
    pprint(D[0]);
    pprint(table[Filter/100%10]);
	pprint(table[Filter/10%10]);
	pprint(table[Filter%10]);
	pprint(0x0d);				                            //回车
	if(Filter>990)
	{
		for(i=0;i<strlen(GHGLQ);i++)pprint(GHGLQ[i]); 	//请更换过滤器	
		pprint(0x0d);				                            //回车
		for(i=0;i<strlen(GLQXNBZ);i++)pprint(GLQXNBZ[i]);  //过滤器效能不足
	}
	else
	{
		for(i=0;i<strlen(GLQXNLH);i++)pprint(GLQXNLH[i]);	//过滤器效能良好
	}  
	pprint(0x0d);				                            //回车
	for(i=0;i<strlen(table2);i++)pprint(table2[i]); 	    //"********"
	pprint(0x0d);				                        	//回车 
	for(i=0;i<strlen(GLQJC);i++)pprint(GLQJC[i]);		   	//过滤器检测    	
	pprint(0x0d);				                            //回车 
	pprint(0x0d);				                            //回车 
	pprint(0x0d);				                       		//回车      
}
/************************************************************/
void print_date_time(void)                                         //打印日期时间
{
    u8 i;
    GetDate();
	GetTime();
    for(i=0;i<strlen(table1);i++)pprint(table1[i]); 	    //"--------------"
    pprint(0x0d);				                            //回车
    for(i=0;i<strlen(SJ);i++)pprint(SJ[i]); 	            //"时间:"       
    pprint(table[timer.hour/10]);
	pprint(table[timer.hour%10]);
    pprint(J[2]);
    pprint(table[timer.min/10]);
	pprint(table[timer.min%10]);
    pprint(0x0d);				                            //回车
    for(i=0;i<strlen(RQ);i++)pprint(RQ[i]); 	            //"日期:" 
    pprint(table[2]);
	pprint(table[0]);
	pprint(table[timer.w_year/10%10]);
	pprint(table[timer.w_year%10]);
    pprint(J[1]);
    pprint(table[timer.w_month/10]);
	pprint(table[timer.w_month%10]);
    pprint(J[1]);
    pprint(table[timer.w_date/10]);
	pprint(table[timer.w_date%10]);
    pprint(0x0d);				                            //回车    
}

void print_temperature(u8 type)
{
	u8 i;
	pprint(0x1c);pprint(0x26);
//	pprint(0x1b);pprint(0x38);pprint(0x00);               	//调用汉字出库指令 	 类型
	pprint(0x1b);pprint(0x55);pprint(2);		            //字体大小
	pprint(0x1b);pprint(0x31);pprint(10);                 	//行间距
	pprint(0x0d);				                            //回车 
	print_time();//打印检测时间
	pprint(0x0d);				                            //回车 
//	pprint(0x1c);pprint(0x26);
	if(type)
		for(i=0;i<strlen(WDZC);i++)pprint(WDZC[i]); 			//＂温度正常＂
	else
		for(i=0;i<strlen(WDYC);i++)pprint(WDYC[i]); 			//＂温度异常＂
	for(i=0;i<strlen(table2);i++)pprint(table2[i]); 	    //"********"
	pprint(0x0d);				                        	//回车 
	for(i=0;i<strlen(WDJC);i++)pprint(WDJC[i]); 			//＂温度检测＂
	pprint(0x0d);				                            //回车
	pprint(0x0d);				                            //回车 
	pprint(0x0d);				                       		//回车 	
}

void deleteRecord(void)    //删除记录
{
	
	SPI_FLASH_SectorErase(0x7E0000);	 //样品编号
	SPI_FLASH_SectorErase(0x9E0000);	 //参数
	SPI_FLASH_SectorErase(0x010000);	//擦除所有检测记录
	SPI_FLASH_SectorErase(0x020000);
	SPI_FLASH_SectorErase(0x030000);
	SPI_FLASH_SectorErase(0x040000);
	SPI_FLASH_SectorErase(0x050000);
	SPI_FLASH_SectorErase(0x060000);
	SPI_FLASH_SectorErase(0x070000);
	SPI_FLASH_SectorErase(0x080000);
	SPI_FLASH_SectorErase(0x090000);
	SPI_FLASH_SectorErase(0x100000);
	SPI_FLASH_SectorErase(0x110000);	//擦除所有检测记录
	SPI_FLASH_SectorErase(0x120000);
	SPI_FLASH_SectorErase(0x130000);
	SPI_FLASH_SectorErase(0x140000);
	SPI_FLASH_SectorErase(0x150000);
	SPI_FLASH_SectorErase(0x160000);
	SPI_FLASH_SectorErase(0x170000);
	SPI_FLASH_SectorErase(0x180000);
	SPI_FLASH_SectorErase(0x190000);
	SPI_FLASH_SectorErase(0x200000);
	Delay5ms(60);

	SPI_FLASH_BufferRead(N,0x9E0000,8);	   //
	if(N[0]==0xff)flag_dob=40;
    else    flag_dob=N[0];
    if(N[2]==0||N[2]==1)flag_cz=N[2];
		else  flag_cz=1;
    if(N[3]==0xff)cuozhi=40;
    else    cuozhi=N[3];
//	SPI_FLASH_SectorErase(0x8F0000);            //擦除仪器编号
    SPI_FLASH_BufferRead(N,0x8F0000,8);
    if(N[0]==0xff)
    {
        YH08=0;
    }
	else 
	{
		YH08=N[1]*100000+N[2]*10000+N[3]*1000+N[4]*100+N[5]*10+N[6];
	}
//	SPI_FLASH_SectorErase(0x7E0000);            //擦除样品编号
	SPI_FLASH_BufferRead(ID,0x7E0000,30);		//读取样品编号
	if(ID[2]==0xff&&ID[3]==0xff)
	{
		Num=0;		
	}
	else
	{					
		Num=ID[2]*256+ID[3];
	}		
	Delay5ms(500);			
}
void sampleScan(void)
{
	if(SAMPLE1==0)
	{	 
		sampleOn|=0x01;
	}
	else 
	{
		sampleOn&=0xFE;
	}
	//collin mask20141027
/*	
	if(SAMPLE2==0)
	{	
		sampleOn|=0x02;
	}
	else 
	{
		sampleOn&=0xFD;
	}
*/	
	//end
	if(SAMPLE3==0)
	{	
		sampleOn|=0x04;
	}
	else 
	{
		sampleOn&=0xFB;
	}
	if(SAMPLE4==0)
	{	
		sampleOn|=0x08;
	}
	else 
	{
		sampleOn&=0xF7;
	}
	if(SAMPLE5==0)
	{	
		sampleOn|=0x10;
	}
	else 
	{
		sampleOn&=0xEF;
	}
	if(SAMPLE6==0)
	{	
		sampleOn|=0x20;
	}
	else 
	{
		sampleOn&=0xDF;
	}			
}
