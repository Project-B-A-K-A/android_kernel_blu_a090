
/*****************************************************************************************************************************
* 描述：CTS02 IC 内部部分代码，移至驱动内。
20160512匹配20160505软件版本
******************************************************************************************************************************/
#include <linux/i2c.h>       // i2c_client, i2c_master_send(), i2c_master_recv(), i2c_transfer(), I2C_M_RD, i2c_smbus_read_i2c_block_data(), i2c_smbus_write_i2c_block_data()
#include <linux/delay.h>     // mdelay()

extern int byd_ts_get_resolution(struct i2c_client *client, int *xy_res);

//固定值宏定义
#define  N			           2
#define  FILTER_HANDLE       1
#define  SEND_HANDLE         1
#define TREND_HANDLE         1

#define READ_PARA_LEN       50

#define READ_SELADDPOINT_ADDR  0x1b
#define READ_PARA_BASEADDR       0x1c


#define X_COOR_FILTER_MAX     36
#define  SHAKE_FFDIS                3
#define  SHAKE_FSDIS               1
#define  SHAKE_FDIS         	0
#define  SHAKE_SDIS         	0

#define COOR_FILTER_MULA       40
#define COOR_WEIGHT              200
#define X_DIS_MULA                 320
#define Y_DIS_MULA                 512
#define X_LAST_WEIGHT		85
#define X_WEIGHT_DIS		18
#define Y_LAST_WEIGHT		85
#define Y_WEIGHT_DIS		18

#define ADDPOINT_CNT             3
#define COOR_DIS_MULA          10


#define DISTANCE		   32
#define FAST_DISTANCE	   32
#define EDGE_X_MUL           10
#define EDGE_Y_MUL           10
#define DIS_MUL                 2
#define HOLD_FRAME          3



const unsigned char  BYD_CTS02_ArithCode_Version[] = {0x20,0x16,0x05,0x12,0x01,0x06};

typedef struct _struct_filter
{
	unsigned short AveCoorX[N];//读取的平均值坐标
	unsigned short AveCoorY[N];

	unsigned short LastCoorX[N];
	unsigned short LastCoorY[N];

	unsigned short NewPointX[N];
	unsigned short NewPointY[N];

	unsigned short LastNewPointX[N];
	unsigned short LastNewPointY[N];

	unsigned char AddCoorCnt[N];

}structfilter;

structfilter G_structPosInfor;

unsigned char buf_coor[10];
extern unsigned char buf[];

unsigned char ucTouchState[N];
unsigned char ucTouchId[N];

unsigned short PANEL_reso_x;
unsigned short PANEL_reso_y;

static unsigned char sNoTouchSendCnt = 0;

static unsigned char g_ucFirstTouchFlag = 0;
static unsigned char g_ucFrameCountFlag = 0;
static unsigned char g_ucFrameSendFlag = 0;
static unsigned char g_ucFrameShakeFlag = 0;

static unsigned short g_filter_max = 0;
static unsigned short g_backfilter_max = 0;
static unsigned char g_shake_ffdis = 0;
static unsigned char g_shake_fsdis = 0;
static unsigned char g_shake_fdis = 0;
static unsigned char g_shake_sdis = 0;
//static unsigned char g_move_point = 10;
static unsigned char g_coor_filter_mula = 0;
static unsigned short g_coor_weight = 0;
static unsigned short g_xdis_mula = 0;
static unsigned short g_ydis_mula = 0;

static unsigned char g_xLastweight = 0;
static unsigned char g_xweight_dis = 0;
static unsigned char g_yLastweight = 0;
static unsigned char g_yweight_dis = 0;

static unsigned char g_addpoint_cnt = 0;
static unsigned char g_coor_dis_mula = 0;
static unsigned short l_SaveCoorY[N]={0,0};

static unsigned char  g_distance= 0;
static unsigned short g_fast_distance= 0;
static unsigned char  g_edge_x_mul= 0;
static unsigned char  g_edge_y_mul= 0;
static unsigned char  g_dis_mul= 0;
static unsigned char  g_hold_frame= 0;
static int dx1[N] = {0},dy1[N] = {0};
static int pa1= 0,pa2= 0,pa3= 0,pa4= 0,pa5= 0;
static unsigned char  compensateCount[N]={0,0};
static unsigned char  compensateFlag[N]={0,0};
static unsigned char  xCompensateFlag[N]={0,0};
static unsigned char  yCompensateFlag[N]={0,0};
static unsigned char  xLongCompensateFlag[N]={0,0};
static unsigned char  yLongCompensateFlag[N]={0,0};
static unsigned char  holdCount[N]={0,0};
static int saveNewPointX[N];
static int saveNewPointY[N];
static int saveNewPointXTemp[N];
static int saveNewPointYTemp[N];
static int saveNewPointXTempTemp[N]={0,0};
static int saveNewPointYTempTemp[N]={0,0};


static unsigned char slopFlag[N]={0,0};
static unsigned char xMoveFlag[N]={0,0};
static unsigned char yMoveFlag[N]={0,0};
static unsigned short NewPointXTemp[N];
static unsigned short NewPointYTemp[N];
static unsigned short NewPointXTempTemp[N];
static unsigned short NewPointYTempTemp[N];

static unsigned char g_sel_addpoint = 0;
static unsigned char g_touchup_cnt = 2;
static unsigned char g_ucFingerNum = 0;
/*******************************************************************************
* 函数名称：int abs(int Value)
* 功能描述：绝对值
* 输入参数：无
* 返回值   ：绝对值
*--------------------------------------------------------------------------------------------------------------
*******************************************************************************/
int abs1(int Value)
{

	return (Value >= 0 ? Value : -Value);
}


unsigned short CalcAddCoor(unsigned short CurrentCoor,unsigned short LastCoor,unsigned short CoorDis)
{
	unsigned short ReCoor;

	if (CurrentCoor > LastCoor)
	{
		ReCoor = (CurrentCoor + CoorDis);
	}
	else
	{
		if (CurrentCoor > CoorDis)
			ReCoor = (CurrentCoor - CoorDis);
		else
			ReCoor = 0;
	}

	return ReCoor;
}

int addPoint(void)
{
	return pa1+(long)(pa2-pa3)*(pa4-pa1)/(pa5-pa3);
}

/*******************************************************************************
* 函数名称：void CoorJitFilter(void)
* 功能描述：坐标防抖滤波

*******************************************************************************/
void CoorJitFilter0(void)
{
	unsigned char i;
	unsigned char Coor_Bit;
	unsigned short x_err;
	unsigned short y_err;
	unsigned short tempx[N];
	unsigned short tempy[N];

	unsigned short x_dis;
	unsigned short y_dis;

#if FILTER_HANDLE
	unsigned short filter_step_x, filter_step_y;
	unsigned short TempFilter1,TempFilter2,TempFilter3,TempFilter4;
	unsigned short l_coor_filter = g_filter_max;

	if (g_filter_max > (PANEL_reso_x/g_coor_filter_mula))
		g_filter_max = PANEL_reso_x/g_coor_filter_mula;

	TempFilter1 = g_filter_max/3;
	TempFilter2 = g_filter_max*2;
	TempFilter3 = g_filter_max*3;
	TempFilter4 = g_filter_max*4;
#endif

	x_dis = PANEL_reso_x/g_xdis_mula;
	y_dis = PANEL_reso_y/g_ydis_mula;

	for (i = 0; i < 2; i++)
	{
		Coor_Bit = 1<<i;

		if ((G_structPosInfor.AveCoorX[i]!= 0) || (G_structPosInfor.AveCoorY[i]!= 0))
		{

			G_structPosInfor.LastCoorX[i] = G_structPosInfor.NewPointX[i];
			G_structPosInfor.LastCoorY[i] = G_structPosInfor.NewPointY[i];
			G_structPosInfor.AddCoorCnt[i] = 0;

			tempx[i] = G_structPosInfor.AveCoorX[i];
			tempy[i] = G_structPosInfor.AveCoorY[i];

			if ((g_ucFrameCountFlag & Coor_Bit) != 0)
			{
				tempx[i] = ((unsigned long)tempx[i]*(g_coor_weight-l_coor_filter)+((unsigned long)G_structPosInfor.LastCoorX[i]*l_coor_filter))/g_coor_weight;
				tempy[i] = ((unsigned long)tempy[i]*(g_coor_weight-l_coor_filter)+((unsigned long)G_structPosInfor.LastCoorY[i]*l_coor_filter))/g_coor_weight;
			}

#if FILTER_HANDLE

			filter_step_x = 0;
			filter_step_y = 0;

			if ((g_ucFrameCountFlag & Coor_Bit) == 0)
			{
				g_ucFrameCountFlag |= Coor_Bit;
			}
			else
			{
				x_err = tempx[i] > G_structPosInfor.LastCoorX[i] ?
				        (tempx[i] - G_structPosInfor.LastCoorX[i]):(G_structPosInfor.LastCoorX[i] - tempx[i]);

				y_err = tempy[i] > G_structPosInfor.LastCoorY[i] ?
				        (tempy[i] - G_structPosInfor.LastCoorY[i]):(G_structPosInfor.LastCoorY[i] - tempy[i]);

				if ( (x_err > g_filter_max && y_err > TempFilter1) ||
				        (x_err > TempFilter1 && y_err > g_filter_max) )
				{
					filter_step_x = x_err;
					filter_step_y = y_err;
				}
				else
				{
					if (x_err > g_filter_max)
						filter_step_x = x_err;
					if (y_err > g_filter_max)
						filter_step_y = y_err;
				}

				if (x_err <= TempFilter2 && y_err <= TempFilter2)
				{
					filter_step_x = filter_step_x>>2;
					filter_step_y = filter_step_y>>2;
				}
				else if (x_err <= TempFilter3 && y_err <= TempFilter3)
				{
					filter_step_x = filter_step_x>>1;
					filter_step_y = filter_step_y>>1;
				}
				else if (x_err <= TempFilter4 && y_err <= TempFilter4)
				{
					filter_step_x = filter_step_x*3/4;
					filter_step_y = filter_step_y*3/4;
				}
#if TREND_HANDLE

				if (slopFlag[i]==1)
				{
					tempx[i] = tempx[i] > G_structPosInfor.LastCoorX[i] ?
					           (G_structPosInfor.LastCoorX[i] + filter_step_x) : (G_structPosInfor.LastCoorX[i] - filter_step_x);

					tempy[i] = tempy[i] > G_structPosInfor.LastCoorY[i] ?
					           (G_structPosInfor.LastCoorY[i] + filter_step_y) : (G_structPosInfor.LastCoorY[i] - filter_step_y);
				}
				else
				{
					if ((xMoveFlag[i]==0))
					{
						tempx[i] = tempx[i] > G_structPosInfor.LastCoorX[i] ?
						           (G_structPosInfor.LastCoorX[i] + filter_step_x) : (G_structPosInfor.LastCoorX[i] - filter_step_x);
					}
					else
					{
						//tempx[i]=(((unsigned long)tempx[i]*(16-g_xLastweight))+((unsigned long)G_structPosInfor.LastCoorX[i]*g_xLastweight))>>4;
						x_err = abs1(tempy[i] - G_structPosInfor.LastCoorY[i]);

						if (x_err > g_xweight_dis)
						{
							x_err = g_xLastweight+x_err/g_xweight_dis;
							if (x_err >= 98)
								x_err = 98;
						}
						else
						{
							x_err = g_xLastweight;
						}

						tempx[i]=((unsigned long)tempx[i]*(100-x_err)+(unsigned long)G_structPosInfor.LastCoorX[i]*x_err)/100;
					}

					if ((yMoveFlag[i]==0))
					{
						tempy[i] = tempy[i] > G_structPosInfor.LastCoorY[i] ?
						           (G_structPosInfor.LastCoorY[i] + filter_step_y) : (G_structPosInfor.LastCoorY[i] - filter_step_y);
					}
					else
					{
						//tempy[i]=(((unsigned long)tempy[i]*(16-g_yLastweight))+((unsigned long)G_structPosInfor.LastCoorY[i]*g_yLastweight))>>4;
						y_err = abs1(tempx[i] - G_structPosInfor.LastCoorX[i]);

						if (y_err > g_yweight_dis)
						{
							y_err = g_yLastweight+y_err/g_yweight_dis;
							if (y_err >= 98)
								y_err = 98;
						}
						else
						{
							y_err = g_yLastweight;
						}

						tempy[i]=((unsigned long)tempy[i]*(100-y_err)+(unsigned long)G_structPosInfor.LastCoorY[i]*y_err)/100;
					}
				}
#else

				tempx[i] = tempx[i] > G_structPosInfor.LastCoorX[i] ?
				           (G_structPosInfor.LastCoorX[i] + filter_step_x) : (G_structPosInfor.LastCoorX[i] - filter_step_x);

				tempy[i] = tempy[i] > G_structPosInfor.LastCoorY[i] ?
				           (G_structPosInfor.LastCoorY[i] + filter_step_y) : (G_structPosInfor.LastCoorY[i] - filter_step_y);
#endif
			}

#endif
#if SEND_HANDLE
			if ((g_ucFrameSendFlag & Coor_Bit)== 0)
			{
				g_ucFrameSendFlag |= Coor_Bit;
				G_structPosInfor.NewPointX[i] = tempx[i];
				G_structPosInfor.NewPointY[i] = tempy[i];
#if TREND_HANDLE == 0
				G_structPosInfor.LastNewPointX[i] = tempx[i];
				G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
			}
			else
			{
				tempx[i] = (G_structPosInfor.LastNewPointX[i] + tempx[i])>>1;
				tempy[i] = (G_structPosInfor.LastNewPointY[i] + tempy[i])>>1;

				x_err = tempx[i] > G_structPosInfor.LastNewPointX[i] ?
				        (tempx[i] - G_structPosInfor.LastNewPointX[i]):(G_structPosInfor.LastNewPointX[i] - tempx[i]);

				y_err = tempy[i] > G_structPosInfor.LastNewPointY[i] ?
				        (tempy[i] - G_structPosInfor.LastNewPointY[i]):(G_structPosInfor.LastNewPointY[i] - tempy[i]);

				if ((g_ucFrameShakeFlag & Coor_Bit)== 0)
				{
					if ( (x_err > g_shake_ffdis && y_err > g_shake_fsdis) ||
					        (x_err > g_shake_fsdis && y_err > g_shake_ffdis) )
					{
						G_structPosInfor.NewPointX[i] = tempx[i];
						G_structPosInfor.NewPointY[i] = tempy[i];
#if TREND_HANDLE == 0
						G_structPosInfor.LastNewPointX[i] = tempx[i];
						G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
						g_ucFrameShakeFlag |= Coor_Bit;
					}
					else
					{
						if (x_err > g_shake_ffdis)
						{
							G_structPosInfor.NewPointX[i] = tempx[i];
#if TREND_HANDLE == 0
							G_structPosInfor.LastNewPointX[i] = tempx[i];
#endif
							g_ucFrameShakeFlag |= Coor_Bit;
						}
#if TREND_HANDLE == 0
						else
						{
							G_structPosInfor.NewPointX[i] = G_structPosInfor.LastNewPointX[i];
						}
#endif

						if (y_err > g_shake_ffdis)
						{
							G_structPosInfor.NewPointY[i] = tempy[i];
#if TREND_HANDLE == 0
							G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
							g_ucFrameShakeFlag |= Coor_Bit;
						}
#if TREND_HANDLE == 0
						else
						{
							G_structPosInfor.NewPointY[i] = G_structPosInfor.LastNewPointY[i];
						}
#endif
					}
				}
				else
				{

					if ( (x_err > g_shake_fdis && y_err > g_shake_sdis) ||
					        (x_err > g_shake_sdis && y_err > g_shake_fdis) )
					{
						G_structPosInfor.NewPointX[i] = tempx[i];
						G_structPosInfor.NewPointY[i] = tempy[i];
#if TREND_HANDLE == 0
						G_structPosInfor.LastNewPointX[i] = tempx[i];
						G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
					}
					else
					{
						if (x_err > g_shake_fdis)
						{
							G_structPosInfor.NewPointX[i]     = tempx[i];
#if TREND_HANDLE == 0
							G_structPosInfor.LastNewPointX[i] = tempx[i];
#endif
						}
#if TREND_HANDLE == 0
						else
						{
							G_structPosInfor.NewPointX[i] = G_structPosInfor.LastNewPointX[i];
						}
#endif

						if (y_err > g_shake_fdis)
						{
							G_structPosInfor.NewPointY[i]     = tempy[i];
#if TREND_HANDLE == 0
							G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
						}
#if TREND_HANDLE == 0
						else
						{
							G_structPosInfor.NewPointY[i] = G_structPosInfor.LastNewPointY[i];
						}
#endif
					}
				}
			}
#else
			G_structPosInfor.NewPointX[i] = tempx[i];
			G_structPosInfor.NewPointY[i] = tempy[i];
#endif
#if TREND_HANDLE
			x_err = PANEL_reso_x>>4;

			if (xMoveFlag[i]==1)
			{
				if (G_structPosInfor.NewPointX[i]>NewPointXTempTemp[i])
				{
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				}
				else if ((NewPointXTempTemp[i]-G_structPosInfor.NewPointX[i])>x_err)
				{
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				}
				G_structPosInfor.NewPointX[i]=NewPointXTempTemp[i];
			}
			if (xMoveFlag[i]==2)
			{
				if (G_structPosInfor.NewPointX[i]<NewPointXTempTemp[i])
				{
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				}
				else if ((G_structPosInfor.NewPointX[i]-NewPointXTempTemp[i])>x_err)
				{
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				}
				G_structPosInfor.NewPointX[i]=NewPointXTempTemp[i];
			}

			if (yMoveFlag[i]==1)
			{
				if (G_structPosInfor.NewPointY[i]<NewPointYTempTemp[i])
				{
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				else if ((G_structPosInfor.NewPointY[i]-NewPointYTempTemp[i])>x_err)
				{
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				G_structPosInfor.NewPointY[i]=NewPointYTempTemp[i];
			}
			if (yMoveFlag[i]==2)
			{
				if (G_structPosInfor.NewPointY[i]>NewPointYTempTemp[i])
				{
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				else if ((NewPointYTempTemp[i]-G_structPosInfor.NewPointY[i])>x_err)
				{
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				G_structPosInfor.NewPointY[i]=NewPointYTempTemp[i];
			}


			if ((NewPointXTemp[i]!=0)||(NewPointYTemp[i]!=0))
			{
				if ((abs1(NewPointXTemp[i]-G_structPosInfor.NewPointX[i])>x_dis)&&
				        (abs1(NewPointYTemp[i]-G_structPosInfor.NewPointY[i])<y_dis))
				{
					if (G_structPosInfor.NewPointX[i]>NewPointXTemp[i])
					{
						xMoveFlag[i]=1;
					}
					else
					{
						xMoveFlag[i]=2;
					}
					yMoveFlag[i]=0;
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];

				}
				if ((abs1(NewPointYTemp[i]-G_structPosInfor.NewPointY[i])>y_dis)&&
				        (abs1(NewPointXTemp[i]-G_structPosInfor.NewPointX[i])<x_dis))
				{
					if (G_structPosInfor.NewPointY[i]>NewPointYTemp[i])
					{
						yMoveFlag[i]=2;
					}
					else
					{
						yMoveFlag[i]=1;
					}
					xMoveFlag[i]=0;
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				if ((abs1(NewPointXTemp[i]-G_structPosInfor.NewPointX[i])>x_dis)&&
				        (abs1(NewPointYTemp[i]-G_structPosInfor.NewPointY[i])>y_dis))
				{
					slopFlag[i]=1;
					xMoveFlag[i]=0;
					yMoveFlag[i]=0;
				}
			}
			G_structPosInfor.LastNewPointX[i]=G_structPosInfor.NewPointX[i];
			G_structPosInfor.LastNewPointY[i]=G_structPosInfor.NewPointY[i];


			//GestureJudge(i);

			NewPointXTemp[i] = G_structPosInfor.NewPointX[i];
			NewPointYTemp[i] = G_structPosInfor.NewPointY[i];
#endif
		}
		else
		{

			G_structPosInfor.NewPointX[i] = 0;
			G_structPosInfor.NewPointY[i] = 0;
			G_structPosInfor.LastCoorX[i] = 0;
			G_structPosInfor.LastCoorY[i] = 0;
			G_structPosInfor.LastNewPointX[i] = 0;
			G_structPosInfor.LastNewPointY[i] = 0;

#if TREND_HANDLE
			xMoveFlag[i]=0;
			yMoveFlag[i]=0;
			slopFlag[i]=0;
//			firstComFlag[i]=0;
//			firstMoveTouch[i]=1;
			NewPointXTemp[i] = 0;
			NewPointYTemp[i] = 0;
			NewPointXTempTemp[i]=0;
			NewPointYTempTemp[i]=0;
#endif
			g_ucFrameCountFlag &= ~Coor_Bit;
			g_ucFrameSendFlag &= ~Coor_Bit;
			g_ucFrameShakeFlag &= ~Coor_Bit;
		}

	}

}

void CoorJitFilter1(void)
{
	unsigned char i;
	unsigned char Coor_Bit;
	unsigned short x_err;
	unsigned short y_err;
	unsigned short tempx[N];
	unsigned short tempy[N];

	unsigned short x_err1;
	unsigned short y_err1;
	unsigned short l_TempCoorX;
	unsigned short l_TempCoorY;

	unsigned short x_dis;
	unsigned short y_dis;

#if FILTER_HANDLE
	unsigned short filter_step_x, filter_step_y;
	unsigned short TempFilter1,TempFilter2,TempFilter3,TempFilter4;
	unsigned short l_coor_filter = g_filter_max;

	if (g_filter_max > (PANEL_reso_x/g_coor_filter_mula))
		g_filter_max = PANEL_reso_x/g_coor_filter_mula;

	TempFilter1 = g_filter_max/3;
	TempFilter2 = g_filter_max*2;
	TempFilter3 = g_filter_max*3;
	TempFilter4 = g_filter_max*4;
#endif

	x_dis = PANEL_reso_x/g_xdis_mula;
	y_dis = PANEL_reso_y/g_ydis_mula;

	for (i = 0; i < 2; i++)
	{
		Coor_Bit = 1<<i;

		if ((G_structPosInfor.AveCoorX[i]!= 0) || (G_structPosInfor.AveCoorY[i]!= 0))
		{

			G_structPosInfor.LastCoorX[i] = G_structPosInfor.NewPointX[i];
			G_structPosInfor.LastCoorY[i] = G_structPosInfor.NewPointY[i];
			G_structPosInfor.AddCoorCnt[i] = 0;

			tempx[i] = G_structPosInfor.AveCoorX[i];
			tempy[i] = G_structPosInfor.AveCoorY[i];

			if ((g_ucFrameCountFlag & Coor_Bit) != 0)
			{
				tempx[i] = ((unsigned long)tempx[i]*(g_coor_weight-l_coor_filter)+((unsigned long)G_structPosInfor.LastCoorX[i]*l_coor_filter))/g_coor_weight;
				tempy[i] = ((unsigned long)tempy[i]*(g_coor_weight-l_coor_filter)+((unsigned long)G_structPosInfor.LastCoorY[i]*l_coor_filter))/g_coor_weight;
			}

			l_SaveCoorY[i] = tempy[i];

#if FILTER_HANDLE

			filter_step_x = 0;
			filter_step_y = 0;

			if ((g_ucFrameCountFlag & Coor_Bit) == 0)
			{
				g_ucFrameCountFlag |= Coor_Bit;
			}
			else
			{
				x_err = tempx[i] > G_structPosInfor.LastCoorX[i] ?
				        (tempx[i] - G_structPosInfor.LastCoorX[i]):(G_structPosInfor.LastCoorX[i] - tempx[i]);

				y_err = tempy[i] > G_structPosInfor.LastCoorY[i] ?
				        (tempy[i] - G_structPosInfor.LastCoorY[i]):(G_structPosInfor.LastCoorY[i] - tempy[i]);

				if ( (x_err > g_filter_max && y_err > TempFilter1) ||
				        (x_err > TempFilter1 && y_err > g_filter_max) )
				{
					filter_step_x = x_err;
					filter_step_y = y_err;
				}
				else
				{
					if (x_err > g_filter_max)
						filter_step_x = x_err;
					if (y_err > g_filter_max)
						filter_step_y = y_err;
				}

				if (x_err <= TempFilter2 && y_err <= TempFilter2)
				{
					filter_step_x = filter_step_x>>2;
					filter_step_y = filter_step_y>>2;
				}
				else if (x_err <= TempFilter3 && y_err <= TempFilter3)
				{
					filter_step_x = filter_step_x>>1;
					filter_step_y = filter_step_y>>1;
				}
				else if (x_err <= TempFilter4 && y_err <= TempFilter4)
				{
					filter_step_x = filter_step_x*3/4;
					filter_step_y = filter_step_y*3/4;
				}
#if TREND_HANDLE

				if (slopFlag[i]==1)
				{
					tempx[i] = tempx[i] > G_structPosInfor.LastCoorX[i] ?
					           (G_structPosInfor.LastCoorX[i] + filter_step_x) : (G_structPosInfor.LastCoorX[i] - filter_step_x);

					tempy[i] = tempy[i] > G_structPosInfor.LastCoorY[i] ?
					           (G_structPosInfor.LastCoorY[i] + filter_step_y) : (G_structPosInfor.LastCoorY[i] - filter_step_y);
				}
				else
				{
					if ((xMoveFlag[i]==0))
					{
						tempx[i] = tempx[i] > G_structPosInfor.LastCoorX[i] ?
						           (G_structPosInfor.LastCoorX[i] + filter_step_x) : (G_structPosInfor.LastCoorX[i] - filter_step_x);
					}
					else
					{
						//tempx[i]=(((unsigned long)tempx[i]*(16-g_xLastweight))+((unsigned long)G_structPosInfor.LastCoorX[i]*g_xLastweight))>>4;
						x_err = abs1(tempy[i] - G_structPosInfor.LastCoorY[i]);

						if (x_err > g_xweight_dis)
						{
							x_err = g_xLastweight+x_err/g_xweight_dis;
							if (x_err >= 98)
								x_err = 98;
						}
						else
						{
							x_err = g_xLastweight;
						}

						tempx[i]=((unsigned long)tempx[i]*(100-x_err)+(unsigned long)G_structPosInfor.LastCoorX[i]*x_err)/100;
					}

					if ((yMoveFlag[i]==0))
					{
						tempy[i] = tempy[i] > G_structPosInfor.LastCoorY[i] ?
						           (G_structPosInfor.LastCoorY[i] + filter_step_y) : (G_structPosInfor.LastCoorY[i] - filter_step_y);
					}
					else
					{
						//tempy[i]=(((unsigned long)tempy[i]*(16-g_yLastweight))+((unsigned long)G_structPosInfor.LastCoorY[i]*g_yLastweight))>>4;
						y_err = abs1(tempx[i] - G_structPosInfor.LastCoorX[i]);

						if (y_err > g_yweight_dis)
						{
							y_err = g_yLastweight+y_err/g_yweight_dis;
							if (y_err >= 98)
								y_err = 98;
						}
						else
						{
							y_err = g_yLastweight;
						}

						tempy[i]=((unsigned long)tempy[i]*(100-y_err)+(unsigned long)G_structPosInfor.LastCoorY[i]*y_err)/100;
					}
				}
#else

				tempx[i] = tempx[i] > G_structPosInfor.LastCoorX[i] ?
				           (G_structPosInfor.LastCoorX[i] + filter_step_x) : (G_structPosInfor.LastCoorX[i] - filter_step_x);

				tempy[i] = tempy[i] > G_structPosInfor.LastCoorY[i] ?
				           (G_structPosInfor.LastCoorY[i] + filter_step_y) : (G_structPosInfor.LastCoorY[i] - filter_step_y);
#endif
			}

#endif
#if SEND_HANDLE
			if ((g_ucFrameSendFlag & Coor_Bit)== 0)
			{
				g_ucFrameSendFlag |= Coor_Bit;
				G_structPosInfor.NewPointX[i] = tempx[i];
				G_structPosInfor.NewPointY[i] = tempy[i];
#if TREND_HANDLE == 0
				G_structPosInfor.LastNewPointX[i] = tempx[i];
				G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
			}
			else
			{
				tempx[i] = (G_structPosInfor.LastNewPointX[i] + tempx[i])>>1;
				tempy[i] = (G_structPosInfor.LastNewPointY[i] + tempy[i])>>1;

				x_err = tempx[i] > G_structPosInfor.LastNewPointX[i] ?
				        (tempx[i] - G_structPosInfor.LastNewPointX[i]):(G_structPosInfor.LastNewPointX[i] - tempx[i]);

				y_err = tempy[i] > G_structPosInfor.LastNewPointY[i] ?
				        (tempy[i] - G_structPosInfor.LastNewPointY[i]):(G_structPosInfor.LastNewPointY[i] - tempy[i]);

				if ((g_ucFrameShakeFlag & Coor_Bit)== 0)
				{
					if ( (x_err > g_shake_ffdis && y_err > g_shake_fsdis) ||
					        (x_err > g_shake_fsdis && y_err > g_shake_ffdis) )
					{
						G_structPosInfor.NewPointX[i] = tempx[i];
						G_structPosInfor.NewPointY[i] = tempy[i];
#if TREND_HANDLE == 0
						G_structPosInfor.LastNewPointX[i] = tempx[i];
						G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
						g_ucFrameShakeFlag |= Coor_Bit;
					}
					else
					{
						if (x_err > g_shake_ffdis)
						{
							G_structPosInfor.NewPointX[i] = tempx[i];
#if TREND_HANDLE == 0
							G_structPosInfor.LastNewPointX[i] = tempx[i];
#endif
							g_ucFrameShakeFlag |= Coor_Bit;
						}
#if TREND_HANDLE == 0
						else
						{
							G_structPosInfor.NewPointX[i] = G_structPosInfor.LastNewPointX[i];
						}
#endif

						if (y_err > g_shake_ffdis)
						{
							G_structPosInfor.NewPointY[i] = tempy[i];
#if TREND_HANDLE == 0
							G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
							g_ucFrameShakeFlag |= Coor_Bit;
						}
#if TREND_HANDLE == 0
						else
						{
							G_structPosInfor.NewPointY[i] = G_structPosInfor.LastNewPointY[i];
						}
#endif
					}
				}
				else
				{

					if ( (x_err > g_shake_fdis && y_err > g_shake_sdis) ||
					        (x_err > g_shake_sdis && y_err > g_shake_fdis) )
					{
						G_structPosInfor.NewPointX[i] = tempx[i];
						G_structPosInfor.NewPointY[i] = tempy[i];
#if TREND_HANDLE == 0
						G_structPosInfor.LastNewPointX[i] = tempx[i];
						G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
					}
					else
					{
						if (x_err > g_shake_fdis)
						{
							G_structPosInfor.NewPointX[i]     = tempx[i];
#if TREND_HANDLE == 0
							G_structPosInfor.LastNewPointX[i] = tempx[i];
#endif
						}
#if TREND_HANDLE == 0
						else
						{
							G_structPosInfor.NewPointX[i] = G_structPosInfor.LastNewPointX[i];
						}
#endif

						if (y_err > g_shake_fdis)
						{
							G_structPosInfor.NewPointY[i]     = tempy[i];
#if TREND_HANDLE == 0
							G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
						}
#if TREND_HANDLE == 0
						else
						{
							G_structPosInfor.NewPointY[i] = G_structPosInfor.LastNewPointY[i];
						}
#endif
					}
				}
			}
#else
			G_structPosInfor.NewPointX[i] = tempx[i];
			G_structPosInfor.NewPointY[i] = tempy[i];
#endif
#if TREND_HANDLE
			x_err = PANEL_reso_x>>4;

			if (xMoveFlag[i]==1)
			{
				if (G_structPosInfor.NewPointX[i]>NewPointXTempTemp[i])
				{
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				}
				else if ((NewPointXTempTemp[i]-G_structPosInfor.NewPointX[i])>x_err)
				{
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				}
				G_structPosInfor.NewPointX[i]=NewPointXTempTemp[i];
			}
			if (xMoveFlag[i]==2)
			{
				if (G_structPosInfor.NewPointX[i]<NewPointXTempTemp[i])
				{
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				}
				else if ((G_structPosInfor.NewPointX[i]-NewPointXTempTemp[i])>x_err)
				{
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				}
				G_structPosInfor.NewPointX[i]=NewPointXTempTemp[i];
			}

			if (yMoveFlag[i]==1)
			{
				if (G_structPosInfor.NewPointY[i]<NewPointYTempTemp[i])
				{
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				else if ((G_structPosInfor.NewPointY[i]-NewPointYTempTemp[i])>x_err)
				{
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				G_structPosInfor.NewPointY[i]=NewPointYTempTemp[i];
			}
			if (yMoveFlag[i]==2)
			{
				if (G_structPosInfor.NewPointY[i]>NewPointYTempTemp[i])
				{
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				else if ((NewPointYTempTemp[i]-G_structPosInfor.NewPointY[i])>x_err)
				{
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				G_structPosInfor.NewPointY[i]=NewPointYTempTemp[i];
			}


			if ((NewPointXTemp[i]!=0)||(NewPointYTemp[i]!=0))
			{
				if ((abs1(NewPointXTemp[i]-G_structPosInfor.NewPointX[i])>x_dis)&&
				        (abs1(NewPointYTemp[i]-G_structPosInfor.NewPointY[i])<y_dis))
				{
					if (G_structPosInfor.NewPointX[i]>NewPointXTemp[i])
					{
						xMoveFlag[i]=1;
					}
					else
					{
						xMoveFlag[i]=2;
					}
					yMoveFlag[i]=0;
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];

				}
				if ((abs1(NewPointYTemp[i]-G_structPosInfor.NewPointY[i])>y_dis)&&
				        (abs1(NewPointXTemp[i]-G_structPosInfor.NewPointX[i])<x_dis))
				{
					if (G_structPosInfor.NewPointY[i]>NewPointYTemp[i])
					{
						yMoveFlag[i]=2;
					}
					else
					{
						yMoveFlag[i]=1;
					}
					xMoveFlag[i]=0;
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				if ((abs1(NewPointXTemp[i]-G_structPosInfor.NewPointX[i])>x_dis)&&
				        (abs1(NewPointYTemp[i]-G_structPosInfor.NewPointY[i])>y_dis))
				{
					slopFlag[i]=1;
					xMoveFlag[i]=0;
					yMoveFlag[i]=0;
				}
			}
			G_structPosInfor.LastNewPointX[i]=G_structPosInfor.NewPointX[i];
			G_structPosInfor.LastNewPointY[i]=G_structPosInfor.NewPointY[i];


			//GestureJudge(i);

			NewPointXTemp[i] = G_structPosInfor.NewPointX[i];
			NewPointYTemp[i] = G_structPosInfor.NewPointY[i];
#endif
		}
		else
		{
			if ((((G_structPosInfor.LastCoorX[i]!=0)||(G_structPosInfor.LastCoorY[i] != 0)))&&
			        (G_structPosInfor.AddCoorCnt[i] < g_addpoint_cnt)
			   )
			{
				x_err = (abs1(G_structPosInfor.LastCoorX[i] - G_structPosInfor.NewPointX[i]));
				y_err = (abs1(G_structPosInfor.LastCoorY[i] - G_structPosInfor.NewPointY[i]));

				if (x_err < (PANEL_reso_x/g_coor_dis_mula)) x_err = x_err>>2;

				if (G_structPosInfor.AddCoorCnt[i] == 0)
				{
					if (x_err < (PANEL_reso_x/g_coor_dis_mula))
					{
						x_err = x_err>>1;

						if (x_err > ((PANEL_reso_x/g_coor_dis_mula)>>1))
							y_err = y_err>>1;
					}
					x_err = x_err<<1;
					y_err = y_err<<1;
				}

				G_structPosInfor.AddCoorCnt[i]++;

				G_structPosInfor.LastNewPointX[i] = G_structPosInfor.NewPointX[i];
				G_structPosInfor.LastNewPointY[i] = G_structPosInfor.NewPointY[i];

				l_TempCoorX = CalcAddCoor(G_structPosInfor.NewPointX[i],G_structPosInfor.LastCoorX[i],x_err);
				l_TempCoorY = CalcAddCoor(G_structPosInfor.NewPointY[i],G_structPosInfor.LastCoorY[i],y_err);

				if (l_TempCoorX > PANEL_reso_x)
				{
					l_TempCoorX = PANEL_reso_x;
					y_err1 = ((PANEL_reso_x-G_structPosInfor.NewPointX[i])*10)/x_err;
					y_err =  (y_err*(y_err1))/10;
					l_TempCoorY = CalcAddCoor(G_structPosInfor.NewPointY[i],G_structPosInfor.LastCoorY[i],y_err);
				}
				else if (l_TempCoorX == 0)
				{
					y_err1 = (G_structPosInfor.NewPointX[i]*10)/x_err;
					y_err =  (y_err*y_err1)/10;
					l_TempCoorY = CalcAddCoor(G_structPosInfor.NewPointY[i],G_structPosInfor.LastCoorY[i],y_err);
				}
				else if (l_TempCoorY > PANEL_reso_y)
				{
					l_TempCoorY = PANEL_reso_y;
					x_err1 = ((PANEL_reso_y - G_structPosInfor.NewPointY[i])*10)/y_err;
					x_err =  (x_err*(x_err1))/10;
					l_TempCoorX = CalcAddCoor(G_structPosInfor.NewPointX[i],G_structPosInfor.LastCoorX[i],x_err);
				}
				else if (l_TempCoorY == 0)
				{
					x_err1 = (G_structPosInfor.NewPointY[i]*10)/y_err;
					x_err =  (x_err*x_err1)/10;
					l_TempCoorX = CalcAddCoor(G_structPosInfor.NewPointX[i],G_structPosInfor.LastCoorX[i],x_err);
				}

				if ((l_TempCoorY > (PANEL_reso_y - (PANEL_reso_y/g_edge_y_mul)))&&
				        (x_err < ((PANEL_reso_x/g_coor_dis_mula)>>1))
				   )
				{
					l_TempCoorY = l_SaveCoorY[i];
				}

				G_structPosInfor.NewPointX[i] = l_TempCoorX;
				G_structPosInfor.NewPointY[i] = l_TempCoorY;

				// if((G_structPosInfor.AddCoorCnt[i] >= g_addpoint_cnt))
				// {
				// G_structPosInfor.LastCoorX[i] = 0;
				// G_structPosInfor.LastCoorY[i] = 0;
				// G_structPosInfor.NewPointX[i] = 0;
				// G_structPosInfor.NewPointY[i] = 0;
				// G_structPosInfor.LastNewPointX[i] = 0;
				// G_structPosInfor.LastNewPointY[i] = 0;
				// G_structPosInfor.AddCoorCnt[i] = 0;
				// }
				// else
				{
					g_ucFingerNum = i+1;
					G_structPosInfor.LastCoorX[i] = G_structPosInfor.LastNewPointX[i];
					G_structPosInfor.LastCoorY[i] = G_structPosInfor.LastNewPointY[i];
					G_structPosInfor.LastNewPointX[i] = 0;
					G_structPosInfor.LastNewPointY[i] = 0;
				}


				if ((G_structPosInfor.NewPointX[i] == 0)||(G_structPosInfor.NewPointX[i] == PANEL_reso_x)||
				        (G_structPosInfor.NewPointY[i] == 0)||(G_structPosInfor.NewPointY[i] == PANEL_reso_y)
				   )
				{
					G_structPosInfor.AddCoorCnt[i] = g_addpoint_cnt;
				}

				if ((x_err > (PANEL_reso_x>>2)) || (y_err > (PANEL_reso_y>>2)))
				{
					G_structPosInfor.AddCoorCnt[i] = g_addpoint_cnt;
				}

			}
			else
			{
				G_structPosInfor.NewPointX[i] = 0;
				G_structPosInfor.NewPointY[i] = 0;
				G_structPosInfor.LastCoorX[i] = 0;
				G_structPosInfor.LastCoorY[i] = 0;
				G_structPosInfor.LastNewPointX[i] = 0;
				G_structPosInfor.LastNewPointY[i] = 0;
				G_structPosInfor.AddCoorCnt[i] = 0;
				l_SaveCoorY[i] = 0;
			}

#if TREND_HANDLE
			xMoveFlag[i]=0;
			yMoveFlag[i]=0;
			slopFlag[i]=0;
//			firstComFlag[i]=0;
//			firstMoveTouch[i]=1;
			NewPointXTemp[i] = 0;
			NewPointYTemp[i] = 0;
			NewPointXTempTemp[i]=0;
			NewPointYTempTemp[i]=0;
#endif
			g_ucFrameCountFlag &= ~Coor_Bit;
			g_ucFrameSendFlag &= ~Coor_Bit;
			g_ucFrameShakeFlag &= ~Coor_Bit;
		}

	}

}


void CoorJitFilter2(void)
{
	unsigned char i;
	unsigned char Coor_Bit;
	unsigned short x_err;
	unsigned short y_err;
	unsigned short tempx[N];
	unsigned short tempy[N];
	unsigned short x_edge;
	unsigned short y_edge;

	unsigned short weight;
	unsigned short x_dis;
	unsigned short y_dis;
    unsigned char  ucTempFingerNum;
#if FILTER_HANDLE
	unsigned short filter_step_x, filter_step_y;
	unsigned short TempFilter1,TempFilter2,TempFilter3,TempFilter4;
	unsigned short l_coor_filter = g_filter_max;

	if (g_filter_max > (PANEL_reso_x/g_coor_filter_mula))
		g_filter_max = PANEL_reso_x/g_coor_filter_mula;

	TempFilter1 = g_filter_max/3;
	TempFilter2 = g_filter_max*2;
	TempFilter3 = g_filter_max*3;
	TempFilter4 = g_filter_max*4;
#endif

	x_edge = PANEL_reso_x/g_edge_x_mul;
	y_edge = PANEL_reso_y/g_edge_y_mul;
	x_dis  = PANEL_reso_x/g_xdis_mula;
	y_dis  = PANEL_reso_y/g_ydis_mula;

	for (i = 0; i < 2; i++)
	{
		Coor_Bit = 1<<i;

		if ((G_structPosInfor.AveCoorX[i]!= 0) || (G_structPosInfor.AveCoorY[i]!= 0))
		{

			G_structPosInfor.LastCoorX[i] = G_structPosInfor.NewPointX[i];
			G_structPosInfor.LastCoorY[i] = G_structPosInfor.NewPointY[i];
			G_structPosInfor.AddCoorCnt[i] = 0;

			tempx[i] = G_structPosInfor.AveCoorX[i];
			tempy[i] = G_structPosInfor.AveCoorY[i];

			if ((g_ucFrameCountFlag & Coor_Bit) != 0)
			{
				tempx[i] = ((unsigned long)tempx[i]*(g_coor_weight-l_coor_filter)+((unsigned long)G_structPosInfor.LastCoorX[i]*l_coor_filter))/g_coor_weight;
				tempy[i] = ((unsigned long)tempy[i]*(g_coor_weight-l_coor_filter)+((unsigned long)G_structPosInfor.LastCoorY[i]*l_coor_filter))/g_coor_weight;
			}

#if FILTER_HANDLE

			filter_step_x = 0;
			filter_step_y = 0;

			if ((g_ucFrameCountFlag & Coor_Bit) == 0)
			{
				g_ucFrameCountFlag |= Coor_Bit;
			}
			else
			{
				x_err = tempx[i] > G_structPosInfor.LastCoorX[i] ?
				        (tempx[i] - G_structPosInfor.LastCoorX[i]):(G_structPosInfor.LastCoorX[i] - tempx[i]);

				y_err = tempy[i] > G_structPosInfor.LastCoorY[i] ?
				        (tempy[i] - G_structPosInfor.LastCoorY[i]):(G_structPosInfor.LastCoorY[i] - tempy[i]);

				if ( (x_err > g_filter_max && y_err > TempFilter1) ||
				        (x_err > TempFilter1 && y_err > g_filter_max) )
				{
					filter_step_x = x_err;
					filter_step_y = y_err;
				}
				else
				{
					if (x_err > g_filter_max)
						filter_step_x = x_err;
					if (y_err > g_filter_max)
						filter_step_y = y_err;
				}

				if (x_err <= TempFilter2 && y_err <= TempFilter2)
				{
					filter_step_x = filter_step_x>>2;
					filter_step_y = filter_step_y>>2;
				}
				else if (x_err <= TempFilter3 && y_err <= TempFilter3)
				{
					filter_step_x = filter_step_x>>1;
					filter_step_y = filter_step_y>>1;
				}
				else if (x_err <= TempFilter4 && y_err <= TempFilter4)
				{
					filter_step_x = filter_step_x*3/4;
					filter_step_y = filter_step_y*3/4;
				}
#if TREND_HANDLE

				if (slopFlag[i]==1)
				{
					tempx[i] = tempx[i] > G_structPosInfor.LastCoorX[i] ?
					           (G_structPosInfor.LastCoorX[i] + filter_step_x) : (G_structPosInfor.LastCoorX[i] - filter_step_x);

					tempy[i] = tempy[i] > G_structPosInfor.LastCoorY[i] ?
					           (G_structPosInfor.LastCoorY[i] + filter_step_y) : (G_structPosInfor.LastCoorY[i] - filter_step_y);
				}
				else
				{
					if ((xMoveFlag[i]==0))
					{
						tempx[i] = tempx[i] > G_structPosInfor.LastCoorX[i] ?
						           (G_structPosInfor.LastCoorX[i] + filter_step_x) : (G_structPosInfor.LastCoorX[i] - filter_step_x);
					}
					else
					{
						//tempx[i]=(((unsigned long)tempx[i]*(16-g_xLastweight))+((unsigned long)G_structPosInfor.LastCoorX[i]*g_xLastweight))>>4;
						x_err = abs1(tempy[i] - G_structPosInfor.LastCoorY[i]);

						if (x_err > g_xweight_dis)
						{
							x_err = g_xLastweight+x_err/g_xweight_dis;
							if (x_err >= 98)
								x_err = 98;
						}
						else
						{
							x_err = g_xLastweight;
						}

						tempx[i]=((unsigned long)tempx[i]*(100-x_err)+(unsigned long)G_structPosInfor.LastCoorX[i]*x_err)/100;
					}

					if ((yMoveFlag[i]==0))
					{
						tempy[i] = tempy[i] > G_structPosInfor.LastCoorY[i] ?
						           (G_structPosInfor.LastCoorY[i] + filter_step_y) : (G_structPosInfor.LastCoorY[i] - filter_step_y);
					}
					else
					{
						//tempy[i]=(((unsigned long)tempy[i]*(16-g_yLastweight))+((unsigned long)G_structPosInfor.LastCoorY[i]*g_yLastweight))>>4;
						y_err = abs1(tempx[i] - G_structPosInfor.LastCoorX[i]);

						if (y_err > g_yweight_dis)
						{
							y_err = g_yLastweight+y_err/g_yweight_dis;
							if (y_err >= 98)
								y_err = 98;
						}
						else
						{
							y_err = g_yLastweight;
						}

						tempy[i]=((unsigned long)tempy[i]*(100-y_err)+(unsigned long)G_structPosInfor.LastCoorY[i]*y_err)/100;
					}
				}
#else

				tempx[i] = tempx[i] > G_structPosInfor.LastCoorX[i] ?
				           (G_structPosInfor.LastCoorX[i] + filter_step_x) : (G_structPosInfor.LastCoorX[i] - filter_step_x);

				tempy[i] = tempy[i] > G_structPosInfor.LastCoorY[i] ?
				           (G_structPosInfor.LastCoorY[i] + filter_step_y) : (G_structPosInfor.LastCoorY[i] - filter_step_y);
#endif
			}

#endif
#if SEND_HANDLE
			if ((g_ucFrameSendFlag & Coor_Bit)== 0)
			{
				g_ucFrameSendFlag |= Coor_Bit;
				G_structPosInfor.NewPointX[i] = tempx[i];
				G_structPosInfor.NewPointY[i] = tempy[i];
#if TREND_HANDLE == 0
				G_structPosInfor.LastNewPointX[i] = tempx[i];
				G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
			}
			else
			{
				tempx[i] = (G_structPosInfor.LastNewPointX[i] + tempx[i])>>1;
				tempy[i] = (G_structPosInfor.LastNewPointY[i] + tempy[i])>>1;

				x_err = tempx[i] > G_structPosInfor.LastNewPointX[i] ?
				        (tempx[i] - G_structPosInfor.LastNewPointX[i]):(G_structPosInfor.LastNewPointX[i] - tempx[i]);

				y_err = tempy[i] > G_structPosInfor.LastNewPointY[i] ?
				        (tempy[i] - G_structPosInfor.LastNewPointY[i]):(G_structPosInfor.LastNewPointY[i] - tempy[i]);

				if ((g_ucFrameShakeFlag & Coor_Bit)== 0)
				{
					if ( (x_err > g_shake_ffdis && y_err > g_shake_fsdis) ||
					        (x_err > g_shake_fsdis && y_err > g_shake_ffdis) )
					{
						G_structPosInfor.NewPointX[i] = tempx[i];
						G_structPosInfor.NewPointY[i] = tempy[i];
#if TREND_HANDLE == 0
						G_structPosInfor.LastNewPointX[i] = tempx[i];
						G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
						g_ucFrameShakeFlag |= Coor_Bit;
					}
					else
					{
						if (x_err > g_shake_ffdis)
						{
							G_structPosInfor.NewPointX[i] = tempx[i];
#if TREND_HANDLE == 0
							G_structPosInfor.LastNewPointX[i] = tempx[i];
#endif
							g_ucFrameShakeFlag |= Coor_Bit;
						}
#if TREND_HANDLE == 0
						else
						{
							G_structPosInfor.NewPointX[i] = G_structPosInfor.LastNewPointX[i];
						}
#endif

						if (y_err > g_shake_ffdis)
						{
							G_structPosInfor.NewPointY[i] = tempy[i];
#if TREND_HANDLE == 0
							G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
							g_ucFrameShakeFlag |= Coor_Bit;
						}
#if TREND_HANDLE == 0
						else
						{
							G_structPosInfor.NewPointY[i] = G_structPosInfor.LastNewPointY[i];
						}
#endif
					}
				}
				else
				{

					if ( (x_err > g_shake_fdis && y_err > g_shake_sdis) ||
					        (x_err > g_shake_sdis && y_err > g_shake_fdis) )
					{
						G_structPosInfor.NewPointX[i] = tempx[i];
						G_structPosInfor.NewPointY[i] = tempy[i];
#if TREND_HANDLE == 0
						G_structPosInfor.LastNewPointX[i] = tempx[i];
						G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
					}
					else
					{
						if (x_err > g_shake_fdis)
						{
							G_structPosInfor.NewPointX[i]     = tempx[i];
#if TREND_HANDLE == 0
							G_structPosInfor.LastNewPointX[i] = tempx[i];
#endif
						}
#if TREND_HANDLE == 0
						else
						{
							G_structPosInfor.NewPointX[i] = G_structPosInfor.LastNewPointX[i];
						}
#endif

						if (y_err > g_shake_fdis)
						{
							G_structPosInfor.NewPointY[i]     = tempy[i];
#if TREND_HANDLE == 0
							G_structPosInfor.LastNewPointY[i] = tempy[i];
#endif
						}
#if TREND_HANDLE == 0
						else
						{
							G_structPosInfor.NewPointY[i] = G_structPosInfor.LastNewPointY[i];
						}
#endif
					}
				}
			}
#else
			G_structPosInfor.NewPointX[i] = tempx[i];
			G_structPosInfor.NewPointY[i] = tempy[i];
#endif
#if TREND_HANDLE
			x_err = PANEL_reso_x>>4;

			if (xMoveFlag[i]==1)
			{
				if (G_structPosInfor.NewPointX[i]>NewPointXTempTemp[i])
				{
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				}
				else if ((NewPointXTempTemp[i]-G_structPosInfor.NewPointX[i])>x_err)
				{
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				}
				G_structPosInfor.NewPointX[i]=NewPointXTempTemp[i];
			}
			if (xMoveFlag[i]==2)
			{
				if (G_structPosInfor.NewPointX[i]<NewPointXTempTemp[i])
				{
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				}
				else if ((G_structPosInfor.NewPointX[i]-NewPointXTempTemp[i])>x_err)
				{
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				}
				G_structPosInfor.NewPointX[i]=NewPointXTempTemp[i];
			}

			if (yMoveFlag[i]==1)
			{
				if (G_structPosInfor.NewPointY[i]<NewPointYTempTemp[i])
				{
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				else if ((G_structPosInfor.NewPointY[i]-NewPointYTempTemp[i])>x_err)
				{
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				G_structPosInfor.NewPointY[i]=NewPointYTempTemp[i];
			}
			if (yMoveFlag[i]==2)
			{
				if (G_structPosInfor.NewPointY[i]>NewPointYTempTemp[i])
				{
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				else if ((NewPointYTempTemp[i]-G_structPosInfor.NewPointY[i])>x_err)
				{
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				G_structPosInfor.NewPointY[i]=NewPointYTempTemp[i];
			}


			if ((NewPointXTemp[i]!=0)||(NewPointYTemp[i]!=0))
			{
				if ((abs1(NewPointXTemp[i]-G_structPosInfor.NewPointX[i])>x_dis)&&
				        (abs1(NewPointYTemp[i]-G_structPosInfor.NewPointY[i])<y_dis))
				{
					if (G_structPosInfor.NewPointX[i]>NewPointXTemp[i])
					{
						xMoveFlag[i]=1;
					}
					else
					{
						xMoveFlag[i]=2;
					}
					yMoveFlag[i]=0;
					NewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];

				}
				if ((abs1(NewPointYTemp[i]-G_structPosInfor.NewPointY[i])>y_dis)&&
				        (abs1(NewPointXTemp[i]-G_structPosInfor.NewPointX[i])<x_dis))
				{
					if (G_structPosInfor.NewPointY[i]>NewPointYTemp[i])
					{
						yMoveFlag[i]=2;
					}
					else
					{
						yMoveFlag[i]=1;
					}
					xMoveFlag[i]=0;
					NewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				}
				if ((abs1(NewPointXTemp[i]-G_structPosInfor.NewPointX[i])>x_dis)&&
				        (abs1(NewPointYTemp[i]-G_structPosInfor.NewPointY[i])>y_dis))
				{
					slopFlag[i]=1;
					xMoveFlag[i]=0;
					yMoveFlag[i]=0;
				}
			}
			G_structPosInfor.LastNewPointX[i]=G_structPosInfor.NewPointX[i];
			G_structPosInfor.LastNewPointY[i]=G_structPosInfor.NewPointY[i];


			//GestureJudge(i);

			NewPointXTemp[i] = G_structPosInfor.NewPointX[i];
			NewPointYTemp[i] = G_structPosInfor.NewPointY[i];
#endif
			if ((saveNewPointXTempTemp[i]==0)&&(saveNewPointYTempTemp[i]==0))
			{
				saveNewPointXTempTemp[i]=G_structPosInfor.NewPointX[i];
				saveNewPointYTempTemp[i]=G_structPosInfor.NewPointY[i];
				xCompensateFlag[i]=0;
				yCompensateFlag[i]=0;
				xLongCompensateFlag[i]=0;
				yLongCompensateFlag[i]=0;
			}
			else if ((saveNewPointXTemp[i]==0)&&(saveNewPointYTemp[i]==0))
			{
				if ((abs1(saveNewPointXTempTemp[i]-G_structPosInfor.NewPointX[i])>g_distance)||
				        (abs1(saveNewPointYTempTemp[i]-G_structPosInfor.NewPointY[i])>g_distance))
				{
					saveNewPointXTemp[i]=G_structPosInfor.NewPointX[i];
					saveNewPointYTemp[i]=G_structPosInfor.NewPointY[i];
					compensateFlag[i]=1;
					compensateCount[i]=0;
				}
			}
			else
			{
				if ((abs1(saveNewPointXTemp[i]-G_structPosInfor.NewPointX[i])>g_distance)||
				        (abs1(saveNewPointYTemp[i]-G_structPosInfor.NewPointY[i])>g_distance))
				{
					saveNewPointXTempTemp[i]=saveNewPointXTemp[i];
					saveNewPointYTempTemp[i]=saveNewPointYTemp[i];
					saveNewPointXTemp[i]=G_structPosInfor.NewPointX[i];
					saveNewPointYTemp[i]=G_structPosInfor.NewPointY[i];
					holdCount[i]=0;
				}
				else
				{
					if (holdCount[i]<g_hold_frame)
					{
						holdCount[i]++;
					}
				}

			}
			//*/
			if (compensateFlag[i]==1)
			{
				dx1[i]=abs1(saveNewPointXTempTemp[i]-saveNewPointXTemp[i]);
				dy1[i]=abs1(saveNewPointYTempTemp[i]-saveNewPointYTemp[i]);
			}
		}
		else
		{
			if (compensateFlag[i]==1)
			{
				if (compensateCount[i]<2)
				{
					ucTempFingerNum = g_ucFingerNum;

					if ((saveNewPointXTemp[i]<x_edge)&&(saveNewPointXTempTemp[i]>=saveNewPointXTemp[i]))
					{
						
						if (dx1[i]>2)
						{
							saveNewPointX[i]=0;
							///*
							pa1=saveNewPointYTemp[i];
							pa2=0;
							pa3=saveNewPointXTemp[i];
							pa4=saveNewPointYTempTemp[i];
							pa5=saveNewPointXTempTemp[i];
							saveNewPointY[i]=addPoint();
							//*/
							if (abs(saveNewPointY[i]-saveNewPointYTemp[i])<=(dy1[i]*4))
							{
								if (saveNewPointY[i] < 0)
								{
									saveNewPointY[i] = 0;
								}
								if (saveNewPointY[i] > PANEL_reso_y)
								{
									saveNewPointY[i] = PANEL_reso_y;
								}
								xCompensateFlag[i]=1;
								G_structPosInfor.NewPointX[i]=saveNewPointX[i];
								G_structPosInfor.NewPointY[i]=saveNewPointY[i];
								g_ucFingerNum++;
							}
							//printk("saveNewPointY[i] = %d\n", saveNewPointY[i]);
						}
						compensateCount[i]=2;
					}
					else if ((saveNewPointXTemp[i])>(PANEL_reso_x-x_edge)&&(saveNewPointXTempTemp[i]<=saveNewPointXTemp[i]))/////////
					{
						
						if (dx1[i]>2)
						{
							saveNewPointX[i]=PANEL_reso_x;
							///*
							pa1=saveNewPointYTemp[i];
							pa2=PANEL_reso_x;
							pa3=saveNewPointXTemp[i];
							pa4=saveNewPointYTempTemp[i];
							pa5=saveNewPointXTempTemp[i];
							saveNewPointY[i]=addPoint();
							///*/
							if (abs(saveNewPointY[i]-saveNewPointYTemp[i])<=(dy1[i]*4))
							{
								if (saveNewPointY[i] < 0)
								{
									saveNewPointY[i] = 0;
								}
								if (saveNewPointY[i] > PANEL_reso_y)
								{
									saveNewPointY[i] = PANEL_reso_y;
								}
								xCompensateFlag[i]=1;
								G_structPosInfor.NewPointX[i]=saveNewPointX[i];
								G_structPosInfor.NewPointY[i]=saveNewPointY[i];
								g_ucFingerNum++;
							}
							//printk("saveNewPointY[i] = %d\n", saveNewPointY[i]);
						}
						compensateCount[i]=2;
					}
					else
					{
						if (holdCount[i]>=g_hold_frame)////////////
						{
							xCompensateFlag[i]=0;
						}
						else
						{
							saveNewPointX[i]=saveNewPointXTemp[i]>saveNewPointXTempTemp[i]?(saveNewPointXTemp[i]+dx1[i]*g_dis_mul):(saveNewPointXTemp[i]-dx1[i]*g_dis_mul);//////////
							
							xCompensateFlag[i]=1;
							if (saveNewPointX[i]<0)
							{
								compensateCount[i]=2;
								///*
								pa1=saveNewPointYTemp[i];
								pa2=0;
								pa3=saveNewPointXTemp[i];
								pa4=saveNewPointYTempTemp[i];
								pa5=saveNewPointXTempTemp[i];
								saveNewPointY[i]=addPoint();
								///*/
								if (saveNewPointY[i] < 0)
								{
									saveNewPointY[i] = 0;
								}

								if (saveNewPointY[i] > PANEL_reso_y)
								{
									saveNewPointY[i] = PANEL_reso_y;
								}

								G_structPosInfor.NewPointY[i]=saveNewPointY[i];
								saveNewPointX[i]=0;
								//printk("saveNewPointY[i] = %d\n", saveNewPointY[i]);

							}
							if (saveNewPointX[i]>PANEL_reso_x)
							{
								compensateCount[i]=2;
								///*
								pa1=saveNewPointYTemp[i];
								pa2=PANEL_reso_x;
								pa3=saveNewPointXTemp[i];
								pa4=saveNewPointYTempTemp[i];
								pa5=saveNewPointXTempTemp[i];
								saveNewPointY[i]=addPoint();
								///*/
								if (saveNewPointY[i] < 0)
								{
									saveNewPointY[i] = 0;
								}

								if (saveNewPointY[i] > PANEL_reso_y)
								{
									saveNewPointY[i] = PANEL_reso_y;
								}

								G_structPosInfor.NewPointY[i]=saveNewPointY[i];
								saveNewPointX[i]=PANEL_reso_x;
								//printk("saveNewPointY[i] = %d\n", saveNewPointY[i]);

							}
							G_structPosInfor.NewPointX[i]=saveNewPointX[i];
							g_ucFingerNum++;
							if (dx1[i]<g_fast_distance)
							{
								xLongCompensateFlag[i]=1;
							}
							else
							{
								xLongCompensateFlag[i]=0;								
							}
						}
					}
					if (compensateCount[i]<2)
					{
						if ((saveNewPointYTemp[i]<y_edge)&&(saveNewPointYTempTemp[i]>=saveNewPointYTemp[i]))
						{
							
							if(dy1[i]>(2))
							{
								saveNewPointY[i]=0;
								///*
								pa1=saveNewPointXTemp[i];
								pa2=0;
								pa3=saveNewPointYTemp[i];
								pa4=saveNewPointXTempTemp[i];
								pa5=saveNewPointYTempTemp[i];
								saveNewPointX[i]=addPoint();
								//*/
								if(abs(saveNewPointX[i]-saveNewPointXTemp[i])<=(dx1[i]*4))
								{
									if (saveNewPointX[i] < 0)
									{
										saveNewPointX[i] = 0;
									}
									if (saveNewPointX[i] > PANEL_reso_x)
									{
										saveNewPointX[i] = PANEL_reso_x;
									}
									yCompensateFlag[i]=1;
									G_structPosInfor.NewPointX[i]=saveNewPointX[i];
									G_structPosInfor.NewPointY[i]=saveNewPointY[i];
									if(xCompensateFlag[i]==0)g_ucFingerNum++;
								}
								//printk("saveNewPointX[i] = %d\n", saveNewPointX[i]);
							}
							compensateCount[i]=2;
						}
						else if ((saveNewPointYTemp[i])>(PANEL_reso_y-y_edge)&&(saveNewPointYTempTemp[i]<=saveNewPointYTemp[i]))
						{
							
							if(dy1[i]>(2))
							{
								saveNewPointY[i]=PANEL_reso_y;
								//saveNewPointX[i]=saveNewPointXTemp[i]+(long)(PANEL_reso_y-saveNewPointYTemp[i])*(saveNewPointXTempTemp[i]-saveNewPointXTemp[i])/(saveNewPointYTempTemp[i]-saveNewPointYTemp[i]);
								///*
								pa1=saveNewPointXTemp[i];
								pa2=PANEL_reso_y;
								pa3=saveNewPointYTemp[i];
								pa4=saveNewPointXTempTemp[i];
								pa5=saveNewPointYTempTemp[i];
								saveNewPointX[i]=addPoint();
								//*/
								if(abs(saveNewPointX[i]-saveNewPointXTemp[i])<=(dx1[i]*4))
								{
									if (saveNewPointX[i] < 0)
									{
										saveNewPointX[i] = 0;
									}

									if (saveNewPointX[i] > PANEL_reso_x)
									{
										saveNewPointX[i] = PANEL_reso_x;
									}
									yCompensateFlag[i]=1;
									G_structPosInfor.NewPointX[i]=saveNewPointX[i];
									G_structPosInfor.NewPointY[i]=saveNewPointY[i];
									if(xCompensateFlag[i]==0)g_ucFingerNum++;
								}
								//printk("saveNewPointX[i] = %d\n", saveNewPointX[i]);
							}
							compensateCount[i]=2;
						}
						else
						{
							if (holdCount[i]>=g_hold_frame)
							{
								if (xCompensateFlag[i]==0)
								{
									compensateCount[i]=2;
								}
							}
							else
							{
								saveNewPointY[i]=saveNewPointYTemp[i]>saveNewPointYTempTemp[i]?(saveNewPointYTemp[i]+dy1[i]*g_dis_mul):(saveNewPointYTemp[i]-dy1[i]*g_dis_mul);/////////////////
								
								yCompensateFlag[i]=1;
								if (saveNewPointY[i]<0)
								{
									compensateCount[i]=2;
									///*
									pa1=saveNewPointXTemp[i];
									pa2=0;
									pa3=saveNewPointYTemp[i];
									pa4=saveNewPointXTempTemp[i];
									pa5=saveNewPointYTempTemp[i];
									saveNewPointX[i]=addPoint();
									///*/
									if (saveNewPointX[i] < 0)
									{
										saveNewPointX[i] = 0;
									}

									if (saveNewPointX[i] > PANEL_reso_x)
									{
										saveNewPointX[i] = PANEL_reso_x;
									}

									G_structPosInfor.NewPointX[i]=saveNewPointX[i];
									saveNewPointY[i]=0;

									//printk("saveNewPointX[i] = %d\n", saveNewPointX[i]);

								}
								if (saveNewPointY[i]>PANEL_reso_y)
								{
									compensateCount[i]=2;
									///*
									pa1=saveNewPointXTemp[i];
									pa2=PANEL_reso_y;
									pa3=saveNewPointYTemp[i];
									pa4=saveNewPointXTempTemp[i];
									pa5=saveNewPointYTempTemp[i];
									saveNewPointX[i]=addPoint();
									//*/
									if (saveNewPointX[i] < 0)
									{
										saveNewPointX[i] = 0;
									}

									if (saveNewPointX[i] > PANEL_reso_x)
									{
										saveNewPointX[i] = PANEL_reso_x;
									}
									G_structPosInfor.NewPointX[i]=saveNewPointX[i];
									saveNewPointY[i]=PANEL_reso_y;

									//printk("saveNewPointX[i] = %d\n", saveNewPointX[i]);

								}
								G_structPosInfor.NewPointY[i]=saveNewPointY[i];
								if(xCompensateFlag[i]==0)g_ucFingerNum++;
								
								if (dy1[i]<g_fast_distance)
								{
									if (xLongCompensateFlag[i]==1)
									{
										compensateCount[i]=2;
									}
								}
								else
								{
									yLongCompensateFlag[i]=0;									
								}
							}
						}
					}
					if(ucTempFingerNum == g_ucFingerNum)
					{
						G_structPosInfor.NewPointX[i]=0;
						G_structPosInfor.NewPointY[i]=0;
					}
					compensateCount[i]++;
					saveNewPointXTempTemp[i]=saveNewPointXTemp[i];
					saveNewPointXTemp[i]=G_structPosInfor.NewPointX[i];
					saveNewPointYTempTemp[i]=saveNewPointYTemp[i];
					saveNewPointYTemp[i]=G_structPosInfor.NewPointY[i];
				}
				else
				{
					G_structPosInfor.NewPointX[i] = 0;
					G_structPosInfor.NewPointY[i] = 0;
					saveNewPointXTempTemp[i]=0;
					saveNewPointYTempTemp[i]=0;
					saveNewPointXTemp[i]=0;
					saveNewPointYTemp[i]=0;
					compensateCount[i]=0;
					compensateFlag[i]=0;
				}
			}
			else
			{
				G_structPosInfor.NewPointX[i] = 0;
				G_structPosInfor.NewPointY[i] = 0;
				saveNewPointXTempTemp[i]=0;
				saveNewPointYTempTemp[i]=0;
				saveNewPointXTemp[i]=0;
				saveNewPointYTemp[i]=0;
				compensateCount[i]=0;
				compensateFlag[i]=0;
				dx1[i]=0;
				dy1[i]=0;
			}
			G_structPosInfor.LastCoorX[i] = 0;
			G_structPosInfor.LastCoorY[i] = 0;

			G_structPosInfor.LastNewPointX[i] = 0;
			G_structPosInfor.LastNewPointY[i] = 0;

#if TREND_HANDLE
			xMoveFlag[i]=0;
			yMoveFlag[i]=0;
			slopFlag[i]=0;
//			firstComFlag[i]=0;
//			firstMoveTouch[i]=1;
			NewPointXTemp[i] = 0;
			NewPointYTemp[i] = 0;
			NewPointXTempTemp[i]=0;
			NewPointYTempTemp[i]=0;
#endif
			g_ucFrameCountFlag &= ~Coor_Bit;
			g_ucFrameSendFlag &= ~Coor_Bit;
			g_ucFrameShakeFlag &= ~Coor_Bit;
		}

	}

}

/*******************************************************************************
* 函数名称：void InitBuffer(uint32_t value, uint8_t *buf_coor)
* 功能描述：将IIC的buf清0
*******************************************************************************/
void InitBuffer(unsigned int value)
{
	unsigned int i;
	for (i = 0; i < sizeof(buf_coor); i++)
	{
		buf_coor[i] = value;
	}
}
/*******************************************************************************
* 函数名称：void CoorComunicateFormat(uint32_t touchnum)
* 功能描述：坐标通信格式
* 输入参数：touchnum:有效触摸点数，抬起时其值为0xff
*******************************************************************************/
void CoorComunicateFormat(unsigned int touchnum)
{
	unsigned int i, k = 0, n = 0;

	unsigned char Coord_Bit = 0;

	if (touchnum == 0xff)
	{
		InitBuffer(0x00);
		buf_coor[0] = 0x80;
		g_ucFirstTouchFlag = 0;
	}
	else
	{
		buf_coor[0] = 0x00;

		for (i = 0,k = 1; i < 2; i++)
		{
			Coord_Bit = 1<<i;

			ucTouchState[i] = 0;
			ucTouchId[i] = 0;

			if ((G_structPosInfor.NewPointX[i] != 0) || (G_structPosInfor.NewPointY[i] != 0))
			{
				n++;

				ucTouchId[i] = i+1;//id

				if ((g_ucFirstTouchFlag & Coord_Bit) == 0)//first touch
				{
					g_ucFirstTouchFlag |= Coord_Bit;

					ucTouchState[i] = 0xa0;//touchstate
				}
				else//一直触摸
				{
					ucTouchState[i] = 0x90;
				}
			}
			else
			{
				if ((g_ucFirstTouchFlag & Coord_Bit) == Coord_Bit)//有手指抬起
				{
					g_ucFirstTouchFlag &= ~Coord_Bit;
					ucTouchId[i] = i+1; //id
					ucTouchState[i] = 0xc0;//touchup
					n++;
				}
			}

			if (ucTouchState[i])
			{
				if (ucTouchState[i]!=0xc0)
				{
					buf_coor[k] = ((G_structPosInfor.NewPointX[i])>>8)|(ucTouchId[i]<<4);
					buf_coor[k+1] = G_structPosInfor.NewPointX[i];
					buf_coor[k+2] = (G_structPosInfor.NewPointY[i]>>8)|ucTouchState[i];
					buf_coor[k+3] = G_structPosInfor.NewPointY[i];
				}
				else
				{
					buf_coor[k] &= 0x0f;
					buf_coor[k] |= (ucTouchId[i]<<4);
					buf_coor[k+2] &= 0x0f;
					buf_coor[k+2] |= ucTouchState[i];
				}

				k+=4;
			}
		}

		if (k == 5)
		{
			buf_coor[5] = 0;
			buf_coor[6] = 0;
			buf_coor[7] = 0;
			buf_coor[8] = 0;
		}

		buf_coor[0] = n|0x80;

	}
}
/************************************************************************************
* 函数名称： int byd_ts_read_coor(struct i2c_client *client)
* 功能描述： 读取平均坐标，经过防抖得到最终坐标
************************************************************************************/
int byd_ts_read_coor(struct i2c_client *client){

	unsigned char i;
	int ret=0;
	
	unsigned char Para[3] = {0};

	ret = i2c_smbus_read_i2c_block_data(client, READ_PARA_BASEADDR, 2, Para);
	//Para[0] = READ_PARA_BASEADDR;
	//ret = i2c_read_bytes(client, Para, 1, Para, 2);
	// 确认读取数据正常
	if (ret <= 0) {
		return ret;
	}
	g_filter_max  = ((unsigned short)Para[0]<<8)+Para[1];

	g_ucFingerNum = buf[0] & 0x0f;

	for (i = 0; i < N; i++){
		G_structPosInfor.AveCoorX[i] = 0;
		G_structPosInfor.AveCoorY[i] = 0;
	}

	for (i = 0; i < N; i++){
		G_structPosInfor.AveCoorX[i] = ((unsigned short)buf[4*i+1]<<8) | buf[4*i+2];
		G_structPosInfor.AveCoorY[i] = ((unsigned short)buf[4*i+3]<<8) | buf[4*i+4];
	}

	if (g_sel_addpoint == 0)
		CoorJitFilter0();
	else if (g_sel_addpoint == 1)
		CoorJitFilter1();
	else
		CoorJitFilter2();


	if (g_ucFingerNum){
		sNoTouchSendCnt = g_touchup_cnt;
		CoorComunicateFormat(g_ucFingerNum);
	}
	else{
		if (sNoTouchSendCnt){

			if (sNoTouchSendCnt)
			{
				CoorComunicateFormat(g_ucFingerNum);//0xc0
			}
			else
			{
				CoorComunicateFormat(0xff);//0x80 00
			}
			sNoTouchSendCnt--;
		}
		else{

			ret = 0;//first touch throw points
		}
	}


	for (i=0;i<9;i++){
		buf[i] = buf_coor[i];
		//	TS_DBG("-----buf[%d]:  %x \n", i, buf_coor[i]);
	}
	return ret;
}


/************************************************************************************
* 函数名称： int byd_ts_read_IC_para(struct i2c_client *client)
* 功能描述： 读取IC内参数，供防抖算法使用
************************************************************************************/
int byd_ts_read_IC_para(struct i2c_client *client){

	unsigned char i,index,len = 8;
	unsigned char Para[READ_PARA_LEN] = {0};
	int ret = 0;
	unsigned char para_len = 0;

	unsigned int xy_res[2]={0,0};

	for (i = 0; i<N;i++)
	{
		G_structPosInfor.NewPointX[i] = 0;
		G_structPosInfor.NewPointY[i] = 0;
		G_structPosInfor.LastCoorX[i] = 0;
		G_structPosInfor.LastCoorY[i] = 0;
		G_structPosInfor.LastNewPointX[i] = 0;
		G_structPosInfor.LastNewPointY[i] = 0;
		G_structPosInfor.AddCoorCnt[i] = 0;
	}

/////////
	ret = byd_ts_get_resolution(client, xy_res);
	PANEL_reso_x = xy_res[0] - 1;
	PANEL_reso_y = xy_res[1] - 1;

	if (xy_res[0] < 1 || xy_res[1] < 1
	        || xy_res[0] > 2048 || xy_res[1] > 2048) {
		pr_err("%s: error resolution range: %d x %d !\n", __FUNCTION__,
		       xy_res[0], xy_res[1]);

	}
	printk(" PANEL_reso_x = %d,PANEL_reso_y = %d \n", PANEL_reso_x,PANEL_reso_y);

////////

	ret = i2c_smbus_read_i2c_block_data(client, READ_SELADDPOINT_ADDR, 1, &Para[0]);
	//Para[0] = READ_SELADDPOINT_ADDR;
	//ret = i2c_read_bytes(client, Para, 1, Para, 1);
	g_sel_addpoint = Para[0];

	printk(" g_sel_addpoint = %d\n", g_sel_addpoint);

/////
	//msleep(2);

	if (g_sel_addpoint)
	{
		if (g_sel_addpoint == 1)
			para_len = 21;
		else
			para_len = 25;
	}
	else
	{
		para_len = 18;
	}

	for (i = 0; i <= (para_len/len);i++)
	{
		index = (len*i);
		ret = i2c_smbus_read_i2c_block_data(client, READ_PARA_BASEADDR+index, len, &Para[index]);
	}
	//Para[0] = READ_PARA_BASEADDR;
	//ret = i2c_read_bytes(client, Para, 1, Para, para_len);

	if (ret <= 0) {

		g_filter_max  = X_COOR_FILTER_MAX;
		g_backfilter_max = g_filter_max;

		g_shake_ffdis = SHAKE_FFDIS;
		g_shake_fsdis = SHAKE_FSDIS;
		g_shake_fdis  = SHAKE_FDIS;
		g_shake_sdis  = SHAKE_SDIS;

		g_coor_filter_mula = COOR_FILTER_MULA;
		g_coor_weight      = COOR_WEIGHT;

		g_xdis_mula = X_DIS_MULA;
		g_ydis_mula = Y_DIS_MULA;
		g_xLastweight = X_LAST_WEIGHT;
		g_xweight_dis = X_WEIGHT_DIS;
		g_yLastweight = Y_LAST_WEIGHT;
		g_yweight_dis = Y_WEIGHT_DIS;

		if (g_sel_addpoint == 1)
		{
			g_addpoint_cnt     = ADDPOINT_CNT;
			g_coor_dis_mula    = COOR_DIS_MULA;
			g_edge_y_mul       = EDGE_Y_MUL;
		}
		else if (g_sel_addpoint == 2)
		{
			g_distance      = DISTANCE;
			g_fast_distance = FAST_DISTANCE;
			g_edge_x_mul    = EDGE_X_MUL;
			g_edge_y_mul    = EDGE_Y_MUL;
			g_dis_mul       = DIS_MUL;
			g_hold_frame    = HOLD_FRAME;
		}
		g_touchup_cnt = 2;
		return ret;
	}

	g_filter_max  = ((unsigned short)Para[0]<<8)+Para[1];
	if (g_filter_max > 2048)
		g_filter_max = X_COOR_FILTER_MAX;

	g_backfilter_max = g_filter_max;

	g_shake_ffdis = Para[2];
	g_shake_fsdis = Para[3];
	g_shake_fdis  = Para[4];
	g_shake_sdis  = Para[5];


	if (Para[6] > 0)
		g_coor_filter_mula = Para[6];
	else
		g_coor_filter_mula = COOR_FILTER_MULA;

	g_coor_weight = ((unsigned short)Para[7]<<8)+Para[8];
	if (g_coor_weight == 0)
		g_coor_weight  = COOR_WEIGHT;

	g_xdis_mula  = ((unsigned short)Para[9]<<8)+Para[10];
	if (g_xdis_mula == 0)
		g_xdis_mula  = X_DIS_MULA;

	g_ydis_mula  = ((unsigned short)Para[11]<<8)+Para[12];
	if (g_ydis_mula == 0)
		g_ydis_mula  = Y_DIS_MULA;

	g_xLastweight = Para[13];
	g_xweight_dis = Para[14];
	g_yLastweight = Para[15];
	g_yweight_dis = Para[16];

	printk(" g_filter_max = %d, g_shake_ffdis = %d,g_shake_fsdis = %d, g_shake_fdis = %d,g_shake_sdis = %d \n",
	       g_filter_max, g_shake_ffdis, g_shake_fsdis,g_shake_fdis,g_shake_sdis);

	printk(" g_coor_filter_mula = %d, g_coor_weight = %d \n",
	       g_coor_filter_mula, g_coor_weight);


	printk(" g_xdis_mula = %d,g_ydis_mula = %d,g_xLastweight = %d,g_xweight_dis = %d,g_yLastweight = %d,g_yweight_dis = %d \n",
	       g_xdis_mula,g_ydis_mula,g_xLastweight,g_xweight_dis,g_yLastweight,g_yweight_dis);

	g_touchup_cnt = 2;
    if (g_sel_addpoint == 0)
	{
		if (Para[17] > 0)
			g_touchup_cnt = Para[17];
	}
	else if (g_sel_addpoint == 1)
	{
		g_addpoint_cnt = Para[17];		

		if (Para[18] > 0)
			g_coor_dis_mula = Para[18];
		else
			g_coor_dis_mula = COOR_DIS_MULA;

		if (Para[19] > 0)
			g_edge_y_mul = Para[19];
		else
			g_edge_y_mul = EDGE_Y_MUL;
		
		if (Para[20] > 0)
			g_touchup_cnt = Para[20];
		
		printk(" g_addpoint_cnt = %d,g_coor_dis_mula = %d,g_edge_y_mul = %d \n", g_addpoint_cnt,g_coor_dis_mula,g_edge_y_mul);
	}
	else if (g_sel_addpoint == 2)
	{
		g_distance      = Para[17];
		g_fast_distance = ((unsigned short)Para[18]<<8)+Para[19];

		if (Para[20] > 0)
			g_edge_x_mul    = Para[20];
		else
			g_edge_x_mul    = EDGE_X_MUL;

		if (Para[21] > 0)
			g_edge_y_mul    = Para[21];
		else
			g_edge_y_mul    = EDGE_Y_MUL;

		g_dis_mul       = Para[22];
		g_hold_frame    = Para[23];
		
		if (Para[24] > 0)
			g_touchup_cnt = Para[24];

		printk(" g_distance = %d,g_fast_distance = %d,g_edge_x_mul = %d,g_edge_y_mul = %d,g_dis_mul = %d,g_hold_frame = %d \n",
		       g_distance,g_fast_distance,g_edge_x_mul,g_edge_y_mul,g_dis_mul,g_hold_frame);
	}

	printk(" g_touchup_cnt = %d \n", g_touchup_cnt);

	return ret;
}




