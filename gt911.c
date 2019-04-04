#include "bcm2835.h"
#include <stdio.h>

typedef char u8;
typedef int u16;

#define TPINT 4
#define TPRES 14

#define GT911_RST_1() bcm2835_gpio_set(TPRES) // RST = 1
#define GT911_RST_0() bcm2835_gpio_clr(TPRES) // RST = 0

#define GT911_INT_1() bcm2835_gpio_set(TPINT) // INT = 1
#define GT911_INT_0() bcm2835_gpio_clr(TPINT) // INT = 0

//I2C读写命令
#define GT_CMD_WR 0X28 //写命令
#define GT_CMD_RD 0X29 //读命令

//GT9147 部分寄存器定义
#define GT_CTRL_REG 0X8040  //GT9147控制寄存器
#define GT_CFGS_REG 0X8047  //GT9147配置起始地址寄存器
#define GT_CHECK_REG 0X80FF //GT9147校验和寄存器
#define GT_PID_REG 0X8140   //GT9147产品ID寄存器

#define GT_GSTID_REG 0X814E //GT9147当前检测到的触摸情况
#define GT_TP1_REG 0X8150   //第一个触摸点数据地址
#define GT_TP2_REG 0X8158   //第二个触摸点数据地址
#define GT_TP3_REG 0X8160   //第三个触摸点数据地址
#define GT_TP4_REG 0X8168   //第四个触摸点数据地址
#define GT_TP5_REG 0X8170   //第五个触摸点数据地址

//一下未使用
#define GT911_READ_XY_REG 0x814E          /* 坐标寄存器 */
#define GT911_CLEARBUF_REG 0x814E         /* 清除坐标寄存器 */
#define GT911_CONFIG_REG 0x8047           /* 配置参数寄存器 */
#define GT911_COMMAND_REG 0x8040          /* 实时命令 */
#define GT911_PRODUCT_ID_REG 0x8140       /*productid*/
#define GT911_VENDOR_ID_REG 0x814A        /* 当前模组选项信息 */
#define GT911_CONFIG_VERSION_REG 0x8047   /* 配置文件版本号 */
#define GT911_CONFIG_CHECKSUM_REG 0x80FF  /* 配置文件校验码 */
#define GT911_FIRMWARE_VERSION_REG 0x8144 /* 固件版本号 */

#define LCD_WIDTH 800
#define LCD_HEIGHT 480

#define TP_PRES_DOWN 0x80 //触屏被按下
#define TP_CATH_PRES 0x40 //有按键按下了

#define CT_MAX_TOUCH 5

void delay_ms(int t);
void GT911_RD_Reg(int reg, char *buf, int len);
void GT911_WR_Reg(int reg, char *buf, int len);
u8 GT911_Send_Cfg(u8 mode); //修改
u8 GT911_Init(void);
u8 GT911_Scan(u8 mode);
void ctp_test(void);

const u8 GT911_CFG_TBL[] = //数组内容修改了
    {
        0x00, 0xE0, 0x01, 0x10, 0x01, 0x05, 0x3C, 0x00, 0x01, 0x08,
        0x14, 0x05, 0x55, 0x37, 0x03, 0x05, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x18, 0x1A, 0x1E, 0x14, 0x8A, 0x2A, 0x0D,
        0x24, 0x26, 0x31, 0x0D, 0x00, 0x00, 0x00, 0x9A, 0x03, 0x1D,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x64, 0x32, 0x00, 0x00,
        0x00, 0x19, 0x46, 0x94, 0xC5, 0x02, 0x07, 0x00, 0x00, 0x04,
        0xA5, 0x1B, 0x00, 0x86, 0x22, 0x00, 0x6F, 0x2A, 0x00, 0x5E,
        0x33, 0x00, 0x4F, 0x3F, 0x00, 0x4F, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10,
        0x12, 0x14, 0x16, 0x18, 0x1A, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0F, 0x10,
        0x12, 0x13, 0x16, 0x18, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21,
        0x22, 0x24, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xA7, 0x01};

char writeBuffer[4];
char readBuffer[4];

typedef struct
{
  //u8 (*init)(void);			//初始化触摸屏控制器
  u8 (*scan)(u8); //扫描触摸屏.0,屏幕扫描;1,物理坐标;
  //void (*adjust)(void);		//触摸屏校准
  u16 x[CT_MAX_TOUCH]; //当前坐标
  u16 y[CT_MAX_TOUCH]; //电容屏有最多5组坐标,电阻屏则用x[0],y[0]代表:此次扫描时,触屏的坐标,用
                       //x[4],y[4]存储第一次按下时的坐标.
  u8 sta;              //笔的状态
                       //b7:按下1/松开0;
                       //b6:0,没有按键按下;1,有按键按下.
                       //b5:保留
                       //b4~b0:电容触摸屏按下的点数(0,表示未按下,1表示按下)
                       /////////////////////触摸屏校准参数(电容屏不需要校准)//////////////////////
  float xfac;
  float yfac;
  short xoff;
  short yoff;
  //新增的参数,当触摸屏的左右上下完全颠倒时需要用到.
  //b0:0,竖屏(适合左右为X坐标,上下为Y坐标的TP)
  //   1,横屏(适合左右为Y坐标,上下为X坐标的TP)
  //b1~6:保留.
  //b7:0,电阻屏
  //   1,电容屏
  u8 touchtype;
} _m_tp_dev;

_m_tp_dev tp_dev =
    {
        //TP_Init,
        GT911_Scan, //TP_Scan,
        //TP_Adjust,
        0,
        0,
        0,
        0,

        0,
        0,
        0,
        0xff,
};

void delay_ms(int t)
{
  bcm2835_delay(t);
}

void GT911_RD_Reg(int reg, char *buf, int len)
{
  char addr[2] = {(char)(reg >> 8), (char)reg};

  bcm2835_i2c_write_read_rs(addr, 2, buf, len);
}

void GT911_WR_Reg(int reg, char *buf, int len)
{
  char addr[255];
  addr[0] = (char)(reg >> 8);
  addr[1] = (char)reg;

  for (int i = 0; i < len; i++)
  {
    addr[2 + i] = buf[i];
  }

  bcm2835_i2c_write(addr, len + 2);
}

u8 GT911_Send_Cfg(u8 mode) //修改
{
  u8 buf[2];
  u8 i = 0;
  buf[0] = 0;
  buf[1] = mode; //是否写入到GT911 FLASH?  即是否掉电保存
  for (i = 0; i < sizeof(GT911_CFG_TBL); i++)
    buf[0] += GT911_CFG_TBL[i]; //计算校验和
  buf[0] = (~buf[0]) + 1;
  GT911_WR_Reg(GT_CFGS_REG, (u8 *)GT911_CFG_TBL, sizeof(GT911_CFG_TBL)); //发送寄存器配置
  GT911_WR_Reg(GT_CHECK_REG, buf, 2);                                    //写入校验和,和配置更新标记
  return 0;
}

//初始化GT9147触摸屏
//返回值:0,初始化成功;1,初始化失败
u8 GT911_Init(void)
{
  u8 temp[4];

  //GPIO_Init(GT_GPIO_PORT, (GPIO_Pin_TypeDef)GT_INT, GPIO_MODE_IN_PU_NO_IT); //TODO
  bcm2835_gpio_fsel(TPINT, BCM2835_GPIO_FSEL_INPT);

  //GPIO_Init(GT_GPIO_PORT, (GPIO_Pin_TypeDef)GT_RST, GPIO_MODE_OUT_PP_LOW_FAST);
  bcm2835_gpio_fsel(TPRES, BCM2835_GPIO_FSEL_OUTP);

  //I2C_Initializes(); //初始化电容屏的I2C总线
  bcm2835_i2c_begin();
  bcm2835_i2c_setSlaveAddress(0x14);
  bcm2835_i2c_set_baudrate(10000);
  //GT_RST = 0;								 //复位
  GT911_RST_0();
  delay_ms(10);
  //GT_RST = 1; //释放复位
  GT911_RST_1();
  delay_ms(10);

  //GPIO_Init(GT_GPIO_PORT, (GPIO_Pin_TypeDef)GT_INT, GPIO_MODE_IN_FL_NO_IT); //TODO

  delay_ms(100);

  GT911_RD_Reg(GT_PID_REG, temp, 4); //读取产品ID

  if (temp[0] == 0x39 && temp[1] == 0x31 && temp[2] == 0x31 && temp[3] == 0x00) //ID==9147
  {
    temp[0] = 0X02;
    GT911_WR_Reg(GT_CTRL_REG, temp, 1); //软复位GT9147
    GT911_RD_Reg(GT_CFGS_REG, temp, 1); //读取GT_CFGS_REG寄存器
    //printf("读出版本:%x\r\n", temp[0]);
    if (temp[0] < 0X68) //默认版本比较低,需要更新flash配置
    {
      GT911_Send_Cfg(0);
    }

    delay_ms(10);
    temp[0] = 0X00;
    GT911_WR_Reg(GT_CTRL_REG, temp, 1); //结束复位
    return 0;
  }
  return 1;
}

const u16 GT911_TPX_TBL[5] = {GT_TP1_REG, GT_TP2_REG, GT_TP3_REG, GT_TP4_REG, GT_TP5_REG};

u8 GT911_Scan(u8 mode)
{
  u8 buf[4];
  u8 i = 0;
  u8 res = 0;
  u8 temp;
  u8 tempsta;

  GT911_RD_Reg(GT_GSTID_REG, &mode, 1); //读取触摸点的状态

  if ((mode & 0XF) && ((mode & 0XF) < 6))
  {
    temp = 0XFF << (mode & 0XF); //将点的个数转换为1的位数,匹配tp_dev.sta定义
    tempsta = tp_dev.sta;        //保存当前的tp_dev.sta值
    tp_dev.sta = (~temp) | TP_PRES_DOWN | TP_CATH_PRES;

    for (i = 0; i < 5; i++) //for(i=0;i<5;i++)
    {
      if (tp_dev.sta & (1 << i)) //触摸有效?
      {
        //读取XY坐标值
        GT911_RD_Reg(GT911_TPX_TBL[i], buf, 4);

        tp_dev.x[i] = ((u16)(buf[1] & 0X0F) << 8) + buf[0];
        tp_dev.y[i] = ((u16)(buf[3] & 0X0F) << 8) + buf[2];

        printf("GT911 ID: 0x%02X 0x%02X 0x%02X 0x%02X\n", buf[0], buf[1], buf[2], buf[3]);
      }
    }
    res = 1;
  }

  if (mode & 0X80 && ((mode & 0XF) < 6))
  {
    temp = 0;
    GT911_WR_Reg(GT_GSTID_REG, &temp, 1); //清标志
  }

  if ((mode & 0X8F) == 0X80) //无触摸点按下
  {
    if (tp_dev.sta & TP_PRES_DOWN) //之前是被按下的
    {
      tp_dev.sta &= ~(1 << 7); //标记按键松开
    }
    else //之前就没有被按下
    {
      tp_dev.x[0] = 0xffff;
      tp_dev.y[0] = 0xffff;
      tp_dev.sta &= 0XE0; //清除点有效标记
    }
  }
  //if (t > 240)
  //	t = 10; //重新从10开始计数
  //printf("运行到SCAN\r\n");
  return res;
}

void ctp_test(void)
{
  u16 lastpos[1][2]; //最后一次数据
  while (1)
  {
    tp_dev.scan(0);
    if ((tp_dev.sta) & 1) //判断是否有触摸点
    {
      if (tp_dev.x[0] < LCD_WIDTH && tp_dev.y[0] < LCD_HEIGHT) //在LCD范围内
      {
        lastpos[0][0] = tp_dev.x[0];
        lastpos[0][1] = tp_dev.y[0];
        //GUI_ShowNum1(263,250,lastpos[0][0],3,12,0);
        //GUI_ShowNum1(263,280,lastpos[0][1],3,12,0);
        //GUI_ShowNum1(tp_dev.x[0],tp_dev.y[0],tp_dev.x[0],3,12,0);
      }
    }
    else
      lastpos[0][0] = 0XFFFF;
  }
}

int main(void)
{
  printf("program start\n");
  if (!bcm2835_init())
  {
    printf("bcm2835_init failed. Are you running as root??\n");
    return 1;
  }

  GT911_Init();
  ctp_test();

  /*bcm2835_i2c_begin();
  bcm2835_i2c_setSlaveAddress(0x14);
  bcm2835_i2c_set_baudrate(10000);*/

  GT911_RD_Reg(0x8140, readBuffer, 4);
  printf("GT911 ID: 0x%02X 0x%02X 0x%02X 0x%02X\n", readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3]);

  while (1)
  {

    GT911_RD_Reg(0x8150, readBuffer, 4);
    printf("touch point 1: 0x%02X 0x%02X 0x%02X 0x%02X\n", readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3]);

    bcm2835_delay(500);
  }

  bcm2835_i2c_end();
  bcm2835_close();
}
