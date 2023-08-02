#include "Headfile.h"
#include "rgb.h"


Bling_Light light_red={
	.port=GPIO_PORTF_BASE,
	.pin=BLING_R_PIN,
};

Bling_Light light_green={
	.port=GPIO_PORTF_BASE,
	.pin=BLING_G_PIN,
};

Bling_Light light_blue={
	.port=GPIO_PORTF_BASE,
	.pin=BLING_B_PIN,
};

void rgb_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);	
	GPIOPinWrite(GPIO_PORTF_BASE,BLING_B_PIN,BLING_B_PIN);//蓝色
	GPIOPinWrite(GPIO_PORTF_BASE,BLING_R_PIN,BLING_R_PIN);//红色
	GPIOPinWrite(GPIO_PORTF_BASE,BLING_G_PIN,BLING_G_PIN);//绿色
	
	// rgb_start_bling();            //开机LED预显示
}


/***************************************************
函数名: void Bling_Set(Bling_Light *Light,
uint32_t Continue_time,//持续时间
uint16_t Period,//周期100ms~1000ms
float Percent,//0~100%
uint16_t  Cnt,
GPIO_TypeDef* Port,
uint16_t Pin
,uint8_t Flag)
说明:	状态指示灯设置函数
入口:	时间、周期、占空比、端口等
出口:	无
备注:	程序初始化后、始终运行
作者:	无名创新
****************************************************/
void bling_set(Bling_Light *light,
               uint32_t Continue_time,//持续时间
               uint16_t Period,//周期100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               uint8_t Flag)
{
  light->contiune_t=(uint16_t)(Continue_time/5);//持续时间
  light->period=Period;//周期
  light->percent=Percent;//占空比
  light->cnt=Cnt;
  //light->port=Port;//端口
  //light->pin=Pin;//引脚
  light->endless_flag=Flag;//无尽模式
}

/***************************************************
函数名: void Bling_Process(Bling_Light *Light)//闪烁运行线程
说明:	状态指示灯实现
入口:	状态灯结构体
出口:	无
备注:	程序初始化后、始终运行
作者:	无名创新
****************************************************/
void bling_process(Bling_Light *light)//闪烁运行线程
{
  if(light->contiune_t>=1)  light->contiune_t--;
  else GPIOPinWrite(light->port,light->pin,light->pin);//置高,亮
  if(light->contiune_t!=0//总时间未清0
     ||light->endless_flag==1)//判断无尽模式是否开启
  {
    light->cnt++;
    if(5*light->cnt>=light->period) light->cnt=0;//计满清零
    if(5*light->cnt<=light->period*light->percent)
    GPIOPinWrite(light->port,light->pin,0);//置低，亮
    else GPIOPinWrite(light->port,light->pin,light->pin);//置高，灭
  }
}

void GPIO_SetBits(Bling_Light *light)
{
  GPIOPinWrite(light->port,light->pin,light->pin);//置高，灭
}

void GPIO_ResetBits(Bling_Light *light)
{
  GPIOPinWrite(light->port,light->pin,0);//置低，亮
}


/***************************************************
函数名: Bling_Working(uint16 bling_mode)
说明:	状态指示灯状态机
入口:	当前模式
出口:	无
备注:	程序初始化后、始终运行
作者:	无名创新
****************************************************/
void bling_working(uint16_t bling_mode)
{
  if(bling_mode==0)//全灭
  {
    bling_process(&light_blue);
    bling_process(&light_red);
    bling_process(&light_green);
  }
  else if(bling_mode==3)//全灭
  {
    GPIOPinWrite(GPIO_PORTF_BASE,BLING_B_PIN,BLING_B_PIN);
    GPIOPinWrite(GPIO_PORTF_BASE,BLING_R_PIN,BLING_R_PIN);
    GPIOPinWrite(GPIO_PORTF_BASE,BLING_G_PIN,BLING_G_PIN);
  } 
}


/***************************************
函数名:	void rgb_start_bling(void)
说明: 开机后初始闪烁
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void rgb_start_bling(void)
{
    bling_set(&light_red  ,1000,500,0.5,0,0);//红色
    bling_set(&light_green,1000,500,0.5,0,0);//绿色
    bling_set(&light_blue ,1000,500,0.5,0,0);//蓝色
}
