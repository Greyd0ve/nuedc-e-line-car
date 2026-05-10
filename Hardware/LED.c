/* 项目适配说明：PC13 LED 用作声光提示中的“光提示”。如果外接蜂鸣器，可在 main.c 的 Prompt_Start/Prompt_Tick1ms 中同步控制。 */
/*
 * 文件：Hardware/LED.c
 * 作用：板载 LED 控制，用于 H 题启动、过点、停车提示。
 *
 * 当前训练板未集成蜂鸣器，因此默认只做光提示。若后续外接有源蜂鸣器，
 * 可以在 main.c 的 Prompt_Start()/Prompt_Tick1ms() 中同步控制蜂鸣器 IO。
 *
 * 注释版说明：仅增加注释，未改变 LED 逻辑。
 */
#include "stm32f10x.h"                  // Device header

/**
  * 函    数：LED初始化
  * 参    数：无
  * 返 回 值：无
  */
void LED_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	//开启GPIOC的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);					//将PC13引脚初始化为推挽输出
	
	/*设置GPIO初始化后的默认电平*/
	GPIO_SetBits(GPIOC, GPIO_Pin_13);						//设置PC13引脚为高电平
}

/**
  * 函    数：LED开启
  * 参    数：无
  * 返 回 值：无
  */
void LED_ON(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);			//设置PC13引脚为低电平
}

/**
  * 函    数：LED关闭
  * 参    数：无
  * 返 回 值：无
  */
void LED_OFF(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_13);			//设置PC13引脚为高电平
}

/**
  * 函    数：LED状态翻转
  * 参    数：无
  * 返 回 值：无
  */
void LED_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) == 0)	//获取输出寄存器的状态，如果当前引脚输出低电平
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_13);					//设置PC13引脚为高电平
	}
	else													//否则，即当前引脚输出高电平
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);					//设置PC13引脚为低电平
	}
}
