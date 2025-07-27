#include "key.h"
int8_t Key_KeyNumber;
/**
  * @brief  获取当前按键的状态，无消抖及松手检测
  * @param  无
  * @retval 按下按键的键码，范围：0,1~4,0表示无按键按下
  */
unsigned char Key_GetState()
{
	unsigned char KeyNumber=0;
	
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==0){KeyNumber=1;}
//	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==0){KeyNumber=2;}
//	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)==0){KeyNumber=3;}
//	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==0){KeyNumber=4;}
	
	return KeyNumber;
}

/**
  * @brief  按键驱动函数，在中断中调用
  * @param  无
  * @retval 无
  */
void Key_Loop(void)
{
	static unsigned char NowState,LastState;//要定义成局部静态变量或定义成全局变量
	LastState=NowState;				//按键状态更新
	NowState=Key_GetState();		//获取当前按键状态
	//如果上个时间点按键按下，这个时间点未按下，则是松手瞬间，以此避免消抖和松手检测
	if(LastState==0 && NowState==1)
	{
		TIM3->CNT=0;
	}
	if(LastState==1 && NowState==0)
	{
		if(TIM3->CNT>=10000)
		{
			Key_KeyNumber=5;
		}
		else Key_KeyNumber=1;
	}
	if(LastState==0 && NowState==2)
	{
		TIM3->CNT=0;
	}
	if(LastState==2 && NowState==0)
	{
		if(TIM3->CNT>=10000)
		{
			Key_KeyNumber=6;
		}
		else Key_KeyNumber=2;
	}
	if(LastState==0 && NowState==3)
	{
		TIM3->CNT=0;
	}
	if(LastState==3 && NowState==0)
	{
		if(TIM3->CNT>=10000)
		{
			Key_KeyNumber=7;
		}
		else Key_KeyNumber=3;
	}
	if(LastState==0 && NowState==3)
	{
		TIM3->CNT=0;
	}
	if(LastState==4 && NowState==0)
	{
		if(TIM3->CNT>=10000)
		{
			Key_KeyNumber=8;
		}
		else Key_KeyNumber=4;
	}
}
/**
  * @brief  获取按键键码
  * @param  无
  * @retval 按下按键的键码，范围：0,1~4,0表示无按键按下
  */
unsigned char Key(void)
{
	unsigned char Temp=0;
	Temp=Key_KeyNumber;
	Key_KeyNumber=0;
	return Temp;
	
}

