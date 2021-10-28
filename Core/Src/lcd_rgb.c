/***
	************************************************************************************************************************************************************************************************
	*	@file  	lcd_rgb.c
	*	@version V1.1
	*  @date    2020-10-13
	*	@author  反客科技
	*	@brief   驱动RGB显示屏进行显示	
   **********************************************************************************************************************************************************************************************
   *  @description
	*
	*	实验平台：反客STM32F429BIT6核心板(型号：FK429M1) + 800*480分辨率的RGB屏幕
	*	淘宝地址：https://shop212360197.taobao.com
	*	QQ交流群：536665479
	*
>>>>> V1.1 版本变更说明：
	*
	*	1. 增加汉字显示函数，基于小字库(只取模用到的汉字)，存储在单片机片内flash，
	*	2. 增加简单的图片显示函数，图片数据存储在单片机片内flash，这种方式只适用于单色图片
	*	3.	增加对 ARGB4444 的支持
	*	4. 刷屏时，不再开辟新的显存作为缓冲，而是改为判断寄存器，等待 垂直数据使能显示状态，有效避免刷屏时带来的撕裂
	*	5. 完善各个函数的注释，进一步简化程序
	*	6. 将多款800*480分辨率的屏幕整合到一起
	*
>>>>> 重要说明：
	*
	*	1. FK429M1 核心板 使用的是外部SDRAM作为显存，起始地址0xD0000000, SDRAM 大小为16M字节
	*	2. 不管是单层显示还是双层显示，都不能超过 SDRAM 的大小
	*	3. 在刚下载完程序时，屏幕有轻微抖动和闪烁属于正常现象，等待片刻或者重新上电即可恢复正常
	* 	4. LTDC时钟在 lcd_rgb.h 文件里的宏 LCD_CLK 设置，配置为33MHz，即刷新率在60帧左右，过高或者过低都会造成闪烁	
	*
>>>>> 其他说明：
	*
	*	1. 中文字库使用的是小字库，即用到了对应的汉字再去取模，用户可以根据需求自行增添或删减
	*	2. 各个函数的功能和使用可以参考函数的说明以及 lcd_test.c 文件里的测试函数
	*
	*********************************************************************************************************************************************************************************************FANKE*****
***/

#include "lcd_rgb.h"

LTDC_HandleTypeDef hltdc;	// LTDC_HandleTypeDef 结构体变量

static pFONT *LCD_Fonts;		// 英文字体
static pFONT *LCD_CHFonts;		// 中文字体

//LCD相关参数结构体
struct	
{
	uint32_t Color; 				//	LCD当前画笔颜色
	uint32_t BackColor;			//	背景色
	uint32_t ColorMode;			//	颜色格式
	uint32_t LayerMemoryAdd;	//	层显存地址
	uint8_t  BytesPerPixel;		//	每个像素所占字节数	
	uint8_t  Layer; 				//	当前层
	uint8_t  Direction;			//	显示方向
	uint8_t  ShowNum_Mode;		// 设置变量显示时多余位补0还是补空格
}LCD;

/*************************************************************************************************
*	函 数 名:	LCD_PanelModify
*	入口参数:	无
*	返 回 值:	无
*	函数功能:	旧面板识别修改，此函数只对旧款5寸屏有用，也就是 RGB050M1  V1.0的版本
*	说    明:	因为旧款5寸屏面板只引出了18位的接口，所以需要相应的把低2位的数据脚输出低电平,
*					其它屏幕无需理会
*************************************************************************************************/

void	LCD_PanelModify(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;	

	GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;				//	输出模式
	GPIO_InitStruct.Pull 	= GPIO_NOPULL;							//	无上下拉
	GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;				// 速度等级2M
	
	GPIO_InitStruct.Pin 		= LTDC_R0_PIN; 			//	R0
	HAL_GPIO_Init(LTDC_R0_PORT, &GPIO_InitStruct);		
	
	GPIO_InitStruct.Pin 		= LTDC_R1_PIN; 			//	R1
	HAL_GPIO_Init(LTDC_R1_PORT, &GPIO_InitStruct);		
	
	GPIO_InitStruct.Pin 		= LTDC_G0_PIN; 			//	G0
	HAL_GPIO_Init(LTDC_G0_PORT, &GPIO_InitStruct);		
	
	GPIO_InitStruct.Pin 		= LTDC_G1_PIN; 			//	G1
	HAL_GPIO_Init(LTDC_G1_PORT, &GPIO_InitStruct);	
	
	GPIO_InitStruct.Pin 		= LTDC_B0_PIN; 			//	B0
	HAL_GPIO_Init(LTDC_B0_PORT, &GPIO_InitStruct);		

	GPIO_InitStruct.Pin 		= LTDC_B1_PIN; 			//	B1
	HAL_GPIO_Init(LTDC_B1_PORT, &GPIO_InitStruct);			
	
	// 旧款5寸屏只引出了18位的接口，其余数据脚是接地的
	// 因此需要将LTDC对应的低2位数据脚输出低电平
	
	HAL_GPIO_WritePin(LTDC_R0_PORT, LTDC_R0_PIN, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(LTDC_R1_PORT, LTDC_R1_PIN, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(LTDC_G0_PORT, LTDC_G0_PIN, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(LTDC_G1_PORT, LTDC_G1_PIN, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(LTDC_B0_PORT, LTDC_B0_PIN, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(LTDC_B1_PORT, LTDC_B1_PIN, GPIO_PIN_RESET);	
			
}


/*************************************************************************************************
*	函 数 名:	HAL_LTDC_MspInit
*	入口参数:	无
*	返 回 值:	无
*	函数功能:	初始化LTDC引脚的IO口
*	说    明:	无	
*************************************************************************************************/

void HAL_LTDC_MspInit(LTDC_HandleTypeDef* hltdc)
{ 
   uint8_t Modify_Flag = 0;	// 触摸坐标修改标志位
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(hltdc->Instance==LTDC)
	{
		__HAL_RCC_LTDC_CLK_ENABLE();	// 开启LTDC时钟

		GPIO_LTDC_Black_CLK_ENABLE;	// 开启背光引脚端口时钟
		__HAL_RCC_GPIOI_CLK_ENABLE();
		__HAL_RCC_GPIOJ_CLK_ENABLE();
		__HAL_RCC_GPIOK_CLK_ENABLE();
		
		 /**LTDC GPIO Configuration    
		 PI12     ------> LTDC_HSYNC
		 PI13     ------> LTDC_VSYNC
		 PI14     ------> LTDC_CLK
		 PK7      ------> LTDC_DE 
			
		 PI15    ------> LTDC_R0		 PJ7     ------> LTDC_G0		    PJ12     ------> LTDC_B0
		 PJ0     ------> LTDC_R1       PJ8     ------> LTDC_G1          PJ13     ------> LTDC_B1
		 PJ1     ------> LTDC_R2       PJ9     ------> LTDC_G2          PJ14     ------> LTDC_B2
		 PJ2     ------> LTDC_R3       PJ10    ------> LTDC_G3          PJ15     ------> LTDC_B3
		 PJ3     ------> LTDC_R4       PJ11    ------> LTDC_G4          PK3      ------> LTDC_B4
		 PJ4     ------> LTDC_R5       PK0     ------> LTDC_G5          PK4      ------> LTDC_B5
		 PJ5     ------> LTDC_R6       PK1     ------> LTDC_G6          PK5      ------> LTDC_B6
		 PJ6     ------> LTDC_R7       PK2     ------> LTDC_G7          PK6      ------> LTDC_B7

		*******/
	
/*----------------------------旧面板识别---------------------------------*/
	

		GPIO_InitStruct.Mode 	= GPIO_MODE_INPUT;		//	输入模式
		GPIO_InitStruct.Pull 	= GPIO_NOPULL;				//	不上下拉
		
		GPIO_InitStruct.Pin 		= LTDC_R0_PIN; 			//	R0
		HAL_GPIO_Init(LTDC_R0_PORT, &GPIO_InitStruct);		

		GPIO_InitStruct.Pin 		= LTDC_G0_PIN; 			//	G0
		HAL_GPIO_Init(LTDC_G0_PORT, &GPIO_InitStruct);		

		GPIO_InitStruct.Pin 		= LTDC_B0_PIN; 			//	B0
		HAL_GPIO_Init(LTDC_B0_PORT, &GPIO_InitStruct);		


		// 旧面板识别修改，此处只对旧款5寸屏有用，也就是 RGB050M1  V1.0的版本
		// 旧款5寸屏只引出了18位的接口，低2位数据脚直接接地，如果对应的引脚为低电平则可以确定接入的是旧款5寸屏
		// 其它面板无需理会此处，用户移植的时候可以直接删掉
		if(   (HAL_GPIO_ReadPin(LTDC_R0_PORT,LTDC_R0_PIN) == 0) \
			&& (HAL_GPIO_ReadPin(LTDC_G0_PORT,LTDC_G0_PIN) == 0) \
			&& (HAL_GPIO_ReadPin(LTDC_B0_PORT,LTDC_B0_PIN) == 0) )	
		{

			Modify_Flag	= 1;	// 如果检测到了旧款5寸屏，则置位标志位	
		}

/*-----------------------------------------------------------------------*/

	
		GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
		HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
								  |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
								  |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
								  |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
		HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
								  |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
		HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);


		// 背光引脚
		GPIO_InitStruct.Pin 		= LTDC_Black_PIN;
		GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull 	= GPIO_PULLUP;
		GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(LTDC_Black_PORT, &GPIO_InitStruct);

		HAL_GPIO_WritePin(LTDC_Black_PORT, LTDC_Black_PIN, GPIO_PIN_RESET);	// 先关闭背光引脚，初始化之后再开启
	}

// 旧面板识别修改，此处只对旧款5寸屏有用，也就是 RGB050M1  V1.0的版本
// 旧款5寸屏只引出了18位的接口，低2位数据脚直接接地，如果对应的引脚为低电平则可以确定接入的是旧款5寸屏
// 其它面板无需理会此处，用户移植的时候可以直接删掉
	if( Modify_Flag == 1 )
	{
		LCD_PanelModify();	// 将对应的低2位引脚输出低电平，否则单片机功耗会很高
	}

}


/*************************************************************************************************
*	函 数 名:	LCD_Init
*	入口参数:	无
*	返 回 值:	无
*	函数功能:	初始化LTDC引脚的IO口、全局参数、层设置等
*	说    明:	无			
*************************************************************************************************/

void LTDC_Init(void)
{ 
	__HAL_RCC_DMA2D_CLK_ENABLE();	// 使能DMA2D
	
	LTDC_LayerCfgTypeDef pLayerCfg = {0};

	hltdc.Instance 		 = LTDC;		
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;	// 低电平有效
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;	// 低电平有效
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;	// 低电平有效，要注意的是，很多面板都是高电平有效，但是429需要设置成低电平才能正常显示
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;	// 正常时钟信号

	hltdc.Init.HorizontalSync 		= HSW - 1;									// 根据屏幕设置参数即可
	hltdc.Init.VerticalSync 		= VSW	-1 ;
	hltdc.Init.AccumulatedHBP		= HBP + HSW -1;
	hltdc.Init.AccumulatedVBP 		= VBP + VSW -1;
	hltdc.Init.AccumulatedActiveW = LCD_Width  + HSW + HBP -1;
	hltdc.Init.AccumulatedActiveH = LCD_Height + VSW + VBP -1;
	hltdc.Init.TotalWidth 			= LCD_Width  + HSW + HBP + HFP - 1; 
	hltdc.Init.TotalHeigh 			= LCD_Height + VSW + VBP + VFP - 1;
	hltdc.Init.Backcolor.Blue 		= 0;
	hltdc.Init.Backcolor.Green 	= 0;
	hltdc.Init.Backcolor.Red 		= 0;
	
	HAL_LTDC_Init(&hltdc);	// 初始化LTDC

	/*---------------------------------- layer0 显示配置 --------------------------------*/

	pLayerCfg.WindowX0 			= 0;										// 水平起点
	pLayerCfg.WindowX1 			= LCD_Width;							// 水平终点
	pLayerCfg.WindowY0 			= 0;										// 垂直起点
	pLayerCfg.WindowY1 			= LCD_Height;							// 垂直终点
	pLayerCfg.PixelFormat		= ColorMode_0;							// 颜色格式
	pLayerCfg.ImageWidth 		= LCD_Width;							// 显示区域宽度
	pLayerCfg.ImageHeight 		= LCD_Height;							// 显示区域高度	
	
// 配置 layer0 的恒定透明度，最终写入 LTDC_LxCACR 寄存器 
//	需要注意的是，这个参数是直接配置整个 layer0 的透明度，这里设置为255即不透明 	
	pLayerCfg.Alpha 				= 255;									// 透明度
	
	
// 设置 layer1 的层混合系数，最终写入 LTDC_LxBFCR 寄存器 
// 该参数用于设置 layer1 和 (layer0+背景）之间的颜色混合系数，计算公式为 ：
// 混合后的颜色 =  BF1 * layer1的颜色 + BF2 * (layer0+背景混合后的颜色）
// 如果 layer1 使用了透明色，则必须配置成 LTDC_BLENDING_FACTOR1_PAxCA 和 LTDC_BLENDING_FACTOR2_PAxCA，否则ARGB中的A通道不起作用
//	更多的介绍可以查阅 参考手册关于 LTDC_LxBFCR 寄存器的介绍	
	pLayerCfg.BlendingFactor1 	= LTDC_BLENDING_FACTOR1_PAxCA;	// 混合系数
	pLayerCfg.BlendingFactor2 	= LTDC_BLENDING_FACTOR2_PAxCA;	// 混合系数
	
// layer0 的显存地址，本例程使用外部的SDRAM作为显存，起始地址0xD0000000，SDRAM大小为16M
// layer0 显存大小等于 = LCD_Width * LCD_Width * BytesPerPixel_0（每个像素所占字节大小）
// 因为 SDRAM 大小为16M，用户设置的区域一定不能超过这个值！		
	pLayerCfg.FBStartAdress 	= LCD_MemoryAdd;						// 显存地址
	
// 配置layer1 的初始默认颜色，包括A,R,G,B 的值 ，最终写入 LTDC_LxDCCR 寄存器 	
	pLayerCfg.Alpha0 				= 0;										// 透明度
	pLayerCfg.Backcolor.Blue 	= 0;										//	初始颜色
	pLayerCfg.Backcolor.Green 	= 0;										//	初始颜色
	pLayerCfg.Backcolor.Red 	= 0;										//	初始颜色
	
	HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) ;					// 配置层0，背景层

	#if ( ( ColorMode_0 == LTDC_PIXEL_FORMAT_RGB888 )||( ColorMode_0 == LTDC_PIXEL_FORMAT_ARGB8888 ) ) // 判断是否使用24位或者32位色

	//使能颜色抖动，24位以上的颜色必须打开，否则无法达到相应的效果
		HAL_LTDC_EnableDither(&hltdc); // 开启颜色抖动
		
	// 当颜色格式为24位色时，重新设置帧缓冲区的寄存器，按照32位格式来设置，即每个像素占4字节
	//	如果使用HAL库默认的设置，在刷屏或者显示字符的时候，容易造成屏幕花屏
	// 这里设置的只是帧缓冲区的格式，和SDRAM显存无关		
		LTDC_Layer1->CFBLR 	= (LCD_Width * 4 << 16 ) | (LCD_Width * 4 + 3) ;	
		HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING);		// 重新载入参数	
	#endif



/*---------------------------------- layer1 显示配置 --------------------------------*/

#if ( LCD_NUM_LAYERS == 2 )	//当定义了双层时

	LTDC_LayerCfgTypeDef pLayerCfg1 = {0};	
	
	pLayerCfg1.WindowX0 			= 0;													// 水平起点
	pLayerCfg1.WindowX1 			= LCD_Width;                              // 水平终点
	pLayerCfg1.WindowY0 			= 0;                                      // 垂直起点
	pLayerCfg1.WindowY1 			= LCD_Height;                             // 垂直终点
	pLayerCfg1.PixelFormat 		= ColorMode_1;                            // 颜色格式
	pLayerCfg1.ImageWidth 		= LCD_Width;                              // 显示区域宽度
	pLayerCfg1.ImageHeight	 	= LCD_Height;                             // 显示区域高度	
	
// 配置 layer1 的恒定透明度，最终写入 LTDC_LxCACR 寄存器 
//	需要注意的是，这个参数是直接配置整个 layer1 的透明度，这里设置为255即不透明 	
	pLayerCfg1.Alpha 				= 255;	                                 // 透明度
	
// 设置 layer1 的层混合系数，最终写入 LTDC_LxBFCR 寄存器 
// 该参数用于设置 layer1 和 (layer0+背景）之间的颜色混合系数，计算公式为 ：
// 混合后的颜色 =  BF1 * layer1的颜色 + BF2 * (layer0+背景混合后的颜色）
// 如果 layer1 使用了透明色，则必须配置成 LTDC_BLENDING_FACTOR1_PAxCA 和 LTDC_BLENDING_FACTOR2_PAxCA，否则ARGB中的A通道不起作用
//	更多的介绍可以查阅 参考手册关于 LTDC_LxBFCR 寄存器的介绍
	pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;            // 混合系数
	pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;            // 混合系数
	
// layer1 的显存地址，本例程使用外部的SDRAM作为显存，起始地址0xD0000000，SDRAM大小为16M
// 由于 layer0 会占用一部分显存，因此设置 layer1 显存时，需要进行一定偏移
	pLayerCfg1.FBStartAdress 	= LCD_MemoryAdd + LCD_MemoryAdd_OFFSET;   // 显存地址
	
// 配置layer1 的初始默认颜色，包括A,R,G,B 的值 ，最终写入 LTDC_LxDCCR 寄存器 	
	pLayerCfg1.Alpha0 			= 0;                                      // 透明度
	pLayerCfg1.Backcolor.Blue 	= 0;                                      //	初始颜色
	pLayerCfg1.Backcolor.Green = 0;                                      //	初始颜色
	pLayerCfg1.Backcolor.Red 	= 0;                                      //	初始颜色
	
	HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) ;                       // 配置层1，前景层

	#if ( ( ColorMode_1 == LTDC_PIXEL_FORMAT_RGB888 )||( ColorMode_1 == LTDC_PIXEL_FORMAT_ARGB8888 ) ) // 判断是否使用24位或者32位色

	//使能颜色抖动，24位以上的颜色必须打开，否则无法达到相应的效果
		HAL_LTDC_EnableDither(&hltdc); // 开启颜色抖动

	// 当颜色格式为24位色时，重新设置帧缓冲区的寄存器，按照32位格式来设置，即每个像素占4字节
	//	如果使用HAL库默认的设置，在刷屏或者显示字符的时候，容易造成屏幕花屏
	// 这里设置的只是帧缓冲区的格式，和SDRAM显存无关	
		LTDC_Layer2->CFBLR 	= (LCD_Width * 4 << 16 ) | (LCD_Width * 4 + 3) ;
		HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING);		// 重新载入参数			
	#endif

#endif
	
/*---------------------------------- 初始化一些默认配置 --------------------------------*/	
	
	LCD_DisplayDirection(Direction_H); 	//	横屏显示
	LCD_SetFont(&Font24);  					//	设置默认字体	
	LCD_ShowNumMode(Fill_Space);			//	数字显示默认填充空格
	
	LCD_SetLayer(0);  
	LCD_SetBackColor(LCD_BLACK); 			//	设置背景色
	LCD_SetColor(LCD_WHITE);				//	设置画笔颜色
	LCD_Clear(); 								//	清屏，刷背景色
	
/*---------------------------------- 如果开启了两层显示 --------------------------------*/	

#if LCD_NUM_LAYERS == 2	 //	如果定义了第二层
	LCD_SetLayer(1); 
	LCD_SetBackColor(LCD_BLACK); 	//	设置背景色
	LCD_SetColor(LCD_WHITE);		//	设置画笔颜色
	LCD_Clear(); 						//	清屏，刷背景色
#endif
// LTDC在初始化之后，上电的瞬间会有一个短暂的白屏，
//	即使一开始就将背光引脚拉低并且屏幕背光引脚用电阻下拉还是会有这个现象，
//	如果需要消除这个现象，可以在初始化完毕之后，进行一个短暂的延时再打开背光
//
//	HAL_Delay(200);
  	HAL_GPIO_WritePin(LTDC_Black_PORT, LTDC_Black_PIN, GPIO_PIN_SET);	// 开启背光
}  


/*************************************************************************************************
*	函 数 名:	LCD_SetLayer
*	入口参数:	layer - 要显示和操作的层，可以设置为0或1，即选择 layer0 或 layer1
*	返 回 值:	无
*	函数功能:	设置要显示和操作的层，切换相应的显存地址、颜色格式等
*	说    明:	LTDC层顺序是固定的，layer1 在 layer0之上，即开启两层显示时，
*					layer1 是前景层，通常使用带透明色的颜色格式，layer0 是背景层，
*					只开启单层时，默认只操作 layer0
*************************************************************************************************/

void LCD_SetLayer(uint8_t layer)
{
#if LCD_NUM_LAYERS == 2		// 如果开了双层
	
	if (layer == 0)			// 如果设置的是 layer0
	{
		LCD.LayerMemoryAdd = LCD_MemoryAdd; 	// 获取 layer0 的显存地址
		LCD.ColorMode      = ColorMode_0;		// 获取 layer0 的颜色格式
		LCD.BytesPerPixel  = BytesPerPixel_0;	// 获取 layer0 的每个像素所需字节数的大小
	}
	else if(layer == 1)	 // 如果设置的是 layer1
	{
		LCD.LayerMemoryAdd = LCD_MemoryAdd + LCD_MemoryAdd_OFFSET;	// 获取 layer1 的显存地址
		LCD.ColorMode      = ColorMode_1;                           // 获取 layer1 的颜色格式
		LCD.BytesPerPixel  = BytesPerPixel_1;		                  // 获取 layer1 的每个像素所需字节数的大小
	}
	LCD.Layer = layer;	//记录当前所在的层
	
#else		// 如果只开启单层，默认操作 layer0
	
	LCD.LayerMemoryAdd = LCD_MemoryAdd;		// 获取 layer0 的显存地址
	LCD.ColorMode      = ColorMode_0;      // 获取 layer0 的颜色格式
	LCD.BytesPerPixel  = BytesPerPixel_0;	// 获取 layer0 的每个像素所需字节数的大小
	LCD.Layer = 0;		// 层标记设置为 layer0
	
#endif

}  

/***************************************************************************************************************
*	函 数 名:	LCD_SetColor
*
*	入口参数:	Color - 要显示的颜色，示例：0xff0000FF 表示不透明的蓝色，0xAA0000FF 表示透明度为66.66%的蓝色
*
*	函数功能:	此函数用于设置显示字符、画点画线、绘图的颜色
*
*	说    明:	1. 为了方便用户使用自定义颜色，入口参数 Color 使用32位的颜色格式，用户无需关心颜色格式的转换
*					2. 32位的颜色中，从高位到低位分别对应 A、R、G、B  4个颜色通道
*					3. 高8位的透明通道中，ff表示不透明，0表示完全透明
*					4. 除非使用ARGB1555和ARGB8888等支持透明色的颜色格式，不然透明色不起作用，其中ARGB1555仅支持一位
*						透明色，即仅有透明和不透明两种状态，ARGB8888支持255级透明度
*					5. 这里说到的透明，是指 背景层、layer0和layer1 之间的透明
*
***************************************************************************************************************/

void LCD_SetColor(uint32_t Color)
{
	uint16_t Alpha_Value = 0, Red_Value = 0, Green_Value = 0, Blue_Value = 0; //各个颜色通道的值

	if( LCD.ColorMode == LTDC_PIXEL_FORMAT_RGB565	)	//将32位色转换为16位色
	{
		Red_Value   = (uint16_t)((Color&0x00F80000)>>8);
		Green_Value = (uint16_t)((Color&0x0000FC00)>>5);
		Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);
		LCD.Color = (uint16_t)(Red_Value | Green_Value | Blue_Value);		
	}
	else if( LCD.ColorMode == LTDC_PIXEL_FORMAT_ARGB1555 )	//将32位色转换为ARGB1555颜色
	{
		if( (Color & 0xFF000000) == 0 )	//判断是否使用透明色
			Alpha_Value = 0x0000;
		else
			Alpha_Value = 0x8000;

		Red_Value   = (uint16_t)((Color&0x00F80000)>>9);	
		Green_Value = (uint16_t)((Color&0x0000F800)>>6);
		Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);
		LCD.Color = (uint16_t)(Alpha_Value | Red_Value | Green_Value | Blue_Value);	
	}
	else if( LCD.ColorMode == LTDC_PIXEL_FORMAT_ARGB4444 )	//将32位色转换为ARGB4444颜色
	{

		Alpha_Value = (uint16_t)((Color&0xf0000000)>>16);
		Red_Value   = (uint16_t)((Color&0x00F00000)>>12);	
		Green_Value = (uint16_t)((Color&0x0000F000)>>8);
		Blue_Value  = (uint16_t)((Color&0x000000F8)>>4);
		LCD.Color = (uint16_t)(Alpha_Value | Red_Value | Green_Value | Blue_Value);	
	}	
	else
		LCD.Color = Color;	//24位色或32位色不需要转换
}

/***************************************************************************************************************
*	函 数 名:	LCD_SetBackColor
*
*	入口参数:	Color - 要显示的颜色，示例：0xff0000FF 表示不透明的蓝色，0xAA0000FF 表示透明度为66.66%的蓝色
*
*	函数功能:	设置背景色,此函数用于清屏以及显示字符的背景色
*
*	说    明:	1. 为了方便用户使用自定义颜色，入口参数 Color 使用32位的颜色格式，用户无需关心颜色格式的转换
*					2. 32位的颜色中，从高位到低位分别对应 A、R、G、B  4个颜色通道
*					3. 高8位的透明通道中，ff表示不透明，0表示完全透明
*					4. 除非使用ARGB1555和ARGB8888等支持透明色的颜色格式，不然透明色不起作用，其中ARGB1555仅支持一位
*						透明色，即仅有透明和不透明两种状态，ARGB8888支持255级透明度
*					5. 这里说到的透明，是指 背景层、layer0和layer1之间的透明
*
***************************************************************************************************************/

void LCD_SetBackColor(uint32_t Color)
{
	uint16_t Alpha_Value = 0, Red_Value = 0, Green_Value = 0, Blue_Value = 0;  //各个颜色通道的值

	if( LCD.ColorMode == LTDC_PIXEL_FORMAT_RGB565	)	//将32位色转换为16位色
	{
		Red_Value   	= (uint16_t)((Color&0x00F80000)>>8);
		Green_Value 	= (uint16_t)((Color&0x0000FC00)>>5);
		Blue_Value  	= (uint16_t)((Color&0x000000F8)>>3);
		LCD.BackColor	= (uint16_t)(Red_Value | Green_Value | Blue_Value);	
	}
	else if( LCD.ColorMode == LTDC_PIXEL_FORMAT_ARGB1555 )	//将32位色转换为ARGB1555颜色
	{
		if( (Color & 0xFF000000) == 0 )	//判断是否使用透明色
			Alpha_Value = 0x0000;
		else
			Alpha_Value = 0x8000;

		Red_Value   	= (uint16_t)((Color&0x00F80000)>>9);
		Green_Value 	= (uint16_t)((Color&0x0000F800)>>6);
		Blue_Value  	= (uint16_t)((Color&0x000000F8)>>3);
		LCD.BackColor 	= (uint16_t)(Alpha_Value | Red_Value | Green_Value | Blue_Value);	
	}
	else if( LCD.ColorMode == LTDC_PIXEL_FORMAT_ARGB4444 )	//将32位色转换为ARGB4444颜色
	{

		Alpha_Value 	= (uint16_t)((Color&0xf0000000)>>16);
		Red_Value   	= (uint16_t)((Color&0x00F00000)>>12);	
		Green_Value 	= (uint16_t)((Color&0x0000F000)>>8);
		Blue_Value  	= (uint16_t)((Color&0x000000F8)>>4);
		LCD.BackColor 	= (uint16_t)(Alpha_Value | Red_Value | Green_Value | Blue_Value);	
	}		
	
	else	
		LCD.BackColor = Color;	//24位色或32位色不需要转换
	
}


/***************************************************************************************************************
*	函 数 名:	LCD_SetFont
*
*	入口参数:	*fonts - 要设置的ASCII字体
*
*	函数功能:	设置ASCII字体，可选择使用 3216/2412/2010/1608/1206 五种大小的字体
*
*	说    明:	1. 使用示例 LCD_SetFont(&Font24) ，即设置 2412的 ASCII字体
*					2. 相关字模存放在 lcd_fonts.c 			
*
***************************************************************************************************************/

void LCD_SetFont(pFONT *fonts)
{
	LCD_Fonts = fonts;
}

/***************************************************************************************************************
*	函 数 名:	LCD_DisplayDirection
*
*	入口参数:	direction - 要显示的方向
*
*	函数功能:	设置要显示的方向，可输入参数 Direction_H 代表横屏显示，Direction_V 代表竖直显示
*
*	说    明:   使用示例 LCD_DisplayDirection(Direction_H) ，即设置屏幕横屏显示
*
***************************************************************************************************************/

void LCD_DisplayDirection(uint8_t direction)
{
	LCD.Direction = direction;
}

/***************************************************************************************************************
*	函 数 名:	LCD_Clear
*
*	函数功能:	清屏函数，将LCD清除为 LCD.BackColor 的颜色，使用DMA2D实现
*
*	说    明:	先用 LCD_SetBackColor() 设置要清除的背景色，再调用该函数清屏即可
*
***************************************************************************************************************/

void LCD_Clear(void)
{
	DMA2D->CR	  &=	~(DMA2D_CR_START);				//	停止DMA2D
	DMA2D->CR		=	DMA2D_R2M;							//	寄存器到SDRAM
	DMA2D->OPFCCR	=	LCD.ColorMode;						//	设置颜色格式
	DMA2D->OOR		=	0;										//	设置行偏移 
	DMA2D->OMAR		=	LCD.LayerMemoryAdd ;				// 地址
	DMA2D->NLR		=	(LCD_Width<<16)|(LCD_Height);	//	设定长度和宽度
	DMA2D->OCOLR	=	LCD.BackColor;						//	颜色
	
// 等待 垂直数据使能显示状态 ，即LTDC即将刷完一整屏数据的时候
// 因为在屏幕没有刷完一帧时进行刷屏，会有撕裂的现象
// 用户也可以使用 寄存器重载中断 进行判断，不过为了保证例程的简洁以及移植的方便性，这里直接使用判断寄存器的方法
//
//
	while( LTDC->CDSR != 0X00000001);	// 判断 显示状态寄存器LTDC_CDSR 的第0位 VDES：垂直数据使能显示状态
	
	DMA2D->CR	  |=	DMA2D_CR_START;					//	启动DMA2D
		
	while (DMA2D->CR & DMA2D_CR_START) ;				//	等待传输完成
}

/***************************************************************************************************************
*	函 数 名:	LCD_ClearRect
*
*	入口参数:	x - 起始水平坐标，取值范围0~799 
*					y - 起始垂直坐标，取值范围0~479
*					width  - 要清除区域的横向长度
*					height - 要清除区域的纵向宽度
*
*	函数功能:	局部清屏函数，将指定位置对应的区域清除为 LCD.BackColor 的颜色
*
*	说    明:	1. 先用 LCD_SetBackColor() 设置要清除的背景色，再调用该函数清屏即可
*					2. 使用示例 LCD_ClearRect( 10, 10, 100, 50) ，清除坐标(10,10)开始的长100宽50的区域
*
***************************************************************************************************************/

void LCD_ClearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{

	DMA2D->CR	  &=	~(DMA2D_CR_START);				//	停止DMA2D
	DMA2D->CR		=	DMA2D_R2M;							//	寄存器到SDRAM
	DMA2D->OPFCCR	=	LCD.ColorMode;						//	设置颜色格式
	DMA2D->OCOLR	=	LCD.BackColor ;					//	颜色
	
	if(LCD.Direction == Direction_H)  //横屏填充
	{		
		DMA2D->OOR		=	LCD_Width - width;				//	设置行偏移 
		DMA2D->OMAR		=	LCD.LayerMemoryAdd + LCD.BytesPerPixel*(LCD_Width * y + x);	// 地址;
		DMA2D->NLR		=	(width<<16)|(height);			//	设定长度和宽度		
	}
	else	//竖屏填充
	{		
		DMA2D->OOR		=	LCD_Width - height;		//	设置行偏移 
		DMA2D->OMAR		=	LCD.LayerMemoryAdd + LCD.BytesPerPixel*((LCD_Height - x - 1 - width)*LCD_Width + y);	// 地址
		DMA2D->NLR		=	(width)|(height<<16);	//	设定长度和宽度		
	}		

	DMA2D->CR	  |=	DMA2D_CR_START;					//	启动DMA2D
		
	while (DMA2D->CR & DMA2D_CR_START) ;			//	等待传输完成

}


/***************************************************************************************************************
*	函 数 名:	LCD_DrawPoint
*
*	入口参数:	x - 起始水平坐标，取值范围0~799 
*					y - 起始垂直坐标，取值范围0~479
*					color  - 要绘制的颜色，使用32位的颜色格式，用户无需关心颜色格式的转换
*
*	函数功能:	在指定坐标绘制指定颜色的点
*
*	说    明:	1. 直接在对应的显存位置写入颜色值，即可实现画点的功能
*					2. 使用示例 LCD_DrawPoint( 10, 10, 0xff0000FF) ，在坐标(10,10)绘制蓝色的点
*
***************************************************************************************************************/

void LCD_DrawPoint(uint16_t x,uint16_t y,uint32_t color)
{

/*----------------------- 32位色 ARGB8888 模式 ----------------------*/
		
	if( LCD.ColorMode == LTDC_PIXEL_FORMAT_ARGB8888 ) 
	{
		if (LCD.Direction == Direction_H) //水平方向
		{
			*(__IO uint32_t*)( LCD.LayerMemoryAdd + 4*(x + y*LCD_Width) ) = color ; 	
		}
		else if(LCD.Direction == Direction_V)	//垂直方向
		{
			*(__IO uint32_t*)( LCD.LayerMemoryAdd + 4*((LCD_Height - x - 1)*LCD_Width + y) ) = color ;
		}
	}
/*----------------------------- 24位色 RGB888 模式 -------------------------*/	
	
	else if ( LCD.ColorMode == LTDC_PIXEL_FORMAT_RGB888 )
	{		
		if (LCD.Direction == Direction_H) //水平方向
		{
			*(__IO uint16_t*)( LCD.LayerMemoryAdd + 3*(x + y*LCD_Width) ) = color ; 
			*(__IO uint8_t*)( LCD.LayerMemoryAdd + 3*(x + y*LCD_Width) + 2 ) = color>>16 ; 	
		}
		else if(LCD.Direction == Direction_V)	//垂直方向
		{
			*(__IO uint16_t*)( LCD.LayerMemoryAdd + 3*((LCD_Height - x - 1)*LCD_Width + y) ) = color ; 
			*(__IO uint8_t*)( LCD.LayerMemoryAdd + 3*((LCD_Height - x - 1)*LCD_Width + y) +2) = color>>16 ; 	
		}	
	}

/*----------------------- 16位色 ARGB1555、RGB565或者ARGB4444 模式 ----------------------*/	
	else		
	{
		if (LCD.Direction == Direction_H) //水平方向
		{
			*(__IO uint16_t*)( LCD.LayerMemoryAdd + 2*(x + y*LCD_Width) ) = color ; 	
		}
		else if(LCD.Direction == Direction_V)	//垂直方向
		{
			*(__IO uint16_t*)( LCD.LayerMemoryAdd + 2*((LCD_Height - x - 1)*LCD_Width + y) ) = color ;
		}	
	}
}

/***************************************************************************************************************
*	函 数 名:	LCD_ReadPoint
*
*	入口参数:	x - 起始水平坐标，取值范围0~799 
*					y - 起始垂直坐标，取值范围0~479
*
*	返 回 值：  读取到的颜色
*
*	函数功能:	读取指定坐标点的颜色，在使用16或24位色模式时，读出来的颜色数据对应为16位或24位
*
*	说    明:	1. 直接读取对应的显存值，即可实现读点的功能
*					2. 使用示例 color = LCD_ReadPoint( 10, 10) ，color 为读取到的坐标点(10,10)的颜色
*
***************************************************************************************************************/

uint32_t LCD_ReadPoint(uint16_t x,uint16_t y)
{
	uint32_t color = 0;

/*----------------------- 32位色 ARGB8888 模式 ----------------------*/
	if( LCD.ColorMode == LTDC_PIXEL_FORMAT_ARGB8888 ) 
	{
		if (LCD.Direction == Direction_H) //水平方向
		{
			color = *(__IO uint32_t*)( LCD.LayerMemoryAdd + 4*(x + y*LCD_Width) ); 	
		}
		else if(LCD.Direction == Direction_V)	//垂直方向
		{
			color = *(__IO uint32_t*)( LCD.LayerMemoryAdd + 4*((LCD_Height - x - 1)*LCD_Width + y) );
		}
	}
	
/*----------------------------- 24位色 RGB888 模式 -------------------------*/	
	else if ( LCD.ColorMode == LTDC_PIXEL_FORMAT_RGB888 )
	{
		if (LCD.Direction == Direction_H) //水平方向
		{
			color = *(__IO uint32_t*)( LCD.LayerMemoryAdd + 3*(x + y*LCD_Width) ) &0x00ffffff; 	
		}
		else if(LCD.Direction == Direction_V)	//垂直方向
		{
			color = *(__IO uint32_t*)( LCD.LayerMemoryAdd + 3*((LCD_Height - x - 1)*LCD_Width + y) ) &0x00ffffff; 	
		}	
	}
	
/*----------------------- 16位色 ARGB1555、RGB565或者ARGB4444 模式 ----------------------*/	
	else		
	{
		if (LCD.Direction == Direction_H) //水平方向
		{
			color = *(__IO uint16_t*)( LCD.LayerMemoryAdd + 2*(x + y*LCD_Width) ); 	
		}
		else if(LCD.Direction == Direction_V)	//垂直方向
		{
			color = *(__IO uint16_t*)( LCD.LayerMemoryAdd + 2*((LCD_Height - x - 1)*LCD_Width + y) );
		}	
	}
	return color;
}  
 
/***************************************************************************************************************
*	函 数 名:	LCD_DisplayChar
*
*	入口参数:	x - 起始水平坐标，取值范围0~799 
*					y - 起始垂直坐标，取值范围0~479
*					c  - ASCII字符
*
*	函数功能:	在指定坐标显示指定的字符
*
*	说    明:	1. 可设置要显示的字体，例如使用 LCD_SetFont(&Font24) 设置为 2412的ASCII字体
*					2.	可设置要显示的颜色，例如使用 LCD_SetColor(0xff0000FF) 设置为蓝色
*					3. 可设置对应的背景色，例如使用 LCD_SetBackColor(0xff000000) 设置为黑色的背景色
*					4. 使用示例 LCD_DisplayChar( 10, 10, 'a') ，在坐标(10,10)显示字符 'a'
*
***************************************************************************************************************/

void LCD_DisplayChar(uint16_t x, uint16_t y,uint8_t add)
{
	uint16_t  index = 0, counter = 0;
   uint8_t   disChar;	//字模的值
	uint16_t  Xaddress = x; //水平坐标
	
	add = add - 32; 
	for(index = 0; index < LCD_Fonts->Sizes; index++)
	{
		disChar = LCD_Fonts->pTable[add*LCD_Fonts->Sizes + index]; //获取字符的模值
		for(counter = 0; counter < 8; counter++)
		{ 
			if(disChar & 0x01)	
			{		
				LCD_DrawPoint(Xaddress,y,LCD.Color);	//当前模值不为0时，使用画笔色绘点
			}
			else		
			{		
				LCD_DrawPoint(Xaddress,y,LCD.BackColor);	//否则使用背景色绘制点
			}
			disChar >>= 1;
			Xaddress++;  //水平坐标自加
			
			if( (Xaddress - x)==LCD_Fonts->Width ) //如果水平坐标达到了字符宽度，则退出当前循环
			{													//进入下一行的绘制
				Xaddress = x;
				y++;
				break;
			}
		}	
	}
}

/***************************************************************************************************************
*	函 数 名:	LCD_DisplayString
*
*	入口参数:	x - 起始水平坐标，取值范围0~799 
*					y - 起始垂直坐标，取值范围0~479
*					p - ASCII字符串的首地址
*
*	函数功能:	在指定坐标显示指定的字符串
*
*	说    明:	1. 可设置要显示的字体，例如使用 LCD_SetFont(&Font24) 设置为 2412的ASCII字体
*					2.	可设置要显示的颜色，例如使用 LCD_SetColor(0xff0000FF) 设置为蓝色
*					3. 可设置对应的背景色，例如使用 LCD_SetBackColor(0xff000000) 设置为黑色的背景色
*					4. 使用示例 LCD_DisplayString( 10, 10, "FANKE") ，在起始坐标为(10,10)的地方显示字符串"FANKE"
*
***************************************************************************************************************/

void LCD_DisplayString( uint16_t x, uint16_t y,  char *p) 
{  
	while ((x < LCD_Width) && (*p != 0))	//判断显示坐标是否超出显示区域并且字符是否为空字符
	{
		 LCD_DisplayChar( x,y,*p);
		 x += LCD_Fonts->Width; //显示下一个字符
		 p++;	//取下一个字符地址
	}
}

/***************************************************************************************************************
*	函 数 名:	LCD_SetTextFont
*
*	入口参数:	*fonts - 要设置的文本字体
*
*	函数功能:	设置文本字体，包括中文和ASCII字符，
*
*	说    明:	1. 可选择使用 3232/2424/2020/1616/1212 五种大小的中文字体，
*						并且对应的设置ASCII字体为 3216/2412/2010/1608/1206
*					2. 相关字模存放在 lcd_fonts.c 
*					3. 中文字库使用的是小字库，即用到了对应的汉字再去取模
*					4. 使用示例 LCD_SetTextFont(&CH_Font24) ，即设置 2424的中文字体以及2412的ASCII字符字体
*
***************************************************************************************************************/

void LCD_SetTextFont(pFONT *fonts)
{
	LCD_CHFonts = fonts;		// 设置中文字体
	switch(fonts->Width )
	{
		case 12:	LCD_Fonts = &Font12;	break;	// 设置ASCII字符的字体为 1206
		case 16:	LCD_Fonts = &Font16;	break;	// 设置ASCII字符的字体为 1608
		case 20:	LCD_Fonts = &Font20;	break;	// 设置ASCII字符的字体为 2010	
		case 24:	LCD_Fonts = &Font24;	break;	// 设置ASCII字符的字体为 2412
		case 32:	LCD_Fonts = &Font32;	break;	// 设置ASCII字符的字体为 3216		
		default: break;
	}

}

/***************************************************************************************************************
*	函 数 名:	LCD_DisplayChinese
*
*	入口参数:	x - 起始水平坐标，取值范围0~799 
*					y - 起始垂直坐标，取值范围0~479
*					pText - 中文字符
*
*	函数功能:	在指定坐标显示指定的单个中文字符
*
*	说    明:	1. 可设置要显示的字体，例如使用 LCD_SetTextFont(&CH_Font24) 设置为 2424的中文字体以及2412的ASCII字符字体
*					2.	可设置要显示的颜色，例如使用 LCD_SetColor(0xff0000FF) 设置为蓝色
*					3. 可设置对应的背景色，例如使用 LCD_SetBackColor(0xff000000) 设置为黑色的背景色
*					4. 使用示例 LCD_DisplayChinese( 10, 10, "反") ，在坐标(10,10)显示中文字符"反"
*
***************************************************************************************************************/

void LCD_DisplayChinese(uint16_t x, uint16_t y, char *pText) 
{
	uint16_t  i=0,index = 0, counter = 0;	// 计数变量
	uint16_t  addr;	// 字模地址
   uint8_t   disChar;	//字模的值
	uint16_t  Xaddress = x; //水平坐标

	while(1)
	{		
		// 对比数组中的汉字编码，用以定位该汉字字模的地址		
		if ( *(LCD_CHFonts->pTable + (i+1)*LCD_CHFonts->Sizes + 0)==*pText && *(LCD_CHFonts->pTable + (i+1)*LCD_CHFonts->Sizes + 1)==*(pText+1) )	
		{   
			addr=i;	// 字模地址偏移
			break;
		}				
		i+=2;	// 每个中文字符编码占两字节

		if(i >= LCD_CHFonts->Table_Rows)	break;	// 字模列表中无相应的汉字	
	}	
	

	for(index = 0; index <LCD_CHFonts->Sizes; index++)
	{	
		disChar = *(LCD_CHFonts->pTable + (addr)*LCD_CHFonts->Sizes + index);	// 获取相应的字模地址
		
		for(counter = 0; counter < 8; counter++)
		{ 
			if(disChar & 0x01)	
			{		
				LCD_DrawPoint(Xaddress,y,LCD.Color);	//当前模值不为0时，使用画笔色绘点
			}
			else		
			{		
				LCD_DrawPoint(Xaddress,y,LCD.BackColor);	//否则使用背景色绘制点
			}
			disChar >>= 1;
			Xaddress++;  //水平坐标自加
			
			if( (Xaddress - x)==LCD_CHFonts->Width ) 	//	如果水平坐标达到了字符宽度，则退出当前循环
			{														//	进入下一行的绘制
				Xaddress = x;
				y++;
				break;
			}
		}	
	}	

}


/***************************************************************************************************************
*	函 数 名:	LCD_DisplayText
*
*	入口参数:	x - 起始水平坐标，取值范围0~799 
*					y - 起始垂直坐标，取值范围0~479
*					pText - 字符串，可以显示中文或者ASCII字符
*
*	函数功能:	在指定坐标显示指定的字符串
*
*	说    明:	1. 可设置要显示的字体，例如使用 LCD_SetTextFont(&CH_Font24) 设置为 2424的中文字体以及2412的ASCII字符字体
*					2.	可设置要显示的颜色，例如使用 LCD_SetColor(0xff0000FF) 设置为蓝色
*					3. 可设置对应的背景色，例如使用 LCD_SetBackColor(0xff000000) 设置为黑色的背景色
*					4. 使用示例 LCD_DisplayChinese( 10, 10, "反客科技STM32") ，在坐标(10,10)显示字符串"反客科技STM32"
*
***************************************************************************************************************/

void LCD_DisplayText(uint16_t x, uint16_t y, char *pText) 
{  
 	
	while(*pText != 0)	// 判断是否为空字符
	{
		if(*pText<=0x7F)	// 判断是否为ASCII码
		{
			LCD_DisplayChar(x,y,*pText);	// 显示ASCII
			x+=LCD_Fonts->Width;				// 水平坐标调到下一个字符处
			pText++;								// 字符串地址+1
		}
		else					// 若字符为汉字
		{			
			LCD_DisplayChinese(x,y,pText);	// 显示汉字
			x+=LCD_CHFonts->Width;				// 水平坐标调到下一个字符处
			pText+=2;								// 字符串地址+2，汉字的编码要2字节
		}
	}	
}

/***************************************************************************************************************
*	函 数 名:	LCD_ShowNumMode
*
*	入口参数:	mode - 设置变量的显示模式
*
*	函数功能:	设置变量显示时多余位补0还是补空格，可输入参数 Fill_Space 填充空格，Fill_Zero 填充零
*
*	说    明:   1. 只有 LCD_DisplayNumber() 显示整数 和 LCD_DisplayDecimals()显示小数 这两个函数用到
*					2. 使用示例 LCD_ShowNumMode(Fill_Zero) 设置多余位填充0，例如 123 可以显示为 000123
*
***************************************************************************************************************/

void LCD_ShowNumMode(uint8_t mode)
{
	LCD.ShowNum_Mode = mode;
}

/*****************************************************************************************************************************************
*	函 数 名:	LCD_DisplayNumber
*
*	入口参数:	x - 起始水平坐标，取值范围0~799 
*					y - 起始垂直坐标，取值范围0~479
*					number - 要显示的数字,范围在 -2147483648~2147483647 之间
*					len - 数字的位数，如果位数超过len，将按其实际长度输出，不包括正负号。但如果需要显示负数，请预留一个位的符号显示空间
*
*	函数功能:	在指定坐标显示指定的整数变量
*
*	说    明:	1. 可设置要显示的字体，例如使用 LCD_SetTextFont(&CH_Font24) 设置为 2424的中文字体以及2412的ASCII字符字体
*					2.	可设置要显示的颜色，例如使用 LCD_SetColor(0xff0000FF) 设置为蓝色
*					3. 可设置对应的背景色，例如使用 LCD_SetBackColor(0xff000000) 设置为黑色的背景色
*					4. 使用示例 LCD_DisplayNumber( 10, 10, a, 5) ，在坐标(10,10)显示指定变量a,总共5位，多余位补0或空格，
*						例如 a=123 时，会根据 LCD_ShowNumMode()的设置来显示  123(前面两个空格位) 或者00123
*						
*****************************************************************************************************************************************/

void  LCD_DisplayNumber( uint16_t x, uint16_t y, int32_t number, uint8_t len) 
{  
	char   Number_Buffer[15];				// 用于存储转换后的字符串

	if( LCD.ShowNum_Mode == Fill_Zero)	// 多余位补0
	{
		sprintf( Number_Buffer , "%0.*d",len, number );	// 将 number 转换成字符串，便于显示		
	}
	else			// 多余位补空格
	{	
		sprintf( Number_Buffer , "%*d",len, number );	// 将 number 转换成字符串，便于显示		
	}
	
	LCD_DisplayString( x, y,(char *)Number_Buffer) ;  // 将转换得到的字符串显示出来
	
}

/***************************************************************************************************************************************
*	函 数 名:	LCD_DisplayDecimals
*
*	入口参数:	x - 起始水平坐标，取值范围0~799 
*					y - 起始垂直坐标，取值范围0~479
*					decimals - 要显示的数字, double型取值1.7 x 10^（-308）~ 1.7 x 10^（+308），但是能确保准确的有效位数为15~16位
*
*       			len - 整个变量的总位数（包括小数点和负号），若实际的总位数超过了指定的总位数，将按实际的总长度位输出，
*							示例1：小数 -123.123 ，指定 len <=8 的话，则实际照常输出 -123.123
*							示例2：小数 -123.123 ，指定 len =10 的话，则实际输出   -123.123(负号前面会有两个空格位) 
*							示例3：小数 -123.123 ，指定 len =10 的话，当调用函数 LCD_ShowNumMode() 设置为填充0模式时，实际输出 -00123.123 
*
*					decs - 要保留的小数位数，若小数的实际位数超过了指定的小数位，则按指定的宽度四舍五入输出
*							 示例：1.12345 ，指定 decs 为4位的话，则输出结果为1.1235
*
*	函数功能:	在指定坐标显示指定的变量，包括小数
*
*	说    明:	1. 可设置要显示的字体，例如使用 LCD_SetTextFont(&CH_Font24) 设置为 2424的中文字体以及2412的ASCII字符字体
*					2.	可设置要显示的颜色，例如使用 LCD_SetColor(0xff0000FF) 设置为蓝色
*					3. 可设置对应的背景色，例如使用 LCD_SetBackColor(0xff000000) 设置为黑色的背景色
*					4. 使用示例 LCD_DisplayDecimals( 10, 10, a, 5, 3) ，在坐标(10,10)显示字变量a,总长度为5位，其中保留3位小数
*						
*****************************************************************************************************************************************/

void  LCD_DisplayDecimals( uint16_t x, uint16_t y, double decimals, uint8_t len, uint8_t decs) 
{  
	char  Number_Buffer[20];				// 用于存储转换后的字符串
	
	if( LCD.ShowNum_Mode == Fill_Zero)	// 多余位填充0模式
	{
		sprintf( Number_Buffer , "%0*.*lf",len,decs, decimals );	// 将 number 转换成字符串，便于显示		
	}
	else		// 多余位填充空格
	{
		sprintf( Number_Buffer , "%*.*lf",len,decs, decimals );	// 将 number 转换成字符串，便于显示		
	}
	
	LCD_DisplayString( x, y,(char *)Number_Buffer) ;	// 将转换得到的字符串显示出来
}



/***************************************************************************************************************************************
*	函 数 名: LCD_DrawImage
*
*	入口参数: x - 水平坐标，取值范围 0~799
*			 	 y - 垂直坐标，取值范围 0~479
*			 	 width  - 图片的水平宽度，最大取值800
*				 height - 图片的垂直宽度，最大取值480
*				*pImage - 图片数据存储区的首地址
*
*	函数功能: 在指定坐标处显示图片
*
*	说    明: 要显示的图片需要事先进行取模，且只能显示一种颜色，使用 LCD_SetColor() 函数设置画笔色
*						 
*****************************************************************************************************************************************/

void 	LCD_DrawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage) 
{  
   uint8_t   disChar;	//字模的值
	uint16_t  Xaddress = x; //水平坐标
	uint16_t  i=0,j=0,m=0;
	
	for(i = 0; i <height; i++)
	{
		for(j = 0; j <(float)width/8; j++)
		{
			disChar = *pImage;

			for(m = 0; m < 8; m++)
			{ 
				if(disChar & 0x01)	
				{		
					LCD_DrawPoint(Xaddress,y,LCD.Color);	//当前模值不为0时，使用画笔色绘点
				}
				else		
				{		
					LCD_DrawPoint(Xaddress,y,LCD.BackColor);	//否则使用背景色绘制点
				}
				disChar >>= 1;
				Xaddress++;  //水平坐标自加
				
				if( (Xaddress - x)==width ) //如果水平坐标达到了字符宽度，则退出当前循环
				{													//进入下一行的绘制
					Xaddress = x;
					y++;
					break;
				}
			}	
			pImage++;			
		}
	}	
}

/***************************************************************************************************************************************
*	函 数 名: LCD_DrawLine
*
*	入口参数: x1 - 起点 水平坐标，取值范围 0~799
*			 	 y1 - 起点 垂直坐标，取值范围 0~479
*
*				 x2 - 终点 水平坐标，取值范围 0~799
*            y2 - 终点 垂直坐标，取值范围 0~479
*
*	函数功能: 在两点之间画线
*
*	说    明: 该函数移植于ST官方评估板的例程
*						 
*****************************************************************************************************************************************/

#define ABS(X)  ((X) > 0 ? (X) : -(X))    

//	函数：画线
//	参数：x1、y1为起点坐标，x2、y2为终点坐标
//
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
	curpixel = 0;

	deltax = ABS(x2 - x1);        /* The difference between the x's */
	deltay = ABS(y2 - y1);        /* The difference between the y's */
	x = x1;                       /* Start x off at the first pixel */
	y = y1;                       /* Start y off at the first pixel */

	if (x2 >= x1)                 /* The x-values are increasing */
	{
	 xinc1 = 1;
	 xinc2 = 1;
	}
	else                          /* The x-values are decreasing */
	{
	 xinc1 = -1;
	 xinc2 = -1;
	}

	if (y2 >= y1)                 /* The y-values are increasing */
	{
	 yinc1 = 1;
	 yinc2 = 1;
	}
	else                          /* The y-values are decreasing */
	{
	 yinc1 = -1;
	 yinc2 = -1;
	}

	if (deltax >= deltay)         /* There is at least one x-value for every y-value */
	{
	 xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
	 yinc2 = 0;                  /* Don't change the y for every iteration */
	 den = deltax;
	 num = deltax / 2;
	 numadd = deltay;
	 numpixels = deltax;         /* There are more x-values than y-values */
	}
	else                          /* There is at least one y-value for every x-value */
	{
	 xinc2 = 0;                  /* Don't change the x for every iteration */
	 yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
	 den = deltay;
	 num = deltay / 2;
	 numadd = deltax;
	 numpixels = deltay;         /* There are more y-values than x-values */
	}
	for (curpixel = 0; curpixel <= numpixels; curpixel++)
	{
	 LCD_DrawPoint(x,y,LCD.Color);             /* Draw the current pixel */
	 num += numadd;              /* Increase the numerator by the top of the fraction */
	 if (num >= den)             /* Check if numerator >= denominator */
	 {
		num -= den;               /* Calculate the new numerator value */
		x += xinc1;               /* Change the x as appropriate */
		y += yinc1;               /* Change the y as appropriate */
	 }
	 x += xinc2;                 /* Change the x as appropriate */
	 y += yinc2;                 /* Change the y as appropriate */
	}

}

/***************************************************************************************************************************************
*	函 数 名: LCD_DrawRect
*
*	入口参数: x - 水平坐标，取值范围 0~799
*			 	 y - 垂直坐标，取值范围 0~479
*			 	 width  - 图片的水平宽度，最大取值800
*				 height - 图片的垂直宽度，最大取值480
*
*	函数功能: 在指点位置绘制指定长宽的矩形线条
*
*	说    明: 该函数移植于ST官方评估板的例程
*						 
*****************************************************************************************************************************************/

void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
	/* draw horizontal lines */
	LCD_DrawLine(x, y, x+width, y);
	LCD_DrawLine(x, y+height, x+width, y+height);

	/* draw vertical lines */
	LCD_DrawLine(x, y, x, y+height);
	LCD_DrawLine(x+width, y, x+width, y+height);
}

/***************************************************************************************************************************************
*	函 数 名: LCD_DrawCircle
*
*	入口参数: x - 圆心 水平坐标，取值范围 0~799
*			 	 y - 圆心 垂直坐标，取值范围 0~479
*			 	 r  - 半径
*
*	函数功能: 在坐标 (x,y) 绘制半径为 r 的圆形线条
*
*	说    明: 1. 该函数移植于ST官方评估板的例程
*				 2. 要绘制的区域不能超过屏幕的显示区域
*
*****************************************************************************************************************************************/

void LCD_DrawCircle(uint16_t x, uint16_t y, uint16_t r)
{
	int Xadd = -r, Yadd = 0, err = 2-2*r, e2;
	do {   

		LCD_DrawPoint(x-Xadd,y+Yadd,LCD.Color);
		LCD_DrawPoint(x+Xadd,y+Yadd,LCD.Color);
		LCD_DrawPoint(x+Xadd,y-Yadd,LCD.Color);
		LCD_DrawPoint(x-Xadd,y-Yadd,LCD.Color);
		
		e2 = err;
		if (e2 <= Yadd) {
			err += ++Yadd*2+1;
			if (-Xadd == Yadd && e2 <= Xadd) e2 = 0;
		}
		if (e2 > Xadd) err += ++Xadd*2+1;
    }
    while (Xadd <= 0);
    
}

/***************************************************************************************************************************************
*	函 数 名: LCD_DrawEllipse
*
*	入口参数: x - 圆心 水平坐标，取值范围 0~799
*			 	 y - 圆心 垂直坐标，取值范围 0~479
*			 	 r1  - 水平半轴的长度
*				 r2  - 垂直半轴的长度
*
*	函数功能: 在坐标 (x,y) 绘制水平半轴为 r1 垂直半轴为 r2 的椭圆线条
*
*	说    明: 1. 该函数移植于ST官方评估板的例程
*				 2. 要绘制的区域不能超过屏幕的显示区域
*
*****************************************************************************************************************************************/

void LCD_DrawEllipse(int x, int y, int r1, int r2)
{
  int Xadd = -r1, Yadd = 0, err = 2-2*r1, e2;
  float K = 0, rad1 = 0, rad2 = 0;
   
  rad1 = r1;
  rad2 = r2;
  
  if (r1 > r2)
  { 
    do {
      K = (float)(rad1/rad2);
		 
		LCD_DrawPoint(x-Xadd,y+(uint16_t)(Yadd/K),LCD.Color);
		LCD_DrawPoint(x+Xadd,y+(uint16_t)(Yadd/K),LCD.Color);
		LCD_DrawPoint(x+Xadd,y-(uint16_t)(Yadd/K),LCD.Color);
		LCD_DrawPoint(x-Xadd,y-(uint16_t)(Yadd/K),LCD.Color);     
		 
      e2 = err;
      if (e2 <= Yadd) {
        err += ++Yadd*2+1;
        if (-Xadd == Yadd && e2 <= Xadd) e2 = 0;
      }
      if (e2 > Xadd) err += ++Xadd*2+1;
    }
    while (Xadd <= 0);
  }
  else
  {
    Yadd = -r2; 
    Xadd = 0;
    do { 
      K = (float)(rad2/rad1);

		LCD_DrawPoint(x-(uint16_t)(Xadd/K),y+Yadd,LCD.Color);
		LCD_DrawPoint(x+(uint16_t)(Xadd/K),y+Yadd,LCD.Color);
		LCD_DrawPoint(x+(uint16_t)(Xadd/K),y-Yadd,LCD.Color);
		LCD_DrawPoint(x-(uint16_t)(Xadd/K),y-Yadd,LCD.Color);  
		 
      e2 = err;
      if (e2 <= Xadd) {
        err += ++Xadd*3+1;
        if (-Yadd == Xadd && e2 <= Yadd) e2 = 0;
      }
      if (e2 > Yadd) err += ++Yadd*3+1;     
    }
    while (Yadd <= 0);
  }
}

/***************************************************************************************************************************************
*	函 数 名: LCD_FillRect
*
*	入口参数: x - 水平坐标，取值范围 0~799
*			 	 y - 垂直坐标，取值范围 0~479
*			 	 width  - 图片的水平宽度，最大取值800
*				 height - 图片的垂直宽度，最大取值480
*
*	函数功能: 在坐标 (x,y) 填充指定长宽的实心矩形
*
*	说    明: 1. 使用DMA2D实现
*				 2. 要绘制的区域不能超过屏幕的显示区域
*						 
*****************************************************************************************************************************************/

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
	DMA2D->CR	  &=	~(DMA2D_CR_START);				//	停止DMA2D
	DMA2D->CR		=	DMA2D_R2M;							//	寄存器到SDRAM
	DMA2D->OPFCCR	=	LCD.ColorMode;						//	设置颜色格式
	DMA2D->OCOLR	=	LCD.Color;							//	颜色
	
	if(LCD.Direction == Direction_H)  //横屏填充
	{		
		DMA2D->OOR		=	LCD_Width - width;				//	设置行偏移 
		DMA2D->OMAR		=	LCD.LayerMemoryAdd + LCD.BytesPerPixel*(LCD_Width * y + x);	// 地址;
		DMA2D->NLR		=	(width<<16)|(height);			//	设定长度和宽度		
	}
	else	//竖屏填充
	{		
		DMA2D->OOR		=	LCD_Width - height;		//	设置行偏移 
		DMA2D->OMAR		=	LCD.LayerMemoryAdd + LCD.BytesPerPixel*((LCD_Height - x - 1 - width)*LCD_Width + y);	// 地址
		DMA2D->NLR		=	(width)|(height<<16);	//	设定长度和宽度		
	}		

	DMA2D->CR	  |=	DMA2D_CR_START;					//	启动DMA2D
		
	while (DMA2D->CR & DMA2D_CR_START) ;			//	等待传输完成
}

/***************************************************************************************************************************************
*	函 数 名: LCD_FillCircle
*
*	入口参数: x - 圆心 水平坐标，取值范围 0~799
*			 	 y - 圆心 垂直坐标，取值范围 0~479
*			 	 r  - 半径
*
*	函数功能: 在坐标 (x,y) 填充半径为 r 的圆形区域
*
*	说    明: 1. 该函数移植于ST官方评估板的例程
*				 2. 要绘制的区域不能超过屏幕的显示区域
*
*****************************************************************************************************************************************/

void LCD_FillCircle(uint16_t x, uint16_t y, uint16_t r)
{
  int32_t  D;    /* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  
  D = 3 - (r << 1);
  
  CurX = 0;
  CurY = r;
  
  while (CurX <= CurY)
  {
    if(CurY > 0) 
    { 
		LCD_DrawLine(x - CurX, y - CurY,x - CurX,y - CurY + 2*CurY);
		LCD_DrawLine(x + CurX, y - CurY,x + CurX,y - CurY + 2*CurY); 
    }
    
    if(CurX > 0) 
    {
		LCD_DrawLine(x - CurY, y - CurX,x - CurY,y - CurX + 2*CurX);
		LCD_DrawLine(x + CurY, y - CurX,x + CurY,y - CurX + 2*CurX); 		 
    }
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
  
  LCD_DrawCircle(x, y, r);  
}
/**************************************************************************************************************************************************************************************************************************************************************************FANKE***/
// 实验平台：反客 STM32F429核心板
//

