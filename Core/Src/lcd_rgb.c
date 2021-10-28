/***
	************************************************************************************************************************************************************************************************
	*	@file  	lcd_rgb.c
	*	@version V1.1
	*  @date    2020-10-13
	*	@author  ���ͿƼ�
	*	@brief   ����RGB��ʾ��������ʾ	
   **********************************************************************************************************************************************************************************************
   *  @description
	*
	*	ʵ��ƽ̨������STM32F429BIT6���İ�(�ͺţ�FK429M1) + 800*480�ֱ��ʵ�RGB��Ļ
	*	�Ա���ַ��https://shop212360197.taobao.com
	*	QQ����Ⱥ��536665479
	*
>>>>> V1.1 �汾���˵����
	*
	*	1. ���Ӻ�����ʾ����������С�ֿ�(ֻȡģ�õ��ĺ���)���洢�ڵ�Ƭ��Ƭ��flash��
	*	2. ���Ӽ򵥵�ͼƬ��ʾ������ͼƬ���ݴ洢�ڵ�Ƭ��Ƭ��flash�����ַ�ʽֻ�����ڵ�ɫͼƬ
	*	3.	���Ӷ� ARGB4444 ��֧��
	*	4. ˢ��ʱ�����ٿ����µ��Դ���Ϊ���壬���Ǹ�Ϊ�жϼĴ������ȴ� ��ֱ����ʹ����ʾ״̬����Ч����ˢ��ʱ������˺��
	*	5. ���Ƹ���������ע�ͣ���һ���򻯳���
	*	6. �����800*480�ֱ��ʵ���Ļ���ϵ�һ��
	*
>>>>> ��Ҫ˵����
	*
	*	1. FK429M1 ���İ� ʹ�õ����ⲿSDRAM��Ϊ�Դ棬��ʼ��ַ0xD0000000, SDRAM ��СΪ16M�ֽ�
	*	2. �����ǵ�����ʾ����˫����ʾ�������ܳ��� SDRAM �Ĵ�С
	*	3. �ڸ����������ʱ����Ļ����΢��������˸�����������󣬵ȴ�Ƭ�̻��������ϵ缴�ɻָ�����
	* 	4. LTDCʱ���� lcd_rgb.h �ļ���ĺ� LCD_CLK ���ã�����Ϊ33MHz����ˢ������60֡���ң����߻��߹��Ͷ��������˸	
	*
>>>>> ����˵����
	*
	*	1. �����ֿ�ʹ�õ���С�ֿ⣬���õ��˶�Ӧ�ĺ�����ȥȡģ���û����Ը����������������ɾ��
	*	2. ���������Ĺ��ܺ�ʹ�ÿ��Բο�������˵���Լ� lcd_test.c �ļ���Ĳ��Ժ���
	*
	*********************************************************************************************************************************************************************************************FANKE*****
***/

#include "lcd_rgb.h"

LTDC_HandleTypeDef hltdc;	// LTDC_HandleTypeDef �ṹ�����

static pFONT *LCD_Fonts;		// Ӣ������
static pFONT *LCD_CHFonts;		// ��������

//LCD��ز����ṹ��
struct	
{
	uint32_t Color; 				//	LCD��ǰ������ɫ
	uint32_t BackColor;			//	����ɫ
	uint32_t ColorMode;			//	��ɫ��ʽ
	uint32_t LayerMemoryAdd;	//	���Դ��ַ
	uint8_t  BytesPerPixel;		//	ÿ��������ռ�ֽ���	
	uint8_t  Layer; 				//	��ǰ��
	uint8_t  Direction;			//	��ʾ����
	uint8_t  ShowNum_Mode;		// ���ñ�����ʾʱ����λ��0���ǲ��ո�
}LCD;

/*************************************************************************************************
*	�� �� ��:	LCD_PanelModify
*	��ڲ���:	��
*	�� �� ֵ:	��
*	��������:	�����ʶ���޸ģ��˺���ֻ�Ծɿ�5�������ã�Ҳ���� RGB050M1  V1.0�İ汾
*	˵    ��:	��Ϊ�ɿ�5�������ֻ������18λ�Ľӿڣ�������Ҫ��Ӧ�İѵ�2λ�����ݽ�����͵�ƽ,
*					������Ļ�������
*************************************************************************************************/

void	LCD_PanelModify(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;	

	GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;				//	���ģʽ
	GPIO_InitStruct.Pull 	= GPIO_NOPULL;							//	��������
	GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;				// �ٶȵȼ�2M
	
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
	
	// �ɿ�5����ֻ������18λ�Ľӿڣ��������ݽ��ǽӵص�
	// �����Ҫ��LTDC��Ӧ�ĵ�2λ���ݽ�����͵�ƽ
	
	HAL_GPIO_WritePin(LTDC_R0_PORT, LTDC_R0_PIN, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(LTDC_R1_PORT, LTDC_R1_PIN, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(LTDC_G0_PORT, LTDC_G0_PIN, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(LTDC_G1_PORT, LTDC_G1_PIN, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(LTDC_B0_PORT, LTDC_B0_PIN, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(LTDC_B1_PORT, LTDC_B1_PIN, GPIO_PIN_RESET);	
			
}


/*************************************************************************************************
*	�� �� ��:	HAL_LTDC_MspInit
*	��ڲ���:	��
*	�� �� ֵ:	��
*	��������:	��ʼ��LTDC���ŵ�IO��
*	˵    ��:	��	
*************************************************************************************************/

void HAL_LTDC_MspInit(LTDC_HandleTypeDef* hltdc)
{ 
   uint8_t Modify_Flag = 0;	// ���������޸ı�־λ
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(hltdc->Instance==LTDC)
	{
		__HAL_RCC_LTDC_CLK_ENABLE();	// ����LTDCʱ��

		GPIO_LTDC_Black_CLK_ENABLE;	// �����������Ŷ˿�ʱ��
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
	
/*----------------------------�����ʶ��---------------------------------*/
	

		GPIO_InitStruct.Mode 	= GPIO_MODE_INPUT;		//	����ģʽ
		GPIO_InitStruct.Pull 	= GPIO_NOPULL;				//	��������
		
		GPIO_InitStruct.Pin 		= LTDC_R0_PIN; 			//	R0
		HAL_GPIO_Init(LTDC_R0_PORT, &GPIO_InitStruct);		

		GPIO_InitStruct.Pin 		= LTDC_G0_PIN; 			//	G0
		HAL_GPIO_Init(LTDC_G0_PORT, &GPIO_InitStruct);		

		GPIO_InitStruct.Pin 		= LTDC_B0_PIN; 			//	B0
		HAL_GPIO_Init(LTDC_B0_PORT, &GPIO_InitStruct);		


		// �����ʶ���޸ģ��˴�ֻ�Ծɿ�5�������ã�Ҳ���� RGB050M1  V1.0�İ汾
		// �ɿ�5����ֻ������18λ�Ľӿڣ���2λ���ݽ�ֱ�ӽӵأ������Ӧ������Ϊ�͵�ƽ�����ȷ��������Ǿɿ�5����
		// ��������������˴����û���ֲ��ʱ�����ֱ��ɾ��
		if(   (HAL_GPIO_ReadPin(LTDC_R0_PORT,LTDC_R0_PIN) == 0) \
			&& (HAL_GPIO_ReadPin(LTDC_G0_PORT,LTDC_G0_PIN) == 0) \
			&& (HAL_GPIO_ReadPin(LTDC_B0_PORT,LTDC_B0_PIN) == 0) )	
		{

			Modify_Flag	= 1;	// �����⵽�˾ɿ�5����������λ��־λ	
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


		// ��������
		GPIO_InitStruct.Pin 		= LTDC_Black_PIN;
		GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull 	= GPIO_PULLUP;
		GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(LTDC_Black_PORT, &GPIO_InitStruct);

		HAL_GPIO_WritePin(LTDC_Black_PORT, LTDC_Black_PIN, GPIO_PIN_RESET);	// �ȹرձ������ţ���ʼ��֮���ٿ���
	}

// �����ʶ���޸ģ��˴�ֻ�Ծɿ�5�������ã�Ҳ���� RGB050M1  V1.0�İ汾
// �ɿ�5����ֻ������18λ�Ľӿڣ���2λ���ݽ�ֱ�ӽӵأ������Ӧ������Ϊ�͵�ƽ�����ȷ��������Ǿɿ�5����
// ��������������˴����û���ֲ��ʱ�����ֱ��ɾ��
	if( Modify_Flag == 1 )
	{
		LCD_PanelModify();	// ����Ӧ�ĵ�2λ��������͵�ƽ������Ƭ�����Ļ�ܸ�
	}

}


/*************************************************************************************************
*	�� �� ��:	LCD_Init
*	��ڲ���:	��
*	�� �� ֵ:	��
*	��������:	��ʼ��LTDC���ŵ�IO�ڡ�ȫ�ֲ����������õ�
*	˵    ��:	��			
*************************************************************************************************/

void LTDC_Init(void)
{ 
	__HAL_RCC_DMA2D_CLK_ENABLE();	// ʹ��DMA2D
	
	LTDC_LayerCfgTypeDef pLayerCfg = {0};

	hltdc.Instance 		 = LTDC;		
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;	// �͵�ƽ��Ч
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;	// �͵�ƽ��Ч
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;	// �͵�ƽ��Ч��Ҫע����ǣ��ܶ���嶼�Ǹߵ�ƽ��Ч������429��Ҫ���óɵ͵�ƽ����������ʾ
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;	// ����ʱ���ź�

	hltdc.Init.HorizontalSync 		= HSW - 1;									// ������Ļ���ò�������
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
	
	HAL_LTDC_Init(&hltdc);	// ��ʼ��LTDC

	/*---------------------------------- layer0 ��ʾ���� --------------------------------*/

	pLayerCfg.WindowX0 			= 0;										// ˮƽ���
	pLayerCfg.WindowX1 			= LCD_Width;							// ˮƽ�յ�
	pLayerCfg.WindowY0 			= 0;										// ��ֱ���
	pLayerCfg.WindowY1 			= LCD_Height;							// ��ֱ�յ�
	pLayerCfg.PixelFormat		= ColorMode_0;							// ��ɫ��ʽ
	pLayerCfg.ImageWidth 		= LCD_Width;							// ��ʾ������
	pLayerCfg.ImageHeight 		= LCD_Height;							// ��ʾ����߶�	
	
// ���� layer0 �ĺ㶨͸���ȣ�����д�� LTDC_LxCACR �Ĵ��� 
//	��Ҫע����ǣ����������ֱ���������� layer0 ��͸���ȣ���������Ϊ255����͸�� 	
	pLayerCfg.Alpha 				= 255;									// ͸����
	
	
// ���� layer1 �Ĳ���ϵ��������д�� LTDC_LxBFCR �Ĵ��� 
// �ò����������� layer1 �� (layer0+������֮�����ɫ���ϵ�������㹫ʽΪ ��
// ��Ϻ����ɫ =  BF1 * layer1����ɫ + BF2 * (layer0+������Ϻ����ɫ��
// ��� layer1 ʹ����͸��ɫ����������ó� LTDC_BLENDING_FACTOR1_PAxCA �� LTDC_BLENDING_FACTOR2_PAxCA������ARGB�е�Aͨ����������
//	����Ľ��ܿ��Բ��� �ο��ֲ���� LTDC_LxBFCR �Ĵ����Ľ���	
	pLayerCfg.BlendingFactor1 	= LTDC_BLENDING_FACTOR1_PAxCA;	// ���ϵ��
	pLayerCfg.BlendingFactor2 	= LTDC_BLENDING_FACTOR2_PAxCA;	// ���ϵ��
	
// layer0 ���Դ��ַ��������ʹ���ⲿ��SDRAM��Ϊ�Դ棬��ʼ��ַ0xD0000000��SDRAM��СΪ16M
// layer0 �Դ��С���� = LCD_Width * LCD_Width * BytesPerPixel_0��ÿ��������ռ�ֽڴ�С��
// ��Ϊ SDRAM ��СΪ16M���û����õ�����һ�����ܳ������ֵ��		
	pLayerCfg.FBStartAdress 	= LCD_MemoryAdd;						// �Դ��ַ
	
// ����layer1 �ĳ�ʼĬ����ɫ������A,R,G,B ��ֵ ������д�� LTDC_LxDCCR �Ĵ��� 	
	pLayerCfg.Alpha0 				= 0;										// ͸����
	pLayerCfg.Backcolor.Blue 	= 0;										//	��ʼ��ɫ
	pLayerCfg.Backcolor.Green 	= 0;										//	��ʼ��ɫ
	pLayerCfg.Backcolor.Red 	= 0;										//	��ʼ��ɫ
	
	HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) ;					// ���ò�0��������

	#if ( ( ColorMode_0 == LTDC_PIXEL_FORMAT_RGB888 )||( ColorMode_0 == LTDC_PIXEL_FORMAT_ARGB8888 ) ) // �ж��Ƿ�ʹ��24λ����32λɫ

	//ʹ����ɫ������24λ���ϵ���ɫ����򿪣������޷��ﵽ��Ӧ��Ч��
		HAL_LTDC_EnableDither(&hltdc); // ������ɫ����
		
	// ����ɫ��ʽΪ24λɫʱ����������֡�������ļĴ���������32λ��ʽ�����ã���ÿ������ռ4�ֽ�
	//	���ʹ��HAL��Ĭ�ϵ����ã���ˢ��������ʾ�ַ���ʱ�����������Ļ����
	// �������õ�ֻ��֡�������ĸ�ʽ����SDRAM�Դ��޹�		
		LTDC_Layer1->CFBLR 	= (LCD_Width * 4 << 16 ) | (LCD_Width * 4 + 3) ;	
		HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING);		// �����������	
	#endif



/*---------------------------------- layer1 ��ʾ���� --------------------------------*/

#if ( LCD_NUM_LAYERS == 2 )	//��������˫��ʱ

	LTDC_LayerCfgTypeDef pLayerCfg1 = {0};	
	
	pLayerCfg1.WindowX0 			= 0;													// ˮƽ���
	pLayerCfg1.WindowX1 			= LCD_Width;                              // ˮƽ�յ�
	pLayerCfg1.WindowY0 			= 0;                                      // ��ֱ���
	pLayerCfg1.WindowY1 			= LCD_Height;                             // ��ֱ�յ�
	pLayerCfg1.PixelFormat 		= ColorMode_1;                            // ��ɫ��ʽ
	pLayerCfg1.ImageWidth 		= LCD_Width;                              // ��ʾ������
	pLayerCfg1.ImageHeight	 	= LCD_Height;                             // ��ʾ����߶�	
	
// ���� layer1 �ĺ㶨͸���ȣ�����д�� LTDC_LxCACR �Ĵ��� 
//	��Ҫע����ǣ����������ֱ���������� layer1 ��͸���ȣ���������Ϊ255����͸�� 	
	pLayerCfg1.Alpha 				= 255;	                                 // ͸����
	
// ���� layer1 �Ĳ���ϵ��������д�� LTDC_LxBFCR �Ĵ��� 
// �ò����������� layer1 �� (layer0+������֮�����ɫ���ϵ�������㹫ʽΪ ��
// ��Ϻ����ɫ =  BF1 * layer1����ɫ + BF2 * (layer0+������Ϻ����ɫ��
// ��� layer1 ʹ����͸��ɫ����������ó� LTDC_BLENDING_FACTOR1_PAxCA �� LTDC_BLENDING_FACTOR2_PAxCA������ARGB�е�Aͨ����������
//	����Ľ��ܿ��Բ��� �ο��ֲ���� LTDC_LxBFCR �Ĵ����Ľ���
	pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;            // ���ϵ��
	pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;            // ���ϵ��
	
// layer1 ���Դ��ַ��������ʹ���ⲿ��SDRAM��Ϊ�Դ棬��ʼ��ַ0xD0000000��SDRAM��СΪ16M
// ���� layer0 ��ռ��һ�����Դ棬������� layer1 �Դ�ʱ����Ҫ����һ��ƫ��
	pLayerCfg1.FBStartAdress 	= LCD_MemoryAdd + LCD_MemoryAdd_OFFSET;   // �Դ��ַ
	
// ����layer1 �ĳ�ʼĬ����ɫ������A,R,G,B ��ֵ ������д�� LTDC_LxDCCR �Ĵ��� 	
	pLayerCfg1.Alpha0 			= 0;                                      // ͸����
	pLayerCfg1.Backcolor.Blue 	= 0;                                      //	��ʼ��ɫ
	pLayerCfg1.Backcolor.Green = 0;                                      //	��ʼ��ɫ
	pLayerCfg1.Backcolor.Red 	= 0;                                      //	��ʼ��ɫ
	
	HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) ;                       // ���ò�1��ǰ����

	#if ( ( ColorMode_1 == LTDC_PIXEL_FORMAT_RGB888 )||( ColorMode_1 == LTDC_PIXEL_FORMAT_ARGB8888 ) ) // �ж��Ƿ�ʹ��24λ����32λɫ

	//ʹ����ɫ������24λ���ϵ���ɫ����򿪣������޷��ﵽ��Ӧ��Ч��
		HAL_LTDC_EnableDither(&hltdc); // ������ɫ����

	// ����ɫ��ʽΪ24λɫʱ����������֡�������ļĴ���������32λ��ʽ�����ã���ÿ������ռ4�ֽ�
	//	���ʹ��HAL��Ĭ�ϵ����ã���ˢ��������ʾ�ַ���ʱ�����������Ļ����
	// �������õ�ֻ��֡�������ĸ�ʽ����SDRAM�Դ��޹�	
		LTDC_Layer2->CFBLR 	= (LCD_Width * 4 << 16 ) | (LCD_Width * 4 + 3) ;
		HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING);		// �����������			
	#endif

#endif
	
/*---------------------------------- ��ʼ��һЩĬ������ --------------------------------*/	
	
	LCD_DisplayDirection(Direction_H); 	//	������ʾ
	LCD_SetFont(&Font24);  					//	����Ĭ������	
	LCD_ShowNumMode(Fill_Space);			//	������ʾĬ�����ո�
	
	LCD_SetLayer(0);  
	LCD_SetBackColor(LCD_BLACK); 			//	���ñ���ɫ
	LCD_SetColor(LCD_WHITE);				//	���û�����ɫ
	LCD_Clear(); 								//	������ˢ����ɫ
	
/*---------------------------------- ���������������ʾ --------------------------------*/	

#if LCD_NUM_LAYERS == 2	 //	��������˵ڶ���
	LCD_SetLayer(1); 
	LCD_SetBackColor(LCD_BLACK); 	//	���ñ���ɫ
	LCD_SetColor(LCD_WHITE);		//	���û�����ɫ
	LCD_Clear(); 						//	������ˢ����ɫ
#endif
// LTDC�ڳ�ʼ��֮���ϵ��˲�����һ�����ݵİ�����
//	��ʹһ��ʼ�ͽ������������Ͳ�����Ļ���������õ����������ǻ����������
//	�����Ҫ����������󣬿����ڳ�ʼ�����֮�󣬽���һ�����ݵ���ʱ�ٴ򿪱���
//
//	HAL_Delay(200);
  	HAL_GPIO_WritePin(LTDC_Black_PORT, LTDC_Black_PIN, GPIO_PIN_SET);	// ��������
}  


/*************************************************************************************************
*	�� �� ��:	LCD_SetLayer
*	��ڲ���:	layer - Ҫ��ʾ�Ͳ����Ĳ㣬��������Ϊ0��1����ѡ�� layer0 �� layer1
*	�� �� ֵ:	��
*	��������:	����Ҫ��ʾ�Ͳ����Ĳ㣬�л���Ӧ���Դ��ַ����ɫ��ʽ��
*	˵    ��:	LTDC��˳���ǹ̶��ģ�layer1 �� layer0֮�ϣ�������������ʾʱ��
*					layer1 ��ǰ���㣬ͨ��ʹ�ô�͸��ɫ����ɫ��ʽ��layer0 �Ǳ����㣬
*					ֻ��������ʱ��Ĭ��ֻ���� layer0
*************************************************************************************************/

void LCD_SetLayer(uint8_t layer)
{
#if LCD_NUM_LAYERS == 2		// �������˫��
	
	if (layer == 0)			// ������õ��� layer0
	{
		LCD.LayerMemoryAdd = LCD_MemoryAdd; 	// ��ȡ layer0 ���Դ��ַ
		LCD.ColorMode      = ColorMode_0;		// ��ȡ layer0 ����ɫ��ʽ
		LCD.BytesPerPixel  = BytesPerPixel_0;	// ��ȡ layer0 ��ÿ�����������ֽ����Ĵ�С
	}
	else if(layer == 1)	 // ������õ��� layer1
	{
		LCD.LayerMemoryAdd = LCD_MemoryAdd + LCD_MemoryAdd_OFFSET;	// ��ȡ layer1 ���Դ��ַ
		LCD.ColorMode      = ColorMode_1;                           // ��ȡ layer1 ����ɫ��ʽ
		LCD.BytesPerPixel  = BytesPerPixel_1;		                  // ��ȡ layer1 ��ÿ�����������ֽ����Ĵ�С
	}
	LCD.Layer = layer;	//��¼��ǰ���ڵĲ�
	
#else		// ���ֻ�������㣬Ĭ�ϲ��� layer0
	
	LCD.LayerMemoryAdd = LCD_MemoryAdd;		// ��ȡ layer0 ���Դ��ַ
	LCD.ColorMode      = ColorMode_0;      // ��ȡ layer0 ����ɫ��ʽ
	LCD.BytesPerPixel  = BytesPerPixel_0;	// ��ȡ layer0 ��ÿ�����������ֽ����Ĵ�С
	LCD.Layer = 0;		// ��������Ϊ layer0
	
#endif

}  

/***************************************************************************************************************
*	�� �� ��:	LCD_SetColor
*
*	��ڲ���:	Color - Ҫ��ʾ����ɫ��ʾ����0xff0000FF ��ʾ��͸������ɫ��0xAA0000FF ��ʾ͸����Ϊ66.66%����ɫ
*
*	��������:	�˺�������������ʾ�ַ������㻭�ߡ���ͼ����ɫ
*
*	˵    ��:	1. Ϊ�˷����û�ʹ���Զ�����ɫ����ڲ��� Color ʹ��32λ����ɫ��ʽ���û����������ɫ��ʽ��ת��
*					2. 32λ����ɫ�У��Ӹ�λ����λ�ֱ��Ӧ A��R��G��B  4����ɫͨ��
*					3. ��8λ��͸��ͨ���У�ff��ʾ��͸����0��ʾ��ȫ͸��
*					4. ����ʹ��ARGB1555��ARGB8888��֧��͸��ɫ����ɫ��ʽ����Ȼ͸��ɫ�������ã�����ARGB1555��֧��һλ
*						͸��ɫ��������͸���Ͳ�͸������״̬��ARGB8888֧��255��͸����
*					5. ����˵����͸������ָ �����㡢layer0��layer1 ֮���͸��
*
***************************************************************************************************************/

void LCD_SetColor(uint32_t Color)
{
	uint16_t Alpha_Value = 0, Red_Value = 0, Green_Value = 0, Blue_Value = 0; //������ɫͨ����ֵ

	if( LCD.ColorMode == LTDC_PIXEL_FORMAT_RGB565	)	//��32λɫת��Ϊ16λɫ
	{
		Red_Value   = (uint16_t)((Color&0x00F80000)>>8);
		Green_Value = (uint16_t)((Color&0x0000FC00)>>5);
		Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);
		LCD.Color = (uint16_t)(Red_Value | Green_Value | Blue_Value);		
	}
	else if( LCD.ColorMode == LTDC_PIXEL_FORMAT_ARGB1555 )	//��32λɫת��ΪARGB1555��ɫ
	{
		if( (Color & 0xFF000000) == 0 )	//�ж��Ƿ�ʹ��͸��ɫ
			Alpha_Value = 0x0000;
		else
			Alpha_Value = 0x8000;

		Red_Value   = (uint16_t)((Color&0x00F80000)>>9);	
		Green_Value = (uint16_t)((Color&0x0000F800)>>6);
		Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);
		LCD.Color = (uint16_t)(Alpha_Value | Red_Value | Green_Value | Blue_Value);	
	}
	else if( LCD.ColorMode == LTDC_PIXEL_FORMAT_ARGB4444 )	//��32λɫת��ΪARGB4444��ɫ
	{

		Alpha_Value = (uint16_t)((Color&0xf0000000)>>16);
		Red_Value   = (uint16_t)((Color&0x00F00000)>>12);	
		Green_Value = (uint16_t)((Color&0x0000F000)>>8);
		Blue_Value  = (uint16_t)((Color&0x000000F8)>>4);
		LCD.Color = (uint16_t)(Alpha_Value | Red_Value | Green_Value | Blue_Value);	
	}	
	else
		LCD.Color = Color;	//24λɫ��32λɫ����Ҫת��
}

/***************************************************************************************************************
*	�� �� ��:	LCD_SetBackColor
*
*	��ڲ���:	Color - Ҫ��ʾ����ɫ��ʾ����0xff0000FF ��ʾ��͸������ɫ��0xAA0000FF ��ʾ͸����Ϊ66.66%����ɫ
*
*	��������:	���ñ���ɫ,�˺������������Լ���ʾ�ַ��ı���ɫ
*
*	˵    ��:	1. Ϊ�˷����û�ʹ���Զ�����ɫ����ڲ��� Color ʹ��32λ����ɫ��ʽ���û����������ɫ��ʽ��ת��
*					2. 32λ����ɫ�У��Ӹ�λ����λ�ֱ��Ӧ A��R��G��B  4����ɫͨ��
*					3. ��8λ��͸��ͨ���У�ff��ʾ��͸����0��ʾ��ȫ͸��
*					4. ����ʹ��ARGB1555��ARGB8888��֧��͸��ɫ����ɫ��ʽ����Ȼ͸��ɫ�������ã�����ARGB1555��֧��һλ
*						͸��ɫ��������͸���Ͳ�͸������״̬��ARGB8888֧��255��͸����
*					5. ����˵����͸������ָ �����㡢layer0��layer1֮���͸��
*
***************************************************************************************************************/

void LCD_SetBackColor(uint32_t Color)
{
	uint16_t Alpha_Value = 0, Red_Value = 0, Green_Value = 0, Blue_Value = 0;  //������ɫͨ����ֵ

	if( LCD.ColorMode == LTDC_PIXEL_FORMAT_RGB565	)	//��32λɫת��Ϊ16λɫ
	{
		Red_Value   	= (uint16_t)((Color&0x00F80000)>>8);
		Green_Value 	= (uint16_t)((Color&0x0000FC00)>>5);
		Blue_Value  	= (uint16_t)((Color&0x000000F8)>>3);
		LCD.BackColor	= (uint16_t)(Red_Value | Green_Value | Blue_Value);	
	}
	else if( LCD.ColorMode == LTDC_PIXEL_FORMAT_ARGB1555 )	//��32λɫת��ΪARGB1555��ɫ
	{
		if( (Color & 0xFF000000) == 0 )	//�ж��Ƿ�ʹ��͸��ɫ
			Alpha_Value = 0x0000;
		else
			Alpha_Value = 0x8000;

		Red_Value   	= (uint16_t)((Color&0x00F80000)>>9);
		Green_Value 	= (uint16_t)((Color&0x0000F800)>>6);
		Blue_Value  	= (uint16_t)((Color&0x000000F8)>>3);
		LCD.BackColor 	= (uint16_t)(Alpha_Value | Red_Value | Green_Value | Blue_Value);	
	}
	else if( LCD.ColorMode == LTDC_PIXEL_FORMAT_ARGB4444 )	//��32λɫת��ΪARGB4444��ɫ
	{

		Alpha_Value 	= (uint16_t)((Color&0xf0000000)>>16);
		Red_Value   	= (uint16_t)((Color&0x00F00000)>>12);	
		Green_Value 	= (uint16_t)((Color&0x0000F000)>>8);
		Blue_Value  	= (uint16_t)((Color&0x000000F8)>>4);
		LCD.BackColor 	= (uint16_t)(Alpha_Value | Red_Value | Green_Value | Blue_Value);	
	}		
	
	else	
		LCD.BackColor = Color;	//24λɫ��32λɫ����Ҫת��
	
}


/***************************************************************************************************************
*	�� �� ��:	LCD_SetFont
*
*	��ڲ���:	*fonts - Ҫ���õ�ASCII����
*
*	��������:	����ASCII���壬��ѡ��ʹ�� 3216/2412/2010/1608/1206 ���ִ�С������
*
*	˵    ��:	1. ʹ��ʾ�� LCD_SetFont(&Font24) �������� 2412�� ASCII����
*					2. �����ģ����� lcd_fonts.c 			
*
***************************************************************************************************************/

void LCD_SetFont(pFONT *fonts)
{
	LCD_Fonts = fonts;
}

/***************************************************************************************************************
*	�� �� ��:	LCD_DisplayDirection
*
*	��ڲ���:	direction - Ҫ��ʾ�ķ���
*
*	��������:	����Ҫ��ʾ�ķ��򣬿�������� Direction_H ���������ʾ��Direction_V ������ֱ��ʾ
*
*	˵    ��:   ʹ��ʾ�� LCD_DisplayDirection(Direction_H) ����������Ļ������ʾ
*
***************************************************************************************************************/

void LCD_DisplayDirection(uint8_t direction)
{
	LCD.Direction = direction;
}

/***************************************************************************************************************
*	�� �� ��:	LCD_Clear
*
*	��������:	������������LCD���Ϊ LCD.BackColor ����ɫ��ʹ��DMA2Dʵ��
*
*	˵    ��:	���� LCD_SetBackColor() ����Ҫ����ı���ɫ���ٵ��øú�����������
*
***************************************************************************************************************/

void LCD_Clear(void)
{
	DMA2D->CR	  &=	~(DMA2D_CR_START);				//	ֹͣDMA2D
	DMA2D->CR		=	DMA2D_R2M;							//	�Ĵ�����SDRAM
	DMA2D->OPFCCR	=	LCD.ColorMode;						//	������ɫ��ʽ
	DMA2D->OOR		=	0;										//	������ƫ�� 
	DMA2D->OMAR		=	LCD.LayerMemoryAdd ;				// ��ַ
	DMA2D->NLR		=	(LCD_Width<<16)|(LCD_Height);	//	�趨���ȺͿ��
	DMA2D->OCOLR	=	LCD.BackColor;						//	��ɫ
	
// �ȴ� ��ֱ����ʹ����ʾ״̬ ����LTDC����ˢ��һ�������ݵ�ʱ��
// ��Ϊ����Ļû��ˢ��һ֡ʱ����ˢ��������˺�ѵ�����
// �û�Ҳ����ʹ�� �Ĵ��������ж� �����жϣ�����Ϊ�˱�֤���̵ļ���Լ���ֲ�ķ����ԣ�����ֱ��ʹ���жϼĴ����ķ���
//
//
	while( LTDC->CDSR != 0X00000001);	// �ж� ��ʾ״̬�Ĵ���LTDC_CDSR �ĵ�0λ VDES����ֱ����ʹ����ʾ״̬
	
	DMA2D->CR	  |=	DMA2D_CR_START;					//	����DMA2D
		
	while (DMA2D->CR & DMA2D_CR_START) ;				//	�ȴ��������
}

/***************************************************************************************************************
*	�� �� ��:	LCD_ClearRect
*
*	��ڲ���:	x - ��ʼˮƽ���꣬ȡֵ��Χ0~799 
*					y - ��ʼ��ֱ���꣬ȡֵ��Χ0~479
*					width  - Ҫ�������ĺ��򳤶�
*					height - Ҫ��������������
*
*	��������:	�ֲ�������������ָ��λ�ö�Ӧ���������Ϊ LCD.BackColor ����ɫ
*
*	˵    ��:	1. ���� LCD_SetBackColor() ����Ҫ����ı���ɫ���ٵ��øú�����������
*					2. ʹ��ʾ�� LCD_ClearRect( 10, 10, 100, 50) ���������(10,10)��ʼ�ĳ�100��50������
*
***************************************************************************************************************/

void LCD_ClearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{

	DMA2D->CR	  &=	~(DMA2D_CR_START);				//	ֹͣDMA2D
	DMA2D->CR		=	DMA2D_R2M;							//	�Ĵ�����SDRAM
	DMA2D->OPFCCR	=	LCD.ColorMode;						//	������ɫ��ʽ
	DMA2D->OCOLR	=	LCD.BackColor ;					//	��ɫ
	
	if(LCD.Direction == Direction_H)  //�������
	{		
		DMA2D->OOR		=	LCD_Width - width;				//	������ƫ�� 
		DMA2D->OMAR		=	LCD.LayerMemoryAdd + LCD.BytesPerPixel*(LCD_Width * y + x);	// ��ַ;
		DMA2D->NLR		=	(width<<16)|(height);			//	�趨���ȺͿ��		
	}
	else	//�������
	{		
		DMA2D->OOR		=	LCD_Width - height;		//	������ƫ�� 
		DMA2D->OMAR		=	LCD.LayerMemoryAdd + LCD.BytesPerPixel*((LCD_Height - x - 1 - width)*LCD_Width + y);	// ��ַ
		DMA2D->NLR		=	(width)|(height<<16);	//	�趨���ȺͿ��		
	}		

	DMA2D->CR	  |=	DMA2D_CR_START;					//	����DMA2D
		
	while (DMA2D->CR & DMA2D_CR_START) ;			//	�ȴ��������

}


/***************************************************************************************************************
*	�� �� ��:	LCD_DrawPoint
*
*	��ڲ���:	x - ��ʼˮƽ���꣬ȡֵ��Χ0~799 
*					y - ��ʼ��ֱ���꣬ȡֵ��Χ0~479
*					color  - Ҫ���Ƶ���ɫ��ʹ��32λ����ɫ��ʽ���û����������ɫ��ʽ��ת��
*
*	��������:	��ָ���������ָ����ɫ�ĵ�
*
*	˵    ��:	1. ֱ���ڶ�Ӧ���Դ�λ��д����ɫֵ������ʵ�ֻ���Ĺ���
*					2. ʹ��ʾ�� LCD_DrawPoint( 10, 10, 0xff0000FF) ��������(10,10)������ɫ�ĵ�
*
***************************************************************************************************************/

void LCD_DrawPoint(uint16_t x,uint16_t y,uint32_t color)
{

/*----------------------- 32λɫ ARGB8888 ģʽ ----------------------*/
		
	if( LCD.ColorMode == LTDC_PIXEL_FORMAT_ARGB8888 ) 
	{
		if (LCD.Direction == Direction_H) //ˮƽ����
		{
			*(__IO uint32_t*)( LCD.LayerMemoryAdd + 4*(x + y*LCD_Width) ) = color ; 	
		}
		else if(LCD.Direction == Direction_V)	//��ֱ����
		{
			*(__IO uint32_t*)( LCD.LayerMemoryAdd + 4*((LCD_Height - x - 1)*LCD_Width + y) ) = color ;
		}
	}
/*----------------------------- 24λɫ RGB888 ģʽ -------------------------*/	
	
	else if ( LCD.ColorMode == LTDC_PIXEL_FORMAT_RGB888 )
	{		
		if (LCD.Direction == Direction_H) //ˮƽ����
		{
			*(__IO uint16_t*)( LCD.LayerMemoryAdd + 3*(x + y*LCD_Width) ) = color ; 
			*(__IO uint8_t*)( LCD.LayerMemoryAdd + 3*(x + y*LCD_Width) + 2 ) = color>>16 ; 	
		}
		else if(LCD.Direction == Direction_V)	//��ֱ����
		{
			*(__IO uint16_t*)( LCD.LayerMemoryAdd + 3*((LCD_Height - x - 1)*LCD_Width + y) ) = color ; 
			*(__IO uint8_t*)( LCD.LayerMemoryAdd + 3*((LCD_Height - x - 1)*LCD_Width + y) +2) = color>>16 ; 	
		}	
	}

/*----------------------- 16λɫ ARGB1555��RGB565����ARGB4444 ģʽ ----------------------*/	
	else		
	{
		if (LCD.Direction == Direction_H) //ˮƽ����
		{
			*(__IO uint16_t*)( LCD.LayerMemoryAdd + 2*(x + y*LCD_Width) ) = color ; 	
		}
		else if(LCD.Direction == Direction_V)	//��ֱ����
		{
			*(__IO uint16_t*)( LCD.LayerMemoryAdd + 2*((LCD_Height - x - 1)*LCD_Width + y) ) = color ;
		}	
	}
}

/***************************************************************************************************************
*	�� �� ��:	LCD_ReadPoint
*
*	��ڲ���:	x - ��ʼˮƽ���꣬ȡֵ��Χ0~799 
*					y - ��ʼ��ֱ���꣬ȡֵ��Χ0~479
*
*	�� �� ֵ��  ��ȡ������ɫ
*
*	��������:	��ȡָ����������ɫ����ʹ��16��24λɫģʽʱ������������ɫ���ݶ�ӦΪ16λ��24λ
*
*	˵    ��:	1. ֱ�Ӷ�ȡ��Ӧ���Դ�ֵ������ʵ�ֶ���Ĺ���
*					2. ʹ��ʾ�� color = LCD_ReadPoint( 10, 10) ��color Ϊ��ȡ���������(10,10)����ɫ
*
***************************************************************************************************************/

uint32_t LCD_ReadPoint(uint16_t x,uint16_t y)
{
	uint32_t color = 0;

/*----------------------- 32λɫ ARGB8888 ģʽ ----------------------*/
	if( LCD.ColorMode == LTDC_PIXEL_FORMAT_ARGB8888 ) 
	{
		if (LCD.Direction == Direction_H) //ˮƽ����
		{
			color = *(__IO uint32_t*)( LCD.LayerMemoryAdd + 4*(x + y*LCD_Width) ); 	
		}
		else if(LCD.Direction == Direction_V)	//��ֱ����
		{
			color = *(__IO uint32_t*)( LCD.LayerMemoryAdd + 4*((LCD_Height - x - 1)*LCD_Width + y) );
		}
	}
	
/*----------------------------- 24λɫ RGB888 ģʽ -------------------------*/	
	else if ( LCD.ColorMode == LTDC_PIXEL_FORMAT_RGB888 )
	{
		if (LCD.Direction == Direction_H) //ˮƽ����
		{
			color = *(__IO uint32_t*)( LCD.LayerMemoryAdd + 3*(x + y*LCD_Width) ) &0x00ffffff; 	
		}
		else if(LCD.Direction == Direction_V)	//��ֱ����
		{
			color = *(__IO uint32_t*)( LCD.LayerMemoryAdd + 3*((LCD_Height - x - 1)*LCD_Width + y) ) &0x00ffffff; 	
		}	
	}
	
/*----------------------- 16λɫ ARGB1555��RGB565����ARGB4444 ģʽ ----------------------*/	
	else		
	{
		if (LCD.Direction == Direction_H) //ˮƽ����
		{
			color = *(__IO uint16_t*)( LCD.LayerMemoryAdd + 2*(x + y*LCD_Width) ); 	
		}
		else if(LCD.Direction == Direction_V)	//��ֱ����
		{
			color = *(__IO uint16_t*)( LCD.LayerMemoryAdd + 2*((LCD_Height - x - 1)*LCD_Width + y) );
		}	
	}
	return color;
}  
 
/***************************************************************************************************************
*	�� �� ��:	LCD_DisplayChar
*
*	��ڲ���:	x - ��ʼˮƽ���꣬ȡֵ��Χ0~799 
*					y - ��ʼ��ֱ���꣬ȡֵ��Χ0~479
*					c  - ASCII�ַ�
*
*	��������:	��ָ��������ʾָ�����ַ�
*
*	˵    ��:	1. ������Ҫ��ʾ�����壬����ʹ�� LCD_SetFont(&Font24) ����Ϊ 2412��ASCII����
*					2.	������Ҫ��ʾ����ɫ������ʹ�� LCD_SetColor(0xff0000FF) ����Ϊ��ɫ
*					3. �����ö�Ӧ�ı���ɫ������ʹ�� LCD_SetBackColor(0xff000000) ����Ϊ��ɫ�ı���ɫ
*					4. ʹ��ʾ�� LCD_DisplayChar( 10, 10, 'a') ��������(10,10)��ʾ�ַ� 'a'
*
***************************************************************************************************************/

void LCD_DisplayChar(uint16_t x, uint16_t y,uint8_t add)
{
	uint16_t  index = 0, counter = 0;
   uint8_t   disChar;	//��ģ��ֵ
	uint16_t  Xaddress = x; //ˮƽ����
	
	add = add - 32; 
	for(index = 0; index < LCD_Fonts->Sizes; index++)
	{
		disChar = LCD_Fonts->pTable[add*LCD_Fonts->Sizes + index]; //��ȡ�ַ���ģֵ
		for(counter = 0; counter < 8; counter++)
		{ 
			if(disChar & 0x01)	
			{		
				LCD_DrawPoint(Xaddress,y,LCD.Color);	//��ǰģֵ��Ϊ0ʱ��ʹ�û���ɫ���
			}
			else		
			{		
				LCD_DrawPoint(Xaddress,y,LCD.BackColor);	//����ʹ�ñ���ɫ���Ƶ�
			}
			disChar >>= 1;
			Xaddress++;  //ˮƽ�����Լ�
			
			if( (Xaddress - x)==LCD_Fonts->Width ) //���ˮƽ����ﵽ���ַ���ȣ����˳���ǰѭ��
			{													//������һ�еĻ���
				Xaddress = x;
				y++;
				break;
			}
		}	
	}
}

/***************************************************************************************************************
*	�� �� ��:	LCD_DisplayString
*
*	��ڲ���:	x - ��ʼˮƽ���꣬ȡֵ��Χ0~799 
*					y - ��ʼ��ֱ���꣬ȡֵ��Χ0~479
*					p - ASCII�ַ������׵�ַ
*
*	��������:	��ָ��������ʾָ�����ַ���
*
*	˵    ��:	1. ������Ҫ��ʾ�����壬����ʹ�� LCD_SetFont(&Font24) ����Ϊ 2412��ASCII����
*					2.	������Ҫ��ʾ����ɫ������ʹ�� LCD_SetColor(0xff0000FF) ����Ϊ��ɫ
*					3. �����ö�Ӧ�ı���ɫ������ʹ�� LCD_SetBackColor(0xff000000) ����Ϊ��ɫ�ı���ɫ
*					4. ʹ��ʾ�� LCD_DisplayString( 10, 10, "FANKE") ������ʼ����Ϊ(10,10)�ĵط���ʾ�ַ���"FANKE"
*
***************************************************************************************************************/

void LCD_DisplayString( uint16_t x, uint16_t y,  char *p) 
{  
	while ((x < LCD_Width) && (*p != 0))	//�ж���ʾ�����Ƿ񳬳���ʾ�������ַ��Ƿ�Ϊ���ַ�
	{
		 LCD_DisplayChar( x,y,*p);
		 x += LCD_Fonts->Width; //��ʾ��һ���ַ�
		 p++;	//ȡ��һ���ַ���ַ
	}
}

/***************************************************************************************************************
*	�� �� ��:	LCD_SetTextFont
*
*	��ڲ���:	*fonts - Ҫ���õ��ı�����
*
*	��������:	�����ı����壬�������ĺ�ASCII�ַ���
*
*	˵    ��:	1. ��ѡ��ʹ�� 3232/2424/2020/1616/1212 ���ִ�С���������壬
*						���Ҷ�Ӧ������ASCII����Ϊ 3216/2412/2010/1608/1206
*					2. �����ģ����� lcd_fonts.c 
*					3. �����ֿ�ʹ�õ���С�ֿ⣬���õ��˶�Ӧ�ĺ�����ȥȡģ
*					4. ʹ��ʾ�� LCD_SetTextFont(&CH_Font24) �������� 2424�����������Լ�2412��ASCII�ַ�����
*
***************************************************************************************************************/

void LCD_SetTextFont(pFONT *fonts)
{
	LCD_CHFonts = fonts;		// ������������
	switch(fonts->Width )
	{
		case 12:	LCD_Fonts = &Font12;	break;	// ����ASCII�ַ�������Ϊ 1206
		case 16:	LCD_Fonts = &Font16;	break;	// ����ASCII�ַ�������Ϊ 1608
		case 20:	LCD_Fonts = &Font20;	break;	// ����ASCII�ַ�������Ϊ 2010	
		case 24:	LCD_Fonts = &Font24;	break;	// ����ASCII�ַ�������Ϊ 2412
		case 32:	LCD_Fonts = &Font32;	break;	// ����ASCII�ַ�������Ϊ 3216		
		default: break;
	}

}

/***************************************************************************************************************
*	�� �� ��:	LCD_DisplayChinese
*
*	��ڲ���:	x - ��ʼˮƽ���꣬ȡֵ��Χ0~799 
*					y - ��ʼ��ֱ���꣬ȡֵ��Χ0~479
*					pText - �����ַ�
*
*	��������:	��ָ��������ʾָ���ĵ��������ַ�
*
*	˵    ��:	1. ������Ҫ��ʾ�����壬����ʹ�� LCD_SetTextFont(&CH_Font24) ����Ϊ 2424�����������Լ�2412��ASCII�ַ�����
*					2.	������Ҫ��ʾ����ɫ������ʹ�� LCD_SetColor(0xff0000FF) ����Ϊ��ɫ
*					3. �����ö�Ӧ�ı���ɫ������ʹ�� LCD_SetBackColor(0xff000000) ����Ϊ��ɫ�ı���ɫ
*					4. ʹ��ʾ�� LCD_DisplayChinese( 10, 10, "��") ��������(10,10)��ʾ�����ַ�"��"
*
***************************************************************************************************************/

void LCD_DisplayChinese(uint16_t x, uint16_t y, char *pText) 
{
	uint16_t  i=0,index = 0, counter = 0;	// ��������
	uint16_t  addr;	// ��ģ��ַ
   uint8_t   disChar;	//��ģ��ֵ
	uint16_t  Xaddress = x; //ˮƽ����

	while(1)
	{		
		// �Ա������еĺ��ֱ��룬���Զ�λ�ú�����ģ�ĵ�ַ		
		if ( *(LCD_CHFonts->pTable + (i+1)*LCD_CHFonts->Sizes + 0)==*pText && *(LCD_CHFonts->pTable + (i+1)*LCD_CHFonts->Sizes + 1)==*(pText+1) )	
		{   
			addr=i;	// ��ģ��ַƫ��
			break;
		}				
		i+=2;	// ÿ�������ַ�����ռ���ֽ�

		if(i >= LCD_CHFonts->Table_Rows)	break;	// ��ģ�б�������Ӧ�ĺ���	
	}	
	

	for(index = 0; index <LCD_CHFonts->Sizes; index++)
	{	
		disChar = *(LCD_CHFonts->pTable + (addr)*LCD_CHFonts->Sizes + index);	// ��ȡ��Ӧ����ģ��ַ
		
		for(counter = 0; counter < 8; counter++)
		{ 
			if(disChar & 0x01)	
			{		
				LCD_DrawPoint(Xaddress,y,LCD.Color);	//��ǰģֵ��Ϊ0ʱ��ʹ�û���ɫ���
			}
			else		
			{		
				LCD_DrawPoint(Xaddress,y,LCD.BackColor);	//����ʹ�ñ���ɫ���Ƶ�
			}
			disChar >>= 1;
			Xaddress++;  //ˮƽ�����Լ�
			
			if( (Xaddress - x)==LCD_CHFonts->Width ) 	//	���ˮƽ����ﵽ���ַ���ȣ����˳���ǰѭ��
			{														//	������һ�еĻ���
				Xaddress = x;
				y++;
				break;
			}
		}	
	}	

}


/***************************************************************************************************************
*	�� �� ��:	LCD_DisplayText
*
*	��ڲ���:	x - ��ʼˮƽ���꣬ȡֵ��Χ0~799 
*					y - ��ʼ��ֱ���꣬ȡֵ��Χ0~479
*					pText - �ַ�����������ʾ���Ļ���ASCII�ַ�
*
*	��������:	��ָ��������ʾָ�����ַ���
*
*	˵    ��:	1. ������Ҫ��ʾ�����壬����ʹ�� LCD_SetTextFont(&CH_Font24) ����Ϊ 2424�����������Լ�2412��ASCII�ַ�����
*					2.	������Ҫ��ʾ����ɫ������ʹ�� LCD_SetColor(0xff0000FF) ����Ϊ��ɫ
*					3. �����ö�Ӧ�ı���ɫ������ʹ�� LCD_SetBackColor(0xff000000) ����Ϊ��ɫ�ı���ɫ
*					4. ʹ��ʾ�� LCD_DisplayChinese( 10, 10, "���ͿƼ�STM32") ��������(10,10)��ʾ�ַ���"���ͿƼ�STM32"
*
***************************************************************************************************************/

void LCD_DisplayText(uint16_t x, uint16_t y, char *pText) 
{  
 	
	while(*pText != 0)	// �ж��Ƿ�Ϊ���ַ�
	{
		if(*pText<=0x7F)	// �ж��Ƿ�ΪASCII��
		{
			LCD_DisplayChar(x,y,*pText);	// ��ʾASCII
			x+=LCD_Fonts->Width;				// ˮƽ���������һ���ַ���
			pText++;								// �ַ�����ַ+1
		}
		else					// ���ַ�Ϊ����
		{			
			LCD_DisplayChinese(x,y,pText);	// ��ʾ����
			x+=LCD_CHFonts->Width;				// ˮƽ���������һ���ַ���
			pText+=2;								// �ַ�����ַ+2�����ֵı���Ҫ2�ֽ�
		}
	}	
}

/***************************************************************************************************************
*	�� �� ��:	LCD_ShowNumMode
*
*	��ڲ���:	mode - ���ñ�������ʾģʽ
*
*	��������:	���ñ�����ʾʱ����λ��0���ǲ��ո񣬿�������� Fill_Space ���ո�Fill_Zero �����
*
*	˵    ��:   1. ֻ�� LCD_DisplayNumber() ��ʾ���� �� LCD_DisplayDecimals()��ʾС�� �����������õ�
*					2. ʹ��ʾ�� LCD_ShowNumMode(Fill_Zero) ���ö���λ���0������ 123 ������ʾΪ 000123
*
***************************************************************************************************************/

void LCD_ShowNumMode(uint8_t mode)
{
	LCD.ShowNum_Mode = mode;
}

/*****************************************************************************************************************************************
*	�� �� ��:	LCD_DisplayNumber
*
*	��ڲ���:	x - ��ʼˮƽ���꣬ȡֵ��Χ0~799 
*					y - ��ʼ��ֱ���꣬ȡֵ��Χ0~479
*					number - Ҫ��ʾ������,��Χ�� -2147483648~2147483647 ֮��
*					len - ���ֵ�λ�������λ������len��������ʵ�ʳ�������������������š��������Ҫ��ʾ��������Ԥ��һ��λ�ķ�����ʾ�ռ�
*
*	��������:	��ָ��������ʾָ������������
*
*	˵    ��:	1. ������Ҫ��ʾ�����壬����ʹ�� LCD_SetTextFont(&CH_Font24) ����Ϊ 2424�����������Լ�2412��ASCII�ַ�����
*					2.	������Ҫ��ʾ����ɫ������ʹ�� LCD_SetColor(0xff0000FF) ����Ϊ��ɫ
*					3. �����ö�Ӧ�ı���ɫ������ʹ�� LCD_SetBackColor(0xff000000) ����Ϊ��ɫ�ı���ɫ
*					4. ʹ��ʾ�� LCD_DisplayNumber( 10, 10, a, 5) ��������(10,10)��ʾָ������a,�ܹ�5λ������λ��0��ո�
*						���� a=123 ʱ������� LCD_ShowNumMode()����������ʾ  123(ǰ�������ո�λ) ����00123
*						
*****************************************************************************************************************************************/

void  LCD_DisplayNumber( uint16_t x, uint16_t y, int32_t number, uint8_t len) 
{  
	char   Number_Buffer[15];				// ���ڴ洢ת������ַ���

	if( LCD.ShowNum_Mode == Fill_Zero)	// ����λ��0
	{
		sprintf( Number_Buffer , "%0.*d",len, number );	// �� number ת�����ַ�����������ʾ		
	}
	else			// ����λ���ո�
	{	
		sprintf( Number_Buffer , "%*d",len, number );	// �� number ת�����ַ�����������ʾ		
	}
	
	LCD_DisplayString( x, y,(char *)Number_Buffer) ;  // ��ת���õ����ַ�����ʾ����
	
}

/***************************************************************************************************************************************
*	�� �� ��:	LCD_DisplayDecimals
*
*	��ڲ���:	x - ��ʼˮƽ���꣬ȡֵ��Χ0~799 
*					y - ��ʼ��ֱ���꣬ȡֵ��Χ0~479
*					decimals - Ҫ��ʾ������, double��ȡֵ1.7 x 10^��-308��~ 1.7 x 10^��+308����������ȷ��׼ȷ����Чλ��Ϊ15~16λ
*
*       			len - ������������λ��������С����͸��ţ�����ʵ�ʵ���λ��������ָ������λ��������ʵ�ʵ��ܳ���λ�����
*							ʾ��1��С�� -123.123 ��ָ�� len <=8 �Ļ�����ʵ���ճ���� -123.123
*							ʾ��2��С�� -123.123 ��ָ�� len =10 �Ļ�����ʵ�����   -123.123(����ǰ����������ո�λ) 
*							ʾ��3��С�� -123.123 ��ָ�� len =10 �Ļ��������ú��� LCD_ShowNumMode() ����Ϊ���0ģʽʱ��ʵ����� -00123.123 
*
*					decs - Ҫ������С��λ������С����ʵ��λ��������ָ����С��λ����ָ���Ŀ�������������
*							 ʾ����1.12345 ��ָ�� decs Ϊ4λ�Ļ�����������Ϊ1.1235
*
*	��������:	��ָ��������ʾָ���ı���������С��
*
*	˵    ��:	1. ������Ҫ��ʾ�����壬����ʹ�� LCD_SetTextFont(&CH_Font24) ����Ϊ 2424�����������Լ�2412��ASCII�ַ�����
*					2.	������Ҫ��ʾ����ɫ������ʹ�� LCD_SetColor(0xff0000FF) ����Ϊ��ɫ
*					3. �����ö�Ӧ�ı���ɫ������ʹ�� LCD_SetBackColor(0xff000000) ����Ϊ��ɫ�ı���ɫ
*					4. ʹ��ʾ�� LCD_DisplayDecimals( 10, 10, a, 5, 3) ��������(10,10)��ʾ�ֱ���a,�ܳ���Ϊ5λ�����б���3λС��
*						
*****************************************************************************************************************************************/

void  LCD_DisplayDecimals( uint16_t x, uint16_t y, double decimals, uint8_t len, uint8_t decs) 
{  
	char  Number_Buffer[20];				// ���ڴ洢ת������ַ���
	
	if( LCD.ShowNum_Mode == Fill_Zero)	// ����λ���0ģʽ
	{
		sprintf( Number_Buffer , "%0*.*lf",len,decs, decimals );	// �� number ת�����ַ�����������ʾ		
	}
	else		// ����λ���ո�
	{
		sprintf( Number_Buffer , "%*.*lf",len,decs, decimals );	// �� number ת�����ַ�����������ʾ		
	}
	
	LCD_DisplayString( x, y,(char *)Number_Buffer) ;	// ��ת���õ����ַ�����ʾ����
}



/***************************************************************************************************************************************
*	�� �� ��: LCD_DrawImage
*
*	��ڲ���: x - ˮƽ���꣬ȡֵ��Χ 0~799
*			 	 y - ��ֱ���꣬ȡֵ��Χ 0~479
*			 	 width  - ͼƬ��ˮƽ��ȣ����ȡֵ800
*				 height - ͼƬ�Ĵ�ֱ��ȣ����ȡֵ480
*				*pImage - ͼƬ���ݴ洢�����׵�ַ
*
*	��������: ��ָ�����괦��ʾͼƬ
*
*	˵    ��: Ҫ��ʾ��ͼƬ��Ҫ���Ƚ���ȡģ����ֻ����ʾһ����ɫ��ʹ�� LCD_SetColor() �������û���ɫ
*						 
*****************************************************************************************************************************************/

void 	LCD_DrawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage) 
{  
   uint8_t   disChar;	//��ģ��ֵ
	uint16_t  Xaddress = x; //ˮƽ����
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
					LCD_DrawPoint(Xaddress,y,LCD.Color);	//��ǰģֵ��Ϊ0ʱ��ʹ�û���ɫ���
				}
				else		
				{		
					LCD_DrawPoint(Xaddress,y,LCD.BackColor);	//����ʹ�ñ���ɫ���Ƶ�
				}
				disChar >>= 1;
				Xaddress++;  //ˮƽ�����Լ�
				
				if( (Xaddress - x)==width ) //���ˮƽ����ﵽ���ַ���ȣ����˳���ǰѭ��
				{													//������һ�еĻ���
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
*	�� �� ��: LCD_DrawLine
*
*	��ڲ���: x1 - ��� ˮƽ���꣬ȡֵ��Χ 0~799
*			 	 y1 - ��� ��ֱ���꣬ȡֵ��Χ 0~479
*
*				 x2 - �յ� ˮƽ���꣬ȡֵ��Χ 0~799
*            y2 - �յ� ��ֱ���꣬ȡֵ��Χ 0~479
*
*	��������: ������֮�仭��
*
*	˵    ��: �ú�����ֲ��ST�ٷ������������
*						 
*****************************************************************************************************************************************/

#define ABS(X)  ((X) > 0 ? (X) : -(X))    

//	����������
//	������x1��y1Ϊ������꣬x2��y2Ϊ�յ�����
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
*	�� �� ��: LCD_DrawRect
*
*	��ڲ���: x - ˮƽ���꣬ȡֵ��Χ 0~799
*			 	 y - ��ֱ���꣬ȡֵ��Χ 0~479
*			 	 width  - ͼƬ��ˮƽ��ȣ����ȡֵ800
*				 height - ͼƬ�Ĵ�ֱ��ȣ����ȡֵ480
*
*	��������: ��ָ��λ�û���ָ������ľ�������
*
*	˵    ��: �ú�����ֲ��ST�ٷ������������
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
*	�� �� ��: LCD_DrawCircle
*
*	��ڲ���: x - Բ�� ˮƽ���꣬ȡֵ��Χ 0~799
*			 	 y - Բ�� ��ֱ���꣬ȡֵ��Χ 0~479
*			 	 r  - �뾶
*
*	��������: ������ (x,y) ���ư뾶Ϊ r ��Բ������
*
*	˵    ��: 1. �ú�����ֲ��ST�ٷ������������
*				 2. Ҫ���Ƶ������ܳ�����Ļ����ʾ����
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
*	�� �� ��: LCD_DrawEllipse
*
*	��ڲ���: x - Բ�� ˮƽ���꣬ȡֵ��Χ 0~799
*			 	 y - Բ�� ��ֱ���꣬ȡֵ��Χ 0~479
*			 	 r1  - ˮƽ����ĳ���
*				 r2  - ��ֱ����ĳ���
*
*	��������: ������ (x,y) ����ˮƽ����Ϊ r1 ��ֱ����Ϊ r2 ����Բ����
*
*	˵    ��: 1. �ú�����ֲ��ST�ٷ������������
*				 2. Ҫ���Ƶ������ܳ�����Ļ����ʾ����
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
*	�� �� ��: LCD_FillRect
*
*	��ڲ���: x - ˮƽ���꣬ȡֵ��Χ 0~799
*			 	 y - ��ֱ���꣬ȡֵ��Χ 0~479
*			 	 width  - ͼƬ��ˮƽ��ȣ����ȡֵ800
*				 height - ͼƬ�Ĵ�ֱ��ȣ����ȡֵ480
*
*	��������: ������ (x,y) ���ָ�������ʵ�ľ���
*
*	˵    ��: 1. ʹ��DMA2Dʵ��
*				 2. Ҫ���Ƶ������ܳ�����Ļ����ʾ����
*						 
*****************************************************************************************************************************************/

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
	DMA2D->CR	  &=	~(DMA2D_CR_START);				//	ֹͣDMA2D
	DMA2D->CR		=	DMA2D_R2M;							//	�Ĵ�����SDRAM
	DMA2D->OPFCCR	=	LCD.ColorMode;						//	������ɫ��ʽ
	DMA2D->OCOLR	=	LCD.Color;							//	��ɫ
	
	if(LCD.Direction == Direction_H)  //�������
	{		
		DMA2D->OOR		=	LCD_Width - width;				//	������ƫ�� 
		DMA2D->OMAR		=	LCD.LayerMemoryAdd + LCD.BytesPerPixel*(LCD_Width * y + x);	// ��ַ;
		DMA2D->NLR		=	(width<<16)|(height);			//	�趨���ȺͿ��		
	}
	else	//�������
	{		
		DMA2D->OOR		=	LCD_Width - height;		//	������ƫ�� 
		DMA2D->OMAR		=	LCD.LayerMemoryAdd + LCD.BytesPerPixel*((LCD_Height - x - 1 - width)*LCD_Width + y);	// ��ַ
		DMA2D->NLR		=	(width)|(height<<16);	//	�趨���ȺͿ��		
	}		

	DMA2D->CR	  |=	DMA2D_CR_START;					//	����DMA2D
		
	while (DMA2D->CR & DMA2D_CR_START) ;			//	�ȴ��������
}

/***************************************************************************************************************************************
*	�� �� ��: LCD_FillCircle
*
*	��ڲ���: x - Բ�� ˮƽ���꣬ȡֵ��Χ 0~799
*			 	 y - Բ�� ��ֱ���꣬ȡֵ��Χ 0~479
*			 	 r  - �뾶
*
*	��������: ������ (x,y) ���뾶Ϊ r ��Բ������
*
*	˵    ��: 1. �ú�����ֲ��ST�ٷ������������
*				 2. Ҫ���Ƶ������ܳ�����Ļ����ʾ����
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
// ʵ��ƽ̨������ STM32F429���İ�
//

