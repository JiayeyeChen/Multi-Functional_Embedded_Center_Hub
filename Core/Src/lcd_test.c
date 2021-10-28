/***
	***************************************************************************************************************************
	*	@file  	lcd_test.c
	*	@version V1.0
	*  @date    2020-9-29
	*	@author  ���ͿƼ�
	*	@brief   LTDC���Ժ���	
   ***************************************************************************************************************************
   *  @description
	*
	*	ʵ��ƽ̨������STM32F429BIT6���İ�(�ͺţ�FK429M1) + 800*480�ֱ��ʵ�RGB��Ļ
	*	�Ա���ַ��https://shop212360197.taobao.com
	*	QQ����Ⱥ��536665479
	*
>>>>> �ļ�˵����
	*
	*	1. �����ڲ��ԣ����ļ����Ǳ��룬�û���ֲ��ʱ���������
	*
	****************************************************************************************************************************
***/

#include "lcd_test.h"


/*************************************************************************************************
*	�� �� ��:	LCD_Test_DoubleLayer
*
*	��������:	˫����ʾ
*
*	˵    ��:	������˫���ʱ����Ч			
*************************************************************************************************/

void LCD_Test_DoubleLayer(void)
{
	uint16_t time = 100;	// ��ʱʱ��
	uint16_t i = 0;	
	
// ���Ƴ�ʼ���棬�������⡢LOGO�Լ�������>>>>>

	LCD_SetBackColor(0xffB9EDF8); 			//	���ñ���ɫ��ʹ���Զ�����ɫ
	LCD_Clear(); 									//	������ˢ����ɫ
	
	LCD_SetTextFont(&CH_Font32);				// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0xff333333);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DisplayText(334, 160,"˫�����");	// ��ʾ�ı�
	
	LCD_SetColor(0xfffd7923);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DrawImage(  280, 218, 240, 83, Image_FANKE_240x83) ;		// ��ʾLOGOͼƬ

	LCD_SetColor(LIGHT_YELLOW);		//	���û���ɫ
	for(i=0;i<150;i++)
   {
		LCD_FillRect(100,330,4*i,6);	// ���ƾ��Σ�ʵ�ּ��׽�������Ч��
		HAL_Delay(10);	
   }	
	
	LCD_SetLayer(1);	//�л�����1
	LCD_SetBackColor(LCD_BLACK & 0x00FFFFFF);  // ͸����100%����8λ������ԽС͸����Խ��
	LCD_Clear();										 // ��ʱǰ������ȫ͸���������޷����� layer0 ��ͼ��	
	
// layer0 ��ʾ���� >>>>>
		
	LCD_SetLayer(0);		//�л�����0	

	LCD_SetBackColor(LCD_BLACK);   
	LCD_Clear();
	
	LCD_SetColor(LCD_WHITE);	
	LCD_SetTextFont(&CH_Font32);				// ����2424��������,ASCII�����ӦΪ2412
	LCD_DisplayText(10,10,"STM32F429 LTDC ������, layer0"); 

	HAL_Delay(1000);
	
	LCD_SetColor(LIGHT_CYAN);  	LCD_DrawLine(45, 60,300, 60);	 	HAL_Delay(time);
	LCD_SetColor(LIGHT_MAGENTA);	LCD_DrawLine(45, 90,300, 90);   	HAL_Delay(time);
	LCD_SetColor(LIGHT_YELLOW); 	LCD_DrawLine(45,120,300,120);	 	HAL_Delay(time);
	LCD_SetColor(LIGHT_GREY); 		LCD_DrawLine(45,150,300,150);	 	HAL_Delay(time);
	LCD_SetColor(LIGHT_CYAN);  	LCD_DrawLine(45,180,300,180);	 	HAL_Delay(time);
	LCD_SetColor(LIGHT_MAGENTA);	LCD_DrawLine(45,210,300,210);   	HAL_Delay(time);
	LCD_SetColor(LIGHT_YELLOW); 	LCD_DrawLine(45,240,300,240);	 	HAL_Delay(time);

	LCD_SetColor(LCD_RED);    		LCD_FillCircle(118,350,85);		HAL_Delay(time);
	LCD_SetColor(LCD_GREEN);  		LCD_FillCircle(187,350,85); 	   HAL_Delay(time);
	LCD_SetColor(LCD_BLUE);   		LCD_FillCircle(268,350,85);  		HAL_Delay(time);
	
	LCD_SetColor(LIGHT_RED);	  	LCD_FillRect(387, 53,200,200);	HAL_Delay(time);	  
	LCD_SetColor(LIGHT_GREEN);  	LCD_FillRect(477,141,200,200);  	HAL_Delay(time);  
	LCD_SetColor(LIGHT_BLUE);    	LCD_FillRect(581,248,200,200);  	HAL_Delay(time);
	
	HAL_Delay(1000);
	
// layer1 ��ʾ���� >>>>>

	LCD_SetLayer(1);	//�л�����1

	LCD_SetColor(0Xff348498);		
	LCD_FillRect(100, 80,600,380);		  

	LCD_SetBackColor(0Xff348498);  
	LCD_SetColor(LCD_BLACK);		
	LCD_DisplayText(146,90,"ǰ����, layer1"); 
	HAL_Delay(1000);

#if ColorMode_1 == LTDC_PIXEL_FORMAT_ARGB8888	// ���layer1 ����Ϊ ARGB8888

	LCD_SetBackColor(LCD_BLACK & 0xE0FFFFFF);  // ���ñ���ɫ͸���ȣ���8λ������ԽС͸����Խ�ߣ�ARGB8888֧��8λ͸��ɫ����255��͸��״̬
	LCD_SetColor(LCD_WHITE);						 // ���ʲ�͸�� 
	LCD_DisplayText(110,140,"STM32F429BIT6 LTDC, ARGB8888, layer1"); 
	HAL_Delay(1000);
	
	LCD_SetBackColor(LCD_BLACK & 0xa0FFFFFF);  // ���ñ���ɫ͸���ȣ���8λ������ԽС͸����Խ�ߣ�ARGB8888֧��8λ͸��ɫ����255��͸��״̬
	LCD_SetColor(LCD_WHITE);		 
	LCD_DisplayText(110,200,"STM32F429BIT6 LTDC, ARGB8888, layer1"); 
	HAL_Delay(1000);
		
	LCD_SetBackColor(LCD_BLACK & 0x70FFFFFF );  // ���ñ���ɫ͸���ȣ���8λ������ԽС͸����Խ�ߣ�ARGB8888֧��8λ͸��ɫ����255��͸��״̬
	LCD_SetColor(LCD_WHITE);
	LCD_DisplayText(110,260,"STM32F429BIT6 LTDC, ARGB8888, layer1"); 	
	HAL_Delay(1000);
	
	LCD_SetBackColor(LCD_BLACK & 0x20FFFFFF );  // ���ñ���ɫ͸���ȣ���8λ������ԽС͸����Խ�ߣ�ARGB8888֧��8λ͸��ɫ����255��͸��״̬
	LCD_SetColor(LCD_WHITE);
	LCD_DisplayText(110,320,"STM32F429BIT6 LTDC, ARGB8888, layer1"); 	
	HAL_Delay(1000);	
	
	LCD_SetBackColor(LCD_BLACK);  				 // ������ɫ����Ϊ ��͸��
	LCD_SetColor(LCD_WHITE & 0x00FFFFFF);		 // ������ɫ����Ϊ ��ȫ͸��	 
	LCD_DisplayText(110,380,"STM32F429BIT6 LTDC, ARGB8888, layer1"); 	
	HAL_Delay(2000);	

#elif	ColorMode_1 == LTDC_PIXEL_FORMAT_ARGB4444	// ���layer1 ����Ϊ ARGB4444

	LCD_SetBackColor(LCD_BLACK & 0xB0FFFFFF);  // ���ñ���ɫ͸���ȣ���8λ������ԽС͸����Խ�ߣ�ARGB4444֧��4λ͸��ɫ����16��͸��״̬
	LCD_SetColor(LCD_WHITE);						 // ���ʲ�͸�� 
	LCD_DisplayText(110,140,"STM32F429BIT6 LTDC, ARGB4444, layer1"); 
	HAL_Delay(1000);
	
	LCD_SetBackColor(LCD_BLACK & 0x80FFFFFF);  // ���ñ���ɫ͸���ȣ���8λ������ԽС͸����Խ�ߣ�ARGB4444֧��4λ͸��ɫ����16��͸��״̬
	LCD_SetColor(LCD_WHITE);		 
	LCD_DisplayText(110,200,"STM32F429BIT6 LTDC, ARGB4444, layer1"); 
	HAL_Delay(1000);
		
	LCD_SetBackColor(LCD_BLACK & 0x30FFFFFF );  // ���ñ���ɫ͸���ȣ���8λ������ԽС͸����Խ�ߣ�ARGB4444֧��4λ͸��ɫ����16��͸��״̬
	LCD_SetColor(LCD_WHITE);
	LCD_DisplayText(110,260,"STM32F429BIT6 LTDC, ARGB4444, layer1"); 	
	HAL_Delay(1000);
	
	LCD_SetBackColor(LCD_BLACK  );  				 // ������ɫ����Ϊ ��͸��
	LCD_SetColor(LCD_WHITE & 0x00FFFFFF);		 // ������ɫ����Ϊ ��ȫ͸��	 
	LCD_DisplayText(110,320,"STM32F429BIT6 LTDC, ARGB4444, layer1"); 	
	HAL_Delay(2000);	
	
#elif	ColorMode_1 == LTDC_PIXEL_FORMAT_ARGB1555	// ���layer1 ����Ϊ ARGB1555


	LCD_SetBackColor(LCD_BLACK & 0xffFFFFF);  // ���ñ���ɫ͸���ȣ���8λ������ԽС͸����Խ�ߣ�ARGB1555��֧��һλ͸��ɫ��������͸���Ͳ�͸������״̬
	LCD_SetColor(LCD_WHITE);						// ���ʲ�͸�� 
	LCD_DisplayText(110,140,"STM32F429BIT6 LTDC, ARGB1555, layer1"); 
	HAL_Delay(1000);
	
	LCD_SetBackColor(LCD_BLACK & 0x00FFFFFF);  // ���ñ���ɫ͸���ȣ���8λ������ԽС͸����Խ�ߣ�ARGB1555��֧��һλ͸��ɫ��������͸���Ͳ�͸������״̬
	LCD_SetColor(LCD_WHITE);	  					 // ���ʲ�͸�� 
	LCD_DisplayText(110,200,"STM32F429BIT6 LTDC, ARGB1555, layer1"); 
	HAL_Delay(1000);
		
	LCD_SetBackColor(LCD_BLACK  & 0xffFFFFF);  // ������ɫ����Ϊ ��͸��
	LCD_SetColor(LCD_WHITE & 0x00FFFFFF);		 // ������ɫ����Ϊ ��ȫ͸��	 
	LCD_DisplayText(110,260,"STM32F429BIT6 LTDC, ARGB1555, layer1"); 	
	HAL_Delay(2000);	
	
#endif

}

/*************************************************************************************************
*	�� �� ��:	LCD_Test_Clear
*
*	��������:	��������
*
*	˵    ��:	��	
*************************************************************************************************/

void LCD_Test_Clear(void)
{
	uint16_t time = 1000;	// ��ʱʱ��
	uint8_t	i = 0;			// ��������
	
// ���Ƴ�ʼ���棬�������⡢LOGO�Լ�������>>>>>
	
	LCD_SetBackColor(0xffB9EDF8); 			//	���ñ���ɫ��ʹ���Զ�����ɫ
	LCD_Clear(); 									//	������ˢ����ɫ
	
	LCD_SetTextFont(&CH_Font32);				// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0xff333333);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DisplayText(334, 160,"ˢ������");	// ��ʾ�ı�
	
	LCD_SetColor(0xfffd7923);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DrawImage(  280, 218, 240, 83, Image_FANKE_240x83) ;		// ��ʾLOGOͼƬ

	LCD_SetColor(LIGHT_YELLOW);		//	���û���ɫ
	for(i=0;i<150;i++)
   {
		LCD_FillRect(100,330,4*i,6);	// ���ƾ��Σ�ʵ�ּ��׽�������Ч��
		HAL_Delay(10);	
   }	

		
// ˢ������>>>>>
		
	LCD_SetTextFont(&CH_Font32);			// ����2424��������,ASCII�����ӦΪ2412
	LCD_SetColor(LCD_BLACK);				// ���û�����ɫ

	for(i=0;i<8;i++)
	{
		switch (i)		// �л�����ɫ
		{
			case 0: LCD_SetBackColor(LIGHT_RED); 		break;	
			case 1: LCD_SetBackColor(LIGHT_GREEN); 	break;				
			case 2: LCD_SetBackColor(LIGHT_BLUE); 		break;
			case 3: LCD_SetBackColor(LIGHT_YELLOW); 	break;
			case 4: LCD_SetBackColor(LIGHT_CYAN);		break;
			case 5: LCD_SetBackColor(LIGHT_GREY); 		break;
			case 6: LCD_SetBackColor(LIGHT_MAGENTA); 	break;
			case 7: LCD_SetBackColor(LCD_WHITE); 		break;			
			default:	break;			
		}
		LCD_Clear();		// ����
		LCD_DisplayText(112, 84,"STM32F429 LTDC ˢ������");
		LCD_DisplayText(112,134,"���İ��ͺţ�FK429M1");
		LCD_DisplayText(112,184,"��Ļ�ֱ��ʣ�800*480");	
		HAL_Delay(time);	// ��ʱ
	}
}

/*************************************************************************************************
*	�� �� ��:	LCD_Test_Text
*
*	��������:	�ı���ʾ����
*
*	˵    ��:	��	
*************************************************************************************************/

void LCD_Test_Text(void)
{
	uint16_t i;					// ��������
	uint16_t time = 80;	// ��ʱʱ��
	
// ���Ƴ�ʼ���棬�������⡢LOGO�Լ�������>>>>>

	LCD_SetBackColor(0xffB9EDF8); 			//	���ñ���ɫ��ʹ���Զ�����ɫ
	LCD_Clear(); 									//	������ˢ����ɫ
	
	LCD_SetTextFont(&CH_Font32);				// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0xff333333);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DisplayText(334, 160,"�ı���ʾ");	// ��ʾ�ı�
	
	LCD_SetColor(0xfffd7923);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DrawImage(  280, 218, 240, 83, Image_FANKE_240x83) ;		// ��ʾLOGOͼƬ

	LCD_SetColor(LIGHT_YELLOW);		//	���û���ɫ
	for(i=0;i<150;i++)
   {
		LCD_FillRect(100,330,4*i,6);	// ���ƾ��Σ�ʵ�ּ��׽�������Ч��
		HAL_Delay(10);	
   }	
		
	
// ��ʾ�ı����������������С�����ĺ�ASCII�ַ� >>>>>

	LCD_SetBackColor(LCD_BLACK); 			//	���ñ���ɫ
	LCD_Clear(); 								// ����
	
	LCD_SetColor(LCD_WHITE);	// ���û��ʣ���ɫ
	LCD_SetFont(&Font32); LCD_DisplayString(0,  0,"!#$%&'()*+,-.0123456789:;<=>?@ABCDEFGHIJKLMNOPQRST"); HAL_Delay(time);		
	LCD_SetFont(&Font24); LCD_DisplayString(0, 32,"!#$%&'()*+,-.0123456789:;<=>?@ABCDEFGHIJKLMNOPQRST"); HAL_Delay(time);
	LCD_SetFont(&Font20); LCD_DisplayString(0, 56,"!#$%&'()*+,-.0123456789:;<=>?@ABCDEFGHIJKLMNOPQRST"); HAL_Delay(time);	
	LCD_SetFont(&Font16); LCD_DisplayString(0, 76,"!#$%&'()*+,-.0123456789:;<=>?@ABCDEFGHIJKLMNOPQRST"); HAL_Delay(time);	
	LCD_SetFont(&Font12); LCD_DisplayString(0, 92,"!#$%&'()*+,-.0123456789:;<=>?@ABCDEFGHIJKLMNOPQRST"); HAL_Delay(time);	
																																		  
	LCD_SetColor(LCD_CYAN);                                                                              
	LCD_SetFont(&Font12); LCD_DisplayString(0,104,"!#$%&'()*+,-.0123456789:;<=>?@ABCDEFGHIJKLMNOPQRST"); HAL_Delay(time);	
	LCD_SetFont(&Font16); LCD_DisplayString(0,116,"!#$%&'()*+,-.0123456789:;<=>?@ABCDEFGHIJKLMNOPQRST"); HAL_Delay(time);	
	LCD_SetFont(&Font20); LCD_DisplayString(0,132,"!#$%&'()*+,-.0123456789:;<=>?@ABCDEFGHIJKLMNOPQRST"); HAL_Delay(time);		
	LCD_SetFont(&Font24); LCD_DisplayString(0,152,"!#$%&'()*+,-.0123456789:;<=>?@ABCDEFGHIJKLMNOPQRST"); HAL_Delay(time);		
	LCD_SetFont(&Font32); LCD_DisplayString(0,176,"!#$%&'()*+,-.0123456789:;<=>?@ABCDEFGHIJKLMNOPQRST"); HAL_Delay(time);	

	LCD_SetTextFont(&CH_Font24);			// ����2424��������,ASCII�����ӦΪ2412
	LCD_SetColor(LCD_YELLOW);				// ���û��ʣ���ɫ
	LCD_DisplayText(0, 230,"�ı���ʾ������ʾ���ĺ�ASCII�ַ���");
	LCD_DisplayText(0, 260,"�û��ɸ������󣬶��ֿ���������ɾ��");	

	
	LCD_SetTextFont(&CH_Font12);			// ����1212��������,ASCII�����ӦΪ1206
	LCD_SetColor(0Xff8AC6D1);						// ���û���
	LCD_DisplayText(28, 290,"1212�������壺���ͿƼ�");	
	
	LCD_SetTextFont(&CH_Font16);			// ����1616��������,ASCII�����ӦΪ1608
	LCD_SetColor(0XffC5E1A5);						// ���û���
	LCD_DisplayText(28, 310,"1616�������壺���ͿƼ�");		
	
	LCD_SetTextFont(&CH_Font20);			// ����2020��������,ASCII�����ӦΪ2010
	LCD_SetColor(0Xff2D248A);						// ���û���
	LCD_DisplayText(28, 335,"2020�������壺���ͿƼ�");		

	LCD_SetTextFont(&CH_Font24);			// ����2424��������,ASCII�����ӦΪ2412
	LCD_SetColor(0XffFF585D);						// ���û���
	LCD_DisplayText(28, 365,"2424�������壺���ͿƼ�");		

	LCD_SetTextFont(&CH_Font32);			// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0XffF6003C);						// ���û���
	LCD_DisplayText(28, 405,"3232�������壺���ͿƼ�");		

	HAL_Delay(2000);	// ��ʱ�ȴ�
}

/*************************************************************************************************
*	�� �� ��:	LCD_Test_Variable
*
*	��������:	������ʾ������������С��
*
*	˵    ��:	��	
*************************************************************************************************/

void LCD_Test_Variable (void)
{

	uint16_t i;					// ��������

	int32_t	a = 0;			// �����������������ڲ���
	int32_t	b = 0;			// �����������������ڲ���
	int32_t	c = 0;			// �����������������ڲ���

	double p = 3.1415926;	// ���帡�������������ڲ���
	double f = -1234.1234;	// ���帡�������������ڲ���
	
	
// ���Ƴ�ʼ���棬�������⡢LOGO�Լ�������>>>>>	
		
	LCD_SetBackColor(0xffB9EDF8); 			//	���ñ���ɫ��ʹ���Զ�����ɫ
	LCD_Clear(); 									//	������ˢ����ɫ
	
	LCD_SetTextFont(&CH_Font32);				// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0xff333333);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DisplayText(334, 160,"������ʾ");	// ��ʾ�ı�
	
	LCD_SetColor(0xfffd7923);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DrawImage(  280, 218, 240, 83, Image_FANKE_240x83) ;		// ��ʾLOGOͼƬ

	LCD_SetColor(LIGHT_YELLOW);		//	���û���ɫ
	for(i=0;i<150;i++)
   {
		LCD_FillRect(100,330,4*i,6);	// ���ƾ��Σ�ʵ�ּ��׽�������Ч��
		HAL_Delay(10);	
   }	
			
	
// ��ʾ����������������С��>>>>>	
	
	LCD_SetBackColor(LCD_BLACK); 			//	���ñ���ɫ
	LCD_Clear(); 								// ����
	
	LCD_SetTextFont(&CH_Font32);					// ����2424��������,ASCII�����ӦΪ2412
	LCD_SetColor(LCD_WHITE);						// ���û���,��ɫ
	
	LCD_DisplayText(28, 20,"����λ���ո�");	// ��ʾ�ı�	
	LCD_DisplayText(400,20,"����λ���0");		// ��ʾ�ı�		
	
	LCD_SetColor(LIGHT_CYAN);					// ���û��ʣ�����ɫ
	LCD_DisplayText(28, 60,"��������");				
	LCD_DisplayText(28,100,"��������");				
	LCD_DisplayText(28,140,"��������");					
				
	LCD_SetColor(LIGHT_YELLOW);				// ���û��ʣ�����ɫ		
	LCD_DisplayText(400, 60,"��������");	
	LCD_DisplayText(400,100,"��������");	
	LCD_DisplayText(400,140,"��������");	
			
	LCD_SetColor(LIGHT_RED);					// ���û���	������ɫ		
	LCD_DisplayText(28, 200,"��С����");	
	LCD_DisplayText(28, 240,"��С����");		
	
	for(i=0;i<100;i++)
   {
		LCD_SetColor(LIGHT_CYAN);									// ���û���	������ɫ	
		LCD_ShowNumMode(Fill_Space);								// ����λ���ո�
		LCD_DisplayNumber( 160, 60, a+i*125, 8) ;				// ��ʾ����		
		LCD_DisplayNumber( 160,100, b+i, 	 6) ;				// ��ʾ����			
		LCD_DisplayNumber( 160,140, c-i,     6) ;				// ��ʾ����			
		
		LCD_SetColor(LIGHT_YELLOW);								// ���û��ʣ�����ɫ	
		LCD_ShowNumMode(Fill_Zero);								// ����λ���0
		LCD_DisplayNumber( 560, 60, a+i*125, 8) ;				// ��ʾ����		
		LCD_DisplayNumber( 560,100, b+i, 	 6) ;				// ��ʾ����			
		LCD_DisplayNumber( 560,140, c-i,     6) ;				// ��ʾ����				
		
		LCD_SetColor(LIGHT_RED);									// ���û��ʣ�����ɫ			
		LCD_ShowNumMode(Fill_Space);								// ����λ���ո�		
		LCD_DisplayDecimals( 160, 200, p+i*0.1,  6,3);		// ��ʾС��	
		LCD_DisplayDecimals( 160, 240, f+i*0.01, 11,4);		// ��ʾС��		
		
		HAL_Delay(30);				
   }
	HAL_Delay(2500);		
	
}



/*************************************************************************************************
*	�� �� ��:	LCD_Test_FillRect
*
*	��������:	����������
*
*	˵    ��:	��	
*************************************************************************************************/

void LCD_Test_FillRect(void)
{
	uint16_t i;					// ��������
	
// ���Ƴ�ʼ���棬�������⡢LOGO�Լ�������>>>>>
		
	LCD_SetBackColor(0xffB9EDF8); 			//	���ñ���ɫ��ʹ���Զ�����ɫ
	LCD_Clear(); 									//	������ˢ����ɫ
	
	LCD_SetTextFont(&CH_Font32);				// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0xff333333);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DisplayText(334, 160,"���λ���");	// ��ʾ�ı�
	
	LCD_SetColor(0xfffd7923);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DrawImage(  280, 218, 240, 83, Image_FANKE_240x83) ;		// ��ʾLOGOͼƬ

	LCD_SetColor(LIGHT_YELLOW);		//	���û���ɫ
	for(i=0;i<150;i++)
   {
		LCD_FillRect(100,330,4*i,6);	// ���ƾ��Σ�ʵ�ּ��׽�������Ч��
		HAL_Delay(10);	
   }	
		
	
// �������>>>>>	
	
	LCD_SetBackColor(LCD_BLACK); //���ñ���ɫ
	LCD_Clear(); //������ˢ����ɫ

	LCD_SetFont(&Font16);
	LCD_SetColor(LCD_BLUE);	  	  LCD_FillRect(200,  3,500,16);	LCD_DisplayString(0,  3,"LCD_BLUE");		
	LCD_SetColor(LCD_GREEN);  	  LCD_FillRect(200, 25,500,16);  LCD_DisplayString(0, 25,"LCD_GREEN");		
	LCD_SetColor(LCD_RED);    	  LCD_FillRect(200, 47,500,16);  LCD_DisplayString(0, 47,"LCD_RED");			
	LCD_SetColor(LCD_CYAN);   	  LCD_FillRect(200, 69,500,16);  LCD_DisplayString(0, 69,"LCD_CYAN");		
	LCD_SetColor(LCD_MAGENTA);	  LCD_FillRect(200, 91,500,16);  LCD_DisplayString(0, 91,"LCD_MAGENTA");	
	LCD_SetColor(LCD_YELLOW); 	  LCD_FillRect(200,113,500,16);  LCD_DisplayString(0,113,"LCD_YELLOW");		
	LCD_SetColor(LCD_GREY);   	  LCD_FillRect(200,135,500,16);	LCD_DisplayString(0,135,"LCD_GREY");		

	LCD_SetColor(LIGHT_BLUE);	  LCD_FillRect(200,157,500,16);  LCD_DisplayString(0,157,"LIGHT_BLUE");		
	LCD_SetColor(LIGHT_GREEN);   LCD_FillRect(200,179,500,16);  LCD_DisplayString(0,179,"LIGHT_GREEN");	
	LCD_SetColor(LIGHT_RED);     LCD_FillRect(200,201,500,16);  LCD_DisplayString(0,201,"LIGHT_RED");	   
	LCD_SetColor(LIGHT_CYAN);    LCD_FillRect(200,223,500,16);  LCD_DisplayString(0,223,"LIGHT_CYAN");	   
	LCD_SetColor(LIGHT_MAGENTA); LCD_FillRect(200,245,500,16);  LCD_DisplayString(0,245,"LIGHT_MAGENTA");	
	LCD_SetColor(LIGHT_YELLOW);  LCD_FillRect(200,267,500,16);  LCD_DisplayString(0,267,"LIGHT_YELLOW");	
	LCD_SetColor(LIGHT_GREY);    LCD_FillRect(200,289,500,16);	LCD_DisplayString(0,289,"LIGHT_GREY");  	

	LCD_SetColor(DARK_BLUE);	  LCD_FillRect(200,311,500,16);  LCD_DisplayString(0,311,"DARK_BLUE");		
	LCD_SetColor(DARK_GREEN);    LCD_FillRect(200,333,500,16);  LCD_DisplayString(0,333,"DARK_GREEN");		
	LCD_SetColor(DARK_RED);      LCD_FillRect(200,355,500,16);  LCD_DisplayString(0,355,"DARK_RED");		
	LCD_SetColor(DARK_CYAN);     LCD_FillRect(200,377,500,16);  LCD_DisplayString(0,377,"DARK_CYAN");		
	LCD_SetColor(DARK_MAGENTA);  LCD_FillRect(200,399,500,16);  LCD_DisplayString(0,399,"DARK_MAGENTA");	
	LCD_SetColor(DARK_YELLOW);   LCD_FillRect(200,421,500,16);  LCD_DisplayString(0,421,"DARK_YELLOW");	
	LCD_SetColor(DARK_GREY);     LCD_FillRect(200,443,500,16);	LCD_DisplayString(0,443,"DARK_GREY");	

	HAL_Delay(3000);
}


/*************************************************************************************************
*	�� �� ��:	LCD_Test_Color
*
*	��������:	��ɫ����
*
*	˵    ��:	��	
*************************************************************************************************/

void LCD_Test_Color(void)
{
	uint16_t i;
	
	
// ���Ƴ�ʼ���棬�������⡢LOGO�Լ�������>>>>>

	LCD_SetBackColor(0xffB9EDF8); 			//	���ñ���ɫ��ʹ���Զ�����ɫ
	LCD_Clear(); 									//	������ˢ����ɫ
	
	LCD_SetTextFont(&CH_Font32);				// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0xff333333);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DisplayText(334, 160,"��ɫ����");	// ��ʾ�ı�
	
	LCD_SetColor(0xfffd7923);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DrawImage(  280, 218, 240, 83, Image_FANKE_240x83) ;		// ��ʾLOGOͼƬ

	LCD_SetColor(LIGHT_YELLOW);		//	���û���ɫ
	for(i=0;i<150;i++)
   {
		LCD_FillRect(100,330,4*i,6);	// ���ƾ��Σ�ʵ�ּ��׽�������Ч��
		HAL_Delay(10);	
   }		
	
// ��ɫ����>>>>>
		
	LCD_SetBackColor(LCD_BLACK); //���ñ���ɫ
	LCD_Clear(); //������ˢ����ɫ
	
	LCD_SetTextFont(&CH_Font32);			// ����2424��������,ASCII�����ӦΪ2412
	LCD_SetColor(LCD_WHITE);				// ���û�����ɫ
	
	LCD_DisplayText(42,70,"���İ��ͺţ�FK429M1");
	LCD_DisplayText(42,110,"��Ļ�ֱ��ʣ�800*480");		
	LCD_DisplayText(42,150,"RGB����ɫɫ�ײ���");	
	
	//ʹ�û��ߺ�����������ɫɫ��
	for(i=0;i<255;i++)
	{
		LCD_SetColor( LCD_RED-(i<<16) );
		LCD_DrawLine(30+2*i,  240,30+2*i,  280);	
		LCD_DrawLine(30+2*i+1,240,30+2*i+1,280);
	}
	for(i=0;i<255;i++)
	{
		LCD_SetColor( LCD_GREEN-(i<<8) );
		LCD_DrawLine(30+2*i,  290,30+2*i,  330);	
		LCD_DrawLine(30+2*i+1,290,30+2*i+1,330);
	}
	for(i=0;i<255;i++)
	{
		LCD_SetColor( LCD_BLUE-i );
		LCD_DrawLine(30+2*i,  350,30+2*i,  390);	
		LCD_DrawLine(30+2*i+1,350,30+2*i+1,390);
	}	
	HAL_Delay(3000);	
	
	
}

/*************************************************************************************************
*	�� �� ��:	LCD_Test_GrahicTest
*
*	��������:	2Dͼ�λ���
*
*	˵    ��:	��	
*************************************************************************************************/

void LCD_Test_GrahicTest(void)
{
	uint16_t time = 80;		// ��ʱʱ��
	uint16_t i;					// ��������
		
// ���Ƴ�ʼ���棬�������⡢LOGO�Լ�������>>>>>

	LCD_SetBackColor(0xffB9EDF8); 			//	���ñ���ɫ��ʹ���Զ�����ɫ
	LCD_Clear(); 									//	������ˢ����ɫ
	
	LCD_SetTextFont(&CH_Font32);				// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0xff333333);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DisplayText(334, 160,"��ͼ����");	// ��ʾ�ı�
	
	LCD_SetColor(0xfffd7923);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DrawImage(  280, 218, 240, 83, Image_FANKE_240x83) ;		// ��ʾLOGOͼƬ

	LCD_SetColor(LIGHT_YELLOW);		//	���û���ɫ
	for(i=0;i<150;i++)
   {
		LCD_FillRect(100,330,4*i,6);	// ���ƾ��Σ�ʵ�ּ��׽�������Ч��
		HAL_Delay(10);	
   }			
	
// 2Dͼ�λ���>>>>>>>
	
	LCD_SetBackColor(LCD_BLACK); //���ñ���ɫ
	LCD_Clear(); //������ˢ����ɫ	

	LCD_SetColor(LCD_WHITE);	
	LCD_DrawLine(0,0,799,0);		// ����Ļ�ı߽续��
	LCD_DrawLine(0,479,799,479);
	LCD_DrawLine(0,0,0,479);
	LCD_DrawLine(799,0,799,479);	
	
	LCD_SetColor(LCD_RED);    LCD_FillCircle(120,350,80);		//���Բ��
	LCD_SetColor(LCD_GREEN);  LCD_FillCircle(170,350,80); 	
	LCD_SetColor(LCD_BLUE);   LCD_FillCircle(220,350,80);  	
	
	LCD_SetColor(LIGHT_GREY);
	LCD_DrawLine(5,5,400,5);	HAL_Delay(time);			//��ֱ��
	LCD_DrawLine(5,10,300,10);	HAL_Delay(time);
	LCD_DrawLine(5,15,200,15); HAL_Delay(time);
	LCD_DrawLine(5,20,100,20);	HAL_Delay(time);	

	LCD_SetColor(LIGHT_CYAN);
	LCD_DrawCircle(600,120,100);	HAL_Delay(time);		//����Բ��
	LCD_DrawCircle(600,120,80);   HAL_Delay(time);
	LCD_DrawCircle(600,120,60);   HAL_Delay(time);
	LCD_DrawCircle(600,120,40);   HAL_Delay(time);
	
	LCD_SetColor(LCD_RED);	
	LCD_DrawRect(5,35,400,150);  HAL_Delay(time);			//���ƾ���
	LCD_DrawRect(30,50,350,120); HAL_Delay(time);
	LCD_DrawRect(55,65,300,90);  HAL_Delay(time);
	LCD_DrawRect(80,80,250,60);  HAL_Delay(time);

	LCD_SetColor(LIGHT_MAGENTA);	
	LCD_DrawEllipse(590,350,200,100); HAL_Delay(time);	//������Բ
	LCD_DrawEllipse(590,350,170,80);  HAL_Delay(time);
	LCD_DrawEllipse(590,350,140,60);  HAL_Delay(time);
	LCD_DrawEllipse(590,350,110,40);  HAL_Delay(time);

	HAL_Delay(2000);		
}
/*************************************************************************************************
*	�� �� ��:	LCD_Test_Image
*
*	��������:	ͼƬ��ʾ����
*
*	˵    ��:	��	
*************************************************************************************************/

void LCD_Test_Image(void)
{
	uint16_t i;					// ��������
		
// ���Ƴ�ʼ���棬�������⡢LOGO�Լ�������>>>>>
	
	LCD_SetBackColor(0xffB9EDF8); 			//	���ñ���ɫ��ʹ���Զ�����ɫ
	LCD_Clear(); 									//	������ˢ����ɫ
	
	LCD_SetTextFont(&CH_Font32);				// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0xff333333);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DisplayText(334, 160,"ͼƬ����");	// ��ʾ�ı�
	
	LCD_SetColor(0xfffd7923);					//	���û���ɫ��ʹ���Զ�����ɫ
	LCD_DrawImage(  280, 218, 240, 83, Image_FANKE_240x83) ;		// ��ʾLOGOͼƬ

	LCD_SetColor(LIGHT_YELLOW);		//	���û���ɫ
	for(i=0;i<150;i++)
   {
		LCD_FillRect(100,330,4*i,6);	// ���ƾ��Σ�ʵ�ּ��׽�������Ч��
		HAL_Delay(10);	
   }		
// ͼƬ����>>>>>>>	

	LCD_SetBackColor(LCD_BLACK); 			//	���ñ���ɫ
	LCD_Clear(); 								// ����
	
	LCD_SetColor( 0xffF6E58D);
	LCD_DrawImage( 185, 116, 83, 83, Image_Android_83x83) ;	// ��ʾͼƬ
	
	LCD_SetColor( 0xffFFAB91);
	LCD_DrawImage( 359, 124, 83, 83, Image_Cloud_83x83) ;		// ��ʾͼƬ
	
	LCD_SetColor( 0xff8AC6D1);
	LCD_DrawImage( 533, 124, 83, 83, Image_Folder_83x83) ;	// ��ʾͼƬ

	LCD_SetColor( 0xffDFF9FB);
	LCD_DrawImage( 185, 270, 83, 83, Image_Message_83x83) ;	// ��ʾͼƬ
	
	LCD_SetColor( 0xff9DD3A8);
	LCD_DrawImage( 359, 270, 83, 83, Image_Toys_83x83) ;		// ��ʾͼƬ
	
	LCD_SetColor( 0xffFF8753);
	LCD_DrawImage( 533, 270, 83, 83, Image_Video_83x83) ;		// ��ʾͼƬ

	HAL_Delay(2000);
	
	LCD_SetBackColor(LCD_WHITE); 			//	���ñ���ɫ
	LCD_Clear(); 								// ����
	LCD_SetColor( LCD_BLACK);				// ���û���
	LCD_DrawImage( 159, 120, 480, 239, Image_FANKE_480x239) ;	// ��ʾͼƬ
	HAL_Delay(2000);
	
}


/*************************************************************************************************
*	�� �� ��:	LCD_Test_Vertical
*
*	��������:	��������
*
*	˵    ��:	��	
*************************************************************************************************/

void  LCD_Test_Vertical(void)
{
	uint16_t i;
	uint16_t time = 100;	
	
	LCD_DisplayDirection(Direction_V); // �л���������ʾ
	
// ���Ƴ�ʼ���棬�������⡢LOGO�Լ�������>>>>>
	
	LCD_SetBackColor(0xffB9EDF8); 			//	���ñ���ɫ
	LCD_Clear(); 									//	������ˢ����ɫ
	
	LCD_SetTextFont(&CH_Font32);				// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0xff333333);					//	���û���ɫ
	LCD_DisplayText(180, 260,"������ʾ");	// ��ʾ�ı�
	
	LCD_SetColor(0xfffd7923);					//	���û���ɫ
	LCD_DrawImage( 120, 320, 240, 83, Image_FANKE_240x83) ;		// ��ʾLOGOͼƬ

	LCD_SetColor(LIGHT_YELLOW);		//	���û���ɫ
	for(i=0;i<130;i++)
   {
		LCD_FillRect(45,450,3*i,6);	// ���ƾ��Σ�ʵ�ּ��׽�������Ч��
		HAL_Delay(10);	
   }



// ������������>>>>>>>		
	
	LCD_SetBackColor(LCD_BLACK); //���ñ���ɫ
	LCD_Clear(); //������ˢ����ɫ		

	LCD_SetColor(LCD_WHITE);
	LCD_SetFont(&Font32);LCD_DisplayString(0, 0,"!#$%&'()*+,-.0123456789:;<=>"); HAL_Delay(time);		
	LCD_SetFont(&Font24);LCD_DisplayString(0,32,"!#$%&'()*+,-.0123456789:;<=>"); HAL_Delay(time);
	LCD_SetFont(&Font20);LCD_DisplayString(0,56,"!#$%&'()*+,-.0123456789:;<=>"); HAL_Delay(time);	
	LCD_SetFont(&Font16);LCD_DisplayString(0,76,"!#$%&'()*+,-.0123456789:;<=>"); HAL_Delay(time);	
	LCD_SetFont(&Font12);LCD_DisplayString(0,92,"!#$%&'()*+,-.0123456789:;<=>"); HAL_Delay(time);	
                                                                                                      
	LCD_SetColor(LCD_CYAN);                                                                            
	LCD_SetFont(&Font12);LCD_DisplayString(0,104,"!#$%&'()*+,-.0123456789:;<=>");HAL_Delay(time);	
	LCD_SetFont(&Font16);LCD_DisplayString(0,116,"!#$%&'()*+,-.0123456789:;<=>");HAL_Delay(time);	
	LCD_SetFont(&Font20);LCD_DisplayString(0,132,"!#$%&'()*+,-.0123456789:;<=>");HAL_Delay(time);		
	LCD_SetFont(&Font24);LCD_DisplayString(0,152,"!#$%&'()*+,-.0123456789:;<=>");HAL_Delay(time);		
	LCD_SetFont(&Font32);LCD_DisplayString(0,176,"!#$%&'()*+,-.0123456789:;<=>");HAL_Delay(time);	

	LCD_SetTextFont(&CH_Font24);			// ����2424��������,ASCII�����ӦΪ2412
	LCD_SetColor(LCD_YELLOW);				// ���û��ʣ���ɫ
	LCD_DisplayText(0, 230,"�ı���ʾ������ʾ���ĺ�ASCII�ַ���");
	LCD_DisplayText(0, 260,"�û��ɸ������󣬶��ֿ���������ɾ��");	

	LCD_SetTextFont(&CH_Font12);			// ����1212��������,ASCII�����ӦΪ1206
	LCD_SetColor(0Xff8AC6D1);						// ���û���
	LCD_DisplayText(28, 310,"1212�������壺���ͿƼ�");	
	
	LCD_SetTextFont(&CH_Font16);			// ����1616��������,ASCII�����ӦΪ1608
	LCD_SetColor(0XffC5E1A5);						// ���û���
	LCD_DisplayText(28, 330,"1616�������壺���ͿƼ�");		
	
	LCD_SetTextFont(&CH_Font20);			// ����2020��������,ASCII�����ӦΪ2010
	LCD_SetColor(0Xff2D248A);						// ���û���
	LCD_DisplayText(28, 355,"2020�������壺���ͿƼ�");		

	LCD_SetTextFont(&CH_Font24);			// ����2424��������,ASCII�����ӦΪ2412
	LCD_SetColor(0XffFF585D);						// ���û���
	LCD_DisplayText(28, 385,"2424�������壺���ͿƼ�");		

	LCD_SetTextFont(&CH_Font32);			// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0XffF6003C);						// ���û���
	LCD_DisplayText(28, 425,"3232�������壺���ͿƼ�");		
	
	LCD_SetTextFont(&CH_Font32);			// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0xff587058);
	LCD_DisplayText(40,500,   "STM32F429 LTDC ����");	
	LCD_DisplayText(40,500+40,"���İ��ͺţ�FK429M1");	
	LCD_DisplayText(40,500+80,"��Ļ�ֱ��ʣ�800*480");	
	
	HAL_Delay(2000);

	time = 80;
	
	LCD_SetBackColor(LCD_BLACK); //���ñ���ɫ
	LCD_Clear(); //������ˢ����ɫ	
	
	LCD_SetColor(LCD_RED);    LCD_FillCircle(120,285,80);		// ���Բ
	LCD_SetColor(LCD_GREEN);  LCD_FillCircle(170,285,80); 
	LCD_SetColor(LCD_BLUE);   LCD_FillCircle(220,285,80);  	
	
   LCD_SetColor(LCD_RED);	
	LCD_DrawRect(5,35,350,150);  HAL_Delay(time);		// ������
	LCD_DrawRect(30,50,300,120); HAL_Delay(time);
	LCD_DrawRect(55,65,250,90);  HAL_Delay(time);
	LCD_DrawRect(80,80,200,60);  HAL_Delay(time);
	
	LCD_SetColor(LIGHT_GREY);
	LCD_DrawLine(5,5,400,5);	HAL_Delay(time);	// ����
	LCD_DrawLine(5,10,300,10);	HAL_Delay(time);
	LCD_DrawLine(5,15,200,15); HAL_Delay(time);
	LCD_DrawLine(5,20,100,20);	HAL_Delay(time);		
	
   
   LCD_SetColor(LCD_YELLOW);	LCD_DrawCircle(100,480,80);   HAL_Delay(time);	// ��Բ
	LCD_SetColor(LCD_CYAN);		LCD_DrawCircle(150,480,80);   HAL_Delay(time);
   LCD_SetColor(LCD_MAGENTA); LCD_DrawCircle(200,480,80);   HAL_Delay(time);
   LCD_SetColor(LCD_RED);		LCD_DrawCircle(250,480,80);   HAL_Delay(time);

   LCD_SetColor(LIGHT_MAGENTA);	
	LCD_DrawEllipse(200,680,200,100); HAL_Delay(time);		// ����Բ
	LCD_DrawEllipse(200,680,170,80);  HAL_Delay(time);
	LCD_DrawEllipse(200,680,140,60);  HAL_Delay(time);
	LCD_DrawEllipse(200,680,110,40);  HAL_Delay(time);
                                                        
	HAL_Delay(2000);	
	
	LCD_SetBackColor(LCD_BLACK); //���ñ���ɫ
	LCD_Clear(); //������ˢ����ɫ

	LCD_SetFont(&Font16);
	LCD_SetColor(LCD_BLUE);	  	  LCD_FillRect(150,  3,250,16);	LCD_DisplayString(0,  3,"LCD_BLUE"); 			
	LCD_SetColor(LCD_GREEN);  	  LCD_FillRect(150, 25,250,16);  LCD_DisplayString(0, 25,"LCD_GREEN");	
	LCD_SetColor(LCD_RED);    	  LCD_FillRect(150, 47,250,16);  LCD_DisplayString(0, 47,"LCD_RED");	 	
	LCD_SetColor(LCD_CYAN);   	  LCD_FillRect(150, 69,250,16);  LCD_DisplayString(0, 69,"LCD_CYAN");		
	LCD_SetColor(LCD_MAGENTA);	  LCD_FillRect(150, 91,250,16);  LCD_DisplayString(0, 91,"LCD_MAGENTA");	
	LCD_SetColor(LCD_YELLOW); 	  LCD_FillRect(150,113,250,16);  LCD_DisplayString(0,113,"LCD_YELLOW");		
	LCD_SetColor(LCD_GREY);   	  LCD_FillRect(150,135,250,16);	LCD_DisplayString(0,135,"LCD_GREY");		
                                                                                          
	LCD_SetColor(LIGHT_BLUE);	  LCD_FillRect(150,157,250,16);  LCD_DisplayString(0,157,"LIGHT_BLUE");		
	LCD_SetColor(LIGHT_GREEN);   LCD_FillRect(150,179,250,16);  LCD_DisplayString(0,179,"LIGHT_GREEN");	
	LCD_SetColor(LIGHT_RED);     LCD_FillRect(150,201,250,16);  LCD_DisplayString(0,201,"LIGHT_RED");	   
	LCD_SetColor(LIGHT_CYAN);    LCD_FillRect(150,223,250,16);  LCD_DisplayString(0,223,"LIGHT_CYAN");	   
	LCD_SetColor(LIGHT_MAGENTA); LCD_FillRect(150,245,250,16);  LCD_DisplayString(0,245,"LIGHT_MAGENTA");	
	LCD_SetColor(LIGHT_YELLOW);  LCD_FillRect(150,267,250,16);  LCD_DisplayString(0,267,"LIGHT_YELLOW");	
	LCD_SetColor(LIGHT_GREY);    LCD_FillRect(150,289,250,16);	LCD_DisplayString(0,289,"LIGHT_GREY");  	
	                                                                                       
	LCD_SetColor(DARK_BLUE);	  LCD_FillRect(150,311,250,16);  LCD_DisplayString(0,311,"DARK_BLUE");		
	LCD_SetColor(DARK_GREEN);    LCD_FillRect(150,333,250,16);  LCD_DisplayString(0,333,"DARK_GREEN");		
	LCD_SetColor(DARK_RED);      LCD_FillRect(150,355,250,16);  LCD_DisplayString(0,355,"DARK_RED");		
	LCD_SetColor(DARK_CYAN);     LCD_FillRect(150,377,250,16);  LCD_DisplayString(0,377,"DARK_CYAN");		
	LCD_SetColor(DARK_MAGENTA);  LCD_FillRect(150,399,250,16);  LCD_DisplayString(0,399,"DARK_MAGENTA");	
	LCD_SetColor(DARK_YELLOW);   LCD_FillRect(150,421,250,16);  LCD_DisplayString(0,421,"DARK_YELLOW");	
	LCD_SetColor(DARK_GREY);     LCD_FillRect(150,443,250,16);	LCD_DisplayString(0,443,"DARK_GREY");	
	
	//ˢ��ɫ��
	for(i=0;i<255;i++){LCD_SetColor( LCD_RED-(i<<16) ); 	LCD_DrawLine(150+i,465,150+i,481);}
	for(i=0;i<255;i++){LCD_SetColor( LCD_GREEN-(i<<8) );	LCD_DrawLine(150+i,487,150+i,503);}
	for(i=0;i<255;i++){LCD_SetColor( LCD_BLUE-i );		 	LCD_DrawLine(150+i,509,150+i,525);}
	for(i=0;i<255;i++){LCD_SetColor( LIGHT_RED-(i<<16) );	LCD_DrawLine(150+i,531,150+i,547);}LCD_DisplayString(0,531,"0xFF8080");
	for(i=0;i<255;i++){LCD_SetColor( LIGHT_GREEN-(i<<8) );LCD_DrawLine(150+i,553,150+i,568);}LCD_DisplayString(0,553,"0x80FF80");
	for(i=0;i<255;i++){LCD_SetColor( LIGHT_BLUE-i );		LCD_DrawLine(150+i,575,150+i,591);}LCD_DisplayString(0,575,"0x8080FF");	
	for(i=0;i<255;i++){LCD_SetColor( DARK_BLUE-(i<<16) );	LCD_DrawLine(150+i,597,150+i,613);}LCD_DisplayString(0,597,"0x000080");	
	for(i=0;i<255;i++){LCD_SetColor( DARK_BLUE-(i<<8) );	LCD_DrawLine(150+i,619,150+i,635);}LCD_DisplayString(0,619,"0x000080");	
	for(i=0;i<255;i++){LCD_SetColor( DARK_RED-i );			LCD_DrawLine(150+i,641,150+i,657);}LCD_DisplayString(0,641,"0x800000");		
	for(i=0;i<255;i++){LCD_SetColor( DARK_RED-(i<<8));		LCD_DrawLine(150+i,663,150+i,679);}LCD_DisplayString(0,663,"0x800000");	
	for(i=0;i<255;i++){LCD_SetColor( DARK_GREEN-(i<<16)); LCD_DrawLine(150+i,685,150+i,701);}LCD_DisplayString(0,685,"0x008000");	
	for(i=0;i<255;i++){LCD_SetColor( DARK_GREEN-i);			LCD_DrawLine(150+i,707,150+i,723);}LCD_DisplayString(0,707,"0x008000");	
	for(i=0;i<255;i++){LCD_SetColor( DARK_CYAN-(i<<16));	LCD_DrawLine(150+i,729,150+i,745);}LCD_DisplayString(0,729,"0x008080");	
	for(i=0;i<255;i++){LCD_SetColor( DARK_YELLOW-i);		LCD_DrawLine(150+i,751,150+i,767);}LCD_DisplayString(0,751,"0x808000");	
	for(i=0;i<255;i++){LCD_SetColor( DARK_MAGENTA-(i<<8));LCD_DrawLine(150+i,773,150+i,789);}LCD_DisplayString(0,773,"0x800080");	

	HAL_Delay(3000);		
	LCD_DisplayDirection(Direction_H); // �л��غ�����ʾ
}







