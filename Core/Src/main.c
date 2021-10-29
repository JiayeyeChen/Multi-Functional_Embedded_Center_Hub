#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"
#include "system_periphrals.h"
#include "usbd_cdc_if.h"
#include "usb.h"
#include "lcd_rgb.h"
#include "lcd_pwm.h"
#include "sdram.h"
#include "touch_800x480.h"

osThreadId_t AK10_CalibrationnTaskHandle;
const osThreadAttr_t AK10_Calibration_attributes = {
  .name = "AK10_Calibration",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t LCDTaskHandle;
const osThreadAttr_t LCDTask_attributes = {
  .name = "LCD",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

void SystemClock_Config(void);
void AK10Calibration_Task(void *argument);
void LCD_Task(void *argument);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_FATFS_Init();
  SystemPeriphral_Init();
  USB_Init();
  
  USART1_Init();
	MX_FMC_Init();
  LTDC_Init();
  Touch_Init();
  
  LCD_SetColor(0xff0E0E0E);				//	设置画笔色
	LCD_SetBackColor(0xff8AC6D1); 			//	设置背景色
	LCD_Clear(); 						//	清屏，刷背景色

	LCD_SetTextFont(&CH_Font32);
	LCD_DisplayText( 42, 20,"电容触摸测试");
	LCD_DisplayText( 42, 70,"核心板型号：FK429M1");
	LCD_DisplayText( 42, 120,"屏幕分辨率：800*480");		


	
	LCD_DisplayString(44, 170,"X1:       Y1:");	
	LCD_DisplayString(44, 220,"X2:       Y2:");	
	LCD_DisplayString(44, 270,"X3:       Y3:");	
	LCD_DisplayString(44, 320,"X4:       Y4:");		
	LCD_DisplayString(44, 370,"X5:       Y5:");		
	
	LCD_SetColor(0xffEA7070);	//设置画笔颜色	
  
  osKernelInitialize();
  AK10_CalibrationnTaskHandle = osThreadNew(AK10Calibration_Task, NULL, &AK10_Calibration_attributes);
  LCDTaskHandle = osThreadNew(LCD_Task, NULL, &LCDTask_attributes);
  osKernelStart();
  while (1){}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	uint16_t LCD_PLLSAIN = 0;		//	用于倍频的PLLSAIN参数，可取范围为50~432
	uint8_t  LCD_PLLSAIR = 3;		//	用于分频的PLLSAIR参数，可取范围为2~7
	uint8_t  LCD_CLKDIV	= 2;		//	LCD时钟分频参数，默认设置为2分频
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
  
  
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	// LCD_CLK = LCD_PLLSAIN / LCD_PLLSAIR / RCC_PLLSAIDIVR_2
	LCD_PLLSAIN = LCD_CLK * LCD_PLLSAIR * LCD_CLKDIV;	//	根据需要使用的LCD时钟计算PLLSAIN参数，可取范围为50~432
	PeriphClkInitStruct.PLLSAI.PLLSAIN 	= LCD_PLLSAIN;			// 设置 PLLSAIN
	PeriphClkInitStruct.PLLSAI.PLLSAIR 	= LCD_PLLSAIR;			// 设置 PLLSAIR，这里取值为3
	PeriphClkInitStruct.PLLSAIDivR 		= RCC_PLLSAIDIVR_4;	// 为了方便计算，这里使用2分频，所以 LCD_CLKDIV 定义为2
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}

void AK10Calibration_Task(void *argument)
{
  for(;;)
  {
    GPIO_Digital_Filtered_Input(&hButtonOnboardKey, 30);
    LED_Blink(&hLEDBlue, 2);
//    CDC_Transmit_HS((uint8_t*)usbtxtest, 6);
    osDelay(10);
  }
}

void LCD_Task(void *argument)
{
  for(;;)
  {
    LED_Blink(&hLEDYellowGreen, 15);
    
    Touch_Scan();	// 触摸扫描
		
		if(touchInfo.flag == 1)
		{
		  LCD_DisplayNumber(110,170,touchInfo.x[0],4);	// 显示第1组坐标
			LCD_DisplayNumber(260,170,touchInfo.y[0],4);
			                                        
			LCD_DisplayNumber(110,220,touchInfo.x[1],4);	// 显示第2组坐标
			LCD_DisplayNumber(260,220,touchInfo.y[1],4);
		                                           
			LCD_DisplayNumber(110,270,touchInfo.x[2],4);	// 显示第3组坐标
			LCD_DisplayNumber(260,270,touchInfo.y[2],4);
		                                           
			LCD_DisplayNumber(110,320,touchInfo.x[3],4);	// 显示第4组坐标
			LCD_DisplayNumber(260,320,touchInfo.y[3],4);
		                                           
			LCD_DisplayNumber(110,370,touchInfo.x[4],4);	// 显示第5组坐标
			LCD_DisplayNumber(260,370,touchInfo.y[4],4);
		}
    
    osDelay(20);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}
